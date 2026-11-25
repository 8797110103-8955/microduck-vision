//! Strict source scanning and bounded intermediate export for DeepMimic conversion.

use std::cell::Cell;
use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::Path;

use microduck_body_runtime::models::assets::{OfficialAssets, load_official_body};
use microduck_training::deepmimic::{ConversionRequest, IntervalSummary, ReadFailure, ScanReport};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};

use crate::types::{RecordingFrame, RuntimeState};

/// Absolute tolerance for the native 50 Hz clock, in seconds.
const TIME_TOLERANCE_S: f64 = 1e-8;
const CONTROL_S: f64 = 0.02;
const PHYSICS_STEPS: u64 = 4;

struct CountedRead<'a> {
    reader: BufReader<File>,
    position: &'a Cell<u64>,
}

impl Read for CountedRead<'_> {
    fn read(&mut self, data: &mut [u8]) -> std::io::Result<usize> {
        let count = self.reader.read(data)?;
        self.position.set(self.position.get() + count as u64);
        Ok(count)
    }
}

pub(super) fn source_digest(path: &Path) -> Result<String, String> {
    let mut file = File::open(path).map_err(|error| error.to_string())?;
    let mut digest = Sha256::new();
    let mut buffer = [0; 65536];
    loop {
        let count = file.read(&mut buffer).map_err(|error| error.to_string())?;
        if count == 0 {
            break;
        }
        digest.update(&buffer[..count]);
    }
    Ok(format!("{:x}", digest.finalize()))
}

fn runtime_metadata(metadata: &Value) -> &Value {
    &metadata["runtime"]
}

fn valid_source(frame: &RecordingFrame, metadata: &Value) -> Result<(), &'static str> {
    let runtime = runtime_metadata(metadata);
    let profile = &runtime["simulation_execution"];
    if profile["model_kind"] != "xl330"
        || profile["body"]["id"] != "official"
        || profile["simulation"]["scenario"]["robot_xml"] != "robot_groundcontact_backlash.xml"
        || profile["periods"]["control_s"].as_f64() != Some(CONTROL_S)
        || profile["periods"]["physics_s"].as_f64() != Some(0.005)
        || profile["periods"]["physics_substeps"] != 4
    {
        return Err("unsupported_source_profile");
    }
    if frame.status.runtime_state != RuntimeState::Running
        || frame.status.sensor_failures != 0
        || frame.status.active_body.is_none()
    {
        return Err("invalid_runtime_state");
    }
    let policy = frame.policy.as_ref().ok_or("missing_policy")?;
    let simulation = frame
        .status
        .simulation
        .as_ref()
        .ok_or("missing_simulation")?;
    let names = runtime["joint_order"]
        .as_array()
        .ok_or("missing_joint_names")?;
    let active: Vec<_> = names
        .iter()
        .enumerate()
        .filter(|(_, name)| *name != "mouth")
        .map(|(i, _)| i)
        .collect();
    if names.len() != 15
        || active.len() != 14
        || policy.observation.len() != 61
        || policy.action.len() != 14
        || policy.valid.len() != names.len()
        || simulation.motor_positions.len() != names.len()
        || simulation.passive_positions.len() != names.len()
        || active.iter().any(|i| !policy.valid[*i])
    {
        return Err("invalid_joint_or_policy_layout");
    }
    if !simulation.time.is_finite()
        || simulation.time < 0.0
        || simulation
            .motor_positions
            .iter()
            .chain(&simulation.passive_positions)
            .chain(&simulation.trunk_position)
            .chain(&simulation.trunk_quaternion)
            .chain(&simulation.base_velocity)
            .any(|value| !value.is_finite())
        || policy
            .observation
            .iter()
            .chain(&policy.action)
            .any(|v| !v.is_finite())
    {
        return Err("non_finite_state");
    }
    Ok(())
}

fn boundary(previous: &RecordingFrame, next: &RecordingFrame) -> Option<&'static str> {
    let old = previous
        .status
        .simulation
        .as_ref()
        .expect("validated simulation");
    let new = next
        .status
        .simulation
        .as_ref()
        .expect("validated simulation");
    if previous.segment_id != next.segment_id {
        return Some("segment_changed");
    }
    if previous.status.execution_generation != next.status.execution_generation
        || previous.status.runtime_generation != next.status.runtime_generation
        || previous.status.config_revision != next.status.config_revision
        || previous.duck_id != next.duck_id
    {
        return Some("execution_changed");
    }
    if (new.time - old.time - CONTROL_S).abs() > TIME_TOLERANCE_S {
        return Some("simulation_time_discontinuity");
    }
    if previous.status.tick.checked_add(1) != Some(next.status.tick) {
        return Some("tick_discontinuity");
    }
    if old.physics_step.checked_add(PHYSICS_STEPS) != Some(new.physics_step) {
        return Some("physics_step_discontinuity");
    }
    None
}

/// Visit frames in file order. The caller receives only validated frames and current
/// segment metadata. Read errors retain the last completely decoded frame location.
fn visit(
    config: Option<&Path>,
    id: &str,
    mut consume: impl FnMut(u32, &RecordingFrame, &Value) -> Result<(), String>,
) -> Result<ScanReport, String> {
    let path = super::recording_path(config, id)?.join("runtime.mcap");
    let hash = source_digest(&path)?;
    let position = Cell::new(0);
    let input = CountedRead {
        reader: BufReader::new(File::open(&path).map_err(|e| e.to_string())?),
        position: &position,
    };
    let mut report = ScanReport {
        recording_id: id.into(),
        mcap_sha256: hash,
        intervals: Vec::new(),
        frames_read: 0,
        excluded_reasons: BTreeMap::new(),
        read_failure: None,
    };
    let mut metadata = BTreeMap::new();
    let mut recording_context = json!({});
    let mut previous = None;
    for message in mcap::io::MessageReader::new(input) {
        let parsed = message
            .map_err(|error| error.to_string())
            .and_then(|message| match message.channel.topic.as_str() {
                "/duck/events" => {
                    let event: Value =
                        serde_json::from_slice(&message.data).map_err(|e| e.to_string())?;
                    if event["event"] == "recording_started" {
                        recording_context = event["context"]["metadata"].clone();
                    }
                    if event["event"] == "segment_started" {
                        let segment = event["context"]["segment_id"]
                            .as_u64()
                            .and_then(|v| u32::try_from(v).ok())
                            .ok_or("Invalid segment ID")?;
                        let mut context = recording_context.clone();
                        context["runtime"] = event["context"]["metadata"].clone();
                        metadata.insert(segment, context);
                    }
                    Ok(None)
                }
                "/duck/runtime" => super::decode_frame(&message.data)
                    .map(Some)
                    .map_err(|e| e.to_string()),
                _ => Ok(None),
            });
        let frame = match parsed {
            Ok(Some(frame)) => frame,
            Ok(None) => continue,
            Err(reason) => {
                report.read_failure = Some(ReadFailure {
                    bytes_read: position.get(),
                    last_reliable_frame: report
                        .intervals
                        .last()
                        .map(|interval| interval.last_frame),
                    reason,
                });
                break;
            }
        };
        let index = report.frames_read;
        report.frames_read = report
            .frames_read
            .checked_add(1)
            .ok_or("Too many source frames")?;
        let context = metadata.get(&frame.segment_id).unwrap_or(&Value::Null);
        let invalid = if frame.recording_id != id {
            Err("recording_id_mismatch")
        } else {
            valid_source(&frame, context)
        };
        if let Err(reason) = invalid {
            *report.excluded_reasons.entry(reason.into()).or_default() += 1;
            previous = None;
            continue;
        }
        let reason = previous.as_ref().and_then(|old| boundary(old, &frame));
        if previous.is_none() || reason.is_some() {
            let sim = frame.status.simulation.as_ref().unwrap();
            let mut excluded_reasons = BTreeMap::new();
            if let Some(reason) = reason {
                excluded_reasons.insert(reason.into(), 1);
            }
            report.intervals.push(IntervalSummary {
                id: format!("{}-{index}", &report.mcap_sha256[..16]),
                body_id: context["runtime"]["simulation_execution"]["body"]["id"]
                    .as_str()
                    .unwrap()
                    .into(),
                model_kind: context["runtime"]["simulation_execution"]["model_kind"]
                    .as_str()
                    .unwrap()
                    .into(),
                control_s: CONTROL_S,
                segment_id: frame.segment_id,
                execution_generation: frame.status.execution_generation,
                first_frame: index,
                last_frame: index,
                first_simulation_s: sim.time,
                last_simulation_s: sim.time,
                frames: 0,
                excluded_reasons,
            });
        }
        let interval = report.intervals.last_mut().unwrap();
        interval.last_frame = index;
        interval.last_simulation_s = frame.status.simulation.as_ref().unwrap().time;
        interval.frames += 1;
        consume(index, &frame, context)?;
        previous = Some(frame);
    }
    if source_digest(&path)? != report.mcap_sha256 {
        return Err("Recording changed during source scan".into());
    }
    Ok(report)
}

pub(crate) fn scan(config: Option<&Path>, id: &str) -> Result<ScanReport, String> {
    visit(config, id, |_, _, _| Ok(()))
}

pub(crate) fn export_source(
    config: Option<&Path>,
    request: &ConversionRequest,
    directory: &Path,
) -> Result<Value, String> {
    request.validate()?;
    let report = scan(config, &request.recording_id)?;
    if let Some(error) = &report.read_failure {
        return Err(format!(
            "MCAP read failed after frame {:?}, byte {}: {}",
            error.last_reliable_frame, error.bytes_read, error.reason
        ));
    }
    let interval = report
        .intervals
        .iter()
        .find(|interval| interval.id == request.interval_id)
        .ok_or("Source interval no longer exists")?;
    if request.start_s < 0.0
        || request.end_s
            > interval.last_simulation_s - interval.first_simulation_s + TIME_TOLERANCE_S
    {
        return Err("Selected times exceed source interval".into());
    }
    let frames_path = directory.join("source-frames.jsonl");
    let mut writer = BufWriter::new(File::create(&frames_path).map_err(|e| e.to_string())?);
    let mut selected = Vec::new();
    let mut metadata = None;
    let second = visit(config, &request.recording_id, |index, frame, context| {
        let time = frame.status.simulation.as_ref().unwrap().time - interval.first_simulation_s;
        if index >= interval.first_frame
            && index <= interval.last_frame
            && time >= request.start_s - TIME_TOLERANCE_S
            && time <= request.end_s + TIME_TOLERANCE_S
        {
            serde_json::to_writer(&mut writer, frame).map_err(|e| e.to_string())?;
            writeln!(writer).map_err(|e| e.to_string())?;
            selected.push((index, frame.status.simulation.as_ref().unwrap().time));
            metadata.get_or_insert_with(|| context.clone());
        }
        Ok(())
    })?;
    if second.read_failure.is_some() || second.mcap_sha256 != report.mcap_sha256 {
        return Err("Recording changed before export completed".into());
    }
    if selected.len() < 3 {
        return Err("Select at least three continuous simulation frames".into());
    }
    writer.flush().map_err(|e| e.to_string())?;
    let metadata = metadata.unwrap();
    let profile = &runtime_metadata(&metadata)["simulation_execution"];
    let assets = OfficialAssets::Embedded;
    let export =
        microduck_body_runtime::training::export_recorded_native_simulation(profile, &assets)?;
    let simulation_export: Value = serde_json::from_slice(&export).map_err(|e| e.to_string())?;
    let body = load_official_body(&assets).map_err(|e| e.to_string())?;
    let mut template = body.template().clone();
    template.assets.display_mjcf = profile["simulation"]["scenario"]["robot_xml"]
        .as_str()
        .ok_or("Missing source robot")?
        .into();
    let scene =
        duck_body::OfficialScene::from_body(&template, &assets).map_err(|e| e.to_string())?;
    let names: Vec<_> = scene.model().bodies.iter().map(|body| &body.name).collect();
    let context = json!({"format_version":1, "recording_id":request.recording_id,
        "interval":interval,"first_frame":selected[0].0,"last_frame":selected.last().unwrap().0,
        "first_simulation_s":selected[0].1,"last_simulation_s":selected.last().unwrap().1,
        "frames":selected.len(),"mcap_sha256":report.mcap_sha256,"metadata":metadata,
        "simulation_export":simulation_export,"body_names":names,"excluded_reasons":report.excluded_reasons});
    std::fs::write(
        directory.join("source-context.json"),
        serde_json::to_vec(&context).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    Ok(context)
}
