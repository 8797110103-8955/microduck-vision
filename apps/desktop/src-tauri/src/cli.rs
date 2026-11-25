//! Command-line entry points for training and preview without opening the desktop window.

use std::{
    ffi::OsString,
    fs::File,
    path::{Path, PathBuf},
};

use clap::{Args, Parser, Subcommand};
use microduck_training::deepmimic::TrainingOptions;
use serde::{Serialize, de::DeserializeOwned};

use crate::{
    training::{
        preview,
        process::{self, HeadlessPlan},
    },
    types::{Initialization, InitializationMode, TrainingPreviewInput, TrainingStartInput},
};

#[derive(Parser)]
#[command(
    name = "",
    no_binary_name = true,
    about = "Train DeepMimic policies and preview saved checkpoints",
    after_help = "See train --help and preview --help for command options."
)]
struct Cli {
    /// Directory containing machines.toml. On Windows, defaults to the desktop app's config directory.
    #[arg(long, global = true)]
    config_dir: Option<PathBuf>,
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Train DeepMimic on a configured virtual machine.
    Train(TrainArgs),
    /// Replay a saved training checkpoint on a virtual machine.
    ///
    /// Runs without opening the desktop window. Prints a JSON report with
    /// displacement, minimum height, maximum tilt, and upright fraction. Saves
    /// report.json and samples.jsonl under trainings/previews/preview-<id>/.
    #[command(
        after_help = "Example\n  preview --machine MACHINE_ID --run-id RUN_ID --checkpoint CHECKPOINT_ID"
    )]
    Preview {
        /// Virtual machine ID from machines.toml.
        #[arg(long)]
        machine: String,
        /// Run ID of the training round that owns the checkpoint.
        #[arg(long)]
        run_id: String,
        /// ID of the saved checkpoint within the training round.
        #[arg(long)]
        checkpoint: String,
        #[arg(
            long,
            help = format!("Number of control steps, from 1 to {}. Defaults to {}.", preview::MAX_STEPS, preview::DEFAULT_STEPS),
            value_parser = clap::value_parser!(u32).range(1..=i64::from(preview::MAX_STEPS))
        )]
        steps: Option<u32>,
    },
}

#[derive(Args)]
struct TrainArgs {
    /// Virtual machine ID from machines.toml.
    #[arg(long)]
    machine: String,
    /// ID of the published DeepMimic motion bundle.
    #[arg(long)]
    bundle: String,
    #[arg(long)]
    experiment_id: Option<String>,
    #[arg(long, value_parser = clap::value_parser!(u32).range(1..))]
    num_envs: u32,
    #[arg(long, value_parser = clap::value_parser!(u32).range(1..))]
    iterations: u32,
    #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u32).range(..=2147483647))]
    seed: u32,
    /// Run the PPO baseline.
    #[arg(long)]
    baseline: bool,
    #[arg(long, requires = "checkpoint")]
    parent_run_id: Option<String>,
    #[arg(long, requires = "parent_run_id")]
    checkpoint: Option<String>,
    /// How to inherit the parent checkpoint. Defaults to resume when a parent is supplied.
    #[arg(long, requires = "parent_run_id", value_parser = ["resume", "adapt"])]
    mode: Option<String>,
    #[arg(long, requires = "mode")]
    reset_normalization: bool,
    /// Read an existing walking plan JSON.
    #[arg(long, conflicts_with_all = ["baseline", "teacher_onnx"])]
    plan: Option<PathBuf>,
    #[command(flatten)]
    walking: WalkingArgs,
}

#[derive(Args, Serialize)]
struct WalkingArgs {
    /// Teacher ONNX recorded in the motion bundle. Enables the walking plan.
    #[arg(long, conflicts_with = "baseline")]
    #[serde(skip_serializing_if = "Option::is_none")]
    teacher_onnx: Option<PathBuf>,
    #[arg(long, requires = "teacher_onnx", value_parser = |arg: &str| parse_bounded(arg, 0.001))]
    #[serde(skip_serializing_if = "Option::is_none")]
    learning_rate: Option<f64>,
    #[arg(long, requires = "teacher_onnx", value_parser = |arg: &str| parse_bounded(arg, 1.0))]
    #[serde(skip_serializing_if = "Option::is_none")]
    exploration_std: Option<f64>,
    #[arg(long, requires = "teacher_onnx", value_parser = |arg: &str| parse_bounded(arg, 5.0))]
    #[serde(skip_serializing_if = "Option::is_none")]
    forward_weight: Option<f64>,
    /// Wait for a decision at each review point.
    #[arg(long, requires = "teacher_onnx")]
    hold_reviews: bool,
}

fn parse_bounded(argument: &str, max: f64) -> Result<f64, String> {
    let value = argument.parse::<f64>().map_err(|error| error.to_string())?;
    if 0.0 < value && value <= max {
        Ok(value)
    } else {
        Err(format!(
            "expected a number greater than 0 and at most {max}"
        ))
    }
}

impl TrainArgs {
    fn into_request(self) -> Result<(TrainingStartInput, Option<HeadlessPlan>), String> {
        let mode = if self.mode.as_deref() == Some("adapt") {
            InitializationMode::Adapt
        } else {
            InitializationMode::Resume
        };
        if self.reset_normalization && mode != InitializationMode::Adapt {
            return Err("--reset-normalization requires --mode adapt".into());
        }
        let initialization =
            self.parent_run_id
                .zip(self.checkpoint)
                .map(|(parent_run_id, checkpoint)| Initialization {
                    mode,
                    parent_run_id,
                    checkpoint,
                    reset_normalization: self.reset_normalization,
                });
        let options = TrainingOptions {
            bundle_id: self.bundle,
            num_envs: self.num_envs,
            iterations: self.iterations,
            seed: self.seed,
            baseline: self.baseline,
        };
        options.validate()?;
        let input = TrainingStartInput {
            initialization,
            replace_existing_experiment: false,
            program: "uv".into(),
            args: Vec::new(),
            virtual_machine_id: self.machine,
            policy_id: None,
            experiment_id: self.experiment_id,
            recipe: options.method().into(),
            teacher_motion_provenance: options.bundle_id.clone(),
            dataset_path: None,
            deepmimic: Some(options),
        };
        let mut walking = self.walking;
        let plan = if let Some(path) = self.plan {
            Some(HeadlessPlan::File(existing_path(&path)?))
        } else if let Some(path) = &walking.teacher_onnx {
            walking.teacher_onnx = Some(existing_path(path)?);
            Some(HeadlessPlan::Inline(
                serde_json::to_value(walking).map_err(|error| error.to_string())?,
            ))
        } else {
            None
        };
        Ok((input, plan))
    }
}

pub(crate) fn run(args: &[OsString]) -> i32 {
    #[cfg(windows)]
    attach_console();
    let result = match args.first().and_then(|arg| arg.to_str()) {
        Some("--training-run" | "--training-preview") => run_json(args),
        _ => match Cli::try_parse_from(args) {
            Ok(cli) => run_flags(cli),
            Err(error) => {
                let _ = error.print();
                return error.exit_code();
            }
        },
    };
    match result {
        Ok(()) => 0,
        Err(error) => {
            eprintln!("{error}");
            1
        }
    }
}

fn run_flags(cli: Cli) -> Result<(), String> {
    let config = cli
        .config_dir
        .or_else(default_config_directory)
        .ok_or("pass --config-dir with the directory containing machines.toml")?;
    match cli.command {
        Command::Train(args) => {
            let (input, plan) = args.into_request()?;
            process::run_cli(&config, input, plan)
        }
        Command::Preview {
            machine,
            run_id,
            checkpoint,
            steps,
        } => preview::run_cli(
            &config,
            &TrainingPreviewInput {
                run_id,
                checkpoint_id: checkpoint,
                virtual_machine_id: machine,
                steps,
            },
        ),
    }
}

fn run_json(args: &[OsString]) -> Result<(), String> {
    match args {
        [command, config, request, plan @ ..] if command == "--training-run" && plan.len() <= 1 => {
            let plan = plan
                .first()
                .map(|path| existing_path(Path::new(path)).map(HeadlessPlan::File))
                .transpose()?;
            process::run_cli(Path::new(config), read_request(Path::new(request))?, plan)
        }
        [command, config, request] if command == "--training-preview" => {
            preview::run_cli(Path::new(config), &read_request(Path::new(request))?)
        }
        [command, ..] if command == "--training-run" => {
            Err("usage: --training-run <config-dir> <request.json> [plan.json]".into())
        }
        _ => Err("usage: --training-preview <config-dir> <request.json>".into()),
    }
}

fn read_request<T: DeserializeOwned>(path: &Path) -> Result<T, String> {
    let file = File::open(path).map_err(|error| format!("{}: {error}", path.display()))?;
    serde_json::from_reader(file).map_err(|error| format!("{}: {error}", path.display()))
}

fn existing_path(path: &Path) -> Result<PathBuf, String> {
    path.canonicalize()
        .map_err(|error| format!("{}: {error}", path.display()))
}

fn default_config_directory() -> Option<PathBuf> {
    #[cfg(windows)]
    return std::env::var_os("APPDATA")
        .map(|path| PathBuf::from(path).join("microduck-vision/config"));
    #[cfg(not(windows))]
    None
}

#[cfg(windows)]
fn attach_console() {
    use windows_sys::Win32::System::Console::{
        ATTACH_PARENT_PROCESS, AttachConsole, GetStdHandle, STD_ERROR_HANDLE, STD_OUTPUT_HANDLE,
    };

    // SAFETY: These calls use documented handle selectors and the parent-process sentinel.
    // Preserve inherited stdout and stderr when a caller captures the JSON stream.
    unsafe {
        if GetStdHandle(STD_OUTPUT_HANDLE).is_null() && GetStdHandle(STD_ERROR_HANDLE).is_null() {
            AttachConsole(ATTACH_PARENT_PROCESS);
        }
    }
}
