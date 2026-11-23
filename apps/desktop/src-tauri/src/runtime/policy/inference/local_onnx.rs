//! Stateless ONNX models exported by the desktop training entry points.

use std::collections::BTreeMap;

use duck_control::model::NUM_JOINTS;
use duck_control::obs::{ACTION_LEN, OBS_LEN, Observation};
use loop_body::LoopError;
use ort::session::{Session, builder::GraphOptimizationLevel};
use ort::tensor::TensorElementType;
use ort::value::{Tensor, ValueType};
use serde::Deserialize;

#[derive(Debug, Clone, PartialEq, Eq, Deserialize, serde::Serialize)]
#[serde(untagged)]
pub(crate) enum Dimension {
    Fixed(i64),
    Named(String),
}

#[derive(Debug, Clone, Deserialize, serde::Serialize)]
pub(crate) struct TensorSpec {
    pub(crate) name: String,
    pub(crate) element_type: String,
    pub(crate) dimensions: Vec<Dimension>,
}

impl TensorSpec {
    pub(crate) fn validate(&self, name: &str, width: usize) -> bool {
        self.name == name
            && self.element_type == "FLOAT"
            && matches!(
                self.dimensions.as_slice(),
                [Dimension::Fixed(1) | Dimension::Named(_), Dimension::Fixed(n)] if *n == width as i64
            )
    }

    fn matches(&self, outlet: &ort::value::Outlet) -> bool {
        let ValueType::Tensor {
            ty: TensorElementType::Float32,
            shape,
            dimension_symbols,
        } = outlet.dtype()
        else {
            return false;
        };
        outlet.name() == self.name
            && shape.len() == self.dimensions.len()
            && self
                .dimensions
                .iter()
                .enumerate()
                .all(|(i, dimension)| match dimension {
                    Dimension::Fixed(size) => shape[i] == *size,
                    Dimension::Named(name) => shape[i] < 0 && dimension_symbols[i] == *name,
                })
    }
}

#[derive(Debug, Clone)]
pub(crate) struct LocalPolicySpec {
    pub(crate) input: TensorSpec,
    pub(crate) output: TensorSpec,
    pub(crate) home: [f64; NUM_JOINTS],
    pub(crate) metadata: BTreeMap<String, String>,
}

pub(super) struct LocalPolicy {
    session: Session,
    input: String,
    output: String,
}

impl LocalPolicy {
    pub(super) fn load(bytes: &[u8], spec: &LocalPolicySpec) -> Result<Self, LoopError> {
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let session = Session::builder()
                .and_then(|builder| builder.with_intra_threads(1))
                .and_then(|builder| builder.with_optimization_level(GraphOptimizationLevel::Level3))
                .and_then(|builder| builder.commit_from_memory(bytes))
                .map_err(|error| LoopError::Controller(error.to_string()))?;
            if !matches!(session.inputs(), [input] if spec.input.matches(input))
                || !matches!(session.outputs(), [output] if spec.output.matches(output))
            {
                return Err(LoopError::Controller(
                    "ONNX 图与训练产物张量合同不一致".into(),
                ));
            }
            if !spec.metadata.is_empty() {
                let metadata = session
                    .metadata()
                    .map_err(|error| LoopError::Controller(error.to_string()))?;
                if spec
                    .metadata
                    .iter()
                    .any(|(key, value)| metadata.custom(key).as_ref() != Some(value))
                {
                    return Err(LoopError::Controller(
                        "ONNX 元数据与训练产物合同不一致".into(),
                    ));
                }
            }
            let mut policy = Self {
                session,
                input: spec.input.name.clone(),
                output: spec.output.name.clone(),
            };
            policy.infer(&Observation::zeroed())?;
            Ok(policy)
        }))
        .map_err(|_| LoopError::Controller("ONNX Runtime 初始化失败".into()))?
    }

    pub(super) fn infer(
        &mut self,
        observation: &Observation,
    ) -> Result<[f32; ACTION_LEN], LoopError> {
        let fail = |error: ort::Error| LoopError::Controller(error.to_string());
        if !observation.as_slice().iter().all(|value| value.is_finite()) {
            return Err(LoopError::Controller("策略观测包含非有限值".into()));
        }
        let input =
            Tensor::from_array(([1, OBS_LEN], observation.as_slice().to_vec())).map_err(fail)?;
        let outputs = self
            .session
            .run(ort::inputs![self.input.as_str() => input])
            .map_err(fail)?;
        let (shape, actions) = outputs[self.output.as_str()]
            .try_extract_tensor::<f32>()
            .map_err(fail)?;
        if shape.as_ref() != [1, ACTION_LEN as i64]
            || !actions.iter().all(|value| value.is_finite())
        {
            return Err(LoopError::Controller("训练策略输出形状或数值无效".into()));
        }
        actions
            .try_into()
            .map_err(|_| LoopError::Controller("训练策略输出长度无效".into()))
    }
}
