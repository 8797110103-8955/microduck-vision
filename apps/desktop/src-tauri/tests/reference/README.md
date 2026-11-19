# Native simulation CPU reference

This is a fixed vendor CPU reference, accepted for this PRD on 2026-11-19. It is
not an upstream published golden. Python is only a reference exporter; the product
loads MuJoCo directly through its Rust/C binding and never starts Python.

Run from the repository root with CPython 3.12.16 and the exact dependency set:

```powershell
uv venv --python 3.12.16 .agentic/reference-env
uv pip install --python .agentic/reference-env/Scripts/python.exe -r apps/desktop/src-tauri/tests/reference/requirements.txt
.agentic/reference-env/Scripts/python.exe apps/desktop/src-tauri/tests/reference/native_mujoco.py
cargo test --manifest-path apps/desktop/src-tauri/Cargo.toml native_mujoco_matches_pinned_vendor_cpu
```

The exporter extracts `load_mujoco_with_bam` from the pinned vendor script without
importing its GUI. It imports Rhoban BAM at the commit in `requirements.txt` and
uses MuJoCo 3.10.0 and the scene directly under
`vendor/microduck_rl/src/mjlab_microduck/robot/microduck`, which is the same source
that `build.rs` copies to the output directory. It uses the deployed official scene,
home pose, trunk z=0.125, identity
orientation, 5ms physics steps and 20ms control ticks. Noise and randomization are
disabled. CPU motor encoders and the training encoder-through-backlash adapter
are reported separately. Delay is a deterministic whole-frame absolute physics
step deadline; schedules are 0, fixed 3, fixed 6 and cycling 3..6.

The fixed target fixture is checked in under `tests/fixtures`. It contains five
300-tick cases including hold, reversal and movement. Tolerances are declared in
the exporter before comparison. Tests compare every frame, vector widths, time,
motor/output feedback, base pose/velocity, IMU, motor torque and exact contact
geometry pairs. Quaternion comparisons account for q/-q equivalence. The report
is `.agentic/native-mujoco-comparison.json`.

For the closed-loop stage, place the official cached `velstand.onnx` at
`.agentic/velstand.onnx`. The artifact used for the accepted run has SHA-256
`ce894d30b3343036d499bfac1102bdaa58d59a21c1b58401ecb70fb6a2f63e0a`.
Its release version is not inferred from the filename. Rerun the exporter, then:

```powershell
$env:MICRODUCK_OFFICIAL_ONNX = "$PWD/.agentic/velstand.onnx"
$env:ORT_DYLIB_PATH = "$PWD/.agentic/reference-env/Lib/site-packages/onnxruntime/capi/onnxruntime.dll"
$env:MICRODUCK_OFFICIAL_REFERENCE_TRACE = "$PWD/.agentic/native-policy-reference.json"
$env:MICRODUCK_OFFICIAL_PRODUCT_TRACE = "$PWD/.agentic/native-product-policy-trace.json"
cargo test --manifest-path apps/desktop/src-tauri/Cargo.toml official_velstand_native_parity_for_300_ticks -- --ignored
```

The policy trace uses the same model, initialization, 3..6 delay and current qvel
contract. Forward command is zero for ticks 0..74, 0.2 for 75..224 and zero for
225..299. ONNX input/output is float32; simulation is float64. Feedback/IMU are
sampled before execution, physics/render states after execution. The product test
aligns these boundaries explicitly and writes maximum absolute errors and the
first failure to `.agentic/native-policy-comparison.json`.

The product must not generate its own expected trace. Changing reference code,
dependencies, source assets or model requires regenerating and reviewing the
fixture provenance and reporting the new hashes.
