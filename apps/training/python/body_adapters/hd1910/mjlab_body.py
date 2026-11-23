"""HD1910 uses the native motor fit and task collision geometry."""

from mjlab_adapter import configure_robot


def install(env_cfg, profile, run_dir, *, domain_randomization):
    if profile.model_kind != "hd1910":
        raise ValueError("expected the native HD1910 training profile")
    configure_robot(
        env_cfg, profile, run_dir, encoder_through_backlash=False,
        domain_randomization=domain_randomization,
    )
