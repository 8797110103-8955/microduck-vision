"""XL330 uses the official task entity and its selected encoder feedback."""

from mjlab_adapter import configure_robot


def install(env_cfg, profile, run_dir, *, domain_randomization):
    if profile.model_kind != "xl330":
        raise ValueError("expected the XL330 training profile")
    configure_robot(
        env_cfg, profile, run_dir,
        encoder_through_backlash=profile.simulation["response"]["encoder_through_backlash"],
        domain_randomization=domain_randomization,
    )
