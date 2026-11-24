"""HD1910 with the resolved XL330 simulation response corrections."""

from dataclasses import replace

from mjlab_adapter import configure_robot


def install(env_cfg, profile, run_dir, *, domain_randomization):
    if profile.model_kind != "hd1910-compatible":
        raise ValueError("expected the compatible HD1910 training profile")
    simulation = profile.simulation
    response = simulation["response"]
    motor = dict(simulation["motor"])
    # Preserve kt/R for command torque while scaling kt squared/R for back-EMF.
    motor["kt"] *= response["back_emf_scale"]
    motor["resistance"] *= response["back_emf_scale"]
    for key in (
        "friction_base", "friction_stribeck", "load_friction_motor",
        "load_friction_external", "load_friction_motor_stribeck",
        "load_friction_external_stribeck", "load_friction_motor_quad",
        "load_friction_external_quad",
    ):
        motor[key] *= response["friction_scale"]
    resolved = replace(profile, execution={
        **profile.execution,
        "simulation": {
            **simulation,
            "motor": motor,
            "response": {**response, "back_emf_scale": 1.0, "friction_scale": 1.0},
        },
    })
    configure_robot(
        env_cfg, resolved, run_dir,
        encoder_through_backlash=response["encoder_through_backlash"],
        domain_randomization=domain_randomization,
    )
