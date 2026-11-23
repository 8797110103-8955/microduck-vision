//! Python sources embedded for installation into the training workspace.

pub const PYTHON_ADAPTERS: &[(&str, &str)] = &[
    (
        "simulation_profile.py",
        include_str!("../python/body_adapters/simulation_profile.py"),
    ),
    (
        "mjlab_adapter.py",
        include_str!("../python/body_adapters/mjlab_adapter.py"),
    ),
    (
        "xl330/__init__.py",
        include_str!("../python/body_adapters/xl330/__init__.py"),
    ),
    (
        "xl330/mjlab_body.py",
        include_str!("../python/body_adapters/xl330/mjlab_body.py"),
    ),
    (
        "hd1910/__init__.py",
        include_str!("../python/body_adapters/hd1910/__init__.py"),
    ),
    (
        "hd1910/mjlab_body.py",
        include_str!("../python/body_adapters/hd1910/mjlab_body.py"),
    ),
    (
        "hd1910_compatible/__init__.py",
        include_str!("../python/body_adapters/hd1910_compatible/__init__.py"),
    ),
    (
        "hd1910_compatible/mjlab_body.py",
        include_str!("../python/body_adapters/hd1910_compatible/mjlab_body.py"),
    ),
];
