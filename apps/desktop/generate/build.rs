use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::PathBuf;

use quote::{ToTokens, quote};
use syn::{FnArg, Item, Type, UseTree, parse_quote};

fn injected(argument: &FnArg) -> bool {
    let FnArg::Typed(argument) = argument else {
        return false;
    };
    let ty = match argument.ty.as_ref() {
        Type::Reference(reference) => reference.elem.as_ref(),
        ty => ty,
    };
    let Type::Path(ty) = ty else { return false };
    ty.path.segments.last().is_some_and(|segment| {
        matches!(
            segment.ident.to_string().as_str(),
            "State" | "AppHandle" | "Window" | "Webview" | "WebviewWindow" | "Request"
        )
    })
}

fn shared_types(tree: &UseTree) -> bool {
    matches!(tree, UseTree::Path(root) if root.ident == "crate"
        && matches!(root.tree.as_ref(), UseTree::Path(module) if module.ident == "types"))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    const COMMAND_REGISTRY: &str = "../src-tauri/src/commands/mod.rs";
    const COMMAND_FILES: &[&str] = &[
        "../src-tauri/src/commands/actions.rs",
        "../src-tauri/src/commands/loop_control.rs",
        "../src-tauri/src/commands/machines.rs",
        "../src-tauri/src/commands/machines/physical.rs",
        "../src-tauri/src/commands/machines/registers.rs",
        "../src-tauri/src/commands/machines/virtual_machine.rs",
        "../src-tauri/src/commands/policy.rs",
        "../src-tauri/src/commands/scene.rs",
        "../src-tauri/src/commands/training.rs",
        "../src-tauri/src/state/types.rs",
    ];
    println!("cargo:rerun-if-changed={COMMAND_REGISTRY}");
    for path in COMMAND_FILES {
        println!("cargo:rerun-if-changed={path}");
    }

    // Use the desktop command signatures without compiling command bodies or application
    // services. Every command file is parsed independently, then its shared `crate::types`
    // imports and Tauri command functions are combined into one type-generation module.
    let mut imports = Vec::new();
    let mut seen_imports = HashSet::new();
    let mut commands = Vec::new();
    for path in COMMAND_FILES {
        if *path == "../src-tauri/src/state/types.rs" {
            continue;
        }
        let source = syn::parse_file(&fs::read_to_string(path)?)?;
        for item in source.items {
            match item {
                Item::Use(import) if shared_types(&import.tree) => {
                    let rendered = import.to_token_stream().to_string();
                    if seen_imports.insert(rendered) {
                        imports.push(Item::Use(import));
                    }
                }
                Item::Fn(mut command) if command.attrs.iter().any(|attr| {
                    let mut path = attr.path().segments.iter();
                    matches!((path.next(), path.next(), path.next()),
                        (Some(root), Some(name), None) if root.ident == "tauri" && name.ident == "command")
                }) => {
                    command.sig.inputs = command
                        .sig
                        .inputs
                        .into_iter()
                        .filter(|arg| !injected(arg))
                        .collect();
                    command.block = Box::new(parse_quote!({
                        unreachable!("type generation only")
                    }));
                    commands.push(Item::Fn(command));
                }
                _ => {}
            }
        }
    }

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let client_output = out_dir.join("client.rs");
    let client = quote!(#(#imports)* #(#commands)*).to_string();
    if fs::read_to_string(&client_output).ok().as_deref() != Some(&client) {
        fs::write(client_output, client)?;
    }

    // Reuse the production command list. The generated contract module flattens all command
    // functions, so convert each production domain prefix to `client::` before including it.
    let registry = syn::parse_file(&fs::read_to_string(COMMAND_REGISTRY)?)?;
    let builder = registry
        .items
        .into_iter()
        .find_map(|item| match item {
            Item::Fn(function) if function.sig.ident == "command_builder" => Some(function),
            _ => None,
        })
        .ok_or("command_builder not found")?;
    let mut builder_source = quote!(#builder).to_string();
    // `quote::ToTokens` renders paths with whitespace around `::`, so normalize
    // those rendered prefixes rather than relying on the source spelling.
    for prefix in [
        "machines :: physical ::",
        "machines :: registers ::",
        "machines :: virtual_machine ::",
        "machines ::",
        "actions ::",
        "policy ::",
        "training ::",
        "loop_control ::",
        "scene ::",
    ] {
        builder_source = builder_source.replace(prefix, "client ::");
    }
    let commands_output = out_dir.join("commands.rs");
    let commands = format!(
        "use super::client;\nuse tauri_specta::{{Builder, collect_commands}};\n{builder_source}"
    );
    if fs::read_to_string(&commands_output).ok().as_deref() != Some(&commands) {
        fs::write(commands_output, commands)?;
    }

    Ok(())
}
