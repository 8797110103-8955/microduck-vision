use std::env;
use std::fs;
use std::path::PathBuf;

use quote::quote;
use syn::{FnArg, Item, Type, UseTree, parse_quote};

fn injected(argument: &FnArg) -> bool {
    let FnArg::Typed(argument) = argument else { return false };
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
    const CLIENT: &str = "../src-tauri/src/client.rs";
    println!("cargo:rerun-if-changed={CLIENT}");

    // Use the desktop's contract imports and signatures without compiling the command bodies.
    let source = syn::parse_file(&fs::read_to_string(CLIENT)?)?;
    let items = source.items.into_iter().filter_map(|item| match item {
        Item::Use(import) if shared_types(&import.tree) => Some(Item::Use(import)),
        Item::Fn(mut command) if command.attrs.iter().any(|attr| {
            let mut path = attr.path().segments.iter();
            matches!((path.next(), path.next(), path.next()),
                (Some(root), Some(name), None) if root.ident == "tauri" && name.ident == "command")
        }) => {
            command.sig.inputs = command.sig.inputs.into_iter().filter(|arg| !injected(arg)).collect();
            command.block = Box::new(parse_quote!({ unreachable!("type generation only") }));
            Some(Item::Fn(command))
        }
        _ => None,
    });

    let output = PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("client.rs");
    let client = quote!(#(#items)*).to_string();
    if fs::read_to_string(&output).ok().as_deref() != Some(&client) {
        fs::write(output, client)?;
    }
    Ok(())
}
