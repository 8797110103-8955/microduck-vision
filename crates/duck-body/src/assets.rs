//! Read-only MJCF resources addressed by a path within the caller's source.

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::BodyError;

pub trait AssetResolver {
    fn read(&self, id: &Path) -> Result<Cow<'_, [u8]>, BodyError>;
}

pub struct FileAssets;

impl AssetResolver for FileAssets {
    fn read(&self, id: &Path) -> Result<Cow<'_, [u8]>, BodyError> {
        std::fs::read(id).map(Cow::Owned).map_err(|error| {
            if error.kind() == std::io::ErrorKind::NotFound {
                BodyError::MissingAsset(id.to_owned())
            } else {
                BodyError::AssetRead(id.to_owned(), error.to_string())
            }
        })
    }
}

impl AssetResolver for BTreeMap<PathBuf, Vec<u8>> {
    fn read(&self, id: &Path) -> Result<Cow<'_, [u8]>, BodyError> {
        self.get(id)
            .map(|bytes| Cow::Borrowed(bytes.as_slice()))
            .ok_or_else(|| BodyError::MissingAsset(id.to_owned()))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BodyAssets {
    pub display_mjcf: PathBuf,
    pub pose_scene: Option<PathBuf>,
}

impl BodyAssets {
    pub fn validate(&self, resolver: &dyn AssetResolver) -> Result<(), BodyError> {
        for source in std::iter::once(&self.display_mjcf).chain(&self.pose_scene) {
            visit_mjcf(resolver, source, &mut |id, node| {
                if node.has_tag_name("mesh")
                    && let Some(file) = node.attribute("file")
                {
                    resolver.read(&mesh_directory(id, node.document()).join(file))?;
                }
                Ok(())
            })?;
        }
        Ok(())
    }
}

pub(crate) fn read_xml<'a>(
    resolver: &'a dyn AssetResolver,
    id: &Path,
) -> Result<Cow<'a, str>, BodyError> {
    match resolver.read(id)? {
        Cow::Owned(bytes) => String::from_utf8(bytes)
            .map(Cow::Owned)
            .map_err(|error| BodyError::AssetRead(id.to_owned(), error.to_string())),
        Cow::Borrowed(bytes) => std::str::from_utf8(bytes)
            .map(Cow::Borrowed)
            .map_err(|error| BodyError::AssetRead(id.to_owned(), error.to_string())),
    }
}

pub(crate) fn mesh_directory(id: &Path, doc: &roxmltree::Document<'_>) -> PathBuf {
    let directory = doc
        .root_element()
        .children()
        .find(|node| node.has_tag_name("compiler"))
        .and_then(|node| node.attribute("meshdir"));
    directory.map_or_else(
        || id.parent().unwrap_or(Path::new("")).to_owned(),
        |directory| relative_asset(id, directory),
    )
}

fn relative_asset(id: &Path, reference: &str) -> PathBuf {
    id.parent().unwrap_or(Path::new("")).join(reference)
}

/// Walk the source in document order, expanding includes at their declaration.
pub(crate) fn visit_mjcf(
    resolver: &dyn AssetResolver,
    id: &Path,
    visit: &mut impl FnMut(&Path, roxmltree::Node<'_, '_>) -> Result<(), BodyError>,
) -> Result<(), BodyError> {
    fn walk_source(
        resolver: &dyn AssetResolver,
        id: &Path,
        stack: &mut Vec<PathBuf>,
        visit: &mut impl FnMut(&Path, roxmltree::Node<'_, '_>) -> Result<(), BodyError>,
    ) -> Result<(), BodyError> {
        if stack.iter().any(|source| source == id) || stack.len() >= 32 {
            return Err(BodyError::Mjcf(format!(
                "invalid include chain at {}",
                id.display()
            )));
        }
        stack.push(id.to_owned());
        let xml = read_xml(resolver, id)?;
        let doc = roxmltree::Document::parse(&xml)
            .map_err(|error| BodyError::Mjcf(format!("{}: {error}", id.display())))?;
        for node in doc.descendants().filter(|node| node.is_element()) {
            if node.has_tag_name("include") {
                let file = node.attribute("file").ok_or_else(|| {
                    BodyError::Mjcf(format!("include in {} is missing file", id.display()))
                })?;
                walk_source(resolver, &relative_asset(id, file), stack, visit)?;
            } else {
                visit(id, node)?;
            }
        }
        stack.pop();
        Ok(())
    }
    walk_source(resolver, id, &mut Vec::new(), visit)
}
