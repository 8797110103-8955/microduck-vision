//! Vendor assets addressed by stable paths, with a common file and memory interface.

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Component, Path, PathBuf};

use duck_body::{
    AssetResolver, BodyError, BodyProfile, FileAssets, OfficialScene, official_assets,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

static EMBEDDED: &[(&str, &[u8])] = include!(concat!(env!("OUT_DIR"), "/official_assets.rs"));

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SourceAsset {
    pub path: String,
    pub sha256: String,
}

/// The file resolver is rooted explicitly for development and source verification. Released
/// applications use embedded bytes with the same path and digest for every asset.
#[derive(Debug, Clone, Copy)]
pub enum OfficialAssets<'a> {
    Embedded,
    Files(&'a Path),
}

impl AssetResolver for OfficialAssets<'_> {
    fn read(&self, id: &Path) -> Result<Cow<'_, [u8]>, BodyError> {
        let name = resource_id(id)?;
        let index = EMBEDDED
            .binary_search_by_key(&name.as_str(), |(path, _)| *path)
            .map_err(|_| BodyError::MissingAsset(id.to_owned()))?;
        match self {
            Self::Embedded => Ok(Cow::Borrowed(EMBEDDED[index].1)),
            Self::Files(root) => FileAssets
                .read(&root.join(name))
                .map(|data| Cow::Owned(data.into_owned())),
        }
    }
}

fn resource_id(path: &Path) -> Result<String, BodyError> {
    let mut segments = Vec::new();
    for part in path.components() {
        match part {
            Component::Normal(name) => segments.push(name.to_str().ok_or_else(|| {
                BodyError::AssetRead(path.to_owned(), "asset path must be UTF-8".into())
            })?),
            Component::CurDir => (),
            _ => {
                return Err(BodyError::AssetRead(
                    path.to_owned(),
                    "asset path must be relative to the vendor root".into(),
                ));
            }
        }
    }
    Ok(segments.join("/"))
}

pub fn load_official_body(source: &dyn AssetResolver) -> Result<BodyProfile, BodyError> {
    BodyProfile::official(official_assets(Path::new("")), source)
}

pub fn display_scene(
    body: &BodyProfile,
    source: &dyn AssetResolver,
) -> Result<OfficialScene, duck_body::SceneError> {
    OfficialScene::for_official_body(body.template(), source)
}

/// An immutable bundle for one MJCF compilation. Relative includes and mesh references retain
/// their vendor paths. It owns any file-backed bytes for the lifetime of compilation.
pub struct AssetBundle<'a> {
    files: BTreeMap<String, Cow<'a, [u8]>>,
}

impl<'a> AssetBundle<'a> {
    pub fn read(source: &'a dyn AssetResolver, roots: &[&str]) -> Result<Self, BodyError> {
        let mut files = BTreeMap::new();
        let mut pending = BTreeSet::new();
        for root in roots {
            load_xml(source, Path::new(root), &mut pending, &mut files)?;
        }
        Ok(Self { files })
    }

    pub fn files(&self) -> impl Iterator<Item = (&str, &[u8])> {
        self.files
            .iter()
            .map(|(path, bytes)| (path.as_str(), bytes.as_ref()))
    }

    pub fn manifest(&self) -> Vec<SourceAsset> {
        self.files()
            .map(|(path, bytes)| SourceAsset {
                path: path.into(),
                sha256: format!("{:x}", Sha256::digest(bytes)),
            })
            .collect()
    }
}

impl AssetResolver for AssetBundle<'_> {
    fn read(&self, id: &Path) -> Result<Cow<'_, [u8]>, BodyError> {
        let key = resource_id(id)?;
        self.files
            .get(&key)
            .map(|bytes| Cow::Borrowed(bytes.as_ref()))
            .ok_or_else(|| BodyError::MissingAsset(id.to_owned()))
    }
}

fn load_xml<'a>(
    source: &'a dyn AssetResolver,
    path: &Path,
    pending: &mut BTreeSet<String>,
    files: &mut BTreeMap<String, Cow<'a, [u8]>>,
) -> Result<(), BodyError> {
    let id = resource_id(path)?;
    if files.contains_key(&id) {
        return Ok(());
    }
    if pending.len() >= 32 || !pending.insert(id.clone()) {
        return Err(BodyError::Mjcf(format!("invalid include chain at {id}")));
    }
    let bytes = source.read(path)?;
    let xml = std::str::from_utf8(&bytes)
        .map_err(|error| BodyError::AssetRead(path.to_owned(), error.to_string()))?;
    let doc = roxmltree::Document::parse(xml)
        .map_err(|error| BodyError::Mjcf(format!("{id}: {error}")))?;
    let directory = path.parent().unwrap_or(Path::new(""));
    let mesh_dir = directory.join(
        doc.root_element()
            .children()
            .find(|node| node.has_tag_name("compiler"))
            .and_then(|node| node.attribute("meshdir"))
            .unwrap_or(""),
    );
    for node in doc.descendants() {
        if node.has_tag_name("include") {
            let name = node
                .attribute("file")
                .ok_or_else(|| BodyError::Mjcf(format!("include in {id} has no file")))?;
            load_xml(source, &directory.join(name), pending, files)?;
        } else if node.has_tag_name("mesh")
            && let Some(name) = node.attribute("file")
        {
            let path: PathBuf = mesh_dir.join(name);
            let key = resource_id(&path)?;
            if let std::collections::btree_map::Entry::Vacant(entry) = files.entry(key) {
                entry.insert(source.read(&path)?);
            }
        }
    }
    pending.remove(&id);
    files.insert(id, bytes);
    Ok(())
}
