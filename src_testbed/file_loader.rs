use std::time::Duration;

use bevy::{
    asset::{io::Reader, Asset, AssetLoader, LoadContext},
    reflect::Reflect,
};

#[derive(Asset, Reflect, Debug, Default, Clone)]
#[reflect(opaque)]
#[reflect(Debug)]
pub struct FileBytes {
    pub data: Vec<u8>,
}

#[derive(Asset, Reflect, Debug, Default, Clone)]
pub struct FileBytesLoader;

#[non_exhaustive]
#[derive(Debug)]
pub enum FileBytesLoaderError {
    Unknown,
}

impl core::fmt::Display for FileBytesLoaderError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "FileBytesLoaderError")
    }
}

impl core::error::Error for FileBytesLoaderError {}

impl AssetLoader for FileBytesLoader {
    type Asset = FileBytes;
    type Settings = ();
    type Error = FileBytesLoaderError;

    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &(),
        _load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let mut bytes = Vec::new();

        reader
            .read_to_end(&mut bytes)
            .await
            .map_err(|_| FileBytesLoaderError::Unknown)?;

        Ok(FileBytes { data: bytes })
    }

    fn extensions(&self) -> &[&str] {
        &["gltf", "glb"]
    }
}
