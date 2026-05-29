//! Entry points that parse an MJCF source into a [`MjcfRobot`].

use std::path::{Path, PathBuf};

use mjcf_rs::model::Model;

use super::conversion::Conversion;
use super::{MjcfLoaderOptions, MjcfRobot};

impl MjcfRobot {
    /// Parse an MJCF file from disk and convert it.
    pub fn from_file(
        path: impl AsRef<Path>,
        options: MjcfLoaderOptions,
    ) -> anyhow::Result<(Self, Model)> {
        let path = path.as_ref();
        let model = Model::from_file(path)?;
        let base = path
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_else(|| PathBuf::from("."));
        let robot = Self::from_model(&model, options, &base);
        Ok((robot, model))
    }

    /// Parse an MJCF string and convert it. `<include>` files resolve
    /// relative to `base_dir`.
    pub fn from_str(
        xml: &str,
        options: MjcfLoaderOptions,
        base_dir: impl AsRef<Path>,
    ) -> anyhow::Result<(Self, Model)> {
        let base_dir = base_dir.as_ref().to_path_buf();
        let model = Model::from_str(xml, &base_dir)?;
        let robot = Self::from_model(&model, options, &base_dir);
        Ok((robot, model))
    }

    /// Convert an already-parsed [`Model`] into rapier objects.
    pub fn from_model(model: &Model, options: MjcfLoaderOptions, base_dir: &Path) -> Self {
        let mut conv = Conversion::new(model, &options, base_dir);
        conv.run();
        conv.into_robot()
    }
}
