use std::path::PathBuf;

/// Errors that can be returned while parsing an MJCF document.
#[derive(thiserror::Error, Debug)]
pub enum ParseError {
    /// The XML failed to parse.
    #[error("XML parse error: {0}")]
    Xml(#[from] roxmltree::Error),

    /// I/O error while reading a file (or one of its `<include>` targets).
    #[error("I/O error reading {path:?}: {source}")]
    Io {
        /// Path that failed.
        path: PathBuf,
        /// Underlying I/O error.
        #[source]
        source: std::io::Error,
    },

    /// An attribute couldn't be parsed.
    #[error("invalid attribute `{attr}` on <{tag}>: {message}")]
    BadAttribute {
        /// Tag name.
        tag: String,
        /// Attribute name.
        attr: String,
        /// Reason it failed.
        message: String,
    },

    /// An attribute had an unexpected value.
    #[error("unexpected `{attr}=\"{value}\"` on <{tag}>: {message}")]
    BadAttributeValue {
        /// Tag name.
        tag: String,
        /// Attribute name.
        attr: String,
        /// Bad value.
        value: String,
        /// Reason.
        message: String,
    },

    /// `<include>` referenced a file that wasn't found, or formed a cycle.
    #[error("invalid <include file=\"{file:?}\">: {message}")]
    BadInclude {
        /// Filename.
        file: PathBuf,
        /// Reason.
        message: String,
    },

    /// `<element class="…">` referenced a class that doesn't exist.
    #[error("element <{tag}> referenced unknown <default class=\"{class}\">")]
    UnknownClass {
        /// Tag name.
        tag: String,
        /// Class name.
        class: String,
    },

    /// A feature is parsed but not supported by this version of the loader.
    #[error("unsupported MJCF feature `{feature}`{}", hint.as_ref().map(|h| format!(": {h}")).unwrap_or_default())]
    Unsupported {
        /// What was unsupported.
        feature: String,
        /// Optional hint.
        hint: Option<String>,
    },

    /// A body referenced a parent that wasn't found.
    #[error("invalid model: {0}")]
    InvalidModel(String),
}
