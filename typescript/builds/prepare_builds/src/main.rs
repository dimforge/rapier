use std::{
    error::Error,
    fs::{self, File},
    io::Write,
    path::{Path, PathBuf},
};

use clap::Parser;
use clap_derive::{Parser, ValueEnum};
use tera::{Context, Tera};

/// Simple program to greet a person
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
pub struct Args {
    /// Dimension to use
    #[arg(short, long)]
    dim: Dimension,

    /// Features to enable
    #[arg(short, long)]
    feature_set: FeatureSet,
}

#[derive(ValueEnum, Debug, Clone, Copy)]
pub enum Dimension {
    Dim2,
    Dim3,
}

#[derive(ValueEnum, Default, Debug, Clone, Copy)]
pub enum FeatureSet {
    #[default]
    NonDeterministic,
    Deterministic,
    Simd,
}

/// Values to use when creating the new build folder.
pub struct BuildValues {
    /// Only the number of dimensions, as sometimes it will be prefixed by "dim" and sometimes post-fixed by "d".
    pub dim: String,
    /// real name of the additional features to enable in the project
    pub feature_set: Vec<String>,
    pub target_dir: PathBuf,
    pub template_dir: PathBuf,
    pub additional_rust_flags: String,
    pub additional_wasm_opt_flags: Vec<String>,
    pub js_package_name: String,
}

impl BuildValues {
    pub fn new(args: Args) -> Self {
        let dim = match args.dim {
            Dimension::Dim2 => "2",
            Dimension::Dim3 => "3",
        };
        let feature_set = match args.feature_set {
            FeatureSet::NonDeterministic => vec![],
            FeatureSet::Deterministic => vec!["enhanced-determinism"],
            FeatureSet::Simd => vec!["simd-stable"],
        };
        let js_package_name = match args.feature_set {
            FeatureSet::NonDeterministic => format!("rapier{dim}d"),
            FeatureSet::Deterministic => format!("rapier{dim}d-deterministic"),
            FeatureSet::Simd => format!("rapier{dim}d-simd"),
        };

        let root: PathBuf = env!("CARGO_MANIFEST_DIR").into();

        Self {
            dim: dim.to_string(),
            feature_set: feature_set.iter().map(|f| f.to_string()).collect(),
            template_dir: root.join("templates/").clone(),
            target_dir: root.parent().unwrap().join(&js_package_name).into(),
            additional_rust_flags: match args.feature_set {
                FeatureSet::Simd => "RUSTFLAGS='-C target-feature=+simd128'".to_string(),
                _ => "".to_string(),
            },
            additional_wasm_opt_flags: match args.feature_set {
                FeatureSet::Simd => vec!["--enable-simd".to_string()],
                _ => vec![],
            },
            js_package_name,
        }
    }
}

fn main() {
    let args = Args::parse();
    dbg!(&args);

    let build_values = BuildValues::new(args);
    copy_top_level_files_in_directory(&build_values.template_dir, &build_values.target_dir)
        .expect("Failed to copy directory");
    process_templates(&build_values).expect("Failed to process templates");
}

fn copy_top_level_files_in_directory(
    src: impl AsRef<Path>,
    dest: impl AsRef<std::ffi::OsStr>,
) -> std::io::Result<()> {
    let src = src.as_ref();
    let dest = Path::new(&dest);
    if dest.exists() {
        fs::remove_dir_all(&dest)?;
    }
    fs::create_dir_all(&dest)?;

    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let path = entry.path();
        let dest_path = dest.join(path.file_name().unwrap());

        if !path.is_dir() {
            fs::copy(&path, &dest_path)?;
        }
    }
    Ok(())
}

/// Process all tera templates in the target directory:
/// - Remove the extension
/// - Render the templates to
///
fn process_templates(build_values: &BuildValues) -> std::io::Result<()> {
    let target_dir = build_values.target_dir.clone();

    let mut context = Context::new();
    context.insert("dimension", &build_values.dim);
    context.insert("additional_features", &build_values.feature_set);
    context.insert("additional_rust_flags", &build_values.additional_rust_flags);
    context.insert(
        "additional_wasm_opt_flags",
        &build_values.additional_wasm_opt_flags,
    );
    context.insert("js_package_name", &build_values.js_package_name);

    let tera = match Tera::new(target_dir.join("**/*.tera").to_str().unwrap()) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("Parsing error(s): {}", e);
            ::std::process::exit(1);
        }
    };
    dbg!(tera.templates.keys(), &context);

    for entry in fs::read_dir(target_dir)? {
        let entry = entry?;
        let path = entry.path();
        // For tera templates, remove extension.
        if path.extension() == Some(std::ffi::OsStr::new("tera")) {
            let mut i = path.iter();

            // Get path from target directory
            for _ in i
                .by_ref()
                .take_while(|c| *c != build_values.target_dir.file_name().unwrap())
            {}
            let path_template = i.as_path();
            match tera.render(path_template.to_str().unwrap(), &context) {
                Ok(s) => {
                    let old_path = path.clone();
                    let new_path = path.with_extension("");
                    let mut file = File::create(path.join(new_path))?;
                    file.write_all(s.as_bytes())?;
                    std::fs::remove_file(old_path)?;
                }
                Err(e) => {
                    eprintln!("Error: {}", e);
                    let mut cause = e.source();
                    while let Some(e) = cause {
                        eprintln!("Reason: {}", e);
                        cause = e.source();
                    }
                }
            };
        }
    }

    Ok(())
}
