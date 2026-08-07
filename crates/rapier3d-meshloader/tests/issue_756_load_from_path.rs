//! Regression test for #756: `load_from_path` auto-detects the file format
//! (STL, OBJ, …) instead of supporting STL only.

use rapier3d::math::Vector;
use rapier3d::prelude::MeshConverter;
use rapier3d_meshloader::load_from_path;
use std::io::Write;
use std::path::PathBuf;

fn write_temp(name: &str, contents: &str) -> PathBuf {
    let path = std::env::temp_dir().join(name);
    let mut file = std::fs::File::create(&path).unwrap();
    file.write_all(contents.as_bytes()).unwrap();
    path
}

const ASCII_STL: &str = "solid tri
facet normal 0 0 1
  outer loop
    vertex 0 0 0
    vertex 1 0 0
    vertex 0 1 0
  endloop
endfacet
endsolid tri
";

const OBJ: &str = "v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
";

#[test]
#[cfg(feature = "stl")]
fn load_from_path_detects_stl() {
    let path = write_temp("issue_756_test.stl", ASCII_STL);
    let shapes = load_from_path(&path, &MeshConverter::TriMesh, Vector::splat(1.0)).unwrap();
    assert_eq!(shapes.len(), 1);
    assert!(shapes[0].is_ok());
}

#[test]
#[cfg(feature = "wavefront")]
fn load_from_path_detects_obj() {
    let path = write_temp("issue_756_test.obj", OBJ);
    let shapes = load_from_path(&path, &MeshConverter::TriMesh, Vector::splat(1.0)).unwrap();
    assert_eq!(shapes.len(), 1);
    assert!(shapes[0].is_ok());
}
