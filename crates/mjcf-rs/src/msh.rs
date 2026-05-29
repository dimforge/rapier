//! Parser for MuJoCo's custom binary `.msh` mesh format.
//!
//! Layout (little-endian):
//! ```text
//! i32 nvertex
//! i32 nnormal
//! i32 ntexcoord
//! i32 nface
//! f32[nvertex * 3]   positions (x, y, z)
//! f32[nnormal * 3]   normals  (x, y, z)
//! f32[ntexcoord * 2] texcoords (u, v)
//! i32[nface * 3]     triangle indices
//! ```
//!
//! Only the positions and faces are required for collision; normals and
//! texcoords are returned for callers that want them.

use std::io::Read;

use byteorder::{LittleEndian, ReadBytesExt};

/// A parsed `.msh` file.
#[derive(Clone, Debug)]
pub struct MshMesh {
    /// Vertex positions, flat `[x0, y0, z0, x1, y1, z1, …]`.
    pub vertices: Vec<f32>,
    /// Vertex normals (may be empty).
    pub normals: Vec<f32>,
    /// Texture coordinates (may be empty).
    pub texcoords: Vec<f32>,
    /// Triangle indices.
    pub faces: Vec<u32>,
}

/// Parse a `.msh` file from raw bytes.
pub fn parse(bytes: &[u8]) -> std::io::Result<MshMesh> {
    let mut cur = bytes;
    let nvertex = cur.read_i32::<LittleEndian>()? as usize;
    let nnormal = cur.read_i32::<LittleEndian>()? as usize;
    let ntexcoord = cur.read_i32::<LittleEndian>()? as usize;
    let nface = cur.read_i32::<LittleEndian>()? as usize;

    let mut vertices = vec![0.0f32; nvertex * 3];
    cur.read_exact(bytemuck_safe_cast(&mut vertices))?;
    let mut normals = vec![0.0f32; nnormal * 3];
    cur.read_exact(bytemuck_safe_cast(&mut normals))?;
    let mut texcoords = vec![0.0f32; ntexcoord * 2];
    cur.read_exact(bytemuck_safe_cast(&mut texcoords))?;
    let mut faces = vec![0i32; nface * 3];
    {
        // i32 chunks; LE.
        for f in faces.iter_mut() {
            *f = cur.read_i32::<LittleEndian>()?;
        }
    }
    Ok(MshMesh {
        vertices,
        normals,
        texcoords,
        faces: faces.into_iter().map(|i| i as u32).collect(),
    })
}

fn bytemuck_safe_cast(slice: &mut [f32]) -> &mut [u8] {
    let len = std::mem::size_of_val(slice);
    let ptr = slice.as_mut_ptr() as *mut u8;
    // Safety: f32 has no invalid bit patterns, and the slice borrows for
    // exactly its lifetime.
    unsafe { std::slice::from_raw_parts_mut(ptr, len) }
}

/// Read a `.msh` file from disk.
pub fn parse_file(path: impl AsRef<std::path::Path>) -> std::io::Result<MshMesh> {
    let bytes = std::fs::read(path)?;
    parse(&bytes)
}
