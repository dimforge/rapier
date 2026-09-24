//! Regenerate examples2d/utils/logo_mesh.h with the Rust demo's exact SVG tessellation.
#[path = "../../../../../../examples2d/utils/svg.rs"]
mod svg;

fn main() {
    println!("/* Generated from examples2d/utils/svg.rs by tools/logo-mesh. Do not edit. */");
    println!("#ifndef EXAMPLE_LOGO_MESH_H\n#define EXAMPLE_LOGO_MESH_H\n#include \"rapier.h\"");
    println!(
        "typedef struct LogoMesh {{ const R2Vector *vertices; size_t vertex_count; const uint32_t *indices; size_t triangle_count; const uint32_t *outline; size_t edge_count; }} LogoMesh;"
    );
    let meshes = svg::rapier_logo();
    for (i, (vertices, indices)) in meshes.iter().enumerate() {
        println!("static const R2Vector logo_vertices_{i}[] = {{");
        for p in vertices {
            println!("    {{{:.9e}, {:.9e}}},", p.x, p.y);
        }
        println!("}};\nstatic const uint32_t logo_indices_{i}[] = {{");
        for t in indices {
            println!("    {}, {}, {},", t[0], t[1], t[2]);
        }
        println!("}};\nstatic const uint32_t logo_outline_{i}[] = {{");
        for e in svg::outline(indices) {
            println!("    {}, {},", e[0], e[1]);
        }
        println!("}};");
    }
    println!("static const LogoMesh logo_meshes[] = {{");
    for (i, (vertices, indices)) in meshes.iter().enumerate() {
        println!(
            "    {{logo_vertices_{i}, {}, logo_indices_{i}, {}, logo_outline_{i}, {}}},",
            vertices.len(),
            indices.len(),
            svg::outline(indices).len()
        );
    }
    println!("}};\n#endif");
}
