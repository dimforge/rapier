// This should work, but it's bugged right now so we have to use 2 shaders: https://github.com/bevyengine/bevy/issues/4011
#ifdef LINES_3D
    #import bevy_pbr::mesh_view_bind_group
    //#import bevy_pbr::mesh_struct
#else
    //#import bevy_sprite::mesh2d_view_bind_group
#endif

struct Vertex {
    [[location(0)]] pos: vec3<f32>;
    [[location(1)]] color: u32;
};

struct VertexOutput {
    [[builtin(position)]] clip_position: vec4<f32>;
    [[location(0)]] color: vec4<f32>;
};

struct FragmentOutput {
    [[builtin(frag_depth)]] depth: f32;
    [[location(0)]] color: vec4<f32>;
};

[[stage(vertex)]]
fn vertex(vertex: Vertex) -> VertexOutput {
    var out: VertexOutput;
    out.clip_position = view.view_proj * vec4<f32>(vertex.pos, 1.0);
    // https://github.com/bevyengine/bevy/blob/328c26d02c50de0bc77f0d24a376f43ba89517b1/examples/2d/mesh2d_manual.rs#L234
    // ... except the above doesn't seem to work in 3d.  Not sure what's going on there.
    var r = f32(vertex.color & 255u) / 255.0;
    var g = f32(vertex.color >> 8u & 255u) / 255.0;
    var b = f32(vertex.color >> 16u & 255u) / 255.0;
    var a = f32(vertex.color >> 24u & 255u) / 255.0;
    out.color = vec4<f32>(r, g, b, a);

    return out;
}

[[stage(fragment)]]
fn fragment(in: VertexOutput) -> FragmentOutput {
    var out: FragmentOutput;

// This should be #ifdef DEPTH_TEST_ENABLED && LINES_3D, but the
// preprocessor doesn't support that yet.
// Luckily, DEPTH_TEST_ENABLED isn't set in 2d anyway.
#ifdef DEPTH_TEST_ENABLED
    out.depth = in.clip_position.z;
#else
    out.depth = 1.0;
#endif
    out.color = in.color;
    return out;
}
