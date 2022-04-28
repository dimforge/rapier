#import bevy_sprite::mesh2d_view_bind_group
[[group(0), binding(0)]]
var<uniform> view: View;

struct Vertex {
    //[[location(0)]] color: vec4<f32>;
    [[location(0)]] place: vec3<f32>;
    [[location(1)]] color: u32;
};

struct VertexOutput {
    [[builtin(position)]] clip_position: vec4<f32>;
    [[location(0)]] color: vec4<f32>;
};

[[stage(vertex)]]
fn vertex(vertex: Vertex) -> VertexOutput {
    var out: VertexOutput;
    out.clip_position = view.view_proj * vec4<f32>(vertex.place, 1.0);
    // What is this craziness?
    out.color = vec4<f32>((vec4<u32>(vertex.color) >> vec4<u32>(0u, 8u, 16u, 24u)) & vec4<u32>(255u)) / 255.0;
    //out.color = vertex.color;
    //out.color = vec4<f32>(1.0, 0.0, 0.0, 1.0);

    return out;
}

[[stage(fragment)]]
fn fragment(in: VertexOutput) -> [[location(0)]] vec4<f32> {
    return in.color;
}
