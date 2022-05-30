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
    var r = f32(vertex.color & 255u) / 255.0;
    var g = f32(vertex.color >> 8u & 255u) / 255.0;
    var b = f32(vertex.color >> 16u & 255u) / 255.0;
    var a = f32(vertex.color >> 24u & 255u) / 255.0;
    out.color = vec4<f32>(r, g, b, a);

    return out;
}

[[stage(fragment)]]
fn fragment(in: VertexOutput) -> [[location(0)]] vec4<f32> {
    return in.color;
}
