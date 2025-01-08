use rapier2d::prelude::*;
use rapier_testbed2d::Testbed;

use lyon::math::Point;
use lyon::path::PathEvent;
use lyon::tessellation::geometry_builder::*;
use lyon::tessellation::{self, FillOptions, FillTessellator};
use usvg::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let ground_size = 25.0;

    let rigid_body = RigidBodyBuilder::fixed();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::fixed()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(vector![ground_size, ground_size]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::fixed()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(vector![-ground_size, ground_size]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, 1.2);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Create the trimeshes from a tessellated SVG.
     */
    let mut fill_tess = FillTessellator::new();
    let opt = usvg::Options::default();
    let rtree = usvg::Tree::from_str(RAPIER_SVG_STR, &opt).unwrap();
    let mut ith = 0;

    for node in rtree.root().descendants() {
        if let usvg::NodeKind::Path(ref p) = *node.borrow() {
            let transform = node.transform();
            if p.fill.is_some() {
                let path = PathConvIter {
                    iter: p.data.iter(),
                    first: Point::new(0.0, 0.0),
                    prev: Point::new(0.0, 0.0),
                    deferred: None,
                    needs_end: false,
                };

                let mut mesh: VertexBuffers<_, u32> = VertexBuffers::new();
                fill_tess
                    .tessellate(
                        path,
                        &FillOptions::tolerance(0.01),
                        &mut BuffersBuilder::new(&mut mesh, VertexCtor { prim_id: 0 }),
                    )
                    .expect("Tessellation failed.");

                let angle = transform.get_rotate() as f32;

                let (sx, sy) = (
                    transform.get_scale().0 as f32 * 0.2,
                    transform.get_scale().1 as f32 * 0.2,
                );

                let indices: Vec<_> = mesh.indices.chunks(3).map(|v| [v[0], v[1], v[2]]).collect();
                let vertices: Vec<_> = mesh
                    .vertices
                    .iter()
                    .map(|v| point![v.position[0] * sx, v.position[1] * -sy])
                    .collect();

                for k in 0..5 {
                    let collider = ColliderBuilder::trimesh(vertices.clone(), indices.clone())
                        .unwrap()
                        .contact_skin(0.2);
                    let rigid_body = RigidBodyBuilder::dynamic()
                        .translation(vector![ith as f32 * 8.0 - 20.0, 20.0 + k as f32 * 11.0])
                        .rotation(angle);
                    let handle = bodies.insert(rigid_body);
                    colliders.insert_with_parent(collider, handle, &mut bodies);
                }

                ith += 1;
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 20.0], 17.0);
}

const RAPIER_SVG_STR: &str = r#"
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg width="100%" height="100%" viewBox="0 0 527 131" version="1.1" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" xml:space="preserve" xmlns:serif="http://www.serif.com/" style="fill-rule:evenodd;clip-rule:evenodd;stroke-linecap:round;stroke-linejoin:round;stroke-miterlimit:1.5;">
    <g transform="matrix(1,0,0,1,1,-673)">
        <g transform="matrix(1,0,0,1,-5.31448,644.547)">
            <g transform="matrix(1.34947,0,0,1.34947,-1559.19,-910.299)">
                <g transform="matrix(50,0,0,50,1277.28,785.746)">
                    <path d="M0.021,-0.074L0.074,-0.08C0.093,-0.082 0.106,-0.086 0.113,-0.092C0.119,-0.097 0.122,-0.111 0.122,-0.132L0.122,-0.597C0.122,-0.607 0.122,-0.616 0.121,-0.623C0.12,-0.63 0.117,-0.636 0.114,-0.641C0.111,-0.646 0.105,-0.65 0.098,-0.653C0.09,-0.656 0.08,-0.659 0.067,-0.661L0.025,-0.668L0.024,-0.685C0.04,-0.685 0.053,-0.685 0.063,-0.685C0.072,-0.684 0.081,-0.685 0.088,-0.686L0.222,-0.693C0.238,-0.695 0.255,-0.696 0.272,-0.696C0.349,-0.696 0.407,-0.685 0.446,-0.662C0.485,-0.639 0.504,-0.6 0.504,-0.543C0.504,-0.504 0.494,-0.473 0.474,-0.45C0.454,-0.427 0.424,-0.406 0.383,-0.387L0.493,-0.194C0.508,-0.167 0.522,-0.147 0.533,-0.132C0.544,-0.117 0.556,-0.106 0.567,-0.099C0.578,-0.092 0.589,-0.087 0.602,-0.085C0.614,-0.082 0.629,-0.08 0.647,-0.077L0.647,-0.059C0.633,-0.06 0.618,-0.06 0.601,-0.06L0.498,-0.06C0.483,-0.06 0.472,-0.06 0.465,-0.059L0.313,-0.341C0.302,-0.362 0.286,-0.372 0.263,-0.372L0.21,-0.372L0.21,-0.138C0.21,-0.122 0.213,-0.11 0.22,-0.103C0.226,-0.096 0.237,-0.09 0.253,-0.086L0.302,-0.074L0.302,-0.056C0.254,-0.059 0.205,-0.06 0.156,-0.06C0.106,-0.06 0.061,-0.059 0.021,-0.056L0.021,-0.074ZM0.21,-0.4C0.343,-0.395 0.41,-0.438 0.41,-0.527C0.41,-0.569 0.396,-0.603 0.367,-0.628C0.337,-0.653 0.297,-0.665 0.245,-0.665C0.24,-0.665 0.234,-0.665 0.228,-0.665C0.222,-0.664 0.216,-0.664 0.21,-0.663L0.21,-0.4Z" style="fill:rgb(235,237,240);fill-rule:nonzero;"/>
                </g>
                <g transform="matrix(50,0,0,50,1309.08,785.746)">
                    <path d="M0.201,-0.049C0.174,-0.049 0.15,-0.054 0.129,-0.064C0.108,-0.073 0.091,-0.086 0.078,-0.103C0.065,-0.12 0.055,-0.139 0.048,-0.162C0.041,-0.185 0.038,-0.209 0.038,-0.235C0.038,-0.27 0.044,-0.302 0.056,-0.329C0.068,-0.356 0.084,-0.378 0.103,-0.396C0.122,-0.414 0.144,-0.428 0.169,-0.437C0.193,-0.446 0.218,-0.451 0.243,-0.451C0.256,-0.451 0.269,-0.45 0.28,-0.447C0.291,-0.444 0.301,-0.441 0.31,-0.438C0.32,-0.435 0.33,-0.431 0.339,-0.426L0.395,-0.455L0.404,-0.452C0.404,-0.4 0.404,-0.348 0.405,-0.297C0.405,-0.245 0.405,-0.193 0.405,-0.141C0.405,-0.114 0.416,-0.102 0.439,-0.103C0.444,-0.103 0.449,-0.104 0.452,-0.106C0.455,-0.107 0.459,-0.109 0.462,-0.111C0.465,-0.114 0.468,-0.116 0.471,-0.118L0.481,-0.112C0.476,-0.1 0.47,-0.089 0.463,-0.08C0.456,-0.072 0.448,-0.065 0.438,-0.059C0.428,-0.052 0.416,-0.049 0.403,-0.049C0.38,-0.049 0.364,-0.056 0.355,-0.071C0.346,-0.085 0.341,-0.105 0.341,-0.13L0.341,-0.16C0.337,-0.146 0.331,-0.132 0.324,-0.119C0.316,-0.106 0.306,-0.094 0.295,-0.084C0.284,-0.073 0.27,-0.065 0.255,-0.059C0.24,-0.052 0.222,-0.049 0.201,-0.049ZM0.341,-0.27C0.341,-0.293 0.339,-0.313 0.336,-0.332C0.332,-0.351 0.326,-0.367 0.317,-0.38C0.308,-0.393 0.297,-0.403 0.284,-0.41C0.27,-0.417 0.253,-0.42 0.232,-0.42C0.218,-0.42 0.203,-0.417 0.188,-0.412C0.172,-0.406 0.158,-0.397 0.145,-0.384C0.132,-0.371 0.122,-0.354 0.114,-0.334C0.105,-0.313 0.101,-0.289 0.101,-0.26C0.101,-0.238 0.104,-0.217 0.11,-0.197C0.115,-0.176 0.123,-0.158 0.134,-0.143C0.145,-0.128 0.158,-0.115 0.173,-0.107C0.188,-0.097 0.206,-0.093 0.226,-0.093C0.242,-0.093 0.257,-0.097 0.271,-0.105C0.284,-0.113 0.296,-0.124 0.306,-0.139C0.316,-0.153 0.324,-0.17 0.331,-0.189C0.337,-0.208 0.34,-0.229 0.341,-0.252L0.341,-0.27Z" style="fill:rgb(235,237,240);fill-rule:nonzero;"/>
                </g>
                <g transform="matrix(50,0,0,50,1334.28,785.746)">
                    <path d="M0.156,-0.398C0.217,-0.443 0.276,-0.463 0.332,-0.459C0.379,-0.456 0.416,-0.437 0.445,-0.402C0.473,-0.367 0.487,-0.32 0.487,-0.262C0.487,-0.232 0.482,-0.204 0.471,-0.177C0.46,-0.15 0.445,-0.127 0.426,-0.108C0.407,-0.089 0.384,-0.073 0.358,-0.062C0.331,-0.051 0.303,-0.045 0.272,-0.045C0.235,-0.045 0.197,-0.055 0.156,-0.074L0.156,0.078C0.156,0.087 0.159,0.093 0.165,0.099C0.17,0.103 0.181,0.107 0.198,0.11L0.258,0.12L0.258,0.137C0.249,0.136 0.238,0.135 0.227,0.135C0.215,0.134 0.201,0.134 0.186,0.133L0.145,0.133C0.122,0.133 0.1,0.133 0.079,0.134C0.058,0.135 0.04,0.136 0.023,0.137L0.023,0.12L0.065,0.112C0.075,0.11 0.082,0.107 0.085,0.103C0.088,0.098 0.089,0.09 0.089,0.077L0.089,-0.364C0.089,-0.389 0.085,-0.401 0.076,-0.402L0.027,-0.41L0.03,-0.424C0.047,-0.431 0.061,-0.437 0.074,-0.443C0.086,-0.449 0.095,-0.454 0.102,-0.458C0.109,-0.463 0.117,-0.468 0.124,-0.475C0.131,-0.482 0.136,-0.489 0.141,-0.496L0.156,-0.496L0.156,-0.398ZM0.156,-0.367L0.156,-0.152C0.156,-0.128 0.167,-0.108 0.188,-0.093C0.209,-0.077 0.236,-0.069 0.27,-0.069C0.291,-0.069 0.311,-0.073 0.328,-0.082C0.345,-0.09 0.36,-0.101 0.373,-0.117C0.386,-0.132 0.395,-0.149 0.402,-0.17C0.409,-0.191 0.412,-0.213 0.412,-0.238C0.412,-0.266 0.408,-0.291 0.401,-0.314C0.394,-0.336 0.383,-0.355 0.37,-0.37C0.356,-0.385 0.34,-0.397 0.321,-0.404C0.302,-0.411 0.282,-0.413 0.259,-0.41C0.226,-0.406 0.192,-0.392 0.156,-0.367Z" style="fill:rgb(235,237,240);fill-rule:nonzero;"/>
                </g>
                <g transform="matrix(50,0,0,50,1360.53,785.746)">
                    <path d="M0.246,-0.056C0.235,-0.057 0.221,-0.058 0.203,-0.059C0.184,-0.06 0.162,-0.06 0.135,-0.06C0.108,-0.06 0.086,-0.06 0.068,-0.059C0.049,-0.058 0.035,-0.057 0.024,-0.056L0.024,-0.073L0.078,-0.084C0.099,-0.088 0.109,-0.1 0.109,-0.12L0.109,-0.319C0.109,-0.335 0.105,-0.347 0.097,-0.354C0.089,-0.361 0.073,-0.368 0.049,-0.373L0.049,-0.388C0.072,-0.394 0.094,-0.403 0.113,-0.415C0.132,-0.427 0.147,-0.442 0.158,-0.46L0.175,-0.46L0.175,-0.117C0.175,-0.107 0.177,-0.1 0.18,-0.095C0.183,-0.09 0.188,-0.087 0.197,-0.085L0.246,-0.073L0.246,-0.056ZM0.137,-0.667C0.15,-0.667 0.161,-0.663 0.17,-0.654C0.179,-0.645 0.184,-0.634 0.184,-0.621C0.184,-0.61 0.179,-0.6 0.17,-0.591C0.16,-0.582 0.148,-0.578 0.135,-0.578C0.123,-0.578 0.113,-0.582 0.105,-0.591C0.096,-0.599 0.092,-0.609 0.092,-0.621C0.092,-0.634 0.096,-0.645 0.106,-0.654C0.115,-0.663 0.125,-0.667 0.137,-0.667Z" style="fill:rgb(235,237,240);fill-rule:nonzero;"/>
                </g>
                <g transform="matrix(50,0,0,50,1374.03,785.746)">
                    <path d="M0.386,-0.309L0.111,-0.309C0.109,-0.296 0.108,-0.283 0.108,-0.271C0.108,-0.22 0.122,-0.178 0.149,-0.147C0.176,-0.115 0.214,-0.099 0.262,-0.099C0.285,-0.099 0.307,-0.103 0.326,-0.113C0.345,-0.122 0.366,-0.137 0.389,-0.158L0.393,-0.135C0.37,-0.105 0.344,-0.082 0.315,-0.067C0.285,-0.052 0.25,-0.044 0.211,-0.044C0.186,-0.044 0.164,-0.049 0.143,-0.06C0.122,-0.07 0.103,-0.084 0.088,-0.103C0.072,-0.122 0.06,-0.143 0.051,-0.169C0.042,-0.194 0.038,-0.221 0.038,-0.25C0.038,-0.279 0.043,-0.307 0.053,-0.333C0.062,-0.358 0.076,-0.38 0.093,-0.399C0.11,-0.418 0.131,-0.432 0.155,-0.443C0.179,-0.454 0.205,-0.459 0.233,-0.459C0.255,-0.459 0.275,-0.455 0.294,-0.448C0.312,-0.441 0.328,-0.431 0.341,-0.418C0.354,-0.405 0.365,-0.389 0.373,-0.37C0.38,-0.351 0.385,-0.331 0.386,-0.309ZM0.116,-0.332L0.261,-0.332C0.271,-0.332 0.28,-0.332 0.287,-0.333C0.294,-0.333 0.3,-0.334 0.305,-0.337C0.31,-0.339 0.313,-0.342 0.314,-0.347C0.315,-0.352 0.314,-0.358 0.312,-0.367C0.308,-0.384 0.3,-0.4 0.288,-0.414C0.275,-0.428 0.256,-0.435 0.231,-0.435C0.201,-0.435 0.176,-0.426 0.156,-0.408C0.136,-0.389 0.123,-0.364 0.116,-0.332Z" style="fill:rgb(235,237,240);fill-rule:nonzero;"/>
                </g>
                <g transform="matrix(50,0,0,50,1395.58,785.746)">
                    <path d="M0.024,-0.056L0.024,-0.072L0.072,-0.081C0.085,-0.084 0.094,-0.089 0.098,-0.096C0.101,-0.103 0.103,-0.115 0.103,-0.132L0.103,-0.324C0.103,-0.337 0.1,-0.347 0.095,-0.353C0.089,-0.359 0.076,-0.364 0.057,-0.368L0.031,-0.374L0.033,-0.389C0.059,-0.398 0.081,-0.409 0.1,-0.422C0.119,-0.435 0.135,-0.455 0.149,-0.482L0.167,-0.482C0.168,-0.464 0.169,-0.445 0.17,-0.424C0.171,-0.403 0.171,-0.381 0.171,-0.358C0.178,-0.37 0.185,-0.382 0.194,-0.393C0.202,-0.404 0.212,-0.415 0.224,-0.426C0.247,-0.448 0.267,-0.459 0.285,-0.459C0.302,-0.459 0.315,-0.454 0.324,-0.445C0.333,-0.436 0.338,-0.424 0.338,-0.409C0.338,-0.396 0.334,-0.385 0.327,-0.378C0.319,-0.37 0.309,-0.366 0.298,-0.366C0.287,-0.366 0.275,-0.369 0.263,-0.376C0.251,-0.383 0.241,-0.386 0.232,-0.386C0.225,-0.386 0.219,-0.383 0.212,-0.376C0.205,-0.369 0.198,-0.361 0.192,-0.352C0.186,-0.343 0.181,-0.333 0.177,-0.323C0.173,-0.312 0.171,-0.304 0.171,-0.297L0.171,-0.13C0.171,-0.113 0.174,-0.101 0.18,-0.096C0.185,-0.09 0.197,-0.086 0.216,-0.083L0.288,-0.072L0.288,-0.056C0.271,-0.057 0.251,-0.058 0.23,-0.059C0.208,-0.06 0.185,-0.06 0.16,-0.06L0.146,-0.06C0.123,-0.06 0.101,-0.06 0.081,-0.059C0.06,-0.058 0.041,-0.057 0.024,-0.056Z" style="fill:rgb(235,237,240);fill-rule:nonzero;"/>
                </g>
            </g>
        </g>
    </g>
</svg>
"#;

pub struct PathConvIter<'a> {
    iter: std::slice::Iter<'a, usvg::PathSegment>,
    prev: Point,
    first: Point,
    needs_end: bool,
    deferred: Option<PathEvent>,
}

impl Iterator for PathConvIter<'_> {
    type Item = PathEvent;
    fn next(&mut self) -> Option<PathEvent> {
        if self.deferred.is_some() {
            return self.deferred.take();
        }

        let next = self.iter.next();
        match next {
            Some(usvg::PathSegment::MoveTo { x, y }) => {
                if self.needs_end {
                    let last = self.prev;
                    let first = self.first;
                    self.needs_end = false;
                    self.prev = Point::new(*x as f32, *y as f32);
                    self.deferred = Some(PathEvent::Begin { at: self.prev });
                    self.first = self.prev;
                    Some(PathEvent::End {
                        last,
                        first,
                        close: false,
                    })
                } else {
                    self.first = Point::new(*x as f32, *y as f32);
                    Some(PathEvent::Begin { at: self.first })
                }
            }
            Some(usvg::PathSegment::LineTo { x, y }) => {
                self.needs_end = true;
                let from = self.prev;
                self.prev = Point::new(*x as f32, *y as f32);
                Some(PathEvent::Line {
                    from,
                    to: self.prev,
                })
            }
            Some(usvg::PathSegment::CurveTo {
                x1,
                y1,
                x2,
                y2,
                x,
                y,
            }) => {
                self.needs_end = true;
                let from = self.prev;
                self.prev = Point::new(*x as f32, *y as f32);
                Some(PathEvent::Cubic {
                    from,
                    ctrl1: Point::new(*x1 as f32, *y1 as f32),
                    ctrl2: Point::new(*x2 as f32, *y2 as f32),
                    to: self.prev,
                })
            }
            Some(usvg::PathSegment::ClosePath) => {
                self.needs_end = false;
                self.prev = self.first;
                Some(PathEvent::End {
                    last: self.prev,
                    first: self.first,
                    close: true,
                })
            }
            None => {
                if self.needs_end {
                    self.needs_end = false;
                    let last = self.prev;
                    let first = self.first;
                    Some(PathEvent::End {
                        last,
                        first,
                        close: false,
                    })
                } else {
                    None
                }
            }
        }
    }
}

pub struct VertexCtor {
    pub prim_id: u32,
}

impl FillVertexConstructor<GpuVertex> for VertexCtor {
    fn new_vertex(&mut self, vertex: tessellation::FillVertex) -> GpuVertex {
        GpuVertex {
            position: vertex.position().to_array(),
            prim_id: self.prim_id,
        }
    }
}

impl StrokeVertexConstructor<GpuVertex> for VertexCtor {
    fn new_vertex(&mut self, vertex: tessellation::StrokeVertex) -> GpuVertex {
        GpuVertex {
            position: vertex.position().to_array(),
            prim_id: self.prim_id,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct GpuVertex {
    pub position: [f32; 2],
    pub prim_id: u32,
}
