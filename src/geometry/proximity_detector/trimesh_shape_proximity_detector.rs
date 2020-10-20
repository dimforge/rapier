use crate::geometry::proximity_detector::{
    PrimitiveProximityDetectionContext, ProximityDetectionContext,
};
use crate::geometry::{Collider, Proximity, Shape, ShapeType, Trimesh};
use crate::ncollide::bounding_volume::{BoundingVolume, AABB};

pub struct TrimeshShapeProximityDetectorWorkspace {
    interferences: Vec<usize>,
    local_aabb2: AABB<f32>,
    old_interferences: Vec<usize>,
}

impl TrimeshShapeProximityDetectorWorkspace {
    pub fn new() -> Self {
        Self {
            interferences: Vec::new(),
            local_aabb2: AABB::new_invalid(),
            old_interferences: Vec::new(),
        }
    }
}

pub fn detect_proximity_trimesh_shape(ctxt: &mut ProximityDetectionContext) -> Proximity {
    let collider1 = &ctxt.colliders[ctxt.pair.pair.collider1];
    let collider2 = &ctxt.colliders[ctxt.pair.pair.collider2];

    if let Some(trimesh1) = collider1.shape().as_trimesh() {
        do_detect_proximity(trimesh1, collider1, collider2, ctxt)
    } else if let Some(trimesh2) = collider2.shape().as_trimesh() {
        do_detect_proximity(trimesh2, collider2, collider1, ctxt)
    } else {
        panic!("Invalid shape types provided.")
    }
}

fn do_detect_proximity(
    trimesh1: &Trimesh,
    collider1: &Collider,
    collider2: &Collider,
    ctxt: &mut ProximityDetectionContext,
) -> Proximity {
    let workspace: &mut TrimeshShapeProximityDetectorWorkspace = ctxt
        .pair
        .detector_workspace
        .as_mut()
        .expect("The TrimeshShapeProximityDetectorWorkspace is missing.")
        .downcast_mut()
        .expect("Invalid workspace type, expected a TrimeshShapeProximityDetectorWorkspace.");

    /*
     * Compute interferences.
     */
    let pos12 = collider1.position.inverse() * collider2.position;
    // TODO: somehow precompute the AABB and reuse it?
    let mut new_local_aabb2 = collider2
        .shape()
        .compute_aabb(&pos12)
        .loosened(ctxt.prediction_distance);
    let same_local_aabb2 = workspace.local_aabb2.contains(&new_local_aabb2);

    if !same_local_aabb2 {
        let extra_margin =
            (new_local_aabb2.maxs - new_local_aabb2.mins).map(|e| (e / 10.0).min(0.1));
        new_local_aabb2.mins -= extra_margin;
        new_local_aabb2.maxs += extra_margin;

        let local_aabb2 = new_local_aabb2; // .loosened(ctxt.prediction_distance * 2.0); // FIXME: what would be the best value?
        std::mem::swap(
            &mut workspace.old_interferences,
            &mut workspace.interferences,
        );

        workspace.interferences.clear();
        trimesh1
            .waabbs()
            .intersect_aabb(&local_aabb2, &mut workspace.interferences);
        workspace.local_aabb2 = local_aabb2;
    }

    /*
     * Dispatch to the specific solver by keeping the previous manifold if we already had one.
     */
    let new_interferences = &workspace.interferences;
    let mut old_inter_it = workspace.old_interferences.drain(..).peekable();
    let mut best_proximity = Proximity::Disjoint;
    let shape_type2 = collider2.shape().shape_type();

    for triangle_id in new_interferences.iter() {
        if *triangle_id >= trimesh1.num_triangles() {
            // Because of SIMD padding, the broad-phase may return tiangle indices greater
            // than the max.
            continue;
        }

        if !same_local_aabb2 {
            loop {
                match old_inter_it.peek() {
                    Some(old_triangle_id) if *old_triangle_id < *triangle_id => {
                        old_inter_it.next();
                    }
                    _ => break,
                }
            }

            if old_inter_it.peek() != Some(triangle_id) {
            } else {
                old_inter_it.next();
            };
        }

        let triangle1 = trimesh1.triangle(*triangle_id);
        let (proximity_detector, mut workspace2) = ctxt
            .dispatcher
            .dispatch_primitives(ShapeType::Triangle, shape_type2);

        let mut ctxt2 = PrimitiveProximityDetectionContext {
            prediction_distance: ctxt.prediction_distance,
            collider1,
            collider2,
            shape1: &triangle1,
            shape2: collider2.shape(),
            position1: collider1.position(),
            position2: collider2.position(),
            workspace: workspace2.as_mut().map(|w| &mut **w),
        };

        match (proximity_detector.detect_proximity)(&mut ctxt2) {
            Proximity::Intersecting => return Proximity::Intersecting,
            Proximity::WithinMargin => best_proximity = Proximity::WithinMargin,
            Proximity::Disjoint => {}
        }
    }

    best_proximity
}
