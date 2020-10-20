use crate::geometry::contact_generator::{
    ContactGenerationContext, PrimitiveContactGenerationContext,
};
use crate::geometry::{Collider, ContactManifold, Shape, ShapeType, Trimesh};
use crate::ncollide::bounding_volume::{BoundingVolume, AABB};

pub struct TrimeshShapeContactGeneratorWorkspace {
    interferences: Vec<usize>,
    local_aabb2: AABB<f32>,
    old_interferences: Vec<usize>,
    old_manifolds: Vec<ContactManifold>,
}

impl TrimeshShapeContactGeneratorWorkspace {
    pub fn new() -> Self {
        Self {
            interferences: Vec::new(),
            local_aabb2: AABB::new_invalid(),
            old_interferences: Vec::new(),
            old_manifolds: Vec::new(),
        }
    }
}

pub fn generate_contacts_trimesh_shape(ctxt: &mut ContactGenerationContext) {
    let collider1 = &ctxt.colliders[ctxt.pair.pair.collider1];
    let collider2 = &ctxt.colliders[ctxt.pair.pair.collider2];

    if let Some(trimesh1) = collider1.shape().as_trimesh() {
        do_generate_contacts(trimesh1, collider1, collider2, ctxt, false)
    } else if let Some(trimesh2) = collider2.shape().as_trimesh() {
        do_generate_contacts(trimesh2, collider2, collider1, ctxt, true)
    }
}

fn do_generate_contacts(
    trimesh1: &Trimesh,
    collider1: &Collider,
    collider2: &Collider,
    ctxt: &mut ContactGenerationContext,
    flipped: bool,
) {
    let ctxt_pair_pair = if flipped {
        ctxt.pair.pair.swap()
    } else {
        ctxt.pair.pair
    };

    let workspace: &mut TrimeshShapeContactGeneratorWorkspace = ctxt
        .pair
        .generator_workspace
        .as_mut()
        .expect("The TrimeshShapeContactGeneratorWorkspace is missing.")
        .downcast_mut()
        .expect("Invalid workspace type, expected a TrimeshShapeContactGeneratorWorkspace.");

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
        std::mem::swap(&mut workspace.old_manifolds, &mut ctxt.pair.manifolds);
        ctxt.pair.manifolds.clear();

        if workspace.old_interferences.is_empty() && !workspace.old_manifolds.is_empty() {
            // This happens if for some reasons the contact generator context was lost
            // and rebuilt. In this case, we hate to reconstruct the `old_interferences`
            // array using the subshape ids from the contact manifolds.
            // TODO: always rely on the subshape ids instead of maintaining `.ord_interferences` ?
            let ctxt_collider1 = ctxt_pair_pair.collider1;
            workspace.old_interferences = workspace
                .old_manifolds
                .iter()
                .map(|manifold| {
                    if manifold.pair.collider1 == ctxt_collider1 {
                        manifold.subshape_index_pair.0
                    } else {
                        manifold.subshape_index_pair.1
                    }
                })
                .collect();
        }
        // This assertion may fire due to the invalid triangle_ids that the
        // near-phase may return (due to SIMD sentinels).
        //
        // assert_eq!(
        //     workspace
        //         .old_interferences
        //         .len()
        //         .min(trimesh1.num_triangles()),
        //     workspace.old_manifolds.len()
        // );

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
    let mut old_manifolds_it = workspace.old_manifolds.drain(..);
    let shape_type2 = collider2.shape().shape_type();

    for (i, triangle_id) in new_interferences.iter().enumerate() {
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
                        old_manifolds_it.next();
                    }
                    _ => break,
                }
            }

            let manifold = if old_inter_it.peek() != Some(triangle_id) {
                // We don't have a manifold for this triangle yet.
                ContactManifold::with_subshape_indices(
                    ctxt_pair_pair,
                    collider1,
                    collider2,
                    *triangle_id,
                    0,
                )
            } else {
                // We already have a manifold for this triangle.
                old_inter_it.next();
                old_manifolds_it.next().unwrap()
            };

            ctxt.pair.manifolds.push(manifold);
        }

        let manifold = &mut ctxt.pair.manifolds[i];
        let triangle1 = trimesh1.triangle(*triangle_id);
        let (generator, mut workspace2) = ctxt
            .dispatcher
            .dispatch_primitives(ShapeType::Triangle, shape_type2);

        let mut ctxt2 = if ctxt_pair_pair.collider1 != manifold.pair.collider1 {
            PrimitiveContactGenerationContext {
                prediction_distance: ctxt.prediction_distance,
                collider1: collider2,
                collider2: collider1,
                shape1: collider2.shape(),
                shape2: &triangle1,
                position1: collider2.position(),
                position2: collider1.position(),
                manifold,
                workspace: workspace2.as_deref_mut(),
            }
        } else {
            PrimitiveContactGenerationContext {
                prediction_distance: ctxt.prediction_distance,
                collider1,
                collider2,
                shape1: &triangle1,
                shape2: collider2.shape(),
                position1: collider1.position(),
                position2: collider2.position(),
                manifold,
                workspace: workspace2.as_deref_mut(),
            }
        };

        (generator.generate_contacts)(&mut ctxt2);
    }
}
