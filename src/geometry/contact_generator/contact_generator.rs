use crate::data::MaybeSerializableData;
use crate::geometry::{
    Collider, ColliderSet, ContactDispatcher, ContactEvent, ContactManifold, ContactPair, Shape,
    SolverFlags,
};
use crate::math::Isometry;
#[cfg(feature = "simd-is-enabled")]
use crate::math::{SimdFloat, SIMD_WIDTH};
use crate::pipeline::EventHandler;

#[derive(Copy, Clone)]
pub enum ContactPhase {
    NearPhase(ContactGenerator),
    ExactPhase(PrimitiveContactGenerator),
}

impl ContactPhase {
    #[inline]
    pub fn generate_contacts(
        self,
        mut context: ContactGenerationContext,
        events: &dyn EventHandler,
    ) {
        let had_contacts_before = context.pair.has_any_active_contact();

        match self {
            Self::NearPhase(gen) => (gen.generate_contacts)(&mut context),
            Self::ExactPhase(gen) => {
                // Build the primitive context from the non-primitive context and dispatch.
                let (collider1, collider2, manifold, workspace) = context
                    .pair
                    .single_manifold(context.colliders, context.solver_flags);
                let mut context2 = PrimitiveContactGenerationContext {
                    prediction_distance: context.prediction_distance,
                    collider1,
                    collider2,
                    shape1: collider1.shape(),
                    shape2: collider2.shape(),
                    position1: collider1.position(),
                    position2: collider2.position(),
                    manifold,
                    workspace,
                };

                (gen.generate_contacts)(&mut context2)
            }
        }

        if had_contacts_before != context.pair.has_any_active_contact() {
            if had_contacts_before {
                events.handle_contact_event(ContactEvent::Stopped(
                    context.pair.pair.collider1,
                    context.pair.pair.collider2,
                ));
            } else {
                events.handle_contact_event(ContactEvent::Started(
                    context.pair.pair.collider1,
                    context.pair.pair.collider2,
                ))
            }
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    #[inline]
    pub fn generate_contacts_simd(
        self,
        mut context: ContactGenerationContextSimd,
        events: &dyn EventHandler,
    ) {
        let mut had_contacts_before = [false; SIMD_WIDTH];

        for (i, pair) in context.pairs.iter().enumerate() {
            had_contacts_before[i] = pair.has_any_active_contact()
        }

        match self {
            Self::NearPhase(gen) => (gen.generate_contacts_simd)(&mut context),
            Self::ExactPhase(gen) => {
                // Build the primitive context from the non-primitive context and dispatch.
                use arrayvec::ArrayVec;
                let mut colliders_arr: ArrayVec<[(&Collider, &Collider); SIMD_WIDTH]> =
                    ArrayVec::new();
                let mut manifold_arr: ArrayVec<[&mut ContactManifold; SIMD_WIDTH]> =
                    ArrayVec::new();
                let mut workspace_arr: ArrayVec<
                    [Option<&mut (dyn Any + Send + Sync)>; SIMD_WIDTH],
                > = ArrayVec::new();

                for (pair, solver_flags) in
                    context.pairs.iter_mut().zip(context.solver_flags.iter())
                {
                    let (collider1, collider2, manifold, workspace) =
                        pair.single_manifold(context.colliders, *solver_flags);
                    colliders_arr.push((collider1, collider2));
                    manifold_arr.push(manifold);
                    workspace_arr.push(workspace);
                }

                let max_index = colliders_arr.len() - 1;
                let colliders1 = array![|ii| colliders_arr[ii.min(max_index)].0; SIMD_WIDTH];
                let colliders2 = array![|ii| colliders_arr[ii.min(max_index)].1; SIMD_WIDTH];

                let mut context2 = PrimitiveContactGenerationContextSimd {
                    prediction_distance: context.prediction_distance,
                    colliders1,
                    colliders2,
                    shapes1: array![|ii| colliders1[ii].shape(); SIMD_WIDTH],
                    shapes2: array![|ii| colliders2[ii].shape(); SIMD_WIDTH],
                    positions1: &Isometry::from(
                        array![|ii| *colliders1[ii].position(); SIMD_WIDTH],
                    ),
                    positions2: &Isometry::from(
                        array![|ii| *colliders2[ii].position(); SIMD_WIDTH],
                    ),
                    manifolds: manifold_arr.as_mut_slice(),
                    workspaces: workspace_arr.as_mut_slice(),
                };

                (gen.generate_contacts_simd)(&mut context2)
            }
        }

        for (i, pair) in context.pairs.iter().enumerate() {
            if had_contacts_before[i] != pair.has_any_active_contact() {
                if had_contacts_before[i] {
                    events.handle_contact_event(ContactEvent::Stopped(
                        pair.pair.collider1,
                        pair.pair.collider2,
                    ))
                } else {
                    events.handle_contact_event(ContactEvent::Started(
                        pair.pair.collider1,
                        pair.pair.collider2,
                    ))
                }
            }
        }
    }
}

pub struct PrimitiveContactGenerationContext<'a> {
    pub prediction_distance: f32,
    pub collider1: &'a Collider,
    pub collider2: &'a Collider,
    pub shape1: &'a dyn Shape,
    pub shape2: &'a dyn Shape,
    pub position1: &'a Isometry<f32>,
    pub position2: &'a Isometry<f32>,
    pub manifold: &'a mut ContactManifold,
    pub workspace: Option<&'a mut (dyn MaybeSerializableData)>,
}

#[cfg(feature = "simd-is-enabled")]
pub struct PrimitiveContactGenerationContextSimd<'a, 'b> {
    pub prediction_distance: f32,
    pub colliders1: [&'a Collider; SIMD_WIDTH],
    pub colliders2: [&'a Collider; SIMD_WIDTH],
    pub shapes1: [&'a dyn Shape; SIMD_WIDTH],
    pub shapes2: [&'a dyn Shape; SIMD_WIDTH],
    pub positions1: &'a Isometry<SimdFloat>,
    pub positions2: &'a Isometry<SimdFloat>,
    pub manifolds: &'a mut [&'b mut ContactManifold],
    pub workspaces: &'a mut [Option<&'b mut (dyn MaybeSerializableData)>],
}

#[derive(Copy, Clone)]
pub struct PrimitiveContactGenerator {
    pub generate_contacts: fn(&mut PrimitiveContactGenerationContext),
    #[cfg(feature = "simd-is-enabled")]
    pub generate_contacts_simd: fn(&mut PrimitiveContactGenerationContextSimd),
}

impl PrimitiveContactGenerator {
    fn unimplemented_fn(_ctxt: &mut PrimitiveContactGenerationContext) {}
    #[cfg(feature = "simd-is-enabled")]
    fn unimplemented_simd_fn(_ctxt: &mut PrimitiveContactGenerationContextSimd) {}
}

impl Default for PrimitiveContactGenerator {
    fn default() -> Self {
        Self {
            generate_contacts: Self::unimplemented_fn,
            #[cfg(feature = "simd-is-enabled")]
            generate_contacts_simd: Self::unimplemented_simd_fn,
        }
    }
}

pub struct ContactGenerationContext<'a> {
    pub dispatcher: &'a dyn ContactDispatcher,
    pub prediction_distance: f32,
    pub colliders: &'a ColliderSet,
    pub pair: &'a mut ContactPair,
    pub solver_flags: SolverFlags,
}

#[cfg(feature = "simd-is-enabled")]
pub struct ContactGenerationContextSimd<'a, 'b> {
    pub dispatcher: &'a dyn ContactDispatcher,
    pub prediction_distance: f32,
    pub colliders: &'a ColliderSet,
    pub pairs: &'a mut [&'b mut ContactPair],
    pub solver_flags: &'a [SolverFlags],
}

#[derive(Copy, Clone)]
pub struct ContactGenerator {
    pub generate_contacts: fn(&mut ContactGenerationContext),
    #[cfg(feature = "simd-is-enabled")]
    pub generate_contacts_simd: fn(&mut ContactGenerationContextSimd),
}

impl ContactGenerator {
    fn unimplemented_fn(_ctxt: &mut ContactGenerationContext) {}
    #[cfg(feature = "simd-is-enabled")]
    fn unimplemented_simd_fn(_ctxt: &mut ContactGenerationContextSimd) {}
}

impl Default for ContactGenerator {
    fn default() -> Self {
        Self {
            generate_contacts: Self::unimplemented_fn,
            #[cfg(feature = "simd-is-enabled")]
            generate_contacts_simd: Self::unimplemented_simd_fn,
        }
    }
}
