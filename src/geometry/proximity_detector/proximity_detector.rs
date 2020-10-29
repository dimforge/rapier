use crate::geometry::{
    Collider, ColliderSet, Proximity, ProximityDispatcher, ProximityEvent, ProximityPair, Shape,
};
use crate::math::Isometry;
#[cfg(feature = "simd-is-enabled")]
use crate::math::{SimdFloat, SIMD_WIDTH};
use crate::pipeline::EventHandler;
use std::any::Any;

#[derive(Copy, Clone)]
pub enum ProximityPhase {
    NearPhase(ProximityDetector),
    ExactPhase(PrimitiveProximityDetector),
}

impl ProximityPhase {
    #[inline]
    pub fn detect_proximity(
        self,
        mut context: ProximityDetectionContext,
        events: &dyn EventHandler,
    ) {
        let proximity = match self {
            Self::NearPhase(gen) => (gen.detect_proximity)(&mut context),
            Self::ExactPhase(gen) => {
                // Build the primitive context from the non-primitive context and dispatch.
                let collider1 = &context.colliders[context.pair.pair.collider1];
                let collider2 = &context.colliders[context.pair.pair.collider2];

                let mut context2 = PrimitiveProximityDetectionContext {
                    prediction_distance: context.prediction_distance,
                    collider1,
                    collider2,
                    shape1: collider1.shape(),
                    shape2: collider2.shape(),
                    position1: collider1.position(),
                    position2: collider2.position(),
                    workspace: context.pair.detector_workspace.as_mut().map(|w| &mut **w),
                };

                (gen.detect_proximity)(&mut context2)
            }
        };

        if context.pair.proximity != proximity {
            events.handle_proximity_event(ProximityEvent::new(
                context.pair.pair.collider1,
                context.pair.pair.collider2,
                context.pair.proximity,
                proximity,
            ))
        }

        context.pair.proximity = proximity;
    }

    #[cfg(feature = "simd-is-enabled")]
    #[inline]
    pub fn detect_proximity_simd(
        self,
        mut context: ProximityDetectionContextSimd,
        events: &dyn EventHandler,
    ) {
        let proximities = match self {
            Self::NearPhase(gen) => (gen.detect_proximity_simd)(&mut context),
            Self::ExactPhase(gen) => {
                // Build the primitive context from the non-primitive context and dispatch.
                use arrayvec::ArrayVec;
                let mut colliders_arr: ArrayVec<[(&Collider, &Collider); SIMD_WIDTH]> =
                    ArrayVec::new();
                let mut workspace_arr: ArrayVec<
                    [Option<&mut (dyn Any + Send + Sync)>; SIMD_WIDTH],
                > = ArrayVec::new();

                for pair in context.pairs.iter_mut() {
                    let collider1 = &context.colliders[pair.pair.collider1];
                    let collider2 = &context.colliders[pair.pair.collider2];
                    colliders_arr.push((collider1, collider2));
                    workspace_arr.push(pair.detector_workspace.as_mut().map(|w| &mut **w));
                }

                let max_index = colliders_arr.len() - 1;
                let colliders1 = array![|ii| colliders_arr[ii.min(max_index)].0; SIMD_WIDTH];
                let colliders2 = array![|ii| colliders_arr[ii.min(max_index)].1; SIMD_WIDTH];

                let mut context2 = PrimitiveProximityDetectionContextSimd {
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
                    workspaces: workspace_arr.as_mut_slice(),
                };

                (gen.detect_proximity_simd)(&mut context2)
            }
        };

        for (i, pair) in context.pairs.iter_mut().enumerate() {
            if pair.proximity != proximities[i] {
                events.handle_proximity_event(ProximityEvent::new(
                    pair.pair.collider1,
                    pair.pair.collider2,
                    pair.proximity,
                    proximities[i],
                ))
            }
            pair.proximity = proximities[i];
        }
    }
}

pub struct PrimitiveProximityDetectionContext<'a> {
    pub prediction_distance: f32,
    pub collider1: &'a Collider,
    pub collider2: &'a Collider,
    pub shape1: &'a dyn Shape,
    pub shape2: &'a dyn Shape,
    pub position1: &'a Isometry<f32>,
    pub position2: &'a Isometry<f32>,
    pub workspace: Option<&'a mut (dyn Any + Send + Sync)>,
}

#[cfg(feature = "simd-is-enabled")]
pub struct PrimitiveProximityDetectionContextSimd<'a, 'b> {
    pub prediction_distance: f32,
    pub colliders1: [&'a Collider; SIMD_WIDTH],
    pub colliders2: [&'a Collider; SIMD_WIDTH],
    pub shapes1: [&'a dyn Shape; SIMD_WIDTH],
    pub shapes2: [&'a dyn Shape; SIMD_WIDTH],
    pub positions1: &'a Isometry<SimdFloat>,
    pub positions2: &'a Isometry<SimdFloat>,
    pub workspaces: &'a mut [Option<&'b mut (dyn Any + Send + Sync)>],
}

#[derive(Copy, Clone)]
pub struct PrimitiveProximityDetector {
    pub detect_proximity: fn(&mut PrimitiveProximityDetectionContext) -> Proximity,
    #[cfg(feature = "simd-is-enabled")]
    pub detect_proximity_simd:
        fn(&mut PrimitiveProximityDetectionContextSimd) -> [Proximity; SIMD_WIDTH],
}

impl PrimitiveProximityDetector {
    fn unimplemented_fn(_ctxt: &mut PrimitiveProximityDetectionContext) -> Proximity {
        Proximity::Disjoint
    }
    #[cfg(feature = "simd-is-enabled")]
    fn unimplemented_simd_fn(
        _ctxt: &mut PrimitiveProximityDetectionContextSimd,
    ) -> [Proximity; SIMD_WIDTH] {
        [Proximity::Disjoint; SIMD_WIDTH]
    }
}

impl Default for PrimitiveProximityDetector {
    fn default() -> Self {
        Self {
            detect_proximity: Self::unimplemented_fn,
            #[cfg(feature = "simd-is-enabled")]
            detect_proximity_simd: Self::unimplemented_simd_fn,
        }
    }
}

pub struct ProximityDetectionContext<'a> {
    pub dispatcher: &'a dyn ProximityDispatcher,
    pub prediction_distance: f32,
    pub colliders: &'a ColliderSet,
    pub pair: &'a mut ProximityPair,
}

#[cfg(feature = "simd-is-enabled")]
pub struct ProximityDetectionContextSimd<'a, 'b> {
    pub dispatcher: &'a dyn ProximityDispatcher,
    pub prediction_distance: f32,
    pub colliders: &'a ColliderSet,
    pub pairs: &'a mut [&'b mut ProximityPair],
}

#[derive(Copy, Clone)]
pub struct ProximityDetector {
    pub detect_proximity: fn(&mut ProximityDetectionContext) -> Proximity,
    #[cfg(feature = "simd-is-enabled")]
    pub detect_proximity_simd: fn(&mut ProximityDetectionContextSimd) -> [Proximity; SIMD_WIDTH],
}

impl ProximityDetector {
    fn unimplemented_fn(_ctxt: &mut ProximityDetectionContext) -> Proximity {
        Proximity::Disjoint
    }
    #[cfg(feature = "simd-is-enabled")]
    fn unimplemented_simd_fn(_ctxt: &mut ProximityDetectionContextSimd) -> [Proximity; SIMD_WIDTH] {
        [Proximity::Disjoint; SIMD_WIDTH]
    }
}

impl Default for ProximityDetector {
    fn default() -> Self {
        Self {
            detect_proximity: Self::unimplemented_fn,
            #[cfg(feature = "simd-is-enabled")]
            detect_proximity_simd: Self::unimplemented_simd_fn,
        }
    }
}
