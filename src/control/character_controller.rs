use crate::dynamics::RigidBodySet;
use crate::geometry::{ColliderHandle, ColliderSet, ContactManifold, Shape, ShapeCastHit};
use crate::math::{Isometry, Point, Real, UnitVector, Vector};
use crate::pipeline::{QueryFilter, QueryFilterFlags, QueryPipeline};
use crate::utils;
use na::{RealField, Vector2};
use parry::bounding_volume::BoundingVolume;
use parry::math::Translation;
use parry::query::details::ShapeCastOptions;
use parry::query::{DefaultQueryDispatcher, PersistentQueryDispatcher};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
/// A length measure used for various options of a character controller.
pub enum CharacterLength {
    /// The length is specified relative to some of the character shape’s size.
    ///
    /// For example setting `CharacterAutostep::max_height` to `CharacterLength::Relative(0.1)`
    /// for a shape with a height equal to 20.0 will result in a maximum step height
    /// of `0.1 * 20.0 = 2.0`.
    Relative(Real),
    /// The length is specified as an absolute value, independent from the character shape’s size.
    ///
    /// For example setting `CharacterAutostep::max_height` to `CharacterLength::Relative(0.1)`
    /// for a shape with a height equal to 20.0 will result in a maximum step height
    /// of `0.1` (the shape height is ignored in for this value).
    Absolute(Real),
}

impl CharacterLength {
    /// Returns `self` with its value changed by the closure `f` if `self` is the `Self::Absolute`
    /// variant.
    pub fn map_absolute(self, f: impl FnOnce(Real) -> Real) -> Self {
        if let Self::Absolute(value) = self {
            Self::Absolute(f(value))
        } else {
            self
        }
    }

    /// Returns `self` with its value changed by the closure `f` if `self` is the `Self::Relative`
    /// variant.
    pub fn map_relative(self, f: impl FnOnce(Real) -> Real) -> Self {
        if let Self::Relative(value) = self {
            Self::Relative(f(value))
        } else {
            self
        }
    }

    fn eval(self, value: Real) -> Real {
        match self {
            Self::Relative(x) => value * x,
            Self::Absolute(x) => x,
        }
    }
}

#[derive(Debug)]
struct HitInfo {
    toi: ShapeCastHit,
    is_wall: bool,
    is_nonslip_slope: bool,
}

/// Configuration for the auto-stepping character controller feature.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CharacterAutostep {
    /// The maximum step height a character can automatically step over.
    pub max_height: CharacterLength,
    /// The minimum width of free space that must be available after stepping on a stair.
    pub min_width: CharacterLength,
    /// Can the character automatically step over dynamic bodies too?
    pub include_dynamic_bodies: bool,
}

impl Default for CharacterAutostep {
    fn default() -> Self {
        Self {
            max_height: CharacterLength::Relative(0.25),
            min_width: CharacterLength::Relative(0.5),
            include_dynamic_bodies: true,
        }
    }
}

#[derive(Debug)]
struct HitDecomposition {
    normal_part: Vector<Real>,
    horizontal_tangent: Vector<Real>,
    vertical_tangent: Vector<Real>,
    // NOTE: we don’t store the penetration part since we don’t really need it
    //       for anything.
}

impl HitDecomposition {
    pub fn unconstrained_slide_part(&self) -> Vector<Real> {
        self.normal_part + self.horizontal_tangent + self.vertical_tangent
    }
}

/// A collision between the character and its environment during its movement.
#[derive(Copy, Clone, Debug)]
pub struct CharacterCollision {
    /// The collider hit by the character.
    pub handle: ColliderHandle,
    /// The position of the character when the collider was hit.
    pub character_pos: Isometry<Real>,
    /// The translation that was already applied to the character when the hit happens.
    pub translation_applied: Vector<Real>,
    /// The translations that was still waiting to be applied to the character when the hit happens.
    pub translation_remaining: Vector<Real>,
    /// Geometric information about the hit.
    pub hit: ShapeCastHit,
}

/// A character controller for kinematic bodies.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug)]
pub struct KinematicCharacterController {
    /// The direction that goes "up". Used to determine where the floor is, and the floor’s angle.
    pub up: UnitVector<Real>,
    /// A small gap to preserve between the character and its surroundings.
    ///
    /// This value should not be too large to avoid visual artifacts, but shouldn’t be too small
    /// (must not be zero) to improve numerical stability of the character controller.
    pub offset: CharacterLength,
    /// Should the character try to slide against the floor if it hits it?
    pub slide: bool,
    /// Should the character automatically step over small obstacles? (disabled by default)
    ///
    /// Note that autostepping is currently a very computationally expensive feature, so it
    /// is disabled by default.
    pub autostep: Option<CharacterAutostep>,
    /// The maximum angle (radians) between the floor’s normal and the `up` vector that the
    /// character is able to climb.
    pub max_slope_climb_angle: Real,
    /// The minimum angle (radians) between the floor’s normal and the `up` vector before the
    /// character starts to slide down automatically.
    pub min_slope_slide_angle: Real,
    /// Should the character be automatically snapped to the ground if the distance between
    /// the ground and its feed are smaller than the specified threshold?
    pub snap_to_ground: Option<CharacterLength>,
    /// Increase this number if your character appears to get stuck when sliding against surfaces.
    ///
    /// This is a small distance applied to the movement toward the contact normals of shapes hit
    /// by the character controller. This helps shape-casting not getting stuck in an always-penetrating
    /// state during the sliding calculation.
    ///
    /// This value should remain fairly small since it can introduce artificial "bumps" when sliding
    /// along a flat surface.
    pub normal_nudge_factor: Real,
}

impl Default for KinematicCharacterController {
    fn default() -> Self {
        Self {
            up: Vector::y_axis(),
            offset: CharacterLength::Relative(0.01),
            slide: true,
            autostep: None,
            max_slope_climb_angle: Real::frac_pi_4(),
            min_slope_slide_angle: Real::frac_pi_4(),
            snap_to_ground: Some(CharacterLength::Relative(0.2)),
            normal_nudge_factor: 1.0e-4,
        }
    }
}

/// The effective movement computed by the character controller.
pub struct EffectiveCharacterMovement {
    /// The movement to apply.
    pub translation: Vector<Real>,
    /// Is the character touching the ground after applying `EffectiveKineamticMovement::translation`?
    pub grounded: bool,
    /// Is the character sliding down a slope due to slope angle being larger than `min_slope_slide_angle`?
    pub is_sliding_down_slope: bool,
}

impl KinematicCharacterController {
    fn check_and_fix_penetrations(&self) {
        /*
        // 1/ Check if the body is grounded and if there are penetrations.
        let mut grounded = false;
        let mut penetrating = false;

        let mut contacts = vec![];

        let aabb = shape
            .compute_aabb(shape_pos)
            .loosened(self.offset);
        queries.colliders_with_aabb_intersecting_aabb(&aabb, |handle| {
            // TODO: apply the filter.
            if let Some(collider) = colliders.get(*handle) {
                if let Ok(Some(contact)) = parry::query::contact(
                    &shape_pos,
                    shape,
                    collider.position(),
                    collider.shape(),
                    self.offset,
                ) {
                    contacts.push((contact, collider));
                }
            }

            true
        });
         */
    }

    /// Computes the possible movement for a shape.
    pub fn move_shape(
        &self,
        dt: Real,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        queries: &QueryPipeline,
        character_shape: &dyn Shape,
        character_pos: &Isometry<Real>,
        desired_translation: Vector<Real>,
        filter: QueryFilter,
        mut events: impl FnMut(CharacterCollision),
    ) -> EffectiveCharacterMovement {
        let mut result = EffectiveCharacterMovement {
            translation: Vector::zeros(),
            grounded: false,
            is_sliding_down_slope: false,
        };
        let dims = self.compute_dims(character_shape);

        // 1. Check and fix penetrations.
        self.check_and_fix_penetrations();

        let mut translation_remaining = desired_translation;

        let grounded_at_starting_pos = self.detect_grounded_status_and_apply_friction(
            dt,
            bodies,
            colliders,
            queries,
            character_shape,
            character_pos,
            &dims,
            filter,
            None,
            None,
        );

        let mut max_iters = 20;
        let mut kinematic_friction_translation = Vector::zeros();
        let offset = self.offset.eval(dims.y);

        while let Some((translation_dir, translation_dist)) =
            UnitVector::try_new_and_get(translation_remaining, 1.0e-5)
        {
            if max_iters == 0 {
                break;
            } else {
                max_iters -= 1;
            }

            // 2. Cast towards the movement direction.
            if let Some((handle, hit)) = queries.cast_shape(
                bodies,
                colliders,
                &(Translation::from(result.translation) * character_pos),
                &translation_dir,
                character_shape,
                ShapeCastOptions {
                    target_distance: offset,
                    stop_at_penetration: false,
                    max_time_of_impact: translation_dist,
                    compute_impact_geometry_on_penetration: true,
                },
                filter,
            ) {
                // We hit something, compute and apply the allowed interference-free translation.
                let allowed_dist = hit.time_of_impact;
                let allowed_translation = *translation_dir * allowed_dist;
                result.translation += allowed_translation;
                translation_remaining -= allowed_translation;

                events(CharacterCollision {
                    handle,
                    character_pos: Translation::from(result.translation) * character_pos,
                    translation_applied: result.translation,
                    translation_remaining,
                    hit,
                });

                let hit_info = self.compute_hit_info(hit);

                // Try to go upstairs.
                if !self.handle_stairs(
                    bodies,
                    colliders,
                    queries,
                    character_shape,
                    &(Translation::from(result.translation) * character_pos),
                    &dims,
                    filter,
                    handle,
                    &hit_info,
                    &mut translation_remaining,
                    &mut result,
                ) {
                    // No stairs, try to move along slopes.
                    translation_remaining = self.handle_slopes(
                        &hit_info,
                        &desired_translation,
                        &translation_remaining,
                        self.normal_nudge_factor,
                        &mut result,
                    );
                }
            } else {
                // No interference along the path.
                result.translation += translation_remaining;
                translation_remaining.fill(0.0);
                break;
            }

            result.grounded = self.detect_grounded_status_and_apply_friction(
                dt,
                bodies,
                colliders,
                queries,
                character_shape,
                &(Translation::from(result.translation) * character_pos),
                &dims,
                filter,
                Some(&mut kinematic_friction_translation),
                Some(&mut translation_remaining),
            );

            if !self.slide {
                break;
            }
        }
        // If needed, and if we are not already grounded, snap to the ground.
        if grounded_at_starting_pos {
            self.snap_to_ground(
                bodies,
                colliders,
                queries,
                character_shape,
                &(Translation::from(result.translation) * character_pos),
                &dims,
                filter,
                &mut result,
            );
        }

        // Return the result.
        result
    }

    fn snap_to_ground(
        &self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        queries: &QueryPipeline,
        character_shape: &dyn Shape,
        character_pos: &Isometry<Real>,
        dims: &Vector2<Real>,
        filter: QueryFilter,
        result: &mut EffectiveCharacterMovement,
    ) -> Option<(ColliderHandle, ShapeCastHit)> {
        if let Some(snap_distance) = self.snap_to_ground {
            if result.translation.dot(&self.up) <= 0.0 {
                let snap_distance = snap_distance.eval(dims.y);
                let offset = self.offset.eval(dims.y);
                if let Some((hit_handle, hit)) = queries.cast_shape(
                    bodies,
                    colliders,
                    character_pos,
                    &-self.up,
                    character_shape,
                    ShapeCastOptions {
                        target_distance: offset,
                        stop_at_penetration: false,
                        max_time_of_impact: snap_distance,
                        compute_impact_geometry_on_penetration: true,
                    },
                    filter,
                ) {
                    // Apply the snap.
                    result.translation -= *self.up * hit.time_of_impact;
                    result.grounded = true;
                    return Some((hit_handle, hit));
                }
            }
        }

        None
    }

    fn predict_ground(&self, up_extends: Real) -> Real {
        self.offset.eval(up_extends) * 1.1
    }

    fn detect_grounded_status_and_apply_friction(
        &self,
        dt: Real,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        queries: &QueryPipeline,
        character_shape: &dyn Shape,
        character_pos: &Isometry<Real>,
        dims: &Vector2<Real>,
        filter: QueryFilter,
        mut kinematic_friction_translation: Option<&mut Vector<Real>>,
        mut translation_remaining: Option<&mut Vector<Real>>,
    ) -> bool {
        let prediction = self.predict_ground(dims.y);

        // TODO: allow custom dispatchers.
        let dispatcher = DefaultQueryDispatcher;

        let mut manifolds: Vec<ContactManifold> = vec![];
        let character_aabb = character_shape
            .compute_aabb(character_pos)
            .loosened(prediction);

        let mut grounded = false;

        queries.colliders_with_aabb_intersecting_aabb(&character_aabb, |handle| {
            if let Some(collider) = colliders.get(*handle) {
                if filter.test(bodies, *handle, collider) {
                    manifolds.clear();
                    let pos12 = character_pos.inv_mul(collider.position());
                    let _ = dispatcher.contact_manifolds(
                        &pos12,
                        character_shape,
                        collider.shape(),
                        prediction,
                        &mut manifolds,
                        &mut None,
                    );

                    if let (Some(kinematic_friction_translation), Some(translation_remaining)) = (
                        kinematic_friction_translation.as_deref_mut(),
                        translation_remaining.as_deref_mut(),
                    ) {
                        let init_kinematic_friction_translation = *kinematic_friction_translation;
                        let kinematic_parent = collider
                            .parent
                            .and_then(|p| bodies.get(p.handle))
                            .filter(|rb| rb.is_kinematic());

                        for m in &manifolds {
                            if self.is_grounded_at_contact_manifold(m, character_pos, dims) {
                                grounded = true;
                            }

                            if let Some(kinematic_parent) = kinematic_parent {
                                let mut num_active_contacts = 0;
                                let mut manifold_center = Point::origin();
                                let normal = -(character_pos * m.local_n1);

                                for contact in &m.points {
                                    if contact.dist <= prediction {
                                        num_active_contacts += 1;
                                        let contact_point = collider.position() * contact.local_p2;
                                        let target_vel =
                                            kinematic_parent.velocity_at_point(&contact_point);

                                        let normal_target_mvt = target_vel.dot(&normal) * dt;
                                        let normal_current_mvt = translation_remaining.dot(&normal);

                                        manifold_center += contact_point.coords;
                                        *translation_remaining +=
                                            normal * (normal_target_mvt - normal_current_mvt);
                                    }
                                }

                                if num_active_contacts > 0 {
                                    let target_vel = kinematic_parent.velocity_at_point(
                                        &(manifold_center / num_active_contacts as Real),
                                    );
                                    let tangent_platform_mvt =
                                        (target_vel - normal * target_vel.dot(&normal)) * dt;
                                    kinematic_friction_translation.zip_apply(
                                        &tangent_platform_mvt,
                                        |y, x| {
                                            if x.abs() > (*y).abs() {
                                                *y = x;
                                            }
                                        },
                                    );
                                }
                            }
                        }

                        *translation_remaining +=
                            *kinematic_friction_translation - init_kinematic_friction_translation;
                    } else {
                        for m in &manifolds {
                            if self.is_grounded_at_contact_manifold(m, character_pos, dims) {
                                grounded = true;
                                return false; // We can stop the search early.
                            }
                        }
                    }
                }
            }
            true
        });

        grounded
    }

    fn is_grounded_at_contact_manifold(
        &self,
        manifold: &ContactManifold,
        character_pos: &Isometry<Real>,
        dims: &Vector2<Real>,
    ) -> bool {
        let normal = -(character_pos * manifold.local_n1);

        if normal.dot(&self.up) >= 1.0e-5 {
            let prediction = self.predict_ground(dims.y);
            for contact in &manifold.points {
                if contact.dist <= prediction {
                    return true;
                }
            }
        }
        false
    }

    fn handle_slopes(
        &self,
        hit: &HitInfo,
        movement_input: &Vector<Real>,
        translation_remaining: &Vector<Real>,
        normal_nudge_factor: Real,
        result: &mut EffectiveCharacterMovement,
    ) -> Vector<Real> {
        let [_vertical_input, horizontal_input] = self.split_into_components(movement_input);
        let horiz_input_decomp = self.decompose_hit(&horizontal_input, &hit.toi);
        let input_decomp = self.decompose_hit(movement_input, &hit.toi);

        let decomp = self.decompose_hit(translation_remaining, &hit.toi);

        // An object is trying to slip if the tangential movement induced by its vertical movement
        // points downward.
        let slipping_intent = self.up.dot(&horiz_input_decomp.vertical_tangent) < 0.0;
        let slipping = self.up.dot(&decomp.vertical_tangent) < 0.0;

        // An object is trying to climb if its indirect vertical motion points upward.
        let climbing_intent = self.up.dot(&input_decomp.vertical_tangent) > 0.0;
        let climbing = self.up.dot(&decomp.vertical_tangent) > 0.0;

        let allowed_movement = if hit.is_wall && climbing && !climbing_intent {
            // Can’t climb the slope, remove the vertical tangent motion induced by the forward motion.
            decomp.horizontal_tangent + decomp.normal_part
        } else if hit.is_nonslip_slope && slipping && !slipping_intent {
            // Prevent the vertical movement from sliding down.
            decomp.horizontal_tangent + decomp.normal_part
        } else {
            // Let it slide (including climbing the slope).
            result.is_sliding_down_slope = true;
            decomp.unconstrained_slide_part()
        };

        allowed_movement + *hit.toi.normal1 * normal_nudge_factor
    }

    fn split_into_components(&self, translation: &Vector<Real>) -> [Vector<Real>; 2] {
        let vertical_translation = *self.up * (self.up.dot(translation));
        let horizontal_translation = *translation - vertical_translation;
        [vertical_translation, horizontal_translation]
    }

    fn compute_hit_info(&self, toi: ShapeCastHit) -> HitInfo {
        let angle_with_floor = self.up.angle(&toi.normal1);
        let is_ceiling = self.up.dot(&toi.normal1) < 0.0;
        let is_wall = angle_with_floor >= self.max_slope_climb_angle && !is_ceiling;
        let is_nonslip_slope = angle_with_floor <= self.min_slope_slide_angle;

        HitInfo {
            toi,
            is_wall,
            is_nonslip_slope,
        }
    }

    fn decompose_hit(&self, translation: &Vector<Real>, hit: &ShapeCastHit) -> HitDecomposition {
        let dist_to_surface = translation.dot(&hit.normal1);
        let normal_part;
        let penetration_part;

        if dist_to_surface < 0.0 {
            normal_part = Vector::zeros();
            penetration_part = dist_to_surface * *hit.normal1;
        } else {
            penetration_part = Vector::zeros();
            normal_part = dist_to_surface * *hit.normal1;
        }

        let tangent = translation - normal_part - penetration_part;
        #[cfg(feature = "dim3")]
        let horizontal_tangent_dir = hit.normal1.cross(&self.up);
        #[cfg(feature = "dim2")]
        let horizontal_tangent_dir = Vector::zeros();

        let horizontal_tangent_dir = horizontal_tangent_dir
            .try_normalize(1.0e-5)
            .unwrap_or_default();
        let horizontal_tangent = tangent.dot(&horizontal_tangent_dir) * horizontal_tangent_dir;
        let vertical_tangent = tangent - horizontal_tangent;

        HitDecomposition {
            normal_part,
            horizontal_tangent,
            vertical_tangent,
        }
    }

    fn compute_dims(&self, character_shape: &dyn Shape) -> Vector2<Real> {
        let extents = character_shape.compute_local_aabb().extents();
        let up_extent = extents.dot(&self.up.abs());
        let side_extent = (extents - (*self.up).abs() * up_extent).norm();
        Vector2::new(side_extent, up_extent)
    }

    fn handle_stairs(
        &self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        queries: &QueryPipeline,
        character_shape: &dyn Shape,
        character_pos: &Isometry<Real>,
        dims: &Vector2<Real>,
        mut filter: QueryFilter,
        stair_handle: ColliderHandle,
        hit: &HitInfo,
        translation_remaining: &mut Vector<Real>,
        result: &mut EffectiveCharacterMovement,
    ) -> bool {
        let Some(autostep) = self.autostep else {
            return false;
        };

        // Only try to autostep on walls.
        if !hit.is_wall {
            return false;
        }

        let offset = self.offset.eval(dims.y);
        let min_width = autostep.min_width.eval(dims.x) + offset;
        let max_height = autostep.max_height.eval(dims.y) + offset;

        if !autostep.include_dynamic_bodies {
            if colliders
                .get(stair_handle)
                .and_then(|co| co.parent)
                .and_then(|p| bodies.get(p.handle))
                .map(|b| b.is_dynamic())
                == Some(true)
            {
                // The "stair" is a dynamic body, which the user wants to ignore.
                return false;
            }

            filter.flags |= QueryFilterFlags::EXCLUDE_DYNAMIC;
        }

        let shifted_character_pos = Translation::from(*self.up * max_height) * character_pos;

        let Some(horizontal_dir) = (*translation_remaining
            - *self.up * translation_remaining.dot(&self.up))
        .try_normalize(1.0e-5) else {
            return false;
        };

        if queries
            .cast_shape(
                bodies,
                colliders,
                character_pos,
                &self.up,
                character_shape,
                ShapeCastOptions {
                    target_distance: offset,
                    stop_at_penetration: false,
                    max_time_of_impact: max_height,
                    compute_impact_geometry_on_penetration: true,
                },
                filter,
            )
            .is_some()
        {
            // We can’t go up.
            return false;
        }

        if queries
            .cast_shape(
                bodies,
                colliders,
                &shifted_character_pos,
                &horizontal_dir,
                character_shape,
                ShapeCastOptions {
                    target_distance: offset,
                    stop_at_penetration: false,
                    max_time_of_impact: min_width,
                    compute_impact_geometry_on_penetration: true,
                },
                filter,
            )
            .is_some()
        {
            // We don’t have enough room on the stair to stay on it.
            return false;
        }

        // Check that we are not getting into a ramp that is too steep
        // after stepping.
        if let Some((_, hit)) = queries.cast_shape(
            bodies,
            colliders,
            &(Translation::from(horizontal_dir * min_width) * shifted_character_pos),
            &-self.up,
            character_shape,
            ShapeCastOptions {
                target_distance: offset,
                stop_at_penetration: false,
                max_time_of_impact: max_height,
                compute_impact_geometry_on_penetration: true,
            },
            filter,
        ) {
            let [vertical_slope_translation, horizontal_slope_translation] = self
                .split_into_components(translation_remaining)
                .map(|remaining| subtract_hit(remaining, &hit));

            let slope_translation = horizontal_slope_translation + vertical_slope_translation;

            let angle_with_floor = self.up.angle(&hit.normal1);
            let climbing = self.up.dot(&slope_translation) >= 0.0;

            if climbing && angle_with_floor > self.max_slope_climb_angle {
                return false; // The target ramp is too steep.
            }
        }

        // We can step, we need to find the actual step height.
        let step_height = max_height
            - queries
                .cast_shape(
                    bodies,
                    colliders,
                    &(Translation::from(horizontal_dir * min_width) * shifted_character_pos),
                    &-self.up,
                    character_shape,
                    ShapeCastOptions {
                        target_distance: offset,
                        stop_at_penetration: false,
                        max_time_of_impact: max_height,
                        compute_impact_geometry_on_penetration: true,
                    },
                    filter,
                )
                .map(|hit| hit.1.time_of_impact)
                .unwrap_or(max_height);

        // Remove the step height from the vertical part of the self.
        let step = *self.up * step_height;
        *translation_remaining -= step;

        // Advance the collider on the step horizontally, to make sure further
        // movement won’t just get stuck on its edge.
        let horizontal_nudge =
            horizontal_dir * horizontal_dir.dot(translation_remaining).min(min_width);
        *translation_remaining -= horizontal_nudge;

        result.translation += step + horizontal_nudge;
        true
    }

    /// For a given collision between a character and its environment, this method will apply
    /// impulses to the rigid-bodies surrounding the character shape at the time of the collision.
    /// Note that the impulse calculation is only approximate as it is not based on a global
    /// constraints resolution scheme.
    pub fn solve_character_collision_impulses(
        &self,
        dt: Real,
        bodies: &mut RigidBodySet,
        colliders: &ColliderSet,
        queries: &QueryPipeline,
        character_shape: &dyn Shape,
        character_mass: Real,
        collision: &CharacterCollision,
        filter: QueryFilter,
    ) {
        let extents = character_shape.compute_local_aabb().extents();
        let up_extent = extents.dot(&self.up.abs());
        let movement_to_transfer =
            *collision.hit.normal1 * collision.translation_remaining.dot(&collision.hit.normal1);
        let prediction = self.predict_ground(up_extent);

        // TODO: allow custom dispatchers.
        let dispatcher = DefaultQueryDispatcher;

        let mut manifolds: Vec<ContactManifold> = vec![];
        let character_aabb = character_shape
            .compute_aabb(&collision.character_pos)
            .loosened(prediction);

        queries.colliders_with_aabb_intersecting_aabb(&character_aabb, |handle| {
            if let Some(collider) = colliders.get(*handle) {
                if let Some(parent) = collider.parent {
                    if filter.test(bodies, *handle, collider) {
                        if let Some(body) = bodies.get(parent.handle) {
                            if body.is_dynamic() {
                                manifolds.clear();
                                let pos12 = collision.character_pos.inv_mul(collider.position());
                                let prev_manifolds_len = manifolds.len();
                                let _ = dispatcher.contact_manifolds(
                                    &pos12,
                                    character_shape,
                                    collider.shape(),
                                    prediction,
                                    &mut manifolds,
                                    &mut None,
                                );

                                for m in &mut manifolds[prev_manifolds_len..] {
                                    m.data.rigid_body2 = Some(parent.handle);
                                    m.data.normal = collision.character_pos * m.local_n1;
                                }
                            }
                        }
                    }
                }
            }
            true
        });

        let velocity_to_transfer = movement_to_transfer * utils::inv(dt);

        for manifold in &manifolds {
            let body_handle = manifold.data.rigid_body2.unwrap();
            let body = &mut bodies[body_handle];

            for pt in &manifold.points {
                if pt.dist <= prediction {
                    let body_mass = body.mass();
                    let contact_point = body.position() * pt.local_p2;
                    let delta_vel_per_contact = (velocity_to_transfer
                        - body.velocity_at_point(&contact_point))
                    .dot(&manifold.data.normal);
                    let mass_ratio = body_mass * character_mass / (body_mass + character_mass);

                    body.apply_impulse_at_point(
                        manifold.data.normal * delta_vel_per_contact.max(0.0) * mass_ratio,
                        contact_point,
                        true,
                    );
                }
            }
        }
    }
}

fn subtract_hit(translation: Vector<Real>, hit: &ShapeCastHit) -> Vector<Real> {
    let surface_correction = (-translation).dot(&hit.normal1).max(0.0);
    // This fixes some instances of moving through walls
    let surface_correction = surface_correction * (1.0 + 1.0e-5);
    translation + *hit.normal1 * surface_correction
}
