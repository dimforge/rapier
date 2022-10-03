use crate::dynamics::RigidBodySet;
use crate::geometry::{ColliderHandle, ColliderSet, ContactManifold, Shape, TOI};
use crate::math::{Isometry, Point, Real, UnitVector, Vector};
use crate::pipeline::{QueryFilter, QueryFilterFlags, QueryPipeline};
use crate::utils;
use na::{RealField, Vector2};
use parry::bounding_volume::BoundingVolume;
use parry::math::Translation;
use parry::query::{DefaultQueryDispatcher, PersistentQueryDispatcher};

#[derive(Copy, Clone, Debug, PartialEq)]
/// A length measure used for various options of a character controller.
pub enum CharacterLength {
    /// The length is specified relative to some of the character shape’s size.
    ///
    /// For example setting `CharacterAutostep::max_height` to `CharaceterLentgh::Relative(0.1)`
    /// for a shape with an height equal to 20.0 will result in a maximum step heigth
    /// of `0.1 * 20.0 = 2.0`.
    Relative(Real),
    /// The lengt his specified as an aboslute value, independent from the character shape’s size.
    ///
    /// For example setting `CharacterAutostep::max_height` to `CharaceterLentgh::Relative(0.1)`
    /// for a shape with an height equal to 20.0 will result in a maximum step heigth
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

/// Configuration for the auto-stepping character controller feature.
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

/// A collision between the character and its environment during its movement.
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
    pub toi: TOI,
}

/// A character controller for kinematic bodies.
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
    /// Should the character automatically step over small obstacles?
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
}

impl Default for KinematicCharacterController {
    fn default() -> Self {
        Self {
            up: Vector::y_axis(),
            offset: CharacterLength::Relative(0.01),
            slide: true,
            autostep: Some(CharacterAutostep::default()),
            max_slope_climb_angle: Real::frac_pi_4(),
            min_slope_slide_angle: Real::frac_pi_4(),
            snap_to_ground: Some(CharacterLength::Relative(0.2)),
        }
    }
}

/// The effective movement computed by the character controller.
pub struct EffectiveCharacterMovement {
    /// The movement to apply.
    pub translation: Vector<Real>,
    /// Is the character touching the ground after applying `EffictiveKineamticMovement::translation`?
    pub grounded: bool,
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
        };

        let extents = character_shape.compute_local_aabb().extents();
        let up_extent = extents.dot(&self.up);
        let side_extent = (extents - *self.up * up_extent).norm();
        let dims = Vector2::new(side_extent, up_extent);

        // 1. Check and fix penetrations.
        self.check_and_fix_penetrations();

        let mut translation_remaining = desired_translation;

        // Check if we are grounded at the initial position.
        let grounded_at_starting_pos = self.detect_grounded_status_and_apply_friction(
            dt,
            bodies,
            colliders,
            queries,
            character_shape,
            &character_pos,
            &dims,
            filter,
            None,
            None,
        );

        // println!("Init grounded status: {grounded_at_starting_pos}");

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
            if let Some((handle, toi)) = queries.cast_shape(
                bodies,
                colliders,
                &(Translation::from(result.translation) * character_pos),
                &translation_dir,
                character_shape,
                translation_dist + offset,
                false,
                filter,
            ) {
                // We hit something, compute the allowed self.
                let allowed_dist =
                    (toi.toi - (-toi.normal1.dot(&translation_dir)) * offset).max(0.0);
                let allowed_translation = *translation_dir * allowed_dist;
                result.translation += allowed_translation;
                translation_remaining -= allowed_translation;

                events(CharacterCollision {
                    handle,
                    character_pos: Translation::from(result.translation) * character_pos,
                    translation_applied: result.translation,
                    translation_remaining,
                    toi,
                });

                if let (Some(translation_on_slope), _) =
                    self.handle_slopes(&toi, &mut translation_remaining)
                {
                    translation_remaining = translation_on_slope;
                } else {
                    // If the slope is too big, try to step on the stair.
                    self.handle_stairs(
                        bodies,
                        colliders,
                        queries,
                        character_shape,
                        &(Translation::from(result.translation) * character_pos),
                        &dims,
                        filter,
                        handle,
                        &mut translation_remaining,
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
    ) -> Option<(ColliderHandle, TOI)> {
        if let Some(snap_distance) = self.snap_to_ground {
            let snap_distance = snap_distance.eval(dims.y);
            if result.translation.dot(&self.up) < 1.0e-5 {
                let offset = self.offset.eval(dims.y);
                if let Some((hit_handle, hit)) = queries.cast_shape(
                    bodies,
                    colliders,
                    character_pos,
                    &-self.up,
                    character_shape,
                    snap_distance + offset,
                    false,
                    filter,
                ) {
                    // Apply the snap.
                    result.translation -= *self.up * (hit.toi - offset).max(0.0);
                    result.grounded = true;
                    return Some((hit_handle, hit));
                }
            }
        }

        None
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
        let prediction = self.offset.eval(dims.y) * 1.1;

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
                            let normal1 = character_pos * m.local_n1;
                            let normal2 = -normal1;

                            if normal1.dot(&self.up) <= -1.0e-5 {
                                grounded = true;
                            }

                            if let Some(kinematic_parent) = kinematic_parent {
                                let mut num_active_contacts = 0;
                                let mut manifold_center = Point::origin();

                                for contact in &m.points {
                                    if contact.dist <= prediction {
                                        num_active_contacts += 1;
                                        let contact_point = collider.position() * contact.local_p2;
                                        let target_vel =
                                            kinematic_parent.velocity_at_point(&contact_point);

                                        let normal_target_mvt = target_vel.dot(&normal2) * dt;
                                        let normal_current_mvt =
                                            translation_remaining.dot(&normal2);

                                        manifold_center += contact_point.coords;
                                        *translation_remaining += normal2
                                            * (normal_target_mvt - normal_current_mvt).max(0.0);
                                    }
                                }

                                if num_active_contacts > 0 {
                                    let target_vel = kinematic_parent.velocity_at_point(
                                        &(manifold_center / num_active_contacts as Real),
                                    );
                                    let tangent_platform_mvt =
                                        (target_vel - normal2 * target_vel.dot(&normal2)) * dt;
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
                            let normal = character_pos * m.local_n1;

                            if normal.dot(&self.up) <= -1.0e-5 {
                                for contact in &m.points {
                                    if contact.dist <= prediction {
                                        grounded = true;
                                        return false; // We can stop the search early.
                                    }
                                }
                            }
                        }
                    }
                }
            }
            true
        });

        grounded
    }

    fn handle_slopes(
        &self,
        hit: &TOI,
        translation_remaining: &Vector<Real>,
    ) -> (Option<Vector<Real>>, Real) {
        let vertical_translation_remaining = *self.up * (self.up.dot(translation_remaining));
        let horizontal_translation_remaining =
            *translation_remaining - vertical_translation_remaining;

        // The idea behind this `if` statement is as follows:
        // - If there is any amount of horizontal translations, then the intended
        //   climb/slide down movement is decided by that translation.
        // - If there is no horizontal translation, then we only have gravity. In that case,
        //   we take the vertical movement into account to decide if we need to slide down.
        let sliding_translation_remaining = if horizontal_translation_remaining != Vector::zeros() {
            horizontal_translation_remaining
                - *hit.normal1 * (horizontal_translation_remaining).dot(&hit.normal1)
        } else {
            vertical_translation_remaining
                - *hit.normal1 * (vertical_translation_remaining).dot(&hit.normal1)
        };

        // Check if there is a slope we can climb.
        let angle_with_floor = self.up.angle(&hit.normal1);
        let climbing = self.up.dot(&sliding_translation_remaining) >= 0.0;

        if !climbing {
            // Moving down the slope.
            let remaining = if angle_with_floor >= self.min_slope_slide_angle {
                // Can slide down.
                sliding_translation_remaining
            } else {
                // To avoid sliding down, we remove the sliding component due to the vertical
                // part of the movement but have to keep the component due to the horizontal
                // part of the self.
                *translation_remaining
                    - (*hit.normal1 * horizontal_translation_remaining.dot(&hit.normal1)
                        + vertical_translation_remaining)
                // Remove the complete vertical part.
            };

            (Some(remaining), -angle_with_floor)
        } else {
            // Moving up the slope.
            let remaining = if angle_with_floor <= self.max_slope_climb_angle {
                // Let’s climb by cancelling from the desired movement the part that
                // doesn’t line up with the slope, and continuing the loop.
                Some(sliding_translation_remaining)
            } else {
                // The slope was too steep.
                None
            };

            (remaining, angle_with_floor)
        }
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
        translation_remaining: &mut Vector<Real>,
        result: &mut EffectiveCharacterMovement,
    ) -> bool {
        if let Some(autostep) = self.autostep {
            let min_width = autostep.min_width.eval(dims.x);
            let max_height = autostep.max_height.eval(dims.y);

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

            if let Some(horizontal_dir) = (*translation_remaining
                - *self.up * translation_remaining.dot(&self.up))
            .try_normalize(1.0e-5)
            {
                if queries
                    .cast_shape(
                        bodies,
                        colliders,
                        character_pos,
                        &self.up,
                        character_shape,
                        max_height,
                        false,
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
                        min_width,
                        false,
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
                    max_height,
                    false,
                    filter,
                ) {
                    let vertical_translation_remaining =
                        *self.up * (self.up.dot(translation_remaining));
                    let horizontal_translation_remaining =
                        *translation_remaining - vertical_translation_remaining;
                    let sliding_movement = horizontal_translation_remaining
                        - *hit.normal1 * horizontal_translation_remaining.dot(&hit.normal1);

                    let angle_with_floor = self.up.angle(&hit.normal1);
                    let climbing = self.up.dot(&sliding_movement) >= 0.0;

                    if climbing && angle_with_floor > self.max_slope_climb_angle {
                        return false; // The target ramp is too steep.
                    }
                }

                // We can step, we need to find the actual step height.
                let step_height = self.offset.eval(dims.y) + max_height
                    - queries
                        .cast_shape(
                            bodies,
                            colliders,
                            &(Translation::from(horizontal_dir * min_width)
                                * shifted_character_pos),
                            &-self.up,
                            character_shape,
                            max_height,
                            false,
                            filter,
                        )
                        .map(|hit| hit.1.toi)
                        .unwrap_or(max_height);

                // Remove the step height from the vertical part of the self.
                *translation_remaining -=
                    *self.up * translation_remaining.dot(&self.up).clamp(0.0, step_height);

                // Advance the collider on the step horizontally, to make sure further
                // movement won’t just get stuck on its edge.
                let horizontal_nudge =
                    horizontal_dir * min_width.min(horizontal_dir.dot(translation_remaining));
                *translation_remaining -= horizontal_nudge;

                result.translation += *self.up * step_height + horizontal_nudge;
                return true;
            }
        }

        false
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
        let up_extent = extents.dot(&self.up);
        let movement_to_transfer =
            *collision.toi.normal1 * collision.translation_remaining.dot(&collision.toi.normal1);
        let prediction = self.offset.eval(up_extent) * 1.1;

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
