use bevy::{ecs::system::SystemParam, prelude::*};
use rapier::{
    geometry::{ContactManifold, SoftPairContacts, SolverContact, SolverContacts},
    pipeline::{ContactModificationContext, ModifiableContacts, PairFilterContext},
    prelude::{PhysicsHooks, SolverFlags},
};

use crate::math::{Real, Vect};

/// Read-only access to the properties of a collision pair filter context.
pub struct PairFilterContextView<'a> {
    /// The raw context from Rapier.
    pub raw: &'a PairFilterContext<'a>,
}

impl PairFilterContextView<'_> {
    /// The entity of the first collider involved in the potential collision.
    pub fn collider1(&self) -> Entity {
        let co1 = &self.raw.colliders[self.raw.collider1];
        Entity::from_bits(co1.user_data as u64)
    }

    /// The entity of the second collider involved in the potential collision.
    pub fn collider2(&self) -> Entity {
        let co2 = &self.raw.colliders[self.raw.collider2];
        Entity::from_bits(co2.user_data as u64)
    }

    /// The entity of the first rigid-body (if `self.collider1()` is attached to a rigid-body)
    /// involved in the potential collision.
    pub fn rigid_body1(&self) -> Option<Entity> {
        self.raw.rigid_body1.map(|h| {
            let co1 = &self.raw.bodies[h];
            Entity::from_bits(co1.user_data as u64)
        })
    }

    /// The entity of the second rigid-body (if `self.collider1()` is attached to a rigid-body)
    /// involved in the potential collision.
    pub fn rigid_body2(&self) -> Option<Entity> {
        self.raw.rigid_body2.map(|h| {
            let co2 = &self.raw.bodies[h];
            Entity::from_bits(co2.user_data as u64)
        })
    }
}

/// Read-write access to the properties of a contact modification context.
pub struct ContactModificationContextView<'a, 'b> {
    /// The raw context from Rapier.
    pub raw: &'a mut ContactModificationContext<'b>,
}

impl ContactModificationContextView<'_, '_> {
    /// The entity of the first collider involved in the potential collision.
    pub fn collider1(&self) -> Entity {
        let co1 = &self.raw.colliders[self.raw.collider1];
        Entity::from_bits(co1.user_data as u64)
    }

    /// The entity of the second collider involved in the potential collision.
    pub fn collider2(&self) -> Entity {
        let co2 = &self.raw.colliders[self.raw.collider2];
        Entity::from_bits(co2.user_data as u64)
    }

    /// The entity of the first rigid-body (if `self.collider1()` is attached to a rigid-body)
    /// involved in the potential collision.
    pub fn rigid_body1(&self) -> Option<Entity> {
        self.raw.rigid_body1.map(|h| {
            let co1 = &self.raw.bodies[h];
            Entity::from_bits(co1.user_data as u64)
        })
    }

    /// The entity of the second rigid-body (if `self.collider1()` is attached to a rigid-body)
    /// involved in the potential collision.
    pub fn rigid_body2(&self) -> Option<Entity> {
        self.raw.rigid_body2.map(|h| {
            let co2 = &self.raw.bodies[h];
            Entity::from_bits(co2.user_data as u64)
        })
    }
}

impl ContactModificationContextView<'_, '_> {
    /// Is this a pair with contact manifolds (every pair except two soft surfaces)?
    ///
    /// All the manifold accessors of this view (e.g. [`Self::normal`], [`Self::friction`],
    /// [`Self::solver_contacts`]) are only meaningful for such pairs.
    pub fn is_rigid(&self) -> bool {
        matches!(self.raw.contacts, ModifiableContacts::Rigid(_))
    }

    /// Is this a pair of two soft surfaces?
    ///
    /// The contacts of such pairs are contact candidates rather than a manifold. They can be
    /// accessed through [`Self::soft`] and [`Self::soft_mut`]; the manifold accessors of this
    /// view return `None` (or do nothing) for them.
    pub fn is_soft(&self) -> bool {
        matches!(self.raw.contacts, ModifiableContacts::Soft(_))
    }

    /// The contact candidates of a pair of two soft surfaces (`None` for other pairs).
    pub fn soft(&self) -> Option<&SoftPairContacts> {
        self.raw.soft()
    }

    /// Mutable access to the contact candidates of a pair of two soft surfaces (`None` for other
    /// pairs).
    ///
    /// A candidate's `dir` and `dist` can be modified, or its `enabled` flag cleared to drop it;
    /// [`SoftPairContacts::disable_all`] ignores the whole pair in the constraints solver.
    pub fn soft_mut(&mut self) -> Option<&mut SoftPairContacts> {
        self.raw.soft_mut()
    }

    /// The contact manifold being modified (`None` for a pair of two soft surfaces).
    pub fn manifold(&self) -> Option<&ContactManifold> {
        self.raw.rigid().map(|m| m.manifold)
    }

    /// The world-space contact normal of the manifold (`None` for a pair of two soft surfaces).
    pub fn normal(&self) -> Option<Vect> {
        self.raw.rigid().map(|m| *m.normal)
    }

    /// Sets the world-space contact normal of the manifold (ignored for a pair of two soft surfaces).
    pub fn set_normal(&mut self, normal: Vect) {
        if let Some(m) = self.raw.rigid_mut() {
            *m.normal = normal;
        }
    }

    /// The friction coefficient applied to every solver contact of the manifold (`None` for a
    /// pair of two soft surfaces).
    pub fn friction(&self) -> Option<Real> {
        self.raw.rigid().map(|m| *m.friction)
    }

    /// Sets the friction coefficient applied to every solver contact of the manifold (ignored for a
    /// pair of two soft surfaces).
    pub fn set_friction(&mut self, friction: Real) {
        if let Some(m) = self.raw.rigid_mut() {
            *m.friction = friction;
        }
    }

    /// The restitution coefficient applied to every solver contact of the manifold (`None` for a
    /// pair of two soft surfaces).
    pub fn restitution(&self) -> Option<Real> {
        self.raw.rigid().map(|m| *m.restitution)
    }

    /// Sets the restitution coefficient applied to every solver contact of the manifold (ignored
    /// for a pair of two soft surfaces).
    pub fn set_restitution(&mut self, restitution: Real) {
        if let Some(m) = self.raw.rigid_mut() {
            *m.restitution = restitution;
        }
    }

    /// The user-defined data of the manifold, persistent across timesteps while the manifold
    /// exists (`None` for a pair of two soft surfaces).
    pub fn user_data(&self) -> Option<u32> {
        self.raw.rigid().map(|m| *m.user_data)
    }

    /// Sets the user-defined data of the manifold (ignored for a pair of two soft surfaces).
    pub fn set_user_data(&mut self, user_data: u32) {
        if let Some(m) = self.raw.rigid_mut() {
            *m.user_data = user_data;
        }
    }

    /// The solver contacts of the manifold (`None` for a pair of two soft surfaces).
    ///
    /// While inside the hook, each solver contact's `anchor1`/`anchor2` hold the world-space
    /// contact points on each body and `dist` their separation.
    pub fn solver_contacts(&self) -> Option<&[SolverContact]> {
        self.raw.rigid().map(|m| &m.solver_contacts[..])
    }

    /// Mutable access to the solver contacts of the manifold (`None` for a pair of two soft
    /// surfaces).
    ///
    /// Contacts can be modified (e.g. their `tangent_velocity` to simulate conveyor belts),
    /// removed, or all cleared to ignore this manifold in the constraints solver.
    pub fn solver_contacts_mut(&mut self) -> Option<&mut SolverContacts> {
        self.raw.rigid_mut().map(|m| &mut *m.solver_contacts)
    }

    /// Updates the contacts to emulate a one-way platform.
    ///
    /// Contacts are only allowed if the contact normal, expressed in the local space of the first
    /// collider, is within `allowed_angle` (in radians) of `allowed_local_n1`. This must be called
    /// at each timestep for each manifold involving the platform, and the manifold's user data
    /// must not be modified elsewhere since it is used to track the contact state. Pairs of two
    /// soft surfaces are left untouched.
    pub fn update_as_oneway_platform(&mut self, allowed_local_n1: Vect, allowed_angle: Real) {
        self.raw
            .update_as_oneway_platform(allowed_local_n1, allowed_angle);
    }
}

/// User-defined functions called by the physics engines during one timestep in order to customize its behavior.
///
/// Note that the default implementations of the filtering methods match Rapier's
/// [`PhysicsHooks`] defaults: [`Self::filter_contact_pair`] returns
/// `Some(SolverFlags::COMPUTE_RIGID_IMPULSES)` and [`Self::filter_intersection_pair`] returns
/// `true`, so enabling a hook on a collider without overriding the corresponding method does not
/// change which pairs interact.
pub trait BevyPhysicsHooks: SystemParam + Send + Sync {
    /// Applies the contact pair filter.
    ///
    /// Note that this method will only be called if at least one of the colliders
    /// involved in the contact contains the `ActiveHooks::FILTER_CONTACT_PAIRS` flags
    /// in its physics hooks flags.
    ///
    /// User-defined filter for potential contact pairs detected by the broad-phase.
    /// This can be used to apply custom logic in order to decide whether two colliders
    /// should have their contact computed by the narrow-phase, and if these contact
    /// should be solved by the constraints solver
    ///
    /// This filter doesn't replace the built-in filtering: it is only called for the pairs that
    /// passed it. Rapier first discards the pairs of colliders attached to the same rigid-body or
    /// to multibody links (or impulse joints) with contacts disabled between them, then the pairs
    /// rejected by the [`ActiveCollisionTypes`] of both colliders (e.g. two non-dynamic bodies by
    /// default), then the pairs rejected by their [`CollisionGroups`]. The [`SolverGroups`] are
    /// applied to the returned solver flags afterwards.
    ///
    /// [`ActiveCollisionTypes`]: crate::geometry::ActiveCollisionTypes
    /// [`CollisionGroups`]: crate::geometry::CollisionGroups
    /// [`SolverGroups`]: crate::geometry::SolverGroups
    ///
    /// If this returns `None`, then the narrow-phase will ignore this contact pair and
    /// not compute any contact manifolds for it.
    /// If this returns `Some`, then the narrow-phase will compute contact manifolds for
    /// this pair of colliders, and configure them with the returned solver flags. For
    /// example, if this returns `Some(SolverFlags::COMPUTE_RIGID_IMPULSES)` then the contacts
    /// will be taken into account by the constraints solver. If this returns
    /// `Some(SolverFlags::empty())` then the constraints solver will ignore these
    /// contacts.
    ///
    /// The default implementation returns `Some(SolverFlags::COMPUTE_RIGID_IMPULSES)`.
    fn filter_contact_pair(&self, _context: PairFilterContextView) -> Option<SolverFlags> {
        Some(SolverFlags::COMPUTE_RIGID_IMPULSES)
    }

    /// Applies the intersection pair filter.
    ///
    /// Note that this method will only be called if at least one of the colliders
    /// involved in the contact contains the `ActiveHooks::FILTER_INTERSECTION_PAIR` flags
    /// in its physics hooks flags.
    ///
    /// User-defined filter for potential intersection pairs detected by the broad-phase.
    ///
    /// This can be used to apply custom logic in order to decide whether two colliders
    /// should have their intersection computed by the narrow-phase.
    ///
    /// This filter doesn't replace the built-in filtering: it is only called for the pairs that
    /// passed it. Rapier first discards the pairs rejected by the [`ActiveCollisionTypes`] of both
    /// colliders (e.g. two non-dynamic bodies by default), then the pairs rejected by their
    /// [`CollisionGroups`].
    ///
    /// [`ActiveCollisionTypes`]: crate::geometry::ActiveCollisionTypes
    /// [`CollisionGroups`]: crate::geometry::CollisionGroups
    ///
    /// If this returns `false`, then the narrow-phase will ignore this pair and
    /// not compute any intersection information for it.
    /// If this return `true` then the narrow-phase will compute intersection
    /// information for this pair.
    ///
    /// The default implementation returns `true`.
    fn filter_intersection_pair(&self, _context: PairFilterContextView) -> bool {
        true
    }

    /// Modifies the set of contacts seen by the constraints solver.
    ///
    /// Note that this method will only be called if at least one of the colliders
    /// involved in the contact contains the `ActiveHooks::MODIFY_SOLVER_CONTACTS` flags
    /// in its physics hooks flags.
    ///
    /// It is called on each contact manifold (with at least one solver contact) of such pairs,
    /// after the solver contacts were computed from the manifold's contact points. This can be
    /// used to modify the set of solver contacts seen by the constraints solver: contacts can be
    /// removed and modified.
    ///
    /// Note that if all the contacts have to be ignored by the constraint solver, you may simply
    /// clear them with `context.solver_contacts_mut()`.
    ///
    /// Modifying the solver contacts allow you to achieve various effects, including:
    /// - Simulating conveyor belts by setting the `tangent_velocity` of a solver contact.
    /// - Simulating shapes with multiply materials by modifying the friction and restitution
    ///   coefficient depending of the features in contacts.
    /// - Simulating one-way platforms depending on the contact normal.
    ///
    /// Each contact manifold is given a `u32` user-defined data that is persistent between
    /// timesteps (as long as the contact manifold exists). This user-defined data is initialized
    /// as 0 and can be modified with [`ContactModificationContextView::set_user_data`].
    ///
    /// The world-space contact normal, friction and restitution can be modified with
    /// [`ContactModificationContextView::set_normal`],
    /// [`ContactModificationContextView::set_friction`] and
    /// [`ContactModificationContextView::set_restitution`].
    fn modify_solver_contacts(&self, _context: ContactModificationContextView) {}
}

impl<T> BevyPhysicsHooks for T
where
    T: 'static + PhysicsHooks + SystemParam + Send + Sync,
    for<'w, 's> T: SystemParam<Item<'w, 's> = T>,
{
    fn filter_contact_pair(&self, context: PairFilterContextView) -> Option<SolverFlags> {
        PhysicsHooks::filter_contact_pair(self, context.raw)
    }

    fn filter_intersection_pair(&self, context: PairFilterContextView) -> bool {
        PhysicsHooks::filter_intersection_pair(self, context.raw)
    }

    fn modify_solver_contacts(&self, context: ContactModificationContextView) {
        PhysicsHooks::modify_solver_contacts(self, context.raw)
    }
}

/// Adapts a type implementing `BevyPhysicsHooks` so that it implements `PhysicsHooks`.
pub(crate) struct BevyPhysicsHooksAdapter<Hooks>
where
    Hooks: BevyPhysicsHooks,
{
    hooks: Hooks,
}

impl<Hooks> BevyPhysicsHooksAdapter<Hooks>
where
    Hooks: BevyPhysicsHooks,
{
    pub(crate) fn new(hooks: Hooks) -> Self {
        Self { hooks }
    }
}

impl<Hooks> PhysicsHooks for BevyPhysicsHooksAdapter<Hooks>
where
    Hooks: BevyPhysicsHooks,
{
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        let context_view = PairFilterContextView { raw: context };
        self.hooks.filter_contact_pair(context_view)
    }

    fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool {
        let context_view = PairFilterContextView { raw: context };
        self.hooks.filter_intersection_pair(context_view)
    }

    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        let context_view = ContactModificationContextView { raw: context };
        self.hooks.modify_solver_contacts(context_view)
    }
}

#[cfg(test)]
mod test {
    use crate::math::{Real, Vect};
    use crate::prelude::*;
    use bevy::{
        ecs::system::SystemParam,
        prelude::*,
        time::{TimePlugin, TimeUpdateStrategy},
    };
    use std::sync::{
        atomic::{AtomicUsize, Ordering},
        Arc,
    };

    const FRICTION: Real = 0.123;
    const RESTITUTION: Real = 0.0;

    #[derive(Resource, Default)]
    struct HookCalls(Arc<AtomicUsize>);

    #[derive(SystemParam)]
    struct ModifyingHooks<'w> {
        calls: Res<'w, HookCalls>,
    }

    impl BevyPhysicsHooks for ModifyingHooks<'_> {
        fn modify_solver_contacts(&self, mut context: ContactModificationContextView) {
            assert!(context.is_rigid());
            assert!(!context.is_soft());
            assert!(context.manifold().is_some());
            assert!(context.solver_contacts().is_some());
            let n = context.normal().unwrap();
            context.set_normal(if n.y > 0.0 { Vect::Y } else { -Vect::Y });
            context.set_friction(FRICTION);
            context.set_restitution(RESTITUTION);
            context.set_user_data(42);
            self.calls.0.fetch_add(1, Ordering::SeqCst);
        }
    }

    #[cfg(feature = "dim3")]
    fn cuboid(hx: Real, hy: Real, hz: Real) -> Collider {
        Collider::cuboid(hx, hy, hz)
    }
    #[cfg(feature = "dim2")]
    fn cuboid(hx: Real, hy: Real, _hz: Real) -> Collider {
        Collider::cuboid(hx, hy)
    }

    #[test]
    fn hook_modifies_friction_and_normal() {
        let mut app = App::new();
        app.add_plugins((
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<ModifyingHooks>::default(),
        ))
        .init_resource::<HookCalls>()
        .insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1f32 / 60f32),
        ));
        let ground = app
            .world_mut()
            .spawn((Transform::from_xyz(0.0, -1.0, 0.0), cuboid(4.0, 1.0, 4.0)))
            .id();
        let body = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 0.6, 0.0),
                RigidBody::Dynamic,
                cuboid(0.5, 0.5, 0.5),
                ActiveHooks::MODIFY_SOLVER_CONTACTS,
            ))
            .id();
        app.finish();
        for _ in 0..60 {
            app.update();
        }

        assert!(app.world().resource::<HookCalls>().0.load(Ordering::SeqCst) > 0);

        let mut query = app.world_mut().query::<(
            &RapierContextSimulation,
            &RapierContextColliders,
            &RapierRigidBodySet,
        )>();
        let (simulation, colliders, bodies) = query.single(app.world()).unwrap();
        let pair = simulation
            .contact_pair(colliders, bodies, ground, body)
            .unwrap();
        assert!(pair.has_any_active_contact());
        for manifold in pair.manifolds() {
            assert_eq!(manifold.friction(), FRICTION);
            assert_eq!(manifold.restitution(), RESTITUTION);
            assert_eq!(manifold.user_data(), 42);
            assert_eq!(manifold.normal().abs(), Vect::Y);
        }
    }
}
