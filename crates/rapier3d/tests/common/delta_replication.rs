//! The snapshot-delta replication oracle shared by the 2D and 3D roundtrip tests (included with
//! `#[path]`; it uses the including file's imports and helpers).

use super::*;

/// Applies one serialized delta of a component to a receiver and checks the receiver's bytes.
fn sync_component<C: SnapshotDelta + serde::Serialize>(
    what: &str,
    component: &str,
    step: usize,
    sender: &mut C,
    receiver: &mut C,
    acked: &mut Option<ChangeTick<C>>,
) {
    let (delta, tick) = sender.delta_since(*acked).expect("delta_since");
    let bytes = bincode::serialize(&delta).unwrap();
    let delta: Delta<C> = bincode::deserialize(&bytes).unwrap();
    let applied = receiver.apply_delta(delta).expect("apply_delta");
    assert_eq!(applied, tick);
    *acked = Some(tick);
    let want = bincode::serialize(sender).unwrap();
    let got = bincode::serialize(receiver).unwrap();
    assert!(
        want == got,
        "{what}: `{component}` differs from the sender after a delta sync at step {step} \
         ({} vs {} bytes, delta {} bytes)",
        want.len(),
        got.len(),
        bytes.len()
    );
}

/// The serialized components of a world.
macro_rules! components {
    ($m: ident) => {
        $m!(islands, IslandManager);
        $m!(broad_phase, BroadPhaseBvh);
        $m!(narrow_phase, NarrowPhase);
        $m!(bodies, RigidBodySet);
        $m!(colliders, ColliderSet);
        $m!(impulse_joints, ImpulseJointSet);
        $m!(multibody_joints, MultibodyJointSet);
        $m!(soft_bodies, SoftBodySet);
    };
}

/// Per-component baselines of the receiver replicating components one by one.
#[derive(Default)]
struct ComponentAcks {
    islands: Option<ChangeTick<IslandManager>>,
    broad_phase: Option<ChangeTick<BroadPhaseBvh>>,
    narrow_phase: Option<ChangeTick<NarrowPhase>>,
    bodies: Option<ChangeTick<RigidBodySet>>,
    colliders: Option<ChangeTick<ColliderSet>>,
    impulse_joints: Option<ChangeTick<ImpulseJointSet>>,
    multibody_joints: Option<ChangeTick<MultibodyJointSet>>,
    soft_bodies: Option<ChangeTick<SoftBodySet>>,
}

/// A world replicated through world deltas, applied every `every` ticks.
struct WorldReceiver {
    world: PhysicsWorld,
    acked: Option<WorldChangeTick>,
    every: usize,
}

impl WorldReceiver {
    /// Returns the size of the serialized delta.
    fn sync(&mut self, what: &str, step: usize, sender: &mut PhysicsWorld) -> usize {
        let (delta, tick) = sender.delta_since(self.acked).expect("delta_since");
        let bytes = bincode::serialize(&delta).unwrap();
        let delta: WorldDelta = bincode::deserialize(&bytes).unwrap();
        assert_eq!(self.world.apply_delta(delta).expect("apply_delta"), tick);
        self.acked = Some(tick);
        assert_same(step, &record(sender), &record(&self.world), what);
        bytes.len()
    }
}

/// Steps `world` while replicating it through deltas into:
/// - a world restored from a full snapshot, synced every tick;
/// - an empty world bootstrapped by a full delta, synced every tick;
/// - a restored world syncing every five ticks against its own acknowledged baseline;
/// - individually deserialized components, synced one by one through component deltas.
///
/// Then the first two step on alongside the sender and must stay byte-identical. Returns the
/// largest serialized per-tick world delta, and the size of the initial full snapshot.
pub fn check_delta_replication(
    what: &str,
    mut world: PhysicsWorld,
    before: usize,
    after: usize,
    mut edit: impl FnMut(&mut PhysicsWorld, usize, &mut Vec<RigidBodyHandle>),
) -> (usize, usize) {
    let mut spawned = Vec::new();
    for step in 0..before {
        edit(&mut world, step, &mut spawned);
        world.step();
    }
    world.enable_change_tracking();

    // Restored receivers start from a full snapshot taken right after their baseline tick.
    let baseline = world.change_tick().unwrap();
    let snapshot = save(&world);
    let restored = || {
        let mut world = restore(&snapshot);
        world.set_delta_baseline(baseline);
        world
    };
    let mut receivers = vec![
        WorldReceiver {
            world: restored(),
            acked: Some(baseline),
            every: 1,
        },
        WorldReceiver {
            world: PhysicsWorld::new(),
            acked: None,
            every: 1,
        },
        WorldReceiver {
            world: restored(),
            acked: Some(baseline),
            every: 5,
        },
    ];
    let mut components = restored();
    let mut acks = ComponentAcks::default();
    macro_rules! component_baseline {
        ($field: ident, $ty: ty) => {
            acks.$field = Some(baseline.$field);
        };
    }
    components!(component_baseline);

    let mut max_delta = 0;
    for step in 0..after {
        edit(&mut world, before + step, &mut spawned);
        world.step();
        for (i, receiver) in receivers.iter_mut().enumerate() {
            if step % receiver.every == 0 {
                let bytes = receiver.sync(what, before + step, &mut world);
                if i == 0 {
                    max_delta = max_delta.max(bytes);
                }
            }
        }
        macro_rules! sync {
            ($field: ident, $ty: ty) => {
                sync_component(
                    what,
                    stringify!($field),
                    before + step,
                    &mut world.$field,
                    &mut components.$field,
                    &mut acks.$field,
                );
            };
        }
        components!(sync);
    }

    // Replicas continue the simulation exactly.
    let mut replicas: Vec<_> = receivers
        .drain(..2)
        .map(|r| (r.world, spawned.clone()))
        .collect();
    for step in 0..30 {
        edit(&mut world, before + after + step, &mut spawned);
        world.step();
        let want = record(&world);
        for (replica, replica_spawned) in &mut replicas {
            edit(replica, before + after + step, replica_spawned);
            replica.step();
            assert_same(step, &want, &record(replica), what);
        }
    }
    (max_delta, snapshot.len())
}
