use super::multibody_link::{MultibodyLink, MultibodyLinkVec};
use super::multibody_workspace::MultibodyWorkspace;
use crate::data::{BundleSet, ComponentSet, ComponentSetMut};
use crate::dynamics::solver::AnyJointVelocityConstraint;
use crate::dynamics::{
    IntegrationParameters, RigidBodyForces, RigidBodyHandle, RigidBodyMassProps, RigidBodyPosition,
    RigidBodyType, RigidBodyVelocity,
};
#[cfg(feature = "dim3")]
use crate::math::Matrix;
use crate::math::{
    AngDim, AngVector, Dim, Isometry, Jacobian, Point, Real, Vector, ANG_DIM, DIM, SPATIAL_DIM,
};
use crate::prelude::MultibodyJoint;
use crate::utils::{IndexMut2, WAngularInertia, WCross, WCrossMatrix};
use na::{
    self, DMatrix, DVector, DVectorSlice, DVectorSliceMut, Dynamic, OMatrix, SMatrix, SVector, LU,
};

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
struct Force {
    linear: Vector<Real>,
    angular: AngVector<Real>,
}

impl Force {
    fn new(linear: Vector<Real>, angular: AngVector<Real>) -> Self {
        Self { linear, angular }
    }

    fn as_vector(&self) -> &SVector<Real, SPATIAL_DIM> {
        unsafe { std::mem::transmute(self) }
    }
}

#[cfg(feature = "dim2")]
fn concat_rb_mass_matrix(
    mass: Vector<Real>,
    inertia: Real,
) -> SMatrix<Real, SPATIAL_DIM, SPATIAL_DIM> {
    let mut result = SMatrix::<Real, SPATIAL_DIM, SPATIAL_DIM>::zeros();
    result[(0, 0)] = mass.x;
    result[(1, 1)] = mass.y;
    result[(2, 2)] = inertia;
    result
}

#[cfg(feature = "dim3")]
fn concat_rb_mass_matrix(
    mass: Vector<Real>,
    inertia: Matrix<Real>,
) -> SMatrix<Real, SPATIAL_DIM, SPATIAL_DIM> {
    let mut result = SMatrix::<Real, SPATIAL_DIM, SPATIAL_DIM>::zeros();
    result[(0, 0)] = mass.x;
    result[(1, 1)] = mass.y;
    result[(2, 2)] = mass.z;
    result
        .fixed_slice_mut::<ANG_DIM, ANG_DIM>(DIM, DIM)
        .copy_from(&inertia);
    result
}

/// An articulated body simulated using the reduced-coordinates approach.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct Multibody {
    // TODO: serialization: skip the workspace fields.
    links: MultibodyLinkVec,
    pub(crate) velocities: DVector<Real>,
    pub(crate) damping: DVector<Real>,
    pub(crate) accelerations: DVector<Real>,

    body_jacobians: Vec<Jacobian<Real>>,
    // TODO: use sparse matrices?
    augmented_mass: DMatrix<Real>,
    inv_augmented_mass: LU<Real, Dynamic, Dynamic>,

    acc_augmented_mass: DMatrix<Real>,
    acc_inv_augmented_mass: LU<Real, Dynamic, Dynamic>,

    ndofs: usize,
    pub(crate) root_is_dynamic: bool,
    pub(crate) solver_id: usize,

    /*
     * Workspaces.
     */
    workspace: MultibodyWorkspace,
    coriolis_v: Vec<OMatrix<Real, Dim, Dynamic>>,
    coriolis_w: Vec<OMatrix<Real, AngDim, Dynamic>>,
    i_coriolis_dt: Jacobian<Real>,
}
impl Default for Multibody {
    fn default() -> Self {
        Multibody::new()
    }
}
impl Multibody {
    /// Creates a new multibody with no link.
    pub fn new() -> Self {
        Multibody {
            links: MultibodyLinkVec(Vec::new()),
            velocities: DVector::zeros(0),
            damping: DVector::zeros(0),
            accelerations: DVector::zeros(0),
            body_jacobians: Vec::new(),
            augmented_mass: DMatrix::zeros(0, 0),
            inv_augmented_mass: LU::new(DMatrix::zeros(0, 0)),
            acc_augmented_mass: DMatrix::zeros(0, 0),
            acc_inv_augmented_mass: LU::new(DMatrix::zeros(0, 0)),
            ndofs: 0,
            solver_id: 0,
            workspace: MultibodyWorkspace::new(),
            coriolis_v: Vec::new(),
            coriolis_w: Vec::new(),
            i_coriolis_dt: Jacobian::zeros(0),
            root_is_dynamic: false,
            // solver_workspace: Some(SolverWorkspace::new()),
        }
    }

    pub fn with_root(handle: RigidBodyHandle) -> Self {
        let mut mb = Multibody::new();
        mb.root_is_dynamic = true;
        let joint = MultibodyJoint::free(Isometry::identity());
        mb.add_link(None, joint, handle);
        mb
    }

    pub fn remove_link(self, to_remove: usize, joint_only: bool) -> Vec<Multibody> {
        let mut result = vec![];
        let mut link2mb = vec![usize::MAX; self.links.len()];
        let mut link_id2new_id = vec![usize::MAX; self.links.len()];

        for (i, mut link) in self.links.0.into_iter().enumerate() {
            let is_new_root = (!joint_only && (i == 0 || link.parent_internal_id == to_remove))
                || (joint_only && (i == 0 || i == to_remove));

            if !joint_only && i == to_remove {
                continue;
            } else if is_new_root {
                link2mb[i] = result.len();
                result.push(Multibody::new());
            } else {
                link2mb[i] = link2mb[link.parent_internal_id]
            }

            let curr_mb = &mut result[link2mb[i]];
            link_id2new_id[i] = curr_mb.links.len();

            if is_new_root {
                let joint = MultibodyJoint::fixed(*link.local_to_world());
                link.joint = joint;
            }

            curr_mb.ndofs += link.joint().ndofs();
            curr_mb.links.push(link);
        }

        // Adjust all the internal ids, and copy the data from the
        // previous multibody to the new one.
        for mb in &mut result {
            mb.grow_buffers(mb.ndofs, mb.links.len());
            mb.workspace.resize(mb.links.len(), mb.ndofs);

            let mut assembly_id = 0;
            for (i, link) in mb.links.iter_mut().enumerate() {
                let link_ndofs = link.joint().ndofs();
                mb.velocities
                    .rows_mut(assembly_id, link_ndofs)
                    .copy_from(&self.velocities.rows(link.assembly_id, link_ndofs));
                mb.damping
                    .rows_mut(assembly_id, link_ndofs)
                    .copy_from(&self.damping.rows(link.assembly_id, link_ndofs));
                mb.accelerations
                    .rows_mut(assembly_id, link_ndofs)
                    .copy_from(&self.accelerations.rows(link.assembly_id, link_ndofs));

                link.internal_id = i;
                link.assembly_id = assembly_id;
                link.parent_internal_id = link_id2new_id[link.parent_internal_id];
                assembly_id += link_ndofs;
            }
        }

        result
    }

    pub fn append(&mut self, mut rhs: Multibody, parent: usize, joint: MultibodyJoint) {
        let rhs_root_ndofs = rhs.links[0].joint.ndofs();
        let rhs_copy_shift = self.ndofs + rhs_root_ndofs;
        let rhs_copy_ndofs = rhs.ndofs - rhs_root_ndofs;

        // Adjust the ids of all the rhs links except the first one.
        let base_assembly_id = self.velocities.len() - rhs_root_ndofs + joint.ndofs();
        let base_internal_id = self.links.len() + 1;
        let base_parent_id = self.links.len();

        for link in &mut rhs.links.0[1..] {
            link.assembly_id += base_assembly_id;
            link.internal_id += base_internal_id;
            link.parent_internal_id += base_parent_id;
        }

        // Adjust the first link.
        {
            rhs.links[0].joint = joint;
            rhs.links[0].assembly_id = self.velocities.len();
            rhs.links[0].internal_id = self.links.len();
            rhs.links[0].parent_internal_id = parent;
        }

        // Grow buffers and append data from rhs.
        self.grow_buffers(rhs_copy_ndofs + rhs.links[0].joint.ndofs(), rhs.links.len());

        if rhs_copy_ndofs > 0 {
            self.velocities
                .rows_mut(rhs_copy_shift, rhs_copy_ndofs)
                .copy_from(&rhs.velocities.rows(rhs_root_ndofs, rhs_copy_ndofs));
            self.damping
                .rows_mut(rhs_copy_shift, rhs_copy_ndofs)
                .copy_from(&rhs.damping.rows(rhs_root_ndofs, rhs_copy_ndofs));
            self.accelerations
                .rows_mut(rhs_copy_shift, rhs_copy_ndofs)
                .copy_from(&rhs.accelerations.rows(rhs_root_ndofs, rhs_copy_ndofs));
        }

        rhs.links[0]
            .joint
            .default_damping(&mut self.damping.rows_mut(base_assembly_id, rhs_root_ndofs));

        self.links.append(&mut rhs.links);
        self.ndofs = self.velocities.len();
        self.workspace.resize(self.links.len(), self.ndofs);
    }

    pub fn inv_augmented_mass(&self) -> &LU<Real, Dynamic, Dynamic> {
        &self.inv_augmented_mass
    }

    /// The first link of this multibody.
    #[inline]
    pub fn root(&self) -> &MultibodyLink {
        &self.links[0]
    }

    /// Mutable reference to the first link of this multibody.
    #[inline]
    pub fn root_mut(&mut self) -> &mut MultibodyLink {
        &mut self.links[0]
    }

    /// Reference `i`-th multibody link of this multibody.
    ///
    /// Return `None` if there is less than `i + 1` multibody links.
    #[inline]
    pub fn link(&self, id: usize) -> Option<&MultibodyLink> {
        self.links.get(id)
    }

    /// Mutable reference to the multibody link with the given id.
    ///
    /// Return `None` if the given id does not identifies a multibody link part of `self`.
    #[inline]
    pub fn link_mut(&mut self, id: usize) -> Option<&mut MultibodyLink> {
        self.links.get_mut(id)
    }

    /// The number of links on this multibody.
    pub fn num_links(&self) -> usize {
        self.links.len()
    }

    /// Iterator through all the links of this multibody.
    ///
    /// All link are guaranteed to be yielded before its descendant.
    pub fn links(&self) -> impl Iterator<Item = &MultibodyLink> {
        self.links.iter()
    }

    /// Mutable iterator through all the links of this multibody.
    ///
    /// All link are guaranteed to be yielded before its descendant.
    pub fn links_mut(&mut self) -> impl Iterator<Item = &mut MultibodyLink> {
        self.links.iter_mut()
    }

    /// The vector of damping applied to this multibody.
    #[inline]
    pub fn damping(&self) -> &DVector<Real> {
        &self.damping
    }

    /// Mutable vector of damping applied to this multibody.
    #[inline]
    pub fn damping_mut(&mut self) -> &mut DVector<Real> {
        &mut self.damping
    }

    pub fn add_link(
        &mut self,
        parent: Option<usize>, // FIXME: should be a RigidBodyHandle?
        dof: MultibodyJoint,
        body: RigidBodyHandle,
    ) -> &mut MultibodyLink {
        assert!(
            parent.is_none() || !self.links.is_empty(),
            "Multibody::build_body: invalid parent id."
        );

        /*
         * Compute the indices.
         */
        let assembly_id = self.velocities.len();
        let internal_id = self.links.len();

        /*
         * Grow the buffers.
         */
        let ndofs = dof.ndofs();
        self.grow_buffers(ndofs, 1);
        self.ndofs += ndofs;

        /*
         * Setup default damping.
         */
        dof.default_damping(&mut self.damping.rows_mut(assembly_id, ndofs));

        /*
         * Create the multibody.
         */
        let local_to_parent = dof.body_to_parent();
        let local_to_world;

        let parent_internal_id;
        if let Some(parent) = parent {
            parent_internal_id = parent;
            let parent_link = &mut self.links[parent_internal_id];
            local_to_world = parent_link.local_to_world * local_to_parent;
        } else {
            parent_internal_id = 0;
            local_to_world = local_to_parent;
        }

        let rb = MultibodyLink::new(
            body,
            internal_id,
            assembly_id,
            parent_internal_id,
            dof,
            local_to_world,
            local_to_parent,
        );

        self.links.push(rb);
        self.workspace.resize(self.links.len(), self.ndofs);

        &mut self.links[internal_id]
    }

    fn grow_buffers(&mut self, ndofs: usize, num_jacobians: usize) {
        let len = self.velocities.len();
        self.velocities.resize_vertically_mut(len + ndofs, 0.0);
        self.damping.resize_vertically_mut(len + ndofs, 0.0);
        self.accelerations.resize_vertically_mut(len + ndofs, 0.0);
        self.body_jacobians
            .extend((0..num_jacobians).map(|_| Jacobian::zeros(0)));
    }

    pub fn update_acceleration<Bodies>(&mut self, bodies: &Bodies)
    where
        Bodies: ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyForces>
            + ComponentSet<RigidBodyVelocity>,
    {
        if self.ndofs == 0 {
            return; // Nothing to do.
        }

        self.accelerations.fill(0.0);

        for i in 0..self.links.len() {
            let link = &self.links[i];

            let (rb_vels, rb_mprops, rb_forces): (
                &RigidBodyVelocity,
                &RigidBodyMassProps,
                &RigidBodyForces,
            ) = bodies.index_bundle(link.rigid_body.0);

            let mut acc = RigidBodyVelocity::zero();

            if i != 0 {
                let parent_id = link.parent_internal_id;
                let parent_link = &self.links[parent_id];
                let parent_rb_vels: &RigidBodyVelocity = bodies.index(parent_link.rigid_body.0);

                acc += self.workspace.accs[parent_id];
                // The 2.0 originates from the two identical terms of Jdot (the terms become
                // identical once they are multiplied by the generalized velocities).
                acc.linvel += 2.0 * parent_rb_vels.angvel.gcross(link.joint_velocity.linvel);
                #[cfg(feature = "dim3")]
                {
                    acc.angvel += parent_rb_vels.angvel.cross(&link.joint_velocity.angvel);
                }

                acc.linvel += parent_rb_vels
                    .angvel
                    .gcross(parent_rb_vels.angvel.gcross(link.shift02));
                acc.linvel += self.workspace.accs[parent_id].angvel.gcross(link.shift02);
            }

            acc.linvel += rb_vels.angvel.gcross(rb_vels.angvel.gcross(link.shift23));
            acc.linvel += self.workspace.accs[i].angvel.gcross(link.shift23);

            self.workspace.accs[i] = acc;

            // TODO: should gyroscopic forces already be computed by the rigid-body itself
            //       (at the same time that we add the gravity force)?
            let gyroscopic;
            let rb_inertia = rb_mprops.effective_angular_inertia();
            let rb_mass = rb_mprops.effective_mass();

            #[cfg(feature = "dim3")]
            {
                gyroscopic = rb_vels.angvel.cross(&(rb_inertia * rb_vels.angvel));
            }
            #[cfg(feature = "dim2")]
            {
                gyroscopic = 0.0;
            }

            let external_forces = Force::new(
                rb_forces.force - rb_mass.component_mul(&acc.linvel),
                rb_forces.torque - gyroscopic - rb_inertia * acc.angvel,
            );
            self.accelerations.gemv_tr(
                1.0,
                &self.body_jacobians[i],
                external_forces.as_vector(),
                1.0,
            );
        }

        self.accelerations
            .cmpy(-1.0, &self.damping, &self.velocities, 1.0);

        self.acc_inv_augmented_mass
            .solve_mut(&mut self.accelerations);
    }

    /// Computes the constant terms of the dynamics.
    pub fn update_dynamics<Bodies>(&mut self, dt: Real, bodies: &mut Bodies)
    where
        Bodies: ComponentSetMut<RigidBodyVelocity> + ComponentSet<RigidBodyMassProps>,
    {
        /*
         * Compute velocities.
         * NOTE: this is needed for kinematic bodies too.
         */
        let link = &mut self.links[0];
        let joint_velocity = link
            .joint
            .jacobian_mul_coordinates(&self.velocities.as_slice()[link.assembly_id..]);

        link.joint_velocity = joint_velocity;
        bodies.set_internal(link.rigid_body.0, link.joint_velocity);

        for i in 1..self.links.len() {
            let (link, parent_link) = self.links.get_mut_with_parent(i);
            let rb_mprops: &RigidBodyMassProps = bodies.index(link.rigid_body.0);
            let (parent_rb_vels, parent_rb_mprops): (&RigidBodyVelocity, &RigidBodyMassProps) =
                bodies.index_bundle(parent_link.rigid_body.0);

            let joint_velocity = link
                .joint
                .jacobian_mul_coordinates(&self.velocities.as_slice()[link.assembly_id..]);
            link.joint_velocity = joint_velocity.transformed(
                &(parent_link.local_to_world.rotation * link.joint.data.local_frame1.rotation),
            );
            let mut new_rb_vels = *parent_rb_vels + link.joint_velocity;
            let shift = rb_mprops.world_com - parent_rb_mprops.world_com;
            new_rb_vels.linvel += parent_rb_vels.angvel.gcross(shift);
            new_rb_vels.linvel += link.joint_velocity.angvel.gcross(link.shift23);

            bodies.set_internal(link.rigid_body.0, new_rb_vels);
        }

        /*
         * Update augmented mass matrix.
         */
        self.update_inertias(dt, bodies);
    }

    fn update_body_jacobians(&mut self) {
        for i in 0..self.links.len() {
            let link = &self.links[i];

            if self.body_jacobians[i].ncols() != self.ndofs {
                // FIXME: use a resize instead.
                self.body_jacobians[i] = Jacobian::zeros(self.ndofs);
            }

            let parent_to_world;

            if i != 0 {
                let parent_id = link.parent_internal_id;
                let parent_link = &self.links[parent_id];
                parent_to_world = parent_link.local_to_world;

                let (link_j, parent_j) = self.body_jacobians.index_mut_const(i, parent_id);
                link_j.copy_from(&parent_j);

                {
                    let mut link_j_v = link_j.fixed_rows_mut::<DIM>(0);
                    let parent_j_w = parent_j.fixed_rows::<ANG_DIM>(DIM);

                    let shift_tr = (link.shift02).gcross_matrix_tr();
                    link_j_v.gemm(1.0, &shift_tr, &parent_j_w, 1.0);
                }
            } else {
                self.body_jacobians[i].fill(0.0);
                parent_to_world = Isometry::identity();
            }

            let ndofs = link.joint.ndofs();
            let mut tmp = SMatrix::<Real, SPATIAL_DIM, SPATIAL_DIM>::zeros();
            let mut link_joint_j = tmp.columns_mut(0, ndofs);
            let mut link_j_part = self.body_jacobians[i].columns_mut(link.assembly_id, ndofs);
            link.joint.jacobian(
                &(parent_to_world.rotation * link.joint.data.local_frame1.rotation),
                &mut link_joint_j,
            );
            link_j_part += link_joint_j;

            {
                let link_j = &mut self.body_jacobians[i];
                let (mut link_j_v, link_j_w) =
                    link_j.rows_range_pair_mut(0..DIM, DIM..DIM + ANG_DIM);
                let shift_tr = link.shift23.gcross_matrix_tr();
                link_j_v.gemm(1.0, &shift_tr, &link_j_w, 1.0);
            }
        }
    }

    fn update_inertias<Bodies>(&mut self, dt: Real, bodies: &Bodies)
    where
        Bodies: ComponentSet<RigidBodyMassProps> + ComponentSet<RigidBodyVelocity>,
    {
        if self.ndofs == 0 {
            return; // Nothing to do.
        }

        if self.augmented_mass.ncols() != self.ndofs {
            // TODO: do a resize instead of a full reallocation.
            self.augmented_mass = DMatrix::zeros(self.ndofs, self.ndofs);
            self.acc_augmented_mass = DMatrix::zeros(self.ndofs, self.ndofs);
        } else {
            self.augmented_mass.fill(0.0);
            self.acc_augmented_mass.fill(0.0);
        }

        if self.coriolis_v.len() != self.links.len() {
            self.coriolis_v.resize(
                self.links.len(),
                OMatrix::<Real, Dim, Dynamic>::zeros(self.ndofs),
            );
            self.coriolis_w.resize(
                self.links.len(),
                OMatrix::<Real, AngDim, Dynamic>::zeros(self.ndofs),
            );
            self.i_coriolis_dt = Jacobian::zeros(self.ndofs);
        }

        for i in 0..self.links.len() {
            let link = &self.links[i];
            let (rb_vels, rb_mprops): (&RigidBodyVelocity, &RigidBodyMassProps) =
                bodies.index_bundle(link.rigid_body.0);
            let rb_mass = rb_mprops.effective_mass();
            let rb_inertia = rb_mprops.effective_angular_inertia().into_matrix();

            let body_jacobian = &self.body_jacobians[i];

            #[allow(unused_mut)] // mut is needed for 3D but not for 2D.
            let mut augmented_inertia = rb_inertia;

            #[cfg(feature = "dim3")]
            {
                // Derivative of gyroscopic forces.
                let gyroscopic_matrix = rb_vels.angvel.gcross_matrix() * rb_inertia
                    - (rb_inertia * rb_vels.angvel).gcross_matrix();

                augmented_inertia += gyroscopic_matrix * dt;
            }

            // TODO: optimize that (knowing the structure of the augmented inertia matrix).
            // TODO: this could be better optimized in 2D.
            let rb_mass_matrix_wo_gyro = concat_rb_mass_matrix(rb_mass, rb_inertia);
            let rb_mass_matrix = concat_rb_mass_matrix(rb_mass, augmented_inertia);
            self.augmented_mass
                .quadform(1.0, &rb_mass_matrix_wo_gyro, body_jacobian, 1.0);
            self.acc_augmented_mass
                .quadform(1.0, &rb_mass_matrix, body_jacobian, 1.0);

            /*
             *
             * Coriolis matrix.
             *
             */
            let rb_j = &self.body_jacobians[i];
            let rb_j_w = rb_j.fixed_rows::<ANG_DIM>(DIM);

            let ndofs = link.joint.ndofs();

            if i != 0 {
                let parent_id = link.parent_internal_id;
                let parent_link = &self.links[parent_id];
                let parent_rb_vels: &RigidBodyVelocity = bodies.index(parent_link.rigid_body.0);
                let parent_j = &self.body_jacobians[parent_id];
                let parent_j_w = parent_j.fixed_rows::<ANG_DIM>(DIM);
                let parent_w = parent_rb_vels.angvel.gcross_matrix();

                let (coriolis_v, parent_coriolis_v) = self.coriolis_v.index_mut2(i, parent_id);
                let (coriolis_w, parent_coriolis_w) = self.coriolis_w.index_mut2(i, parent_id);

                coriolis_v.copy_from(&parent_coriolis_v);
                coriolis_w.copy_from(&parent_coriolis_w);

                // [c1 - c0].gcross() * (JDot + JDot/u * qdot)"
                let shift_cross_tr = link.shift02.gcross_matrix_tr();
                coriolis_v.gemm(1.0, &shift_cross_tr, &parent_coriolis_w, 1.0);

                // JDot (but the 2.0 originates from the sum of two identical terms in JDot and JDot/u * gdot)
                let dvel_cross = (rb_vels.angvel.gcross(link.shift02)
                    + 2.0 * link.joint_velocity.linvel)
                    .gcross_matrix_tr();
                coriolis_v.gemm(1.0, &dvel_cross, &parent_j_w, 1.0);

                // JDot/u * qdot
                coriolis_v.gemm(
                    1.0,
                    &link.joint_velocity.linvel.gcross_matrix_tr(),
                    &parent_j_w,
                    1.0,
                );
                coriolis_v.gemm(1.0, &(parent_w * shift_cross_tr), &parent_j_w, 1.0);

                #[cfg(feature = "dim3")]
                {
                    let vel_wrt_joint_w = link.joint_velocity.angvel.gcross_matrix();
                    coriolis_w.gemm(-1.0, &vel_wrt_joint_w, &parent_j_w, 1.0);
                }

                // JDot (but the 2.0 originates from the sum of two identical terms in JDot and JDot/u * gdot)
                {
                    let mut coriolis_v_part = coriolis_v.columns_mut(link.assembly_id, ndofs);

                    let mut tmp1 = SMatrix::<Real, SPATIAL_DIM, SPATIAL_DIM>::zeros();
                    let mut rb_joint_j = tmp1.columns_mut(0, ndofs);
                    link.joint.jacobian(
                        &(parent_link.local_to_world.rotation
                            * link.joint.data.local_frame1.rotation),
                        &mut rb_joint_j,
                    );

                    let rb_joint_j_v = rb_joint_j.fixed_rows::<DIM>(0);
                    coriolis_v_part.gemm(2.0, &parent_w, &rb_joint_j_v, 1.0);

                    #[cfg(feature = "dim3")]
                    {
                        let rb_joint_j_w = rb_joint_j.fixed_rows::<ANG_DIM>(DIM);
                        let mut coriolis_w_part = coriolis_w.columns_mut(link.assembly_id, ndofs);
                        coriolis_w_part.gemm(1.0, &parent_w, &rb_joint_j_w, 1.0);
                    }
                }
            } else {
                self.coriolis_v[i].fill(0.0);
                self.coriolis_w[i].fill(0.0);
            }

            let coriolis_v = &mut self.coriolis_v[i];
            let coriolis_w = &mut self.coriolis_w[i];

            {
                // [c3 - c2].gcross() * (JDot + JDot/u * qdot)
                let shift_cross_tr = link.shift23.gcross_matrix_tr();
                coriolis_v.gemm(1.0, &shift_cross_tr, &coriolis_w, 1.0);

                // JDot
                let dvel_cross = rb_vels.angvel.gcross(link.shift23).gcross_matrix_tr();
                coriolis_v.gemm(1.0, &dvel_cross, &rb_j_w, 1.0);

                // JDot/u * qdot
                coriolis_v.gemm(
                    1.0,
                    &(rb_vels.angvel.gcross_matrix() * shift_cross_tr),
                    &rb_j_w,
                    1.0,
                );
            }

            let coriolis_v = &mut self.coriolis_v[i];
            let coriolis_w = &mut self.coriolis_w[i];

            /*
             * Meld with the mass matrix.
             */
            {
                let mut i_coriolis_dt_v = self.i_coriolis_dt.fixed_rows_mut::<DIM>(0);
                i_coriolis_dt_v.copy_from(coriolis_v);
                i_coriolis_dt_v
                    .column_iter_mut()
                    .for_each(|mut c| c.component_mul_assign(&(rb_mass * dt)));
            }

            #[cfg(feature = "dim2")]
            {
                let mut i_coriolis_dt_w = self.i_coriolis_dt.fixed_rows_mut::<ANG_DIM>(DIM);
                // NOTE: this is just an axpy, but on row columns.
                i_coriolis_dt_w.zip_apply(&coriolis_w, |o, x| *o = x * dt * rb_inertia);
            }
            #[cfg(feature = "dim3")]
            {
                let mut i_coriolis_dt_w = self.i_coriolis_dt.fixed_rows_mut::<ANG_DIM>(DIM);
                i_coriolis_dt_w.gemm(dt, &rb_inertia, &coriolis_w, 0.0);
            }

            self.acc_augmented_mass
                .gemm_tr(1.0, &rb_j, &self.i_coriolis_dt, 1.0);
        }

        /*
         * Damping.
         */
        for i in 0..self.ndofs {
            self.acc_augmented_mass[(i, i)] += self.damping[i] * dt;
            self.augmented_mass[(i, i)] += self.damping[i] * dt;
        }

        // FIXME: avoid allocation inside LU at each timestep.
        self.acc_inv_augmented_mass = LU::new(self.acc_augmented_mass.clone());
        self.inv_augmented_mass = LU::new(self.augmented_mass.clone());
        // self.acc_inv_augmented_mass = self.inv_augmented_mass.clone();
        // self.augmented_mass = self.acc_augmented_mass.clone();
        // self.inv_augmented_mass = self.acc_inv_augmented_mass.clone();
    }

    /// The generalized velocity at the multibody_joint of the given link.
    #[inline]
    pub(crate) fn joint_velocity(&self, link: &MultibodyLink) -> DVectorSlice<Real> {
        let ndofs = link.joint().ndofs();
        DVectorSlice::from_slice(
            &self.velocities.as_slice()[link.assembly_id..link.assembly_id + ndofs],
            ndofs,
        )
    }

    #[inline]
    pub fn generalized_acceleration(&self) -> DVectorSlice<Real> {
        self.accelerations.rows(0, self.ndofs)
    }

    #[inline]
    pub fn generalized_velocity(&self) -> DVectorSlice<Real> {
        self.velocities.rows(0, self.ndofs)
    }

    #[inline]
    pub fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<Real> {
        self.velocities.rows_mut(0, self.ndofs)
    }

    #[inline]
    pub fn integrate(&mut self, dt: Real) {
        for rb in self.links.iter_mut() {
            rb.joint
                .integrate(dt, &self.velocities.as_slice()[rb.assembly_id..])
        }
    }

    pub fn apply_displacements(&mut self, disp: &[Real]) {
        for link in self.links.iter_mut() {
            link.joint.apply_displacement(&disp[link.assembly_id..])
        }
    }

    pub fn update_root_type<Bodies>(&mut self, bodies: &mut Bodies)
    where
        Bodies: ComponentSet<RigidBodyType> + ComponentSet<RigidBodyPosition>,
    {
        let rb_type: Option<&RigidBodyType> = bodies.get(self.links[0].rigid_body.0);
        if let Some(rb_type) = rb_type {
            let rb_pos: &RigidBodyPosition = bodies.index(self.links[0].rigid_body.0);

            if rb_type.is_dynamic() != self.root_is_dynamic {
                if rb_type.is_dynamic() {
                    let free_joint = MultibodyJoint::free(rb_pos.position);
                    let prev_root_ndofs = self.links[0].joint().ndofs();
                    self.links[0].joint = free_joint;
                    self.links[0].assembly_id = 0;
                    self.ndofs += SPATIAL_DIM;

                    self.velocities = self.velocities.clone().insert_rows(0, SPATIAL_DIM, 0.0);
                    self.damping = self.damping.clone().insert_rows(0, SPATIAL_DIM, 0.0);
                    self.accelerations =
                        self.accelerations.clone().insert_rows(0, SPATIAL_DIM, 0.0);

                    for link in &mut self.links[1..] {
                        link.assembly_id += SPATIAL_DIM - prev_root_ndofs;
                    }
                } else {
                    assert!(self.velocities.len() >= SPATIAL_DIM);
                    assert!(self.damping.len() >= SPATIAL_DIM);
                    assert!(self.accelerations.len() >= SPATIAL_DIM);

                    let fixed_joint = MultibodyJoint::fixed(rb_pos.position);
                    let prev_root_ndofs = self.links[0].joint().ndofs();
                    self.links[0].joint = fixed_joint;
                    self.links[0].assembly_id = 0;
                    self.ndofs -= prev_root_ndofs;

                    if self.ndofs == 0 {
                        self.velocities = DVector::zeros(0);
                        self.damping = DVector::zeros(0);
                        self.accelerations = DVector::zeros(0);
                    } else {
                        self.velocities =
                            self.velocities.index((prev_root_ndofs.., 0)).into_owned();
                        self.damping = self.damping.index((prev_root_ndofs.., 0)).into_owned();
                        self.accelerations = self
                            .accelerations
                            .index((prev_root_ndofs.., 0))
                            .into_owned();
                    }

                    for link in &mut self.links[1..] {
                        link.assembly_id -= prev_root_ndofs;
                    }
                }

                self.root_is_dynamic = rb_type.is_dynamic();
            }

            // Make sure the positions are properly set to match the rigid-body’s.
            if self.links[0].joint.data.locked_axes.is_empty() {
                self.links[0].joint.set_free_pos(rb_pos.position);
            } else {
                self.links[0].joint.data.local_frame1 = rb_pos.position;
            }
        }
    }

    pub fn forward_kinematics<Bodies>(&mut self, bodies: &mut Bodies, update_mass_props: bool)
    where
        Bodies: ComponentSet<RigidBodyType>
            + ComponentSetMut<RigidBodyMassProps>
            + ComponentSetMut<RigidBodyPosition>,
    {
        // Special case for the root, which has no parent.
        {
            let link = &mut self.links[0];
            link.local_to_parent = link.joint.body_to_parent();
            link.local_to_world = link.local_to_parent;

            bodies.map_mut_internal(link.rigid_body.0, |rb_pos: &mut RigidBodyPosition| {
                rb_pos.next_position = link.local_to_world;
            });

            if update_mass_props {
                bodies.map_mut_internal(link.rigid_body.0, |mprops: &mut RigidBodyMassProps| {
                    mprops.update_world_mass_properties(&link.local_to_world)
                });
            }
        }

        // Handle the children. They all have a parent within this multibody.
        for i in 1..self.links.len() {
            let (link, parent_link) = self.links.get_mut_with_parent(i);

            link.local_to_parent = link.joint.body_to_parent();
            link.local_to_world = parent_link.local_to_world * link.local_to_parent;

            {
                let parent_rb_mprops: &RigidBodyMassProps = bodies.index(parent_link.rigid_body.0);
                let rb_mprops: &RigidBodyMassProps = bodies.index(link.rigid_body.0);
                let c0 = parent_link.local_to_world * parent_rb_mprops.local_mprops.local_com;
                let c2 = link.local_to_world
                    * Point::from(link.joint.data.local_frame2.translation.vector);
                let c3 = link.local_to_world * rb_mprops.local_mprops.local_com;

                link.shift02 = c2 - c0;
                link.shift23 = c3 - c2;
            }

            bodies.map_mut_internal(link.rigid_body.0, |rb_pos: &mut RigidBodyPosition| {
                rb_pos.next_position = link.local_to_world;
            });

            let rb_type: &RigidBodyType = bodies.index(link.rigid_body.0);
            assert_eq!(
                *rb_type,
                RigidBodyType::Dynamic,
                "A rigid-body that is not at the root of a multibody must be dynamic."
            );

            if update_mass_props {
                bodies.map_mut_internal(link.rigid_body.0, |rb_mprops: &mut RigidBodyMassProps| {
                    rb_mprops.update_world_mass_properties(&link.local_to_world)
                });
            }
        }

        /*
         * Compute body jacobians.
         */
        self.update_body_jacobians();
    }

    #[inline]
    pub fn ndofs(&self) -> usize {
        self.ndofs
    }

    pub fn fill_jacobians(
        &self,
        link_id: usize,
        unit_force: Vector<Real>,
        unit_torque: SVector<Real, ANG_DIM>,
        j_id: &mut usize,
        jacobians: &mut DVector<Real>,
    ) -> (Real, Real) {
        if self.ndofs == 0 {
            return (0.0, 0.0);
        }

        let wj_id = *j_id + self.ndofs;
        let force = Force {
            linear: unit_force,
            #[cfg(feature = "dim2")]
            angular: unit_torque[0],
            #[cfg(feature = "dim3")]
            angular: unit_torque,
        };

        let link = &self.links[link_id];
        let mut out_j = jacobians.rows_mut(*j_id, self.ndofs);
        self.body_jacobians[link.internal_id].tr_mul_to(force.as_vector(), &mut out_j);

        // TODO: Optimize with a copy_nonoverlapping?
        for i in 0..self.ndofs {
            jacobians[wj_id + i] = jacobians[*j_id + i];
        }

        {
            let mut out_invm_j = jacobians.rows_mut(wj_id, self.ndofs);
            self.inv_augmented_mass.solve_mut(&mut out_invm_j);
        }

        let j = jacobians.rows(*j_id, self.ndofs);
        let invm_j = jacobians.rows(wj_id, self.ndofs);
        *j_id += self.ndofs * 2;

        (j.dot(&invm_j), j.dot(&self.generalized_velocity()))
    }

    #[inline]
    pub fn has_active_internal_constraints(&self) -> bool {
        self.links()
            .any(|link| link.joint().num_velocity_constraints() != 0)
    }

    #[inline]
    pub fn num_active_internal_constraints_and_jacobian_lines(&self) -> (usize, usize) {
        let num_constraints: usize = self
            .links
            .iter()
            .map(|l| l.joint().num_velocity_constraints())
            .sum();
        (num_constraints, num_constraints)
    }

    #[inline]
    pub fn generate_internal_constraints(
        &self,
        params: &IntegrationParameters,
        j_id: &mut usize,
        jacobians: &mut DVector<Real>,
        out: &mut Vec<AnyJointVelocityConstraint>,
        mut insert_at: Option<usize>,
    ) {
        if !cfg!(feature = "parallel") {
            let num_constraints: usize = self
                .links
                .iter()
                .map(|l| l.joint().num_velocity_constraints())
                .sum();

            let required_jacobian_len = *j_id + num_constraints * self.ndofs * 2;
            if jacobians.nrows() < required_jacobian_len {
                jacobians.resize_vertically_mut(required_jacobian_len, 0.0);
            }
        }

        for link in self.links.iter() {
            link.joint().velocity_constraints(
                params,
                self,
                link,
                0,
                j_id,
                jacobians,
                out,
                &mut insert_at,
            );
        }
    }
}
