//! Setters and topology helpers of the soft-body builder: elements, materials, tearing, colliders and per-body settings.

use crate::alloc_prelude::*;
use crate::dynamics::SpringCoefficients;
use crate::geometry::ColliderBuilder;
use crate::math::{DIM, Real, Vector};
use parry::utils::hashmap::HashMap;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::super::{SoftBodyCellModel, SoftBodyMaterial};
use super::{SoftBodyBuilder, SoftBodyParticleSettings};

impl SoftBodyBuilder {
    /*
     * Setters.
     */

    /// The particle positions set so far.
    pub fn particle_positions(&self) -> &[Vector] {
        &self.positions
    }

    /// Sets the world-space positions of the particles.
    pub fn positions(mut self, positions: Vec<Vector>) -> Self {
        self.positions = positions;
        self
    }

    /// Sets the mass of every particle.
    pub fn particle_mass(mut self, mass: Real) -> Self {
        self.particle_mass = mass;
        self.masses.clear();
        self
    }

    /// Sets the total mass of the soft body (split uniformly across the particles).
    pub fn mass(mut self, mass: Real) -> Self {
        self.particle_mass = mass / self.positions.len().max(1) as Real;
        self.masses.clear();
        self
    }

    /// Sets per-particle masses.
    pub fn masses(mut self, masses: Vec<Real>) -> Self {
        self.masses = masses;
        self
    }

    /// Pins the given particles (infinite mass, moved kinematically: see
    /// [`crate::dynamics::SoftBody::set_particle_kinematic_target`]).
    pub fn pinned_particles(mut self, pinned: impl IntoIterator<Item = u32>) -> Self {
        self.pinned.extend(pinned);
        self
    }

    /// Sets the structural edges.
    pub fn edges(mut self, edges: Vec<[u32; 2]>) -> Self {
        self.edges = edges;
        self
    }

    /// Adds structural edges (seams between two appended pieces, extra springs...); their
    /// rest length is the current distance of their particles.
    pub fn add_edges(mut self, edges: impl IntoIterator<Item = [u32; 2]>) -> Self {
        let inserted: Vec<[u32; 2]> = edges.into_iter().collect();
        // The per-edge overrides index the concatenation of `edges` and `bend_edges`: the
        // bending indices shift.
        let shift = inserted.len() as u32;
        let first_bend = self.edges.len() as u32;
        for i in &mut self.tension_only_edges {
            if *i >= first_bend {
                *i += shift;
            }
        }
        for (i, _) in &mut self.edge_softness {
            if *i >= first_bend {
                *i += shift;
            }
        }
        for (i, _) in &mut self.edge_tear_resistance {
            if *i >= first_bend {
                *i += shift;
            }
        }
        self.edges.extend(inserted);
        self
    }

    /// Appends the particles and elements of `other` (its indices shifted past this builder's
    /// particles): the two pieces become one soft body, to be sewn together with
    /// [`Self::add_edges`]. This builder's material, model and settings are kept.
    pub fn append(mut self, other: SoftBodyBuilder) -> Self {
        let offset = self.positions.len() as u32;
        // Per-particle masses become explicit as soon as one side has some.
        if !self.masses.is_empty() || !other.masses.is_empty() {
            if self.masses.is_empty() {
                self.masses = vec![self.particle_mass; self.positions.len()];
            }
            if other.masses.is_empty() {
                self.masses.extend(core::iter::repeat_n(
                    other.particle_mass,
                    other.positions.len(),
                ));
            } else {
                self.masses.extend_from_slice(&other.masses);
            }
        }
        // The per-edge overrides of both sides, re-indexed into the new concatenation.
        let (self_edges, self_bends) = (self.edges.len() as u32, self.bend_edges.len() as u32);
        let (other_edges, other_bends) = (other.edges.len() as u32, other.bend_edges.len() as u32);
        let remap_self = |i: u32| if i < self_edges { i } else { i + other_edges };
        let remap_other = |i: u32| {
            if i < other_edges {
                i + self_edges
            } else {
                i - other_edges + self_edges + other_edges + self_bends
            }
        };
        for i in &mut self.tension_only_edges {
            *i = remap_self(*i);
        }
        for (i, _) in &mut self.edge_softness {
            *i = remap_self(*i);
        }
        for (i, _) in &mut self.edge_tear_resistance {
            *i = remap_self(*i);
        }
        self.tension_only_edges
            .extend(other.tension_only_edges.iter().map(|&i| remap_other(i)));
        self.edge_softness.extend(
            other
                .edge_softness
                .iter()
                .map(|&(i, c)| (remap_other(i), c)),
        );
        self.edge_tear_resistance.extend(
            other
                .edge_tear_resistance
                .iter()
                .map(|&(i, r)| (remap_other(i), r)),
        );
        let _ = other_bends;
        let shift = |v: &[u32]| -> Vec<u32> { v.iter().map(|&i| i + offset).collect() };
        self.positions.extend_from_slice(&other.positions);
        self.pinned.extend(shift(&other.pinned));
        self.edges
            .extend(other.edges.iter().map(|e| [e[0] + offset, e[1] + offset]));
        self.bend_edges.extend(
            other
                .bend_edges
                .iter()
                .map(|e| [e[0] + offset, e[1] + offset]),
        );
        #[cfg(feature = "dim3")]
        self.dihedrals.extend(
            other
                .dihedrals
                .iter()
                .map(|d| [d[0] + offset, d[1] + offset, d[2] + offset, d[3] + offset]),
        );
        self.cells
            .extend(other.cells.iter().map(|c| c.map(|i| i + offset)));
        self.surface
            .extend(other.surface.iter().map(|s| s.map(|i| i + offset)));
        self
    }

    /// Sets the bending edges.
    pub fn bend_edges(mut self, edges: Vec<[u32; 2]>) -> Self {
        self.bend_edges = edges;
        self
    }

    /// Makes every edge (structural and bending) only resist stretching, never compression.
    pub fn tension_only(mut self) -> Self {
        self.tension_only_edges = (0..(self.edges.len() + self.bend_edges.len()) as u32).collect();
        self
    }

    /// Sets the dihedral bending constraints (shared edge then the two opposite vertices).
    #[cfg(feature = "dim3")]
    pub fn dihedrals(mut self, dihedrals: Vec<[u32; 4]>) -> Self {
        self.dihedrals = dihedrals;
        self
    }

    /// Sets the simplex cells (triangles in 2D, tetrahedra in 3D). Structural edges along the
    /// cell edges are added at build time if none were given.
    pub fn cells(mut self, cells: Vec<[u32; DIM + 1]>) -> Self {
        self.cells = cells;
        self
    }

    /// Moves everything the builder holds in world space: the particles and the skin.
    ///
    /// Useful to drop copies of a meshed body at several places without meshing it again; moving
    /// the particles alone would leave the skin behind.
    pub fn translated(mut self, translation: Vector) -> Self {
        for position in &mut self.positions {
            *position += translation;
        }
        if let Some((vertices, _)) = &mut self.skin {
            for vertex in vertices {
                *vertex += translation;
            }
        }
        self
    }

    /// Sets the mesh the cells hold: the body's default collision mesh is then that mesh, bound to
    /// the cells ([`crate::dynamics::SoftMeshMapping::Skinned`]) instead of the cells' boundary.
    /// The vertices are in world space in the cells' build pose, each bound to the closest cell.
    pub fn skin(mut self, vertices: Vec<Vector>, indices: Vec<[u32; DIM]>) -> Self {
        self.skin = Some((vertices, indices));
        self
    }

    /// Whether the body's skin is its collision mesh rather than its cells' boundary (off by
    /// default, and no effect without a skin); on, a contact is resolved through the particles of
    /// the cell holding the touched part of the skin, which must be at least as fine as the cells.
    pub fn skin_collision(mut self, enabled: bool) -> Self {
        self.skin_collision = enabled;
        self
    }

    /// Sets the segments the body collides through: a rope or a wire, whose collision mesh is a
    /// polyline rather than a surface (3D only; in 2D [`Self::surface`] already takes segments).
    /// A wire encloses nothing, so its contacts are two-sided and it never expels anything.
    #[cfg(feature = "dim3")]
    pub fn wire(mut self, segments: Vec<[u32; 2]>) -> Self {
        self.wire = segments;
        self
    }

    /// Sets the computational mesh's boundary elements (segments in 2D, triangles in 3D),
    /// oriented outward: what the volume constraint integrates over, and the geometry of the default
    /// collision mesh.
    pub fn surface(mut self, surface: Vec<[u32; DIM]>) -> Self {
        self.surface = surface;
        self
    }

    /// Sets the material.
    pub fn material(mut self, material: SoftBodyMaterial) -> Self {
        self.material = material;
        self
    }

    /// Sets the same softness for every constraint family.
    pub fn softness(mut self, softness: SpringCoefficients<Real>) -> Self {
        self.material = SoftBodyMaterial::uniform(softness);
        self
    }

    /// Sets the material's tear strain (see [`SoftBodyMaterial::tear_strain`]).
    pub fn tear_strain(mut self, strain: Real) -> Self {
        self.material.tear_strain = Some(strain);
        self
    }

    /// Sets the material's tear force (see [`SoftBodyMaterial::tear_force`]).
    pub fn tear_force(mut self, force: Real) -> Self {
        self.material.tear_force = Some(force);
        self
    }

    /// Sets the smallest piece, in measure elements, a tear may split off (see
    /// [`SoftBodyMaterial::min_piece`]).
    pub fn min_piece(mut self, elements: u32) -> Self {
        self.material.min_piece = Some(elements);
        self
    }

    /// Sets the material's tear smoothing time constant (see
    /// [`SoftBodyMaterial::tear_smoothing`]).
    pub fn tear_smoothing(mut self, seconds: Real) -> Self {
        self.material.tear_smoothing = seconds;
        self
    }

    /// Sets the material's interior strength (see [`SoftBodyMaterial::interior_strength`]).
    pub fn interior_strength(mut self, strength: Real) -> Self {
        self.material.interior_strength = strength;
        self
    }

    /// Sets the tear-threshold multiplier of some edges, as `(edge index, resistance)` pairs
    /// indexing the concatenation of the structural then the bending edges (see
    /// [`crate::dynamics::SoftBodyEdge::tear_resistance`]).
    pub fn edge_tear_resistance(mut self, edges: impl IntoIterator<Item = (u32, Real)>) -> Self {
        self.edge_tear_resistance.extend(edges);
        self
    }

    /// Sets the constitutive model of the cells.
    pub fn cell_model(mut self, model: SoftBodyCellModel) -> Self {
        self.cell_model = model;
        self
    }

    /// Selects the solver simulating the body's elasticity (see [`super::SoftBodySolver`]).
    #[cfg(feature = "fem")]
    pub fn solver(mut self, solver: super::SoftBodySolver) -> Self {
        self.solver = solver;
        self
    }

    /// Enables area/volume preservation: one constraint per piece of material enclosed by a
    /// closed surface (see [`crate::dynamics::SoftBody::volume_pieces`]).
    pub fn volume_preservation(mut self, enabled: bool) -> Self {
        self.volume_preservation = enabled;
        self
    }

    /// Sets the target volume multiplier (`> 1` inflates the body).
    pub fn volume_factor(mut self, factor: Real) -> Self {
        self.volume_factor = factor;
        self
    }

    /// Enables shape matching.
    pub fn shape_matching(mut self, enabled: bool) -> Self {
        self.shape_matching = enabled;
        self
    }

    /// Enables self contacts between the body's surface vertices/edges and its own surface.
    pub fn self_contacts(mut self, enabled: bool) -> Self {
        self.self_contacts = enabled;
        self
    }

    /// Whether the shape of the body's default collision surface is built with parry's
    /// `ORIENTED` flag, like a rigid polyline or mesh. Left unset, it is whenever the surface is
    /// closed, which is what a solid body wants: an oriented closed surface encloses matter, so
    /// nothing is held inside it. Set it to `false` for a shell, whose inner side holds the bodies
    /// inside it. After insertion the collider's shape is the authority: change its flags there,
    /// like for any collider. Independent of the material inside (cells, pressure).
    pub fn oriented(mut self, oriented: bool) -> Self {
        self.oriented = Some(oriented);
        self
    }

    /// Sets the thickness of the particles (the contact skin of the body's surface collider).
    pub fn particle_radius(mut self, radius: Real) -> Self {
        self.particle_radius = radius;
        self
    }

    /// Configures the collider of the body's default collision mesh (its cells' boundary, or its
    /// skin under [`Self::skin_collision`]): friction, restitution, groups, events. The shape is
    /// the mesh's own; the density is ignored (mass from [`Self::mass`] or [`Self::masses`]).
    pub fn surface_collider(mut self, collider: ColliderBuilder) -> Self {
        self.collider_template = Some(collider);
        self
    }

    /// Gives the body no default collision mesh collider: it collides through the meshes added
    /// after insertion, if any.
    pub fn no_surface_collider(mut self) -> Self {
        self.collider_template = None;
        self
    }

    /// Sets the dynamics settings of the particles (damping, gravity scale, sleeping...).
    pub fn particle_settings(mut self, settings: SoftBodyParticleSettings) -> Self {
        self.particle_settings = settings;
        self
    }

    /// Sets the linear damping of the particles.
    pub fn linear_damping(mut self, damping: Real) -> Self {
        self.particle_settings.linear_damping = damping;
        self
    }

    /// Sets the gravity scale of the particles.
    pub fn gravity_scale(mut self, scale: Real) -> Self {
        self.particle_settings.gravity_scale = scale;
        self
    }

    /// Requests extra solver substeps for the particles (and everything they touch).
    pub fn additional_solver_iterations(mut self, iterations: usize) -> Self {
        self.particle_settings.additional_solver_iterations = iterations;
        self
    }

    /// Requests extra internal PGS iterations per substep for the particles (and everything
    /// they touch); see [`SoftBodyParticleSettings::additional_pgs_iterations`].
    ///
    /// [`SoftBodyParticleSettings::additional_pgs_iterations`]: super::SoftBodyParticleSettings::additional_pgs_iterations
    pub fn additional_pgs_iterations(mut self, iterations: usize) -> Self {
        self.particle_settings.additional_pgs_iterations = iterations;
        self
    }

    /// Whether the particles may fall asleep.
    pub fn can_sleep(mut self, can_sleep: bool) -> Self {
        self.particle_settings.can_sleep = can_sleep;
        self
    }

    /// Sets the user data of the soft body.
    pub fn user_data(mut self, data: u128) -> Self {
        self.user_data = data;
        self
    }

    /*
     * Topology helpers.
     */

    /// The unique edges of the surface elements.
    pub fn surface_edges(&self) -> Vec<[u32; 2]> {
        let mut seen: HashMap<[u32; 2], ()> = HashMap::default();
        let mut edges = Vec::new();
        for element in &self.surface {
            for k in 0..DIM {
                let a = element[k];
                let b = element[(k + 1) % DIM];
                let key = [a.min(b), a.max(b)];
                if seen.insert(key, ()).is_none() {
                    edges.push(key);
                }
            }
        }
        edges
    }

    /// The unique edges of the cells.
    pub fn cell_edges(&self) -> Vec<[u32; 2]> {
        let mut seen: HashMap<[u32; 2], ()> = HashMap::default();
        let mut edges = Vec::new();
        for cell in &self.cells {
            for i in 0..DIM + 1 {
                for j in i + 1..DIM + 1 {
                    let key = [cell[i].min(cell[j]), cell[i].max(cell[j])];
                    if seen.insert(key, ()).is_none() {
                        edges.push(key);
                    }
                }
            }
        }
        edges
    }

    /// The dihedral constraints across every interior edge of the triangle surface (3D only).
    #[cfg(feature = "dim3")]
    pub fn surface_dihedrals(&self) -> Vec<[u32; 4]> {
        // edge -> (first opposite vertex, second opposite vertex)
        let mut opposite: HashMap<[u32; 2], (u32, Option<u32>)> = HashMap::default();
        let mut order = Vec::new();
        for tri in &self.surface {
            for k in 0..3 {
                let a = tri[k];
                let b = tri[(k + 1) % 3];
                let c = tri[(k + 2) % 3];
                let key = [a.min(b), a.max(b)];
                match opposite.get_mut(&key) {
                    None => {
                        opposite.insert(key, (c, None));
                        order.push(key);
                    }
                    Some((_, second)) => {
                        if second.is_none() {
                            *second = Some(c);
                        }
                    }
                }
            }
        }
        order
            .into_iter()
            .filter_map(|key| {
                let (c, d) = opposite[&key];
                d.map(|d| [key[0], key[1], c, d])
            })
            .collect()
    }
}
