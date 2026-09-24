use crate::math::Vect;
use rapier::parry::shape::{Polyline, PolylineFlags};

/// Read-only access to the properties of a polyline.
#[derive(Copy, Clone)]
pub struct PolylineView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Polyline,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The number of segments forming this polyline.
            pub fn num_segments(&self) -> usize {
                self.raw.num_segments()
            }

            /// An iterator through all the segments of this mesh.
            pub fn segments(&self) -> impl ExactSizeIterator<Item = (Vect, Vect)> + '_ {
                self.raw.segments().map(|seg| (seg.a.into(), seg.b.into()))
            }

            /// Get the `i`-th segment of this mesh.
            pub fn segment(&self, i: u32) -> (Vect, Vect) {
                let seg = self.raw.segment(i);
                (seg.a.into(), seg.b.into())
            }

            /// The vertex buffer, containing all the vertices of this polyline.
            pub fn vertices(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
                self.raw.vertices().iter().map(|v| (*v).into())
            }

            /// The index buffer, describing all the segments of this polyline.
            pub fn indices(&self) -> &[[u32; 2]] {
                self.raw.indices()
            }

            /// The flags controlling the optional data associated to this polyline.
            pub fn flags(&self) -> PolylineFlags {
                self.raw.flags()
            }

            /// The outward pseudo-normal of each vertex, if this polyline is oriented.
            ///
            /// The pseudo-normals are only computed with the [`PolylineFlags::ORIENTED`] flag.
            /// The returned slice is indexed like [`Self::vertices`].
            #[cfg(feature = "dim2")]
            pub fn pseudo_normals(&self) -> Option<&[Vect]> {
                self.raw.pseudo_normals()
            }
        }
    }
);

impl_ref_methods!(PolylineView);

/// Read-write access to the properties of a polyline.
pub struct PolylineViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut Polyline,
}

impl_ref_methods!(PolylineViewMut);

impl PolylineViewMut<'_> {
    /// Reverse the orientation of this polyline by swapping the indices of all
    /// its segments and reverting its index buffer.
    pub fn reverse(&mut self) {
        self.raw.reverse()
    }

    /// Sets the flags of this polyline, computing or discarding its optional associated data.
    pub fn set_flags(&mut self, flags: PolylineFlags) {
        self.raw.set_flags(flags)
    }

    /// Replaces the vertex positions, keeping the index buffer.
    ///
    /// The acceleration structure is refitted and the pseudo-normals (if any) are recomputed.
    /// This is intended for polylines built with [`PolylineFlags::DEFORMABLE`].
    ///
    /// # Panics
    /// Panics if `vertices.len()` differs from the current number of vertices.
    pub fn set_vertices(&mut self, vertices: &[Vect]) {
        self.raw.set_vertices(vertices)
    }

    /// Modifies the vertex positions in place through `f`, then refits the acceleration
    /// structure and recomputes the pseudo-normals (if any).
    ///
    /// This is intended for polylines built with [`PolylineFlags::DEFORMABLE`].
    pub fn update_vertices(&mut self, f: impl FnOnce(&mut [Vect])) {
        self.raw.update_vertices(f)
    }
}
