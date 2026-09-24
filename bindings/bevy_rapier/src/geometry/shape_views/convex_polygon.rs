use crate::math::Vect;
use rapier::parry::shape::ConvexPolygon;

/// Read-only access to the properties of a convex polygon.
#[derive(Copy, Clone)]
pub struct ConvexPolygonView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a ConvexPolygon,
}

impl ConvexPolygonView<'_> {
    /// The vertices of this convex polygon.
    pub fn points(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
        self.raw.points().iter().copied()
    }

    /// The normals of the edges of this convex polygon.
    pub fn normals(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
        self.raw.normals().iter().copied()
    }
}

/// Read-write access to the properties of a convex polygon.
pub struct ConvexPolygonViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut ConvexPolygon,
}

impl ConvexPolygonViewMut<'_> {
    /// The vertices of this convex polygon.
    pub fn points(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
        self.raw.points().iter().copied()
    }

    /// The normals of the edges of this convex polygon.
    pub fn normals(&self) -> impl ExactSizeIterator<Item = Vect> + '_ {
        self.raw.normals().iter().copied()
    }

    // TODO: add modifications.
}
