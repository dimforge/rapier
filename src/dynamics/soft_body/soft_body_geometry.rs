//! Geometric helpers of a soft body: the end-of-step orientation update and the boundary and cell volumes with their gradients.
use super::SoftBody;
use crate::math::{DIM, Matrix, Real, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

impl SoftBody {
    /// Updates the state derived from the particle positions at the end of a step: the volume
    /// pieces' inside-out flags (their pressure targets) and the collision mesh's orientation.
    pub(crate) fn update_orientation(&mut self) {
            // With hysteresis: a crumpled body hovering around a zero volume must not flip its
            // pressure target every step.
            let volume =
                Self::boundary_volume(&self.boundary, |i| self.particles[i as usize].position);
            let ratio = volume / self.rest_volume;
            if ratio < -0.25 {
            } else if ratio > 0.25 {
        }
        self.for_each_mesh_mut(|body, mesh| mesh.update_orientation(body));
    }

    /// The signed area (2D) or volume (3D) enclosed by the elements of `boundary`.
    pub(crate) fn boundary_volume(
        boundary: &[[u32; DIM]],
        position: impl Fn(u32) -> Vector,
    ) -> Real {
        let mut vol = 0.0;
        for element in boundary {
            #[cfg(feature = "dim2")]
            {
                let a = position(element[0]);
                let b = position(element[1]);
                vol += a.perp_dot(b) * 0.5;
            }
            #[cfg(feature = "dim3")]
            {
                let a = position(element[0]);
                let b = position(element[1]);
                let c = position(element[2]);
                vol += a.dot(b.cross(c)) / 6.0;
            }
        }
        vol
    }

    /// Accumulates the gradient of the enclosed area/volume with respect to every particle.
    pub(crate) fn boundary_volume_gradients(
        boundary: &[[u32; DIM]],
        position: impl Fn(u32) -> Vector,
        gradients: &mut [Vector],
    ) {
        for element in boundary {
            #[cfg(feature = "dim2")]
            {
                let a = position(element[0]);
                let b = position(element[1]);
                gradients[element[0] as usize] += Vector::new(b.y, -b.x) * 0.5;
                gradients[element[1] as usize] += Vector::new(-a.y, a.x) * 0.5;
            }
            #[cfg(feature = "dim3")]
            {
                let a = position(element[0]);
                let b = position(element[1]);
                let c = position(element[2]);
                gradients[element[0] as usize] += b.cross(c) / 6.0;
                gradients[element[1] as usize] += c.cross(a) / 6.0;
                gradients[element[2] as usize] += a.cross(b) / 6.0;
            }
        }
    }

    /// Signed area (2D) or volume (3D) of a simplex cell.
    pub(crate) fn cell_volume(x: [Vector; DIM + 1]) -> Real {
        #[cfg(feature = "dim2")]
        {
            (x[1] - x[0]).perp_dot(x[2] - x[0]) * 0.5
        }
        #[cfg(feature = "dim3")]
        {
            (x[1] - x[0]).dot((x[2] - x[0]).cross(x[3] - x[0])) / 6.0
        }
    }

    /// Gradients of [`Self::cell_volume`] with respect to the cell's vertices.
    pub(crate) fn cell_volume_gradients(x: [Vector; DIM + 1]) -> [Vector; DIM + 1] {
        let mut grad = [Vector::ZERO; DIM + 1];
        #[cfg(feature = "dim2")]
        {
            let a = x[1] - x[0];
            let b = x[2] - x[0];
            grad[1] = Vector::new(b.y, -b.x) * 0.5;
            grad[2] = Vector::new(-a.y, a.x) * 0.5;
            grad[0] = -(grad[1] + grad[2]);
        }
        #[cfg(feature = "dim3")]
        {
            let a = x[1] - x[0];
            let b = x[2] - x[0];
            let c = x[3] - x[0];
            grad[1] = b.cross(c) / 6.0;
            grad[2] = c.cross(a) / 6.0;
            grad[3] = a.cross(b) / 6.0;
            grad[0] = -(grad[1] + grad[2] + grad[3]);
        }
        grad
    }

    /// The edge matrix `[x1 - x0, x2 - x0, (x3 - x0)]` of a simplex cell.
    pub(crate) fn cell_edge_matrix(x: [Vector; DIM + 1]) -> Matrix {
        #[cfg(feature = "dim2")]
        {
            Matrix::from_cols(x[1] - x[0], x[2] - x[0])
        }
        #[cfg(feature = "dim3")]
        {
            Matrix::from_cols(x[1] - x[0], x[2] - x[0], x[3] - x[0])
        }
    }
}
