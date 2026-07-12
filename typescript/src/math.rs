//! Linear algebra primitives.

#[cfg(feature = "dim3")]
use js_sys::Float32Array;
#[cfg(feature = "dim3")]
use na::{Quaternion, Unit};
#[cfg(feature = "dim3")]
use rapier::math::Real;
use rapier::math::{Point, Rotation, Vector};
#[cfg(feature = "dim3")]
use rapier::parry::utils::SdpMatrix3;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
#[repr(transparent)]
#[derive(Copy, Clone)]
/// A rotation quaternion.
pub struct RawRotation(pub(crate) Rotation<f32>);

impl From<Rotation<f32>> for RawRotation {
    fn from(v: Rotation<f32>) -> Self {
        RawRotation(v)
    }
}

#[wasm_bindgen]
#[cfg(feature = "dim2")]
/// A unit complex number describing the orientation of a Rapier entity.
impl RawRotation {
    /// The identity rotation.
    pub fn identity() -> Self {
        Self(Rotation::identity())
    }

    /// The rotation with thegiven angle.
    pub fn fromAngle(angle: f32) -> Self {
        Self(Rotation::new(angle))
    }

    /// The imaginary part of this complex number.
    #[wasm_bindgen(getter)]
    pub fn im(&self) -> f32 {
        self.0.im
    }

    /// The real part of this complex number.
    #[wasm_bindgen(getter)]
    pub fn re(&self) -> f32 {
        self.0.re
    }

    /// The rotation angle in radians.
    #[wasm_bindgen(getter)]
    pub fn angle(&self) -> f32 {
        self.0.angle()
    }
}

#[wasm_bindgen]
#[cfg(feature = "dim3")]
/// A unit quaternion describing the orientation of a Rapier entity.
impl RawRotation {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        RawRotation(Unit::new_unchecked(Quaternion::new(w, x, y, z)))
    }

    /// The identity quaternion.
    pub fn identity() -> Self {
        Self(Rotation::identity())
    }

    /// The `x` component of this quaternion.
    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f32 {
        self.0.i
    }

    /// The `y` component of this quaternion.
    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f32 {
        self.0.j
    }

    /// The `z` component of this quaternion.
    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f32 {
        self.0.k
    }

    /// The `w` component of this quaternion.
    #[wasm_bindgen(getter)]
    pub fn w(&self) -> f32 {
        self.0.w
    }
}

#[wasm_bindgen]
#[repr(transparent)]
#[derive(Copy, Clone)]
/// A vector.
pub struct RawVector(pub(crate) Vector<f32>);

impl From<Vector<f32>> for RawVector {
    fn from(v: Vector<f32>) -> Self {
        RawVector(v)
    }
}

impl From<Point<f32>> for RawVector {
    fn from(pt: Point<f32>) -> Self {
        pt.coords.into()
    }
}

#[wasm_bindgen]
impl RawVector {
    /// Creates a new vector filled with zeros.
    pub fn zero() -> Self {
        Self(Vector::zeros())
    }

    /// Creates a new 2D vector from its two components.
    ///
    /// # Parameters
    /// - `x`: the `x` component of this 2D vector.
    /// - `y`: the `y` component of this 2D vector.
    #[cfg(feature = "dim2")]
    #[wasm_bindgen(constructor)]
    pub fn new(x: f32, y: f32) -> Self {
        Self(Vector::new(x, y))
    }

    /// Creates a new 3D vector from its two components.
    ///
    /// # Parameters
    /// - `x`: the `x` component of this 3D vector.
    /// - `y`: the `y` component of this 3D vector.
    /// - `z`: the `z` component of this 3D vector.
    #[cfg(feature = "dim3")]
    #[wasm_bindgen(constructor)]
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self(Vector::new(x, y, z))
    }

    /// The `x` component of this vector.
    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f32 {
        self.0.x
    }

    /// Sets the `x` component of this vector.
    #[wasm_bindgen(setter)]
    pub fn set_x(&mut self, x: f32) {
        self.0.x = x
    }

    /// The `y` component of this vector.
    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f32 {
        self.0.y
    }

    /// Sets the `y` component of this vector.
    #[wasm_bindgen(setter)]
    pub fn set_y(&mut self, y: f32) {
        self.0.y = y
    }

    /// The `z` component of this vector.
    #[cfg(feature = "dim3")]
    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f32 {
        self.0.z
    }

    /// Sets the `z` component of this vector.
    #[cfg(feature = "dim3")]
    #[wasm_bindgen(setter)]
    pub fn set_z(&mut self, z: f32) {
        self.0.z = z
    }

    /// Create a new 2D vector from this vector with its components rearranged as `{x, y}`.
    #[cfg(feature = "dim2")]
    pub fn xy(&self) -> Self {
        Self(self.0.xy())
    }

    /// Create a new 2D vector from this vector with its components rearranged as `{y, x}`.
    #[cfg(feature = "dim2")]
    pub fn yx(&self) -> Self {
        Self(self.0.yx())
    }

    /// Create a new 2D vector from this vector with its components rearranged as `{z, y}`.
    #[cfg(feature = "dim2")]
    #[cfg(feature = "dim3")]
    pub fn zy(&self) -> Self {
        Self(self.0.zy())
    }

    /// Create a new 3D vector from this vector with its components rearranged as `{x, y, z}`.
    ///
    /// This will effectively return a copy of `this`. This method exist for completeness with the
    /// other swizzling functions.
    #[cfg(feature = "dim3")]
    pub fn xyz(&self) -> Self {
        Self(self.0.xyz())
    }

    /// Create a new 3D vector from this vector with its components rearranged as `{y, x, z}`.
    #[cfg(feature = "dim3")]
    pub fn yxz(&self) -> Self {
        Self(self.0.yxz())
    }

    /// Create a new 3D vector from this vector with its components rearranged as `{z, x, y}`.
    #[cfg(feature = "dim3")]
    pub fn zxy(&self) -> Self {
        Self(self.0.zxy())
    }

    /// Create a new 3D vector from this vector with its components rearranged as `{x, z, y}`.
    #[cfg(feature = "dim3")]
    pub fn xzy(&self) -> Self {
        Self(self.0.xzy())
    }

    /// Create a new 3D vector from this vector with its components rearranged as `{y, z, x}`.
    #[cfg(feature = "dim3")]
    pub fn yzx(&self) -> Self {
        Self(self.0.yzx())
    }

    /// Create a new 3D vector from this vector with its components rearranged as `{z, y, x}`.
    #[cfg(feature = "dim3")]
    pub fn zyx(&self) -> Self {
        Self(self.0.zyx())
    }
}

#[wasm_bindgen]
#[repr(transparent)]
#[derive(Copy, Clone)]
#[cfg(feature = "dim3")]
pub struct RawSdpMatrix3(pub(crate) SdpMatrix3<Real>);

#[cfg(feature = "dim3")]
impl From<SdpMatrix3<Real>> for RawSdpMatrix3 {
    fn from(v: SdpMatrix3<Real>) -> Self {
        RawSdpMatrix3(v)
    }
}

#[wasm_bindgen]
#[cfg(feature = "dim3")]
impl RawSdpMatrix3 {
    /// Row major list of the upper-triangular part of the symmetric matrix.
    pub fn elements(&self) -> Float32Array {
        let m = self.0;
        let output = Float32Array::new_with_length(6);
        output.copy_from(&[m.m11, m.m12, m.m13, m.m22, m.m23, m.m33]);
        output
    }
}
