//! Conversions between the workspace math types (glamx 0.2) and the ones
//! kiss3d re-exports (glamx 0.3). The two glam versions are distinct types,
//! so values crossing the kiss3d boundary must be rebuilt component-wise.

pub fn vec2(v: glamx::Vec2) -> kiss3d::glamx::Vec2 {
    kiss3d::glamx::Vec2::new(v.x, v.y)
}

#[cfg(feature = "dim2")]
pub fn from_vec2(v: kiss3d::glamx::Vec2) -> glamx::Vec2 {
    glamx::Vec2::new(v.x, v.y)
}

#[cfg(feature = "dim3")]
pub fn vec3(v: glamx::Vec3) -> kiss3d::glamx::Vec3 {
    kiss3d::glamx::Vec3::new(v.x, v.y, v.z)
}

#[cfg(feature = "dim3")]
pub fn from_vec3(v: kiss3d::glamx::Vec3) -> glamx::Vec3 {
    glamx::Vec3::new(v.x, v.y, v.z)
}

#[cfg(feature = "dim2")]
pub fn pose(p: rapier::math::Pose) -> kiss3d::glamx::Pose2 {
    kiss3d::glamx::Pose2 {
        rotation: kiss3d::glamx::Rot2::from_cos_sin_unchecked(
            p.rotation.re as f32,
            p.rotation.im as f32,
        ),
        translation: kiss3d::glamx::Vec2::new(p.translation.x as f32, p.translation.y as f32),
    }
}

#[cfg(feature = "dim3")]
pub fn pose(p: rapier::math::Pose) -> kiss3d::glamx::Pose3 {
    kiss3d::glamx::Pose3 {
        rotation: kiss3d::glamx::Quat::from_xyzw(
            p.rotation.x as f32,
            p.rotation.y as f32,
            p.rotation.z as f32,
            p.rotation.w as f32,
        ),
        translation: kiss3d::glamx::Vec3::new(
            p.translation.x as f32,
            p.translation.y as f32,
            p.translation.z as f32,
        ),
        ..kiss3d::glamx::Pose3::IDENTITY
    }
}
