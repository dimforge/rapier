use crate::math::{Rot, Vect};
use bevy::prelude::Transform;
use rapier::math::Pose;

/// Builds a Rapier [`Pose`] from a translation/rotation pair using bevy_rapier's [`Vect`] / [`Rot`] aliases.
#[cfg(feature = "dim2")]
pub(crate) fn pose_from(translation: Vect, rotation: Rot) -> Pose {
    Pose::new(translation, rotation)
}

/// Builds a Rapier [`Pose`] from a translation/rotation pair using bevy_rapier's [`Vect`] / [`Rot`] aliases.
#[cfg(feature = "dim3")]
pub(crate) fn pose_from(translation: Vect, rotation: Rot) -> Pose {
    Pose::from_parts(translation, rotation)
}

/// Converts a Rapier pose to a Bevy transform.
#[cfg(feature = "dim2")]
pub fn iso_to_transform(iso: &Pose) -> Transform {
    Transform {
        translation: iso.translation.extend(0.0),
        rotation: bevy::prelude::Quat::from_rotation_z(iso.rotation.angle()),
        ..Default::default()
    }
}

/// Converts a Rapier pose to a Bevy transform.
#[cfg(feature = "dim3")]
pub fn iso_to_transform(iso: &Pose) -> Transform {
    Transform {
        translation: iso.translation,
        rotation: iso.rotation,
        ..Default::default()
    }
}

/// Converts a Bevy transform to a Rapier pose.
#[cfg(feature = "dim2")]
pub(crate) fn transform_to_iso(transform: &Transform) -> Pose {
    use bevy::math::Vec3Swizzles;
    Pose::new(
        transform.translation.xy(),
        transform.rotation.to_scaled_axis().z,
    )
}

/// Converts a Bevy transform to a Rapier pose.
#[cfg(feature = "dim3")]
pub(crate) fn transform_to_iso(transform: &Transform) -> Pose {
    Pose::from_parts(transform.translation, transform.rotation)
}

#[cfg(test)]
#[cfg(feature = "dim3")]
mod tests {
    use super::*;
    use bevy::prelude::Transform;

    #[test]
    fn convert_back_to_equal_transform() {
        let transform = Transform {
            translation: bevy::prelude::Vec3::new(-2.1855694e-8, 0.0, 0.0),
            rotation: bevy::prelude::Quat::from_xyzw(0.99999994, 0.0, 1.6292068e-7, 0.0)
                .normalize(),
            ..Default::default()
        };
        let converted_transform = iso_to_transform(&transform_to_iso(&transform));
        assert_eq!(converted_transform, transform);
    }
}
