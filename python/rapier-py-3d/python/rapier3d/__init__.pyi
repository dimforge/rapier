"""Stub for `rapier.dim3` (3D, f32)."""

from ._rapier3d import (
    AngVector3 as AngVector3,
    Isometry3 as Isometry3,
    Point3 as Point3,
    Quaternion as Quaternion,
    RapierError as RapierError,
    InvalidHandle as InvalidHandle,
    MeshConversionError as MeshConversionError,
    UrdfError as UrdfError,
    MjcfError as MjcfError,
    SerializationError as SerializationError,
    QueryFailure as QueryFailure,
    MeshLoaderError as MeshLoaderError,
    Rotation3 as Rotation3,
    Vec3 as Vec3,
    rotation_from_angle as rotation_from_angle,
)
from . import f64 as f64, math as math

__version__: str
