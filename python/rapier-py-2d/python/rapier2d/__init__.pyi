"""Stub for `rapier.dim2` (2D, f32)."""

from ._rapier2d import (
    Isometry2 as Isometry2,
    Point2 as Point2,
    RapierError as RapierError,
    InvalidHandle as InvalidHandle,
    MeshConversionError as MeshConversionError,
    UrdfError as UrdfError,
    MjcfError as MjcfError,
    SerializationError as SerializationError,
    QueryFailure as QueryFailure,
    MeshLoaderError as MeshLoaderError,
    Rotation2 as Rotation2,
    Vec2 as Vec2,
    rotation_from_angle as rotation_from_angle,
)
from . import f64 as f64, math as math

__version__: str
