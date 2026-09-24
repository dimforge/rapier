use rapier::prelude::{Aabb, VoxelData, VoxelState, Voxels};

use crate::math::{IVect, Vect};

#[cfg(feature = "dim2")]
use bevy::shape::Aabb2d as BevyAabb;
#[cfg(feature = "dim3")]
use bevy::shape::Aabb3d as BevyAabb;

#[cfg(feature = "dim2")]
fn aabb_na_from_bevy(aabb: &BevyAabb) -> Aabb {
    rapier::parry::bounding_volume::Aabb::new(aabb.min, aabb.max)
}

#[cfg(feature = "dim3")]
fn aabb_na_from_bevy(aabb: &BevyAabb) -> Aabb {
    rapier::parry::bounding_volume::Aabb::new(aabb.min.into(), aabb.max.into())
}

#[cfg(feature = "dim2")]
fn aabb_bevy_from_na(aabb: &Aabb) -> BevyAabb {
    BevyAabb {
        min: aabb.mins,
        max: aabb.maxs,
    }
}

#[cfg(feature = "dim3")]
fn aabb_bevy_from_na(aabb: &Aabb) -> BevyAabb {
    BevyAabb {
        min: aabb.mins.into(),
        max: aabb.maxs.into(),
    }
}

/// Read-only access to the properties of a [`Voxels`] shape.
#[derive(Copy, Clone)]
pub struct VoxelsView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Voxels,
}

// TODO: implement those on VoxelsViewMut too.
macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// Shortcut to [`Voxels::total_memory_size`].
            pub fn total_memory_size(&self) -> usize {
                self.raw.total_memory_size()
            }

            /// Shortcut to [`Voxels::heap_memory_size`].
            pub fn heap_memory_size(&self) -> usize {
                self.raw.heap_memory_size()
            }

            /// Shortcut to [`Voxels::voxel_size`].
            pub fn voxel_size(&self) -> Vect {
                self.raw.voxel_size()
            }

            /// Shortcut to [`Voxels::voxel_center`].
            pub fn voxel_center(&self, key: IVect) -> Vect {
                self.raw.voxel_center(key)
            }

            /// Shortcut to [`Voxels::cropped`].
            ///
            /// Returns `None` if no filled voxel lies in the `[domain_mins, domain_maxs]` range.
            pub fn cropped(&self, domain_mins: IVect, domain_maxs: IVect) -> Option<Voxels> {
                self.raw.cropped(domain_mins, domain_maxs)
            }

            /// Shortcut to [`Voxels::local_aabb`].
            pub fn extents(&self) -> Vect {
                self.raw.local_aabb().extents()
            }

            /// Shortcut to [`Voxels::local_aabb`].
            pub fn domain_center(&self) -> Vect {
                self.raw.local_aabb().center()
            }

            /// Shortcut to [`Voxels::domain`].
            pub fn is_voxel_in_bounds(&self, key: IVect) -> bool {
                let [mins, maxs] = self.raw.domain();
                #[cfg(feature = "dim2")]
                {
                    key.x >= mins.x && key.y >= mins.y && key.x < maxs.x && key.y < maxs.y
                }
                #[cfg(feature = "dim3")]
                {
                    key.x >= mins.x
                        && key.y >= mins.y
                        && key.z >= mins.z
                        && key.x < maxs.x
                        && key.y < maxs.y
                        && key.z < maxs.z
                }
            }

            /// Shortcut to [`Voxels::voxel_aabb`].
            pub fn voxel_aabb(&self, key: IVect) -> BevyAabb {
                aabb_bevy_from_na(&self.raw.voxel_aabb(key.into()))
            }

            /// Shortcut to [`Voxels::voxel_state`].
            pub fn voxel_state(&self, key: IVect) -> Option<VoxelState> {
                self.raw.voxel_state(key.into())
            }

            /// Shortcut to [`Voxels::voxel_at_point`].
            pub fn voxel_at_point_unchecked(&self, point: Vect) -> IVect {
                self.raw.voxel_at_point(point)
            }

            /// Shortcut to [`Voxels::voxel_at_point`].
            pub fn voxel_at_point(&self, pt: Vect) -> Option<IVect> {
                let voxel = self.voxel_at_point_unchecked(pt);
                if self.is_voxel_in_bounds(voxel) {
                    Some(voxel)
                } else {
                    None
                }
            }

            /// Shortcut to [`Voxels::domain`].
            pub fn clamp_voxel(&self, key: IVect) -> IVect {
                let [mins, maxs] = self.raw.domain();
                #[cfg(feature = "dim2")]
                {
                    IVect::new(
                        key.x.clamp(mins.x, maxs.x - 1),
                        key.y.clamp(mins.y, maxs.y - 1),
                    )
                }
                #[cfg(feature = "dim3")]
                {
                    IVect::new(
                        key.x.clamp(mins.x, maxs.x - 1),
                        key.y.clamp(mins.y, maxs.y - 1),
                        key.z.clamp(mins.z, maxs.z - 1),
                    )
                }
            }

            /// Shortcut to [`Voxels::voxel_range_intersecting_local_aabb`].
            pub fn voxel_range_intersecting_local_aabb(&self, aabb: &BevyAabb) -> [IVect; 2] {
                let res = self
                    .raw
                    .voxel_range_intersecting_local_aabb(&aabb_na_from_bevy(aabb));
                [res[0].into(), res[1].into()]
            }

            /// Shortcut to [`Voxels::voxel_range_aabb`].
            pub fn voxel_range_aabb(&self, mins: IVect, maxs: IVect) -> BevyAabb {
                aabb_bevy_from_na(&self.raw.voxel_range_aabb(mins.into(), maxs.into()))
            }

            /// Shortcut to [`Voxels::align_aabb_to_grid`].
            pub fn align_aabb_to_grid(&self, aabb: &BevyAabb) -> BevyAabb {
                aabb_bevy_from_na(&self.raw.align_aabb_to_grid(&aabb_na_from_bevy(aabb)))
            }

            /// Shortcut to [`Voxels::voxels_intersecting_local_aabb`].
            pub fn voxels_intersecting_local_aabb(
                &self,
                aabb: &BevyAabb,
            ) -> impl Iterator<Item = VoxelData> + '_ {
                self.raw
                    .voxels_intersecting_local_aabb(&aabb_na_from_bevy(aabb))
            }

            /// Shortcut to [`Voxels::voxels`].
            pub fn voxels(&self) -> impl Iterator<Item = VoxelData> + '_ {
                self.raw.voxels()
            }

            /// Shortcut to [`Voxels::split_with_box`].
            pub fn split_with_box(&self, aabb: &BevyAabb) -> (Option<Voxels>, Option<Voxels>) {
                self.raw.split_with_box(&aabb_na_from_bevy(aabb))
            }

            /// Shortcut to [`Voxels::voxels_in_range`].
            pub fn voxels_in_range(
                &self,
                mins: IVect,
                maxs: IVect,
            ) -> impl Iterator<Item = VoxelData> + '_ {
                self.raw.voxels_in_range(mins.into(), maxs.into())
            }
        }
    }
);

impl_ref_methods!(VoxelsView);

/// Read-write access to the properties of a [`Voxels`].
pub struct VoxelsViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut Voxels,
}

impl_ref_methods!(VoxelsViewMut);

impl<'a> VoxelsViewMut<'a> {
    /// Shortcut to set voxel state with bounds checking.
    pub fn try_set_voxel(&mut self, key: IVect, is_filled: bool) -> Option<VoxelState> {
        if self.is_voxel_in_bounds(key) {
            Some(self.raw.set_voxel(key, is_filled))
        } else {
            None
        }
    }

    /// Shortcut to to [`Voxels::set_voxel`].
    pub fn set_voxel(&mut self, key: IVect, is_filled: bool) -> VoxelState {
        self.raw.set_voxel(key, is_filled)
    }

    /// Shortcut to [`Voxels::crop`].
    pub fn crop(&mut self, domain_mins: IVect, domain_maxs: IVect) {
        self.raw.crop(domain_mins, domain_maxs);
    }

    /// Shortcut to [`Voxels::set_voxel_size`].
    pub fn set_voxel_size(&mut self, new_size: Vect) {
        self.raw.set_voxel_size(new_size);
    }

    /// Shortcut to [`Voxels::combine_voxel_states`].
    ///
    /// Makes the voxels of `self` and `other` aware of their neighbors from the other shape, so
    /// that contacts transition smoothly across their boundary. A voxel with coordinates `key`
    /// in `other` has coordinates `key + origin_shift` in `self`.
    pub fn combine_voxel_states(&mut self, other: &mut VoxelsViewMut<'_>, origin_shift: IVect) {
        self.raw.combine_voxel_states(other.raw, origin_shift);
    }

    /// Shortcut to [`Voxels::propagate_voxel_change`].
    ///
    /// Same as [`Self::combine_voxel_states`] but restricted to the given `voxel` of `self` (and
    /// its neighbors), after it was modified with [`Self::set_voxel`].
    pub fn propagate_voxel_change(
        &mut self,
        other: &mut VoxelsViewMut<'_>,
        voxel: IVect,
        origin_shift: IVect,
    ) {
        self.raw
            .propagate_voxel_change(other.raw, voxel, origin_shift);
    }
}
