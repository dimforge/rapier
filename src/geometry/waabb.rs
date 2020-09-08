#[cfg(feature = "serde-serialize")]
use crate::math::DIM;
use crate::math::{Point, SIMD_WIDTH};
use ncollide::bounding_volume::AABB;
#[cfg(feature = "simd-is-enabled")]
use {
    crate::math::{SimdBool, SimdFloat},
    simba::simd::{SimdPartialOrd, SimdValue},
};

#[derive(Debug, Copy, Clone)]
#[cfg(feature = "simd-is-enabled")]
pub(crate) struct WAABB {
    pub mins: Point<SimdFloat>,
    pub maxs: Point<SimdFloat>,
}

#[derive(Debug, Copy, Clone)]
#[cfg(not(feature = "simd-is-enabled"))]
pub(crate) struct WAABB {
    pub mins: [Point<f32>; SIMD_WIDTH],
    pub maxs: [Point<f32>; SIMD_WIDTH],
}

#[cfg(feature = "serde-serialize")]
impl serde::Serialize for WAABB {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;

        #[cfg(feature = "simd-is-enabled")]
        let mins: Point<[f32; SIMD_WIDTH]> = Point::from(
            self.mins
                .coords
                .map(|e| array![|ii| e.extract(ii); SIMD_WIDTH]),
        );
        #[cfg(feature = "simd-is-enabled")]
        let maxs: Point<[f32; SIMD_WIDTH]> = Point::from(
            self.maxs
                .coords
                .map(|e| array![|ii| e.extract(ii); SIMD_WIDTH]),
        );

        #[cfg(not(feature = "simd-is-enabled"))]
        let mins = self.mins;
        #[cfg(not(feature = "simd-is-enabled"))]
        let maxs = self.maxs;

        let mut waabb = serializer.serialize_struct("WAABB", 2)?;
        waabb.serialize_field("mins", &mins)?;
        waabb.serialize_field("maxs", &maxs)?;
        waabb.end()
    }
}

#[cfg(feature = "serde-serialize")]
impl<'de> serde::Deserialize<'de> for WAABB {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        struct Visitor {};
        impl<'de> serde::de::Visitor<'de> for Visitor {
            type Value = WAABB;
            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                write!(
                    formatter,
                    "two arrays containing at least {} floats",
                    SIMD_WIDTH * DIM * 2
                )
            }

            #[cfg(feature = "simd-is-enabled")]
            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: serde::de::SeqAccess<'de>,
            {
                let mins: Point<[f32; SIMD_WIDTH]> = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(0, &self))?;
                let maxs: Point<[f32; SIMD_WIDTH]> = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(1, &self))?;
                let mins = Point::from(mins.coords.map(|e| SimdFloat::from(e)));
                let maxs = Point::from(maxs.coords.map(|e| SimdFloat::from(e)));
                Ok(WAABB { mins, maxs })
            }

            #[cfg(not(feature = "simd-is-enabled"))]
            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: serde::de::SeqAccess<'de>,
            {
                let mins = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(0, &self))?;
                let maxs = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(1, &self))?;
                Ok(WAABB { mins, maxs })
            }
        }

        deserializer.deserialize_struct("WAABB", &["mins", "maxs"], Visitor {})
    }
}

#[cfg(feature = "simd-is-enabled")]
impl WAABB {
    pub fn new(mins: Point<SimdFloat>, maxs: Point<SimdFloat>) -> Self {
        Self { mins, maxs }
    }

    pub fn new_invalid() -> Self {
        Self::splat(AABB::new_invalid())
    }

    pub fn splat(aabb: AABB<f32>) -> Self {
        Self {
            mins: Point::splat(aabb.mins),
            maxs: Point::splat(aabb.maxs),
        }
    }

    #[cfg(feature = "dim2")]
    pub fn intersects_lanewise(&self, other: &WAABB) -> SimdBool {
        self.mins.x.simd_le(other.maxs.x)
            & other.mins.x.simd_le(self.maxs.x)
            & self.mins.y.simd_le(other.maxs.y)
            & other.mins.y.simd_le(self.maxs.y)
    }

    #[cfg(feature = "dim3")]
    pub fn intersects_lanewise(&self, other: &WAABB) -> SimdBool {
        self.mins.x.simd_le(other.maxs.x)
            & other.mins.x.simd_le(self.maxs.x)
            & self.mins.y.simd_le(other.maxs.y)
            & other.mins.y.simd_le(self.maxs.y)
            & self.mins.z.simd_le(other.maxs.z)
            & other.mins.z.simd_le(self.maxs.z)
    }
}

#[cfg(feature = "simd-is-enabled")]
impl From<[AABB<f32>; SIMD_WIDTH]> for WAABB {
    fn from(aabbs: [AABB<f32>; SIMD_WIDTH]) -> Self {
        let mins = array![|ii| aabbs[ii].mins; SIMD_WIDTH];
        let maxs = array![|ii| aabbs[ii].maxs; SIMD_WIDTH];

        WAABB {
            mins: Point::from(mins),
            maxs: Point::from(maxs),
        }
    }
}

#[cfg(not(feature = "simd-is-enabled"))]
impl WAABB {
    pub fn new_invalid() -> Self {
        Self::splat(AABB::new_invalid())
    }

    pub fn splat(aabb: AABB<f32>) -> Self {
        Self {
            mins: [aabb.mins; SIMD_WIDTH],
            maxs: [aabb.maxs; SIMD_WIDTH],
        }
    }

    #[cfg(feature = "dim2")]
    pub fn intersects_lanewise(&self, other: &WAABB) -> [bool; SIMD_WIDTH] {
        array![|ii|
            self.mins[ii].x <= other.maxs[ii].x
                && other.mins[ii].x <= self.maxs[ii].x
                && self.mins[ii].y <= other.maxs[ii].y
                && other.mins[ii].y <= self.maxs[ii].y
            ; SIMD_WIDTH
        ]
    }

    #[cfg(feature = "dim3")]
    pub fn intersects_lanewise(&self, other: &WAABB) -> [bool; SIMD_WIDTH] {
        array![|ii|
            self.mins[ii].x <= other.maxs[ii].x
                && other.mins[ii].x <= self.maxs[ii].x
                && self.mins[ii].y <= other.maxs[ii].y
                && other.mins[ii].y <= self.maxs[ii].y
                && self.mins[ii].z <= other.maxs[ii].z
                && other.mins[ii].z <= self.maxs[ii].z
            ; SIMD_WIDTH
        ]
    }
}

#[cfg(not(feature = "simd-is-enabled"))]
impl From<[AABB<f32>; SIMD_WIDTH]> for WAABB {
    fn from(aabbs: [AABB<f32>; SIMD_WIDTH]) -> Self {
        let mins = array![|ii| aabbs[ii].mins; SIMD_WIDTH];
        let maxs = array![|ii| aabbs[ii].maxs; SIMD_WIDTH];

        WAABB { mins, maxs }
    }
}
