use super::SENTINEL_VALUE;
use crate::math::Real;

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SAPEndpoint {
    pub value: Real,
    pub packed_flag_proxy: u32,
}

const START_FLAG_MASK: u32 = 0b1 << 31;
const PROXY_MASK: u32 = u32::MAX ^ START_FLAG_MASK;
const START_SENTINEL_TAG: u32 = u32::MAX;
const END_SENTINEL_TAG: u32 = u32::MAX ^ START_FLAG_MASK;

impl SAPEndpoint {
    pub fn start_endpoint(value: Real, proxy: u32) -> Self {
        Self {
            value,
            packed_flag_proxy: proxy | START_FLAG_MASK,
        }
    }

    pub fn end_endpoint(value: Real, proxy: u32) -> Self {
        Self {
            value,
            packed_flag_proxy: proxy & PROXY_MASK,
        }
    }

    pub fn start_sentinel() -> Self {
        Self {
            value: -SENTINEL_VALUE,
            packed_flag_proxy: START_SENTINEL_TAG,
        }
    }

    pub fn end_sentinel() -> Self {
        Self {
            value: SENTINEL_VALUE,
            packed_flag_proxy: END_SENTINEL_TAG,
        }
    }

    pub fn is_sentinel(self) -> bool {
        self.packed_flag_proxy & PROXY_MASK == PROXY_MASK
    }

    pub fn proxy(self) -> u32 {
        self.packed_flag_proxy & PROXY_MASK
    }

    pub fn is_start(self) -> bool {
        (self.packed_flag_proxy & START_FLAG_MASK) != 0
    }

    pub fn is_end(self) -> bool {
        (self.packed_flag_proxy & START_FLAG_MASK) == 0
    }
}
