// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use core::fmt;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
#[repr(transparent)]
pub struct LayerId(u32);

impl LayerId {
    pub const MIN: LayerId = LayerId(0);
    pub const MAX: LayerId = LayerId(u32::MAX);

    #[inline]
    pub fn checked_add(self, delta: usize) -> Option<Self> {
        self.0.checked_add(u32::try_from(delta).ok()?).map(Self)
    }

    #[inline]
    pub(crate) fn checked_add_one(self) -> Option<Self> {
        self.0.checked_add(1).map(Self)
    }

    #[inline]
    pub fn checked_sub(self, delta: usize) -> Option<Self> {
        self.0.checked_sub(u32::try_from(delta).ok()?).map(Self)
    }

    #[inline]
    pub(crate) fn checked_sub_one(self) -> Option<Self> {
        self.0.checked_sub(1).map(Self)
    }
}

impl fmt::Debug for LayerId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "LayerId({})", self.0)
    }
}

impl From<usize> for LayerId {
    #[inline]
    fn from(x: usize) -> LayerId {
        LayerId(x.try_into().unwrap())
    }
}

impl From<LayerId> for usize {
    #[inline]
    fn from(x: LayerId) -> usize {
        x.0.try_into().unwrap()
    }
}
