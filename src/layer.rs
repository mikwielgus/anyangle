// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
#[repr(transparent)]
pub struct LayerId(usize);

impl LayerId {
    pub const MIN: LayerId = LayerId(0);
    pub const MAX: LayerId = LayerId(usize::MAX);
}

impl From<usize> for LayerId {
    #[inline]
    fn from(x: usize) -> LayerId {
        LayerId(x)
    }
}

impl From<LayerId> for usize {
    #[inline]
    fn from(x: LayerId) -> usize {
        x.0
    }
}
