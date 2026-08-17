// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use core::{fmt, ops};

use crate::LayerId;

#[derive(Clone, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serde", serde(from = "LayerIdsSer", into = "LayerIdsSer"))]
pub struct LayerIds(pub bit_set::BitSet);

impl fmt::Debug for LayerIds {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "LayerIds {}", self.0)
    }
}

#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
struct LayerIdsSer(Vec<usize>);

impl From<LayerIds> for LayerIdsSer {
    fn from(x: LayerIds) -> Self {
        Self(x.0.iter().collect())
    }
}

impl From<LayerIdsSer> for LayerIds {
    fn from(x: LayerIdsSer) -> Self {
        Self(x.0.iter().copied().collect())
    }
}

pub trait GetLayerIds {
    fn layers(&self) -> LayerIds;
}

impl GetLayerIds for LayerId {
    fn layers(&self) -> LayerIds {
        let mut ret = LayerIds::default();
        ret.insert(*self);
        ret
    }
}

impl GetLayerIds for LayerIds {
    fn layers(&self) -> LayerIds {
        self.clone()
    }
}

impl LayerIds {
    pub fn insert(&mut self, i: LayerId) {
        self.0.insert(i.into());
    }

    pub fn remove(&mut self, i: LayerId) {
        self.0.remove(i.into());
    }

    pub fn iter(&self) -> impl Iterator<Item = LayerId> + '_ {
        self.0.iter().map(LayerId::from)
    }

    pub fn bounding_range(&self) -> Option<core::ops::Range<usize>> {
        if self.0.is_empty() {
            None
        } else {
            let ret = self.0.iter().collect::<Vec<_>>();
            Some(*ret.first().unwrap()..*ret.last().unwrap() + 1)
        }
    }

    pub fn adjacent_layers_to(&self, i: LayerId) -> Vec<LayerId> {
        let mut ret = Vec::new();
        let iu: usize = i.into();

        if let Some(j) = iu.checked_sub(1)
            && self.0.contains(j)
        {
            ret.push(LayerId::from(j));
        }

        if let Some(j) = iu.checked_add(1)
            && self.0.contains(j)
        {
            ret.push(LayerId::from(j));
        }

        ret
    }

    pub fn is_on_layer(&self, l: LayerId) -> bool {
        self.0.contains(l.into())
    }
}

impl Extend<LayerId> for LayerIds {
    fn extend<T: IntoIterator<Item = LayerId>>(&mut self, iter: T) {
        for layer in iter {
            self.insert(layer);
        }
    }
}

impl Extend<LayerIds> for LayerIds {
    fn extend<T: IntoIterator<Item = LayerIds>>(&mut self, iter: T) {
        for layers in iter {
            self.0.union_with(&layers.0);
        }
    }
}

impl<'a> Extend<&'a LayerIds> for LayerIds {
    fn extend<T: IntoIterator<Item = &'a LayerIds>>(&mut self, iter: T) {
        for layers in iter {
            self.0.union_with(&layers.0);
        }
    }
}

impl FromIterator<LayerId> for LayerIds {
    fn from_iter<T: IntoIterator<Item = LayerId>>(iter: T) -> Self {
        let mut this = Self::default();
        this.extend(iter);
        this
    }
}

impl FromIterator<LayerIds> for LayerIds {
    fn from_iter<T: IntoIterator<Item = LayerIds>>(iter: T) -> Self {
        let mut this = Self::default();
        this.extend(iter);
        this
    }
}

impl<'a> FromIterator<&'a LayerIds> for LayerIds {
    fn from_iter<T: IntoIterator<Item = &'a LayerIds>>(iter: T) -> Self {
        let mut this = Self::default();
        this.extend(iter);
        this
    }
}

impl<'a> ops::BitAnd for &'a LayerIds {
    type Output = LayerIds;

    fn bitand(self, rhs: Self) -> LayerIds {
        LayerIds(self.0.intersection(&rhs.0).collect())
    }
}

impl<'a> ops::BitAndAssign<&'a LayerIds> for LayerIds {
    fn bitand_assign(&mut self, rhs: &'a LayerIds) {
        self.0.intersect_with(&rhs.0);
    }
}

impl<'a> ops::BitOr for &'a LayerIds {
    type Output = LayerIds;

    fn bitor(self, rhs: Self) -> LayerIds {
        LayerIds(self.0.union(&rhs.0).collect())
    }
}

impl<'a> ops::BitOrAssign<&'a LayerIds> for LayerIds {
    fn bitor_assign(&mut self, rhs: &'a LayerIds) {
        self.0.union_with(&rhs.0);
    }
}

impl<'a> ops::BitXor for &'a LayerIds {
    type Output = LayerIds;

    fn bitxor(self, rhs: Self) -> LayerIds {
        LayerIds(self.0.symmetric_difference(&rhs.0).collect())
    }
}

impl<'a> ops::BitXorAssign<&'a LayerIds> for LayerIds {
    fn bitxor_assign(&mut self, rhs: &'a LayerIds) {
        self.0.symmetric_difference_with(&rhs.0);
    }
}
