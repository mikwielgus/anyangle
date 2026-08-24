// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use core::{fmt, ops};

use crate::LayerId;

type BitBlock = u32;

#[derive(Clone, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serde", serde(from = "LayerIdsSer", into = "LayerIdsSer"))]
pub struct LayerIds(pub bit_set::BitSet<BitBlock>);

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

    pub fn iter(&self) -> Iter<'_> {
        self.into_iter()
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

impl ops::BitAnd for &LayerIds {
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

impl ops::BitOr for &LayerIds {
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

impl ops::BitXor for &LayerIds {
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

pub struct Iter<'a>(bit_set::Iter<'a, BitBlock>);

impl<'a> Iterator for Iter<'a> {
    type Item = LayerId;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.0.next().map(LayerId::from)
    }

    #[inline]
    fn count(self) -> usize {
        self.0.count()
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        self.0.size_hint()
    }
}

impl<'a> IntoIterator for &'a LayerIds {
    type Item = LayerId;
    type IntoIter = Iter<'a>;

    #[inline]
    fn into_iter(self) -> Iter<'a> {
        Iter((&self.0).into_iter())
    }
}
