// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use core::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct LayerId(pub usize);

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
        x.0.iter().copied().map(LayerId).collect()
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
        self.0.insert(i.0);
    }

    pub fn remove(&mut self, i: LayerId) {
        self.0.remove(i.0);
    }

    pub fn iter(&self) -> impl Iterator<Item = LayerId> + '_ {
        self.0.iter().map(LayerId)
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

        if let Some(j) = i.0.checked_sub(1)
            && self.0.contains(j)
        {
            ret.push(LayerId(j));
        }

        if let Some(j) = i.0.checked_add(1)
            && self.0.contains(j)
        {
            ret.push(LayerId(j));
        }

        ret
    }

    pub fn is_on_layer(&self, l: LayerId) -> bool {
        self.0.contains(l.0)
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
