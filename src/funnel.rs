// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0
//! An implementation of the [simple stupid funnel algorithm](https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html).
//! ## Well-formed `portal`s
//!
//! Note that consecutive `portal`s, and the current portal formed by `[self.lhs, self.rhs]`
//! and the currently passed `portal` aren't allowed to intersect anywhere except at their
//! endpoints, otherwise this calculation breaks down / yields usless results.
//!
//! If routing across the faces of some subdivision of 2D space, this means passing
//! edges of a _single_ triangulation (with faces being triangles) yields valid funnels,
//! and similarly, using 2D polygonal complices (complexes) with convex faces and passing
//! their edges into this method also works.

use alloc::vec::Vec;
use approx::AbsDiffEq;
use num_traits::Num;

use crate::math::*;

#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub struct SimpleFunnel<K, D = ()> {
    pub apex: ([K; 2], D),
    pub lhs: ([K; 2], D, usize),
    pub rhs: ([K; 2], D, usize),

    pub past_portals: Vec<[([K; 2], D); 2]>,
}

pub struct SimpleFunnelWithEpsilon<'a, K: AbsDiffEq, D = ()> {
    pub inner: &'a mut SimpleFunnel<K, D>,
    pub epsilon: <K as AbsDiffEq>::Epsilon,
}

impl<K, D> SimpleFunnel<K, D>
where
    K: AbsDiffEq + Clone,
    D: Clone,
{
    pub fn new(apex: ([K; 2], D)) -> Self {
        Self {
            apex: apex.clone(),
            lhs: (apex.0.clone(), apex.1.clone(), 0),
            rhs: (apex.0.clone(), apex.1.clone(), 0),
            past_portals: Vec::new(),
        }
    }

    #[inline]
    pub fn with_epsilon(
        &mut self,
        epsilon: <K as AbsDiffEq>::Epsilon,
    ) -> SimpleFunnelWithEpsilon<'_, K, D> {
        SimpleFunnelWithEpsilon {
            inner: self,
            epsilon,
        }
    }

    #[inline]
    pub fn push(&mut self, portal: [([K; 2], D); 2]) {
        self.past_portals.push(portal);
    }
}

impl<K, D> Iterator for SimpleFunnelWithEpsilon<'_, K, D>
where
    K: AbsDiffEq + Clone + Num + PartialOrd,
    <K as AbsDiffEq>::Epsilon: Clone,
    D: Clone,
{
    type Item = ([K; 2], D);

    fn next(&mut self) -> Option<Self::Item> {
        let apex = &mut self.inner.apex;
        macro_rules! update_vertex {
            ($ahs:expr, $bhs:expr, $bportal_id:expr, $bportal:expr, $flip:expr) => {{
                let comp = |v: K| {
                    if $flip { v > K::zero() } else { v < K::zero() }
                };
                (|| {
                    if !comp(triarea2(&apex.0, &$bhs.0, &$bportal.0)) {
                        if apex.0.abs_diff_eq(&$bhs.0, self.epsilon.clone())
                            || comp(triarea2(&apex.0, &$ahs.0, &$bportal.0))
                        {
                            // Tighten the funnel
                            $bhs.0 = $bportal.0.clone();
                            $bhs.1 = $bportal.1.clone();
                            $bhs.2 = $bportal_id + 1;
                        } else {
                            // B over A, insert A to path
                            apex.0 = $ahs.0.clone();
                            apex.1 = $ahs.1.clone();
                            // Restart scan from portal A point
                            let ret_idx = $ahs.2;
                            $ahs.2 = 0;
                            $bhs.0 = apex.0.clone();
                            $bhs.1 = apex.1.clone();
                            $bhs.2 = 0;
                            return Some(ret_idx);
                        }
                    }
                    None
                })()
            }};
        }

        let mut ret_idx = None;

        for (i, portal) in self.inner.past_portals.iter().enumerate() {
            // Update right vertex
            if let Some(idx) = update_vertex!(
                &mut self.inner.lhs,
                &mut self.inner.rhs,
                i,
                &portal[1],
                false
            ) {
                ret_idx = Some(idx);
                break;
            }

            // Update left vertex
            if let Some(idx) = update_vertex!(
                &mut self.inner.rhs,
                &mut self.inner.lhs,
                i,
                &portal[0],
                true
            ) {
                ret_idx = Some(idx);
                break;
            }
        }

        if let Some(idx) = ret_idx {
            assert_ne!(idx, 0);
            // Restart scan from portal `idx` point
            self.inner.past_portals.drain(..idx);
            Some(apex.clone())
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::SimpleFunnel;
    use alloc::vec::Vec;

    #[test]
    fn t_simple() {
        let mut x = SimpleFunnel::<i32, u8>::new(([0, 0], 0));
        x.push([([30, 0], 1), ([10, 40], 2)]);
        x.push([([60, 20], 3), ([60, 40], 4)]);
        x.push([([70, -10], 5), ([90, 20], 6)]);
        x.push([([80, -10], 7), ([80, -10], 8)]);

        let y = x.with_epsilon(0).collect::<Vec<_>>();
        assert_eq!(&y[..], &[([60, 20], 3), ([80, -10], 8)]);
    }

    #[test]
    // See issue #49
    //
    // TODO(fogti): the behavior below is afaik wrong,
    // this should additionally yield [10,10] and [20,40].
    //
    // This test is here to document the behavior regarding the seemingly-wrong trace that
    // is the main anchor point of the issue mentioned above,
    // such that we have something to compare against when we attempt to fix this later.
    fn t_issue49() {
        let mut x = SimpleFunnel::<i32, u8>::new(([20, 20], 7));
        x.push([([20, 10], 6), ([95, 5], 14)]);
        x.push([([10, 10], 4), ([5, 5], 2)]);
        x.push([([10, 20], 5), ([5, 95], 3)]);
        x.push([([20, 40], 8), ([5, 95], 3)]);
        x.push([([40, 40], 10), ([40, 40], 10)]);

        let y = x.with_epsilon(0).collect::<Vec<_>>();
        assert_eq!(
            &y[..],
            &[([20, 10], 6), ([10, 10], 4), ([10, 20], 5), ([20, 40], 8),]
        );
    }
}
