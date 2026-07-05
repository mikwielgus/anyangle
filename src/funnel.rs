// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0
//! An implementation of the [simple stupid funnel algorithm](https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html).

use core::{cmp::PartialOrd, mem};

use num_traits::sign::Signed;

use crate::math::*;

#[derive(Clone, Copy, Debug, Default)]
pub struct SimpleFunnel<K, D = ()> {
    pub apex: ([K; 2], D),
    pub lhs: ([K; 2], D),
    pub rhs: ([K; 2], D),
    pub epsilon: K,
}

impl<K, D> SimpleFunnel<K, D>
where
    K: Clone + Default + Signed + PartialOrd,
    D: Clone,
{
    pub fn new(apex: ([K; 2], D), epsilon: K) -> Self {
        Self {
            apex: apex.clone(),
            lhs: apex.clone(),
            rhs: apex,
            epsilon,
        }
    }

    pub fn advance(&mut self, portal: [([K; 2], D); 2]) -> Option<(([K; 2], D), &([K; 2], D))> {
        let apex = &mut self.apex;
        /*
        let mut update_vertex = |ahs: &mut ([K; 2], D),
                                 bhs: &mut ([K; 2], D),
                                 bportal: &([K; 2], D),
                                 flip: bool|
         -> Option<(([K; 2], D), &([K; 2], D))> */
        macro_rules! update_vertex {
            ($ahs:expr, $bhs:expr, $bportal:expr, $flip:expr) => {{
                let comp = |v: K| {
                    if $flip {
                        v > K::default()
                    } else {
                        v < K::default()
                    }
                };
                if !comp(triarea2(&apex.0, &$bhs.0, &$bportal.0)) {
                    if approx_eq(&apex.0, &$bhs.0, self.epsilon.clone())
                        || comp(triarea2(&apex.0, &$ahs.0, &$bportal.0))
                    {
                        // Tighten the funnel
                        *$bhs = $bportal.clone();
                    } else {
                        // B over A, insert A to path and restart scan from portal A point
                        let old_apex = mem::replace(apex, (*$ahs).clone());
                        *$bhs = apex.clone();
                        return Some((old_apex, apex));
                    }
                }
                None
            }};
        }

        // Update right vertex
        if let Some(ret) = update_vertex!(&mut self.lhs, &mut self.rhs, &portal[1], false) {
            return Some(ret);
        }

        // Update left vertex
        if let Some(ret) = update_vertex!(&mut self.rhs, &mut self.lhs, &portal[0], true) {
            return Some(ret);
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::SimpleFunnel;

    #[test]
    fn t_simple() {
        let mut x = SimpleFunnel::<i32, u8>::new(([0, 0], 0), 0);
        assert_eq!(x.advance([([30, 0], 1), ([10, 40], 2)]), None);
        assert_eq!(x.advance([([60, 20], 3), ([60, 40], 4)]), None);
        assert_eq!(
            x.advance([([70, -10], 5), ([90, 20], 6)]),
            Some((([0, 0], 0), &([60, 20], 3)))
        );
        assert_eq!(x.advance([([80, -10], 7), ([80, -10], 8)]), None);
    }
}
