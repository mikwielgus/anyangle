// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0
//! An implementation of the [simple stupid funnel algorithm](https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html).

use core::{
    cmp::PartialOrd,
    mem,
    ops::{Mul, Sub},
};

use num_traits::sign::Signed;

#[derive(Clone, Copy, Debug, Default)]
pub struct SimpleFunnel<K> {
    pub apex: [K; 2],
    pub lhs: [K; 2],
    pub rhs: [K; 2],
    pub epsilon: K,
}

impl<K> SimpleFunnel<K>
where
    K: Clone + Default + Signed + PartialOrd,
{
    pub fn new(apex: [K; 2], epsilon: K) -> Self {
        Self {
            apex: apex.clone(),
            lhs: apex.clone(),
            rhs: apex,
            epsilon,
        }
    }

    pub fn advance(&mut self, portal: [[K; 2]; 2]) -> Option<([K; 2], [K; 2])> {
        let apex = &mut self.apex;
        let mut update_vertex = |ahs: &mut [K; 2],
                                 bhs: &mut [K; 2],
                                 bportal: &[K; 2],
                                 flip: bool|
         -> Option<([K; 2], [K; 2])> {
            let comp = |v: K| {
                if flip {
                    v > K::default()
                } else {
                    v < K::default()
                }
            };
            if !comp(triarea2(&apex, &bhs, bportal)) {
                if approx_eq(&apex, &bhs, self.epsilon.clone())
                    || comp(triarea2(&apex, &ahs, bportal))
                {
                    // Tighten the funnel
                    *bhs = bportal.clone();
                } else {
                    // B over A, insert A to path and restart scan from portal A point
                    let old_apex = mem::replace(apex, (*ahs).clone());
                    *bhs = apex.clone();
                    return Some((old_apex, (*apex).clone()));
                }
            }
            None
        };

        // Update right vertex
        if let Some(ret) = update_vertex(&mut self.lhs, &mut self.rhs, &portal[1], false) {
            return Some(ret);
        }

        // Update left vertex
        if let Some(ret) = update_vertex(&mut self.rhs, &mut self.lhs, &portal[0], true) {
            return Some(ret);
        }

        None
    }
}

/// Calculates the oriented area between 3 points
fn triarea2<K>(a: &[K; 2], b: &[K; 2], c: &[K; 2]) -> K
where
    K: Clone + Sub<Output = K> + Mul<Output = K>,
{
    perp_product(delta(b, a), delta(c, a))
}

/// Calculates the 2D perpendicular product between 2 points
fn perp_product<K>(a: [K; 2], b: [K; 2]) -> K
where
    K: Clone + Sub<Output = K> + Mul<Output = K>,
{
    b[0].clone() * a[1].clone() - a[0].clone() * b[1].clone()
}

/// Calculates the difference between 2 points
fn delta<K>(a: &[K; 2], b: &[K; 2]) -> [K; 2]
where
    K: Clone + Sub<Output = K>,
{
    [a[0].clone() - b[0].clone(), a[1].clone() - b[1].clone()]
}

fn approx_eq<K>(a: &[K; 2], b: &[K; 2], epsilon: K) -> bool
where
    K: Clone + Sub<Output = K> + Signed + PartialOrd,
{
    let d = delta(a, b);
    d[0].abs() <= epsilon && d[1].abs() <= epsilon
}

#[cfg(test)]
mod tests {
    use super::SimpleFunnel;

    #[test]
    fn t_simple() {
        let mut x = SimpleFunnel::<i32>::new([0, 0], 0);
        assert_eq!(x.advance([[30, 0], [10, 40]]), None);
        assert_eq!(x.advance([[60, 20], [60, 40]]), None);
        assert_eq!(x.advance([[70, -10], [90, 20]]), Some(([0, 0], [60, 20])));
        assert_eq!(x.advance([[80, -10], [80, -10]]), None);
    }
}
