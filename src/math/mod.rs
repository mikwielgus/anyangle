// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use core::{
    cmp::PartialOrd,
    ops::{Mul, Sub},
};
use num_traits::sign::Signed;

/// Calculates the oriented area between 3 points
pub fn triarea2<K>(a: &[K; 2], b: &[K; 2], c: &[K; 2]) -> K
where
    K: Clone + Sub<Output = K> + Mul<Output = K>,
{
    perp_product(delta(b, a), delta(c, a))
}

/// Calculates the 2D perpendicular product between 2 points
pub fn perp_product<K>(a: [K; 2], b: [K; 2]) -> K
where
    K: Clone + Sub<Output = K> + Mul<Output = K>,
{
    b[0].clone() * a[1].clone() - a[0].clone() * b[1].clone()
}

/// Calculates the difference between 2 points
pub(crate) fn delta<K>(a: &[K; 2], b: &[K; 2]) -> [K; 2]
where
    K: Clone + Sub<Output = K>,
{
    [a[0].clone() - b[0].clone(), a[1].clone() - b[1].clone()]
}

pub(crate) fn approx_eq<K>(a: &[K; 2], b: &[K; 2], epsilon: K) -> bool
where
    K: Clone + Sub<Output = K> + Signed + PartialOrd,
{
    let d = delta(a, b);
    d[0].abs() <= epsilon && d[1].abs() <= epsilon
}
