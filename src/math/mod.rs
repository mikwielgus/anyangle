// SPDX-FileCopyrightText: 2026 anyangle contributors
// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use core::{
    cmp::{Ordering, PartialOrd},
    ops::{Mul, Sub},
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum RotationSense {
    Counterclockwise,
    Clockwise,
}

impl core::ops::Neg for RotationSense {
    type Output = Self;

    fn neg(self) -> Self {
        match self {
            RotationSense::Counterclockwise => RotationSense::Clockwise,
            RotationSense::Clockwise => RotationSense::Counterclockwise,
        }
    }
}

impl RotationSense {
    /// move `pos` by `step` along `self` assuming the list of positions is ordered CCW.
    pub fn step_ccw(self, pos: usize, len: usize, mut step: usize) -> usize {
        step %= len;
        (match self {
            RotationSense::Counterclockwise => pos + step,
            RotationSense::Clockwise => len + pos - step,
        }) % len
    }
}

pub fn poly_convex_hull_rotation_sense<S, I>(
    poly_ext_hull: &[([S; 2], I)],
    pivot: usize,
) -> RotationSense
where
    S: Clone + PartialOrd + num_traits::Num,
{
    let len = poly_ext_hull.len();
    assert!(pivot < 5);

    let prev = &poly_ext_hull[(len + pivot - 1) % len].0;
    let curr = &poly_ext_hull[pivot].0;
    let next = &poly_ext_hull[(pivot + 1) % len].0;

    // see also: https://en.wikipedia.org/w/index.php?title=Curve_orientation&oldid=1250027587#Orientation_of_a_simple_polygon
    #[rustfmt::skip]
    let det = (curr[0].clone() * next[1].clone() + prev[0].clone() * curr[1].clone() + prev[1].clone() * next[0].clone())
            - (curr[1].clone() * next[0].clone() + prev[1].clone() * curr[0].clone() + prev[0].clone() * next[1].clone());

    if det < S::zero() {
        RotationSense::Clockwise
    } else {
        RotationSense::Counterclockwise
    }
}

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

/// [Diagonal taxicab norm](https://fogti.codeberg.page/public-docs/Topology/diagonal-taxicab/diagonal-taxicab.pdf) for `n=2`.
pub fn diagonal_taxicab_norm<K>(x: &[K; 2]) -> Option<K>
where
    K: Clone + CheckedAdd + num_traits::Signed + num_traits::NumCast + Into<f64> + PartialOrd,
{
    let factor: f64 = (2.0_f64).sqrt() - 1.;

    let x = (x[0].clone().abs(), x[1].clone().abs());

    let (min_value, max_value) = match x.0.partial_cmp(&x.1)? {
        Ordering::Less | Ordering::Equal => x,
        Ordering::Greater => (x.1, x.0),
    };

    max_value.checked_add(&<K as num_traits::NumCast>::from::<f64>(
        factor * Into::<f64>::into(min_value),
    )?)
}

pub trait CheckedAdd: Sized {
    fn checked_add(&self, oth: &Self) -> Option<Self>;
}

impl CheckedAdd for f32 {
    fn checked_add(&self, oth: &Self) -> Option<Self> {
        let ret = self + oth;
        if ret.is_finite() { Some(ret) } else { None }
    }
}

impl CheckedAdd for f64 {
    fn checked_add(&self, oth: &Self) -> Option<Self> {
        let ret = self + oth;
        if ret.is_finite() { Some(ret) } else { None }
    }
}

macro_rules! integer_impl {
    ($($ty:ty),*) => {
        $(
        impl CheckedAdd for $ty {
            fn checked_add(&self, oth: &Self) -> Option<Self> {
                <$ty>::checked_add(*self, *oth)
            }
        }
        )*
    }
}

integer_impl!(i8, u8, i16, u16, i32, u32, i64, u64);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diagonal_taxicab_norm_f64() {
        use approx::AbsDiffEq;
        assert!(
            diagonal_taxicab_norm::<f64>(&[8., 8.])
                .unwrap()
                .abs_diff_eq(&11.313708498984761, f64::EPSILON),
        );
        assert!(
            diagonal_taxicab_norm::<f64>(&[8., 10.343])
                .unwrap()
                .abs_diff_eq(&13.656708498984761, f64::EPSILON),
        );
        assert!(
            diagonal_taxicab_norm::<f64>(&[0., 16.])
                .unwrap()
                .abs_diff_eq(&16., f64::EPSILON),
        );
    }

    #[test]
    fn diagonal_taxicab_norm_i32() {
        assert_eq!(diagonal_taxicab_norm::<i32>(&[8, 8]), Some(11));
        assert_eq!(diagonal_taxicab_norm::<i32>(&[8, 10]), Some(13));
        assert_eq!(diagonal_taxicab_norm::<i32>(&[10, 8]), Some(13));
        assert_eq!(diagonal_taxicab_norm::<i32>(&[0, 16]), Some(16));
    }
}
