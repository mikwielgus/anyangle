// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0
//! [Diagonal taxicab norm](https://fogti.codeberg.page/public-docs/Topology/diagonal-taxicab/diagonal-taxicab.pdf) for `n=2`.

use approx::AbsDiffEq;
use core::{
    cmp::Ordering,
    ops::{Add, AddAssign, Mul, Sub, SubAssign},
};
use num_traits::{ConstZero, FloatConst, MulAdd, Signed, Zero};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct DiagonalTaxicabNorm<Scalar> {
    pub along_axis: Scalar,
    pub diagonal: Scalar,
}

impl<Scalar> DiagonalTaxicabNorm<Scalar>
where
    Scalar: Clone + Signed + PartialOrd,
{
    pub fn new(x: [Scalar; 2]) -> Option<Self> {
        let [x0, x1] = x.map(|i| i.abs());

        let (min_value, max_value) = match x0.partial_cmp(&x1)? {
            Ordering::Less | Ordering::Equal => (x0, x1),
            Ordering::Greater => (x1, x0),
        };

        Some(Self {
            along_axis: max_value - min_value.clone(),
            diagonal: min_value,
        })
    }
}

impl<Scalar> From<DiagonalTaxicabNorm<Scalar>> for f32
where
    Scalar: Into<Self>,
    Self: MulAdd,
{
    fn from(x: DiagonalTaxicabNorm<Scalar>) -> Self {
        Self::SQRT_2().mul_add(x.diagonal.into(), x.along_axis.into())
    }
}

impl<Scalar> From<DiagonalTaxicabNorm<Scalar>> for f64
where
    Scalar: Into<Self>,
    Self: MulAdd,
{
    fn from(x: DiagonalTaxicabNorm<Scalar>) -> Self {
        Self::SQRT_2().mul_add(x.diagonal.into(), x.along_axis.into())
    }
}

impl<Scalar: Add<Output = Scalar>> Add for DiagonalTaxicabNorm<Scalar> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self {
            along_axis: self.along_axis + rhs.along_axis,
            diagonal: self.diagonal + rhs.diagonal,
        }
    }
}

impl<Scalar: Sub<Output = Scalar>> Sub for DiagonalTaxicabNorm<Scalar> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self {
            along_axis: self.along_axis - rhs.along_axis,
            diagonal: self.diagonal - rhs.diagonal,
        }
    }
}

impl<Scalar: AddAssign> AddAssign for DiagonalTaxicabNorm<Scalar> {
    fn add_assign(&mut self, rhs: Self) {
        self.along_axis += rhs.along_axis;
        self.diagonal += rhs.diagonal;
    }
}

impl<Scalar: SubAssign> SubAssign for DiagonalTaxicabNorm<Scalar> {
    fn sub_assign(&mut self, rhs: Self) {
        self.along_axis -= rhs.along_axis;
        self.diagonal -= rhs.diagonal;
    }
}

impl<Scalar> PartialOrd for DiagonalTaxicabNorm<Scalar>
where
    Scalar: Clone + Ord + ConstZero + Mul<Output = Scalar> + Sub<Output = Scalar>,
{
    #[inline]
    fn partial_cmp(&self, oth: &Self) -> Option<Ordering> {
        Some(self.cmp(oth))
    }
}

impl<Scalar> Ord for DiagonalTaxicabNorm<Scalar>
where
    Scalar: Clone + Ord + ConstZero + Mul<Output = Scalar> + Sub<Output = Scalar>,
{
    fn cmp(&self, oth: &Self) -> Ordering {
        match (
            self.along_axis.cmp(&oth.along_axis),
            self.diagonal.cmp(&oth.diagonal),
        ) {
            (Ordering::Equal, x) | (x, Ordering::Equal) => x,
            (x, y) if x == y => x,
            (x, y) => {
                let aa = self.along_axis.clone() - oth.along_axis.clone();
                let dd = self.diagonal.clone() - oth.diagonal.clone();
                let aa_sq = aa.clone() * aa;
                let dg_sq = dd.clone() * dd;
                let dg_sq = dg_sq.clone() + dg_sq;
                if aa_sq >= dg_sq { x } else { y }
            }
        }
    }
}

impl<Scalar> AbsDiffEq for DiagonalTaxicabNorm<Scalar>
where
    Scalar: AbsDiffEq,
    Scalar::Epsilon: Clone,
{
    type Epsilon = Scalar::Epsilon;

    #[inline]
    fn default_epsilon() -> Self::Epsilon {
        Scalar::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        Scalar::abs_diff_eq(&self.along_axis, &other.along_axis, epsilon.clone())
            && Scalar::abs_diff_eq(&self.diagonal, &other.diagonal, epsilon)
    }
}

impl<Scalar: Zero> Zero for DiagonalTaxicabNorm<Scalar> {
    #[inline]
    fn zero() -> Self {
        Self {
            along_axis: Scalar::zero(),
            diagonal: Scalar::zero(),
        }
    }

    #[inline]
    fn is_zero(&self) -> bool {
        self.along_axis.is_zero() && self.diagonal.is_zero()
    }
}

impl<Scalar: ConstZero> ConstZero for DiagonalTaxicabNorm<Scalar> {
    const ZERO: Self = Self {
        along_axis: Scalar::ZERO,
        diagonal: Scalar::ZERO,
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diagonal_taxicab_norm_f64() {
        use approx::AbsDiffEq;
        assert!(
            f64::from(DiagonalTaxicabNorm::<f64>::new([8., 8.]).unwrap())
                .abs_diff_eq(&11.313708498984761, f64::EPSILON),
        );
        assert!(
            f64::from(DiagonalTaxicabNorm::<f64>::new([8., 10.343]).unwrap())
                .abs_diff_eq(&13.656708498984761, f64::EPSILON),
        );
        assert!(
            f64::from(DiagonalTaxicabNorm::<f64>::new([0., 16.]).unwrap())
                .abs_diff_eq(&16., f64::EPSILON),
        );
    }

    #[test]
    fn diagonal_taxicab_norm_i32() {
        assert_eq!(
            DiagonalTaxicabNorm::<i32>::new([8, 8]),
            Some(DiagonalTaxicabNorm {
                along_axis: 0,
                diagonal: 8,
            })
        );
        assert_eq!(
            DiagonalTaxicabNorm::<i32>::new([8, 10]),
            Some(DiagonalTaxicabNorm {
                along_axis: 2,
                diagonal: 8,
            })
        );
        assert!(
            DiagonalTaxicabNorm {
                along_axis: 0,
                diagonal: 8,
            } < DiagonalTaxicabNorm {
                along_axis: 2,
                diagonal: 8,
            }
        );
        assert_eq!(
            DiagonalTaxicabNorm::<i32>::new([10, 8]),
            Some(DiagonalTaxicabNorm {
                along_axis: 2,
                diagonal: 8,
            })
        );
        assert_eq!(
            DiagonalTaxicabNorm::<i32>::new([0, 16]),
            Some(DiagonalTaxicabNorm {
                along_axis: 16,
                diagonal: 0,
            })
        );
        assert!(
            DiagonalTaxicabNorm {
                along_axis: 2,
                diagonal: 8,
            } < DiagonalTaxicabNorm {
                along_axis: 16,
                diagonal: 0,
            }
        )
    }
}
