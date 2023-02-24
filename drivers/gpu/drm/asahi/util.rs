// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]

//! Utility functions

use core::ops::{Add, BitAnd, Div, Not, Sub};

pub(crate) fn align<T>(a: T, b: T) -> T
where
    T: Copy,
    T: Default,
    T: BitAnd<Output = T>,
    T: Not<Output = T>,
    T: Add<Output = T>,
    T: Sub<Output = T>,
    T: Div<Output = T>,
{
    let def: T = Default::default();
    #[allow(clippy::eq_op)]
    let one: T = !def / !def;

    (a + b - one) & !(b - one)
}

pub(crate) fn div_ceil<T>(a: T, b: T) -> T
where
    T: Copy,
    T: Default,
    T: BitAnd<Output = T>,
    T: Not<Output = T>,
    T: Add<Output = T>,
    T: Sub<Output = T>,
    T: Div<Output = T>,
{
    let def: T = Default::default();
    #[allow(clippy::eq_op)]
    let one: T = !def / !def;

    (a + b - one) / b
}
