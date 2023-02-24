// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Common types for firmware structure definitions

use crate::{alloc, object};
pub(crate) use kernel::macros::versions;

pub(crate) use crate::f32;
pub(crate) use crate::float::F32;
pub(crate) use crate::object::{GpuPointer, GpuStruct, GpuWeakPointer};
pub(crate) use ::alloc::boxed::Box;
use core::fmt;
use core::ops::{Deref, DerefMut, Index, IndexMut};
use core::sync::atomic;
pub(crate) use core::sync::atomic::{
    AtomicI16, AtomicI32, AtomicI64, AtomicI8, AtomicU16, AtomicU32, AtomicU64, AtomicU8,
};
pub(crate) type GpuObject<T> =
    object::GpuObject<T, alloc::GenericAlloc<T, alloc::DefaultAllocation>>;
pub(crate) type GpuArray<T> = object::GpuArray<T, alloc::GenericAlloc<T, alloc::DefaultAllocation>>;
pub(crate) type GpuOnlyArray<T> =
    object::GpuOnlyArray<T, alloc::GenericAlloc<T, alloc::DefaultAllocation>>;
pub(crate) use crate::alloc::Allocator as _Allocator;
pub(crate) use crate::event::EventValue;
pub(crate) type Allocator = alloc::DefaultAllocator;
pub(crate) use core::fmt::Debug;
pub(crate) use core::marker::PhantomData;

#[derive(Debug, Default)]
pub(crate) struct Stamp(pub(crate) AtomicU32);

#[derive(Debug, Default)]
pub(crate) struct FwStamp(pub(crate) AtomicU32);

#[derive(Copy, Clone, Default)]
#[repr(C, packed(4))]
pub(crate) struct U64(pub(crate) u64);

unsafe impl Zeroed for U64 {}

impl fmt::Debug for U64 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let v = self.0;
        f.write_fmt(format_args!("{:#x}", v))
    }
}

#[macro_export]
macro_rules! no_debug {
    ($type:ty) => {
        impl Debug for $type {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(f, "...")
            }
        }
    };
}

/// Types which can be safely initialized with an all-zero bit pattern
/// See: https://github.com/rust-lang/rfcs/issues/2626
///
/// # Safety
/// This trait must only be implemented if a type only contains primitive types
/// which can be zero-initialized, FFI structs, intended to be zero-initialized,
/// or other types which impl Zeroed.
pub(crate) unsafe trait Zeroed: Default {
    fn zeroed() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

#[macro_export]
macro_rules! default_zeroed {
    (<$($lt:lifetime),*>, $type:ty) => {
        impl<$($lt),*> Default for $type {
            fn default() -> $type {
                Zeroed::zeroed()
            }
        }
        unsafe impl<$($lt),*> Zeroed for $type {}
    };
    ($type:ty) => {
        impl Default for $type {
            fn default() -> $type {
                Zeroed::zeroed()
            }
        }
        unsafe impl Zeroed for $type {}
    };
}

#[derive(Copy, Clone)]
#[repr(C, packed)]
pub(crate) struct Pad<const N: usize>([u8; N]);

impl<const N: usize> Default for Pad<N> {
    fn default() -> Self {
        Pad([0; N])
    }
}

impl<const N: usize> fmt::Debug for Pad<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_fmt(format_args!("<pad>"))
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub(crate) struct Array<const N: usize, T>([T; N]);

impl<const N: usize, T> Array<N, T> {
    pub(crate) fn new(data: [T; N]) -> Self {
        Self(data)
    }
}

unsafe impl<const N: usize, T: Zeroed> Zeroed for Array<N, T> {}

impl<const N: usize, T: Default> Default for Array<N, T> {
    fn default() -> Self {
        Self(core::array::from_fn(|_i| Default::default()))
    }
}

impl<const N: usize, T> Index<usize> for Array<N, T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl<const N: usize, T> IndexMut<usize> for Array<N, T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.0[index]
    }
}

impl<const N: usize, T> Deref for Array<N, T> {
    type Target = [T; N];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const N: usize, T> DerefMut for Array<N, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const N: usize, T: Sized + fmt::Debug> fmt::Debug for Array<N, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

#[macro_export]
macro_rules! trivial_gpustruct {
    ($type:ident) => {
        #[derive(Debug, Default)]
        pub(crate) struct $type {}

        impl GpuStruct for $type {
            type Raw<'a> = raw::$type;
        }
    };
}

/*
#[derive(Debug, Default)]
#[repr(transparent)]
struct Atomic<T>(T);

impl<T: Sized> Deref for Array<N, T> {
    type Target = [T; N];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const N: usize, T: Sized> DerefMut for Array<N, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
*/
