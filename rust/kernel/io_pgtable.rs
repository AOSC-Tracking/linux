// SPDX-License-Identifier: GPL-2.0
#![allow(missing_docs)]

//! IOMMU page tables
//!
//! C header: [`include/io-pgtable.h`](../../../../include/io-pgtable.h)

use crate::{
    bindings, device,
    error::{code::*, to_result, Result},
    types::PointerWrapper,
    ScopeGuard,
};

use core::marker::PhantomData;
use core::mem;
use core::ops::{Deref, DerefMut};

pub mod prot {
    pub const READ: u32 = bindings::IOMMU_READ;
    pub const WRITE: u32 = bindings::IOMMU_WRITE;
    pub const CACHE: u32 = bindings::IOMMU_CACHE;
    pub const NOEXEC: u32 = bindings::IOMMU_NOEXEC;
    pub const MMIO: u32 = bindings::IOMMU_MMIO;
    pub const PRIV: u32 = bindings::IOMMU_PRIV;
}

pub struct Config {
    pub quirks: usize,
    pub pgsize_bitmap: usize,
    pub ias: usize,
    pub oas: usize,
    pub coherent_walk: bool,
}

pub trait FlushOps {
    type Data: PointerWrapper + Send + Sync;

    fn tlb_flush_all(data: <Self::Data as PointerWrapper>::Borrowed<'_>);
    fn tlb_flush_walk(
        data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        iova: usize,
        size: usize,
        granule: usize,
    );
    // TODO: Implement the gather argument
    fn tlb_add_page(
        data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        iova: usize,
        granule: usize,
    );
}

/// An IOMMU page table
///
/// # Invariants
///
///   - [`self.ops`] is valid and non-null.
///   - [`self.cfg`] is valid and non-null.
pub struct IOPagetable<T: FlushOps, U> {
    ops: *mut bindings::io_pgtable_ops,
    cfg: bindings::io_pgtable_cfg,
    data: *mut core::ffi::c_void,
    _p: PhantomData<T>,
    _q: PhantomData<U>,
}

pub trait GetConfig<T: FlushOps> {
    fn cfg(iopt: &IOPagetable<T, Self>) -> Self
    where
        Self: Sized;
}

impl<T: FlushOps, U: GetConfig<T>> IOPagetable<T, U> {
    const FLUSH_OPS: bindings::iommu_flush_ops = bindings::iommu_flush_ops {
        tlb_flush_all: Some(tlb_flush_all_callback::<T>),
        tlb_flush_walk: Some(tlb_flush_walk_callback::<T>),
        tlb_add_page: Some(tlb_add_page_callback::<T>),
    };

    fn new_fmt(
        dev: &dyn device::RawDevice,
        format: u32,
        config: Config,
        data: T::Data,
    ) -> Result<IOPagetable<T, U>> {
        let ptr = data.into_pointer() as *mut _;
        let guard = ScopeGuard::new(|| {
            // SAFETY: `ptr` came from a previous call to `into_pointer`.
            unsafe { T::Data::from_pointer(ptr) };
        });

        let mut raw_cfg = bindings::io_pgtable_cfg {
            quirks: config.quirks.try_into()?,
            pgsize_bitmap: config.pgsize_bitmap.try_into()?,
            ias: config.ias.try_into()?,
            oas: config.oas.try_into()?,
            coherent_walk: config.coherent_walk,
            tlb: &Self::FLUSH_OPS,
            iommu_dev: dev.raw_device(),
            __bindgen_anon_1: unsafe { mem::zeroed() },
        };

        let ops = unsafe {
            bindings::alloc_io_pgtable_ops(format as bindings::io_pgtable_fmt, &mut raw_cfg, ptr)
        };

        if ops.is_null() {
            return Err(EINVAL);
        }

        guard.dismiss();
        Ok(IOPagetable {
            ops,
            cfg: raw_cfg,
            data: ptr,
            _p: PhantomData,
            _q: PhantomData,
        })
    }

    pub fn cfg(&self) -> U {
        <U as GetConfig<T>>::cfg(self)
    }

    pub fn map_pages(
        &mut self,
        iova: usize,
        paddr: usize,
        pgsize: usize,
        pgcount: usize,
        prot: u32,
    ) -> Result<usize> {
        let mut mapped: usize = 0;

        to_result(unsafe {
            (*self.ops).map_pages.unwrap()(
                self.ops,
                iova as u64,
                paddr as u64,
                pgsize,
                pgcount,
                prot as i32,
                bindings::GFP_KERNEL,
                &mut mapped,
            )
        })?;

        Ok(mapped)
    }

    pub fn unmap_pages(
        &mut self,
        iova: usize,
        pgsize: usize,
        pgcount: usize,
        // TODO: gather: *mut iommu_iotlb_gather,
    ) -> usize {
        unsafe {
            (*self.ops).unmap_pages.unwrap()(
                self.ops,
                iova as u64,
                pgsize,
                pgcount,
                core::ptr::null_mut(),
            )
        }
    }

    pub fn iova_to_phys(&mut self, iova: usize) -> u64 {
        unsafe { (*self.ops).iova_to_phys.unwrap()(self.ops, iova as u64) }
    }
}

impl<T: FlushOps, U> Drop for IOPagetable<T, U> {
    fn drop(&mut self) {
        // SAFETY: The pointer is valid by the type invariant.
        unsafe { bindings::free_io_pgtable_ops(self.ops) };

        // Free context data.
        //
        // SAFETY: This matches the call to `into_pointer` from `new` in the success case.
        unsafe { T::Data::from_pointer(self.data) };
    }
}

unsafe impl<T: FlushOps, U> Send for IOPagetable<T, U> {}
unsafe impl<T: FlushOps, U> Sync for IOPagetable<T, U> {}

unsafe extern "C" fn tlb_flush_all_callback<T: FlushOps>(cookie: *mut core::ffi::c_void) {
    T::tlb_flush_all(unsafe { T::Data::borrow(cookie) });
}

unsafe extern "C" fn tlb_flush_walk_callback<T: FlushOps>(
    iova: core::ffi::c_ulong,
    size: usize,
    granule: usize,
    cookie: *mut core::ffi::c_void,
) {
    T::tlb_flush_walk(
        unsafe { T::Data::borrow(cookie) },
        iova as usize,
        size,
        granule,
    );
}

unsafe extern "C" fn tlb_add_page_callback<T: FlushOps>(
    _gather: *mut bindings::iommu_iotlb_gather,
    iova: core::ffi::c_ulong,
    granule: usize,
    cookie: *mut core::ffi::c_void,
) {
    T::tlb_add_page(unsafe { T::Data::borrow(cookie) }, iova as usize, granule);
}

macro_rules! iopt_cfg {
    ($name:ident, $field:ident, $type:ident) => {
        pub type $name = bindings::$type;

        impl<T: FlushOps> GetConfig<T> for $name {
            fn cfg(iopt: &IOPagetable<T, $name>) -> $name {
                unsafe { iopt.cfg.__bindgen_anon_1.$field }
            }
        }
    };
}

impl<T: FlushOps> GetConfig<T> for () {
    fn cfg(_iopt: &IOPagetable<T, ()>) {}
}

macro_rules! iopt_type {
    ($type:ident, $cfg:ty, $fmt:ident) => {
        pub struct $type<T: FlushOps>(IOPagetable<T, $cfg>);

        impl<T: FlushOps> $type<T> {
            pub fn new(dev: &dyn device::RawDevice, config: Config, data: T::Data) -> Result<Self> {
                Ok(Self(IOPagetable::<T, $cfg>::new_fmt(
                    dev,
                    bindings::$fmt,
                    config,
                    data,
                )?))
            }
        }

        impl<T: FlushOps> Deref for $type<T> {
            type Target = IOPagetable<T, $cfg>;

            fn deref(&self) -> &Self::Target {
                &self.0
            }
        }

        impl<T: FlushOps> DerefMut for $type<T> {
            fn deref_mut(&mut self) -> &mut Self::Target {
                &mut self.0
            }
        }
    };
}

// Ew...
iopt_cfg!(
    ARMLPAES1Cfg,
    arm_lpae_s1_cfg,
    io_pgtable_cfg__bindgen_ty_1__bindgen_ty_1
);
iopt_cfg!(
    ARMLPAES2Cfg,
    arm_lpae_s2_cfg,
    io_pgtable_cfg__bindgen_ty_1__bindgen_ty_2
);
iopt_cfg!(
    ARMv7SCfg,
    arm_v7s_cfg,
    io_pgtable_cfg__bindgen_ty_1__bindgen_ty_3
);
iopt_cfg!(
    ARMMaliLPAECfg,
    arm_mali_lpae_cfg,
    io_pgtable_cfg__bindgen_ty_1__bindgen_ty_4
);
iopt_cfg!(
    AppleDARTCfg,
    apple_dart_cfg,
    io_pgtable_cfg__bindgen_ty_1__bindgen_ty_5
);
iopt_cfg!(
    AppleUATCfg,
    apple_uat_cfg,
    io_pgtable_cfg__bindgen_ty_1__bindgen_ty_6
);

iopt_type!(ARM32LPAES1, ARMLPAES1Cfg, io_pgtable_fmt_ARM_32_LPAE_S1);
iopt_type!(ARM32LPAES2, ARMLPAES2Cfg, io_pgtable_fmt_ARM_32_LPAE_S2);
iopt_type!(ARM64LPAES1, ARMLPAES1Cfg, io_pgtable_fmt_ARM_64_LPAE_S1);
iopt_type!(ARM64LPAES2, ARMLPAES2Cfg, io_pgtable_fmt_ARM_64_LPAE_S2);
iopt_type!(ARMv7S, ARMv7SCfg, io_pgtable_fmt_ARM_V7S);
iopt_type!(ARMMaliLPAE, ARMMaliLPAECfg, io_pgtable_fmt_ARM_MALI_LPAE);
iopt_type!(AMDIOMMUV1, (), io_pgtable_fmt_AMD_IOMMU_V1);
iopt_type!(AppleDART, AppleDARTCfg, io_pgtable_fmt_APPLE_DART);
iopt_type!(AppleDART2, AppleDARTCfg, io_pgtable_fmt_APPLE_DART2);
iopt_type!(AppleUAT, AppleUATCfg, io_pgtable_fmt_APPLE_UAT);
