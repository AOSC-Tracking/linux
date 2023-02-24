// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]

//! Support for Apple RTKit coprocessors.
//!
//! C header: [`include/linux/soc/apple/rtkit.h`](../../../../include/linux/gpio/driver.h)

use crate::{
    bindings, device,
    error::{code::*, from_kernel_err_ptr, to_result, Error, Result},
    str::CStr,
    types::PointerWrapper,
    ScopeGuard,
};
use alloc::boxed::Box;

use core::marker::PhantomData;
use core::ptr;

use macros::vtable;

pub struct ShMem(bindings::apple_rtkit_shmem);

pub trait Buffer {
    fn iova(&self) -> Option<usize>;
    fn buf(&mut self) -> Option<&mut [u8]>;
}

#[vtable]
pub trait Operations {
    type Data: PointerWrapper + Send + Sync;
    type Buffer: Buffer;

    fn crashed(_data: <Self::Data as PointerWrapper>::Borrowed<'_>) {}

    fn recv_message(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _endpoint: u8,
        _message: u64,
    ) {
    }

    fn recv_message_early(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _endpoint: u8,
        _message: u64,
    ) -> bool {
        false
    }

    fn shmem_alloc(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _size: usize,
    ) -> Result<Self::Buffer> {
        Err(EINVAL)
    }

    fn shmem_map(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _iova: usize,
        _size: usize,
    ) -> Result<Self::Buffer> {
        Err(EINVAL)
    }
}

/// Represents `struct apple_rtkit *`.
///
/// # Invariants
///
/// The pointer is valid.
pub struct RTKit<T: Operations> {
    rtk: *mut bindings::apple_rtkit,
    data: *mut core::ffi::c_void,
    _p: PhantomData<T>,
}

unsafe extern "C" fn crashed_callback<T: Operations>(cookie: *mut core::ffi::c_void) {
    T::crashed(unsafe { T::Data::borrow(cookie) });
}

unsafe extern "C" fn recv_message_callback<T: Operations>(
    cookie: *mut core::ffi::c_void,
    endpoint: u8,
    message: u64,
) {
    T::recv_message(unsafe { T::Data::borrow(cookie) }, endpoint, message);
}

unsafe extern "C" fn recv_message_early_callback<T: Operations>(
    cookie: *mut core::ffi::c_void,
    endpoint: u8,
    message: u64,
) -> bool {
    T::recv_message_early(unsafe { T::Data::borrow(cookie) }, endpoint, message)
}

unsafe extern "C" fn shmem_setup_callback<T: Operations>(
    cookie: *mut core::ffi::c_void,
    bfr: *mut bindings::apple_rtkit_shmem,
) -> core::ffi::c_int {
    // SAFETY: Argument is a valid buffer
    let bfr_mut = unsafe { &mut *bfr };

    let buf = if bfr_mut.iova != 0 {
        bfr_mut.is_mapped = true;
        T::shmem_map(
            unsafe { T::Data::borrow(cookie) },
            bfr_mut.iova as usize,
            bfr_mut.size,
        )
    } else {
        bfr_mut.is_mapped = false;
        T::shmem_alloc(unsafe { T::Data::borrow(cookie) }, bfr_mut.size)
    };

    let mut buf = match buf {
        Err(e) => {
            return e.to_kernel_errno();
        }
        Ok(buf) => buf,
    };

    let iova = match buf.iova() {
        None => return EIO.to_kernel_errno(),
        Some(iova) => iova,
    };

    let slice = match buf.buf() {
        None => return ENOMEM.to_kernel_errno(),
        Some(slice) => slice,
    };

    if slice.len() < bfr_mut.size {
        return ENOMEM.to_kernel_errno();
    }

    bfr_mut.iova = iova as u64;
    bfr_mut.buffer = slice.as_mut_ptr() as *mut _;

    match Box::try_new(buf) {
        Err(e) => Error::from(e).to_kernel_errno(),
        Ok(boxed) => {
            bfr_mut.private = Box::leak(boxed) as *mut T::Buffer as *mut _;
            0
        }
    }
}

unsafe extern "C" fn shmem_destroy_callback<T: Operations>(
    _cookie: *mut core::ffi::c_void,
    bfr: *mut bindings::apple_rtkit_shmem,
) {
    let bfr_mut = unsafe { &mut *bfr };
    // Per shmem_setup_callback, this has to be a pointer to a Buffer if it is set
    if !bfr_mut.private.is_null() {
        unsafe {
            Box::from_raw(bfr_mut.private as *mut T::Buffer);
        }
        bfr_mut.private = core::ptr::null_mut();
    }
}

impl<T: Operations> RTKit<T> {
    const VTABLE: bindings::apple_rtkit_ops = bindings::apple_rtkit_ops {
        crashed: Some(crashed_callback::<T>),
        recv_message: Some(recv_message_callback::<T>),
        recv_message_early: Some(recv_message_early_callback::<T>),
        shmem_setup: if T::HAS_SHMEM_ALLOC || T::HAS_SHMEM_MAP {
            Some(shmem_setup_callback::<T>)
        } else {
            None
        },
        shmem_destroy: if T::HAS_SHMEM_ALLOC || T::HAS_SHMEM_MAP {
            Some(shmem_destroy_callback::<T>)
        } else {
            None
        },
    };

    pub fn new(
        dev: &dyn device::RawDevice,
        mbox_name: Option<&'static CStr>,
        mbox_idx: usize,
        data: T::Data,
    ) -> Result<Self> {
        let ptr = data.into_pointer() as *mut _;
        let guard = ScopeGuard::new(|| {
            // SAFETY: `ptr` came from a previous call to `into_pointer`.
            unsafe { T::Data::from_pointer(ptr) };
        });
        let rtk = unsafe {
            from_kernel_err_ptr(bindings::apple_rtkit_init(
                dev.raw_device(),
                ptr,
                match mbox_name {
                    Some(s) => s.as_char_ptr(),
                    None => ptr::null(),
                },
                mbox_idx.try_into()?,
                &Self::VTABLE,
            ))
        }?;

        guard.dismiss();
        Ok(Self {
            rtk,
            data: ptr,
            _p: PhantomData,
        })
    }

    pub fn boot(&mut self) -> Result {
        to_result(unsafe { bindings::apple_rtkit_boot(self.rtk) })
    }

    pub fn start_endpoint(&mut self, endpoint: u8) -> Result {
        to_result(unsafe { bindings::apple_rtkit_start_ep(self.rtk, endpoint) })
    }

    pub fn send_message(&mut self, endpoint: u8, message: u64) -> Result {
        to_result(unsafe {
            bindings::apple_rtkit_send_message(self.rtk, endpoint, message, ptr::null_mut(), false)
        })
    }
}

// SAFETY: `RTKit` operations require a mutable reference
unsafe impl<T: Operations> Sync for RTKit<T> {}

// SAFETY: `RTKit` operations require a mutable reference
unsafe impl<T: Operations> Send for RTKit<T> {}

impl<T: Operations> Drop for RTKit<T> {
    fn drop(&mut self) {
        // SAFETY: The pointer is valid by the type invariant.
        unsafe { bindings::apple_rtkit_free(self.rtk) };

        // Free context data.
        //
        // SAFETY: This matches the call to `into_pointer` from `new` in the success case.
        unsafe { T::Data::from_pointer(self.data) };
    }
}
