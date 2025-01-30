// SPDX-License-Identifier: GPL-2.0

//! Generic memory-mapped IO.

use core::ops::Deref;

use crate::device::Device;
use crate::devres::Devres;
use crate::io::resource::Region;
use crate::io::resource::Resource;
use crate::io::Io;
use crate::io::IoRaw;
use crate::prelude::*;

/// An exclusive memory-mapped IO region.
///
/// # Invariants
///
/// - ExclusiveIoMem has exclusive access to the underlying `iomem`.
pub struct ExclusiveIoMem<const SIZE: usize> {
    /// The region abstraction. This represents exclusive access to the
    /// range represented by the underlying `iomem`.
    ///
    /// It's placed first to ensure that the region is released before it is
    /// unmapped as a result of the drop order.
    #[allow(dead_code)]
    region: Region,
    /// The underlying `IoMem` instance.
    iomem: IoMem<SIZE>,
}

impl<const SIZE: usize> ExclusiveIoMem<SIZE> {
    /// Creates a new `ExclusiveIoMem` instance.
    pub(crate) fn ioremap(resource: &Resource) -> Result<Self> {
        let iomem = IoMem::ioremap(resource)?;

        let start = resource.start();
        let size = resource.size();
        let name = resource.name();

        let region = resource
            .request_mem_region(start, size, name)
            .ok_or(EBUSY)?;

        let iomem = ExclusiveIoMem { iomem, region };

        Ok(iomem)
    }

    pub(crate) fn new(resource: &Resource, device: &Device) -> Result<Devres<Self>> {
        let iomem = Self::ioremap(resource)?;
        let devres = Devres::new(device, iomem, GFP_KERNEL)?;

        Ok(devres)
    }
}

impl<const SIZE: usize> Deref for ExclusiveIoMem<SIZE> {
    type Target = Io<SIZE>;

    fn deref(&self) -> &Self::Target {
        &*self.iomem
    }
}

/// A generic memory-mapped IO region.
///
/// Accesses to the underlying region is checked either at compile time, if the
/// region's size is known at that point, or at runtime otherwise.
///
/// # Invariants
///
/// `IoMem` always holds an `IoRaw` inststance that holds a valid pointer to the
/// start of the I/O memory mapped region.
pub struct IoMem<const SIZE: usize = 0> {
    io: IoRaw<SIZE>,
}

impl<const SIZE: usize> IoMem<SIZE> {
    fn ioremap(resource: &Resource) -> Result<Self> {
        let size = resource.size();
        if size == 0 {
            return Err(EINVAL);
        }

        let res_start = resource.start();

        // SAFETY:
        // - `res_start` and `size` are read from a presumably valid `struct resource`.
        // - `size` is known not to be zero at this point.
        let addr = unsafe { bindings::ioremap(res_start, size as kernel::ffi::c_ulong) };
        if addr.is_null() {
            return Err(ENOMEM);
        }

        let io = IoRaw::new(addr as usize, size as usize)?;
        let io = IoMem { io };

        Ok(io)
    }

    /// Creates a new `IoMem` instance.
    pub(crate) fn new(resource: &Resource, device: &Device) -> Result<Devres<Self>> {
        let io = Self::ioremap(resource)?;
        let devres = Devres::new(device, io, GFP_KERNEL)?;

        Ok(devres)
    }
}

impl<const SIZE: usize> Drop for IoMem<SIZE> {
    fn drop(&mut self) {
        // SAFETY: Safe as by the invariant of `Io`.
        unsafe { bindings::iounmap(self.io.addr() as *mut core::ffi::c_void) }
    }
}

impl<const SIZE: usize> Deref for IoMem<SIZE> {
    type Target = Io<SIZE>;

    fn deref(&self) -> &Self::Target {
        // SAFETY: Safe as by the invariant of `IoMem`.
        unsafe { Io::from_raw(&self.io) }
    }
}
