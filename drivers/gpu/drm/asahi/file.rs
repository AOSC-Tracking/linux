// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(clippy::unusual_byte_groupings)]

//! Asahi File state

use crate::debug::*;
use crate::driver::AsahiDevice;
use crate::fw::types::*;
use crate::{alloc, buffer, driver, gem, gpu, mmu, render};
use kernel::drm::gem::BaseObject;
use kernel::prelude::*;
use kernel::sync::{smutex::Mutex, Arc};
use kernel::{bindings, drm};

const DEBUG_CLASS: DebugFlags = DebugFlags::File;

pub(crate) struct File {
    id: u64,
    renderer: Box<dyn render::Renderer>,
    dummy_obj: gem::ObjectRef,
    ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
    ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
    vm: mmu::Vm,
}

pub(crate) type DrmFile = drm::file::File<File>;

const VM_SHADER_START: u64 = 0x11_00000000;
const VM_SHADER_END: u64 = 0x11_ffffffff;
const VM_USER_START: u64 = 0x20_00000000;
const VM_USER_END: u64 = 0x5f_00000000;

const VM_DRV_GPU_START: u64 = 0x60_00000000;
const VM_DRV_GPU_END: u64 = 0x60_ffffffff;
const VM_DRV_GPUFW_START: u64 = 0x61_00000000;
const VM_DRV_GPUFW_END: u64 = 0x61_ffffffff;
const VM_UNK_PAGE: u64 = 0x6f_ffff8000;

impl drm::file::DriverFile for File {
    type Driver = driver::AsahiDriver;

    fn open(device: &AsahiDevice) -> Result<Box<Self>> {
        debug::update_debug_flags();

        let gpu = &device.data().gpu;
        let vm = gpu.new_vm()?;
        let id = gpu.ids().file.next();

        mod_dev_dbg!(device, "[File {}]: DRM device opened", id);
        mod_dev_dbg!(device, "[File {}]: Creating allocators", id);
        let ualloc = Arc::try_new(Mutex::new(alloc::DefaultAllocator::new(
            device,
            &vm,
            VM_DRV_GPU_START,
            VM_DRV_GPU_END,
            buffer::PAGE_SIZE,
            mmu::PROT_GPU_SHARED_RW,
            512 * 1024,
            true,
            fmt!("File {} GPU Shared", id),
        )?))?;
        let ualloc_priv = Arc::try_new(Mutex::new(alloc::DefaultAllocator::new(
            device,
            &vm,
            VM_DRV_GPUFW_START,
            VM_DRV_GPUFW_END,
            buffer::PAGE_SIZE,
            mmu::PROT_GPU_FW_PRIV_RW,
            64 * 1024,
            true,
            fmt!("File {} GPU FW Private", id),
        )?))?;

        let mut dummy_obj = gem::new_kernel_object(device, 0x4000)?;
        dummy_obj.vmap()?.as_mut_slice().fill(0);
        dummy_obj.map_at(&vm, VM_UNK_PAGE, mmu::PROT_GPU_SHARED_RW, true)?;

        mod_dev_dbg!(device, "[File {}]: Creating renderer", id);
        let renderer = device
            .data()
            .gpu
            .new_renderer(ualloc.clone(), ualloc_priv.clone())?;

        mod_dev_dbg!(device, "[File {}]: Opened successfully", id);
        Ok(Box::try_new(Self {
            id,
            vm,
            ualloc,
            ualloc_priv,
            dummy_obj,
            renderer,
        })?)
    }
}

macro_rules! param {
    ($name:ident) => {
        kernel::macros::concat_idents!(bindings::drm_asahi_param_DRM_ASAHI_PARAM_, $name)
    };
}

impl File {
    pub(crate) fn get_param(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_get_param,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(device, "[File {}]: IOCTL: get_param", file.inner().id);

        let gpu = &device.data().gpu;

        let value: u64 = match data.param {
            param!(UNSTABLE_UABI_VERSION) => bindings::DRM_ASAHI_UNSTABLE_UABI_VERSION as u64,
            param!(GPU_GENERATION) => gpu.get_dyncfg().id.gpu_gen as u32 as u64,
            param!(GPU_VARIANT) => gpu.get_dyncfg().id.gpu_variant as u32 as u64,
            param!(GPU_REVISION) => gpu.get_dyncfg().id.gpu_rev as u32 as u64,
            param!(CHIP_ID) => gpu.get_cfg().chip_id as u64,
            param!(FEAT_COMPAT) => gpu.get_cfg().gpu_feat_compat as u64,
            param!(FEAT_INCOMPAT) => gpu.get_cfg().gpu_feat_incompat as u64,
            param!(VM_USER_START) => VM_USER_START,
            param!(VM_USER_END) => VM_USER_END,
            param!(VM_SHADER_START) => VM_SHADER_START,
            param!(VM_SHADER_END) => VM_SHADER_END,
            _ => return Err(EINVAL),
        };

        data.value = value;

        Ok(0)
    }

    pub(crate) fn submit(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_submit,
        file: &DrmFile,
    ) -> Result<u32> {
        debug::update_debug_flags();

        let gpu = &device.data().gpu;
        gpu.update_globals();

        let id = gpu.ids().submission.next();
        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: submit (submission ID: {})",
            file.inner().id,
            id
        );
        let inner = file.inner();
        let ret = inner.renderer.render(&inner.vm, &inner.ualloc, data, id);
        if let Err(e) = ret {
            dev_info!(
                device,
                "[File {}]: IOCTL: submit failed! (submission ID: {} err: {:?})",
                file.inner().id,
                id,
                e
            );
            Err(e)
        } else {
            Ok(0)
        }
    }

    pub(crate) fn wait(
        device: &AsahiDevice,
        _data: &mut bindings::drm_asahi_wait,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(device, "[File {}]: IOCTL: wait", file.inner().id);
        Ok(0)
    }

    pub(crate) fn create_bo(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_create_bo,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: create_bo size={:#x?}",
            file.inner().id,
            data.size
        );

        let mut bo = gem::new_object(device, data.size as usize, data.flags)?;

        if data.flags & bindings::ASAHI_BO_PIPELINE != 0 {
            let iova = bo.map_into_range(
                &file.inner().vm,
                VM_SHADER_START,
                VM_SHADER_END,
                mmu::UAT_PGSZ as u64,
                mmu::PROT_GPU_SHARED_RW,
                true,
            )?;
            data.offset = iova as u64 - VM_SHADER_START;
        } else {
            let iova = bo.map_into_range(
                &file.inner().vm,
                VM_USER_START,
                VM_USER_END,
                mmu::UAT_PGSZ as u64,
                mmu::PROT_GPU_SHARED_RW,
                true,
            )?;
            data.offset = iova as u64;
        }
        let handle = bo.gem.create_handle(file)?;
        data.handle = handle;

        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: create_bo size={:#x} offset={:#x?} handle={:#x?}",
            file.inner().id,
            data.size,
            data.offset,
            data.handle
        );

        Ok(0)
    }

    pub(crate) fn mmap_bo(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_mmap_bo,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: mmap_bo handle={:#x?}",
            file.inner().id,
            data.handle
        );

        let bo = gem::Object::lookup_handle(file, data.handle)?;

        data.offset = bo.create_mmap_offset()?;
        Ok(0)
    }

    pub(crate) fn get_bo_offset(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_get_bo_offset,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: get_bo_offset handle={:#x?}",
            file.inner().id,
            data.handle
        );

        let mut bo = gem::ObjectRef::new(gem::Object::lookup_handle(file, data.handle)?);

        // This can race other threads. Only one will win the map and the
        // others will return EBUSY.
        let iova = bo.map_into_range(
            &file.inner().vm,
            VM_USER_START,
            VM_USER_END,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_SHARED_RW,
            true,
        );

        if let Some(iova) = bo.iova(file.inner().vm.id()) {
            // If we have a mapping, call it good.
            data.offset = iova as u64;
            mod_dev_dbg!(
                device,
                "[File {}]: IOCTL: get_bo_offset handle={:#x?} offset={:#x?}",
                file.inner().id,
                data.handle,
                iova
            );
            Ok(0)
        } else {
            // Otherwise return the error, or a generic one if something
            // went very wrong and we lost the mapping.
            dev_err!(
                device,
                "[File {}]: IOCTL: get_bo_offset failed",
                file.inner().id
            );
            iova?;
            Err(EIO)
        }
    }

    pub(crate) fn vm_id(&self) -> u64 {
        self.vm.id()
    }
}

impl Drop for File {
    fn drop(&mut self) {
        mod_pr_debug!("[File {}]: Closing...", self.id);
    }
}
