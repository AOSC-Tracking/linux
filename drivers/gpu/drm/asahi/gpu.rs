// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(dead_code)]

use core::any::Any;
use core::sync::atomic::{AtomicU64, Ordering};
use core::time::Duration;

use kernel::{
    delay::coarse_sleep,
    macros::versions,
    prelude::*,
    soc::apple::rtkit,
    sync::{smutex::Mutex, Arc, Guard, UniqueArc},
    time, PointerWrapper,
};

use crate::box_in_place;
use crate::debug::*;
use crate::driver::AsahiDevice;
use crate::fw::channels::{DeviceControlMsg, FwCtlMsg, PipeType};
use crate::{
    alloc, buffer, channel, event, fw, gem, hw, initdata, mem, mmu, regs, render, workqueue,
};

const DEBUG_CLASS: DebugFlags = DebugFlags::Gpu;

const EP_FIRMWARE: u8 = 0x20;
const EP_DOORBELL: u8 = 0x21;

const MSG_INIT: u64 = 0x81 << 48;
const INIT_DATA_MASK: u64 = (1 << 44) - 1;

const MSG_TX_DOORBELL: u64 = 0x83 << 48;
const MSG_FWCTL: u64 = 0x84 << 48;
const MSG_HALT: u64 = 0x85 << 48;

const MSG_RX_DOORBELL: u64 = 0x42 << 48;

const DOORBELL_KICKFW: u64 = 0x10;
const DOORBELL_DEVCTRL: u64 = 0x11;

const IOVA_KERN_PRIV_BASE: u64 = 0xffffffa000000000;
const IOVA_KERN_PRIV_TOP: u64 = 0xffffffa7ffffffff;
const IOVA_KERN_SHARED_BASE: u64 = 0xffffffa800000000;
const IOVA_KERN_SHARED_TOP: u64 = 0xffffffa9ffffffff;
const IOVA_KERN_SHARED_RO_BASE: u64 = 0xffffffaa00000000;
const IOVA_KERN_SHARED_RO_TOP: u64 = 0xffffffabffffffff;
const IOVA_KERN_GPU_BASE: u64 = 0xffffffaf00000000;
const IOVA_KERN_GPU_TOP: u64 = 0xffffffafffffffff;

const HALT_ENTER_TIMEOUT_MS: u64 = 100;

pub(crate) struct KernelAllocators {
    pub(crate) private: alloc::DefaultAllocator,
    pub(crate) shared: alloc::DefaultAllocator,
    pub(crate) shared_ro: alloc::DefaultAllocator,
    pub(crate) gpu: alloc::DefaultAllocator,
}

#[versions(AGX)]
struct RxChannels {
    event: channel::EventChannel,
    fw_log: channel::FwLogChannel,
    ktrace: channel::KTraceChannel,
    stats: channel::StatsChannel::ver,
}

struct PipeChannels {
    pub(crate) vtx: Vec<Mutex<channel::PipeChannel>>,
    pub(crate) frag: Vec<Mutex<channel::PipeChannel>>,
    pub(crate) comp: Vec<Mutex<channel::PipeChannel>>,
}

struct TxChannels {
    pub(crate) device_control: channel::DeviceControlChannel,
}

const NUM_PIPES: usize = 4;

pub(crate) struct ID(AtomicU64);

impl ID {
    pub(crate) fn new(val: u64) -> ID {
        ID(AtomicU64::new(val))
    }

    pub(crate) fn next(&self) -> u64 {
        self.0.fetch_add(1, Ordering::Relaxed)
    }
}

impl Default for ID {
    fn default() -> Self {
        ID(AtomicU64::new(2))
    }
}

#[derive(Default)]
pub(crate) struct SequenceIDs {
    pub(crate) file: ID,
    pub(crate) vm: ID,
    pub(crate) buf: ID,
    pub(crate) submission: ID,
    pub(crate) renderer: ID,
}

#[versions(AGX)]
pub(crate) struct GpuManager {
    dev: AsahiDevice,
    cfg: &'static hw::HwConfig,
    dyncfg: Box<hw::DynConfig>,
    initialized: bool,
    pub(crate) initdata: Box<fw::types::GpuObject<fw::initdata::InitData::ver>>,
    uat: Box<mmu::Uat>,
    alloc: Mutex<KernelAllocators>,
    io_mappings: Vec<mmu::Mapping>,
    rtkit: Mutex<Option<Box<rtkit::RTKit<GpuManager::ver>>>>,
    rx_channels: Mutex<Box<RxChannels::ver>>,
    tx_channels: Mutex<Box<TxChannels>>,
    fwctl_channel: Mutex<Box<channel::FwCtlChannel>>,
    pipes: PipeChannels,
    event_manager: Arc<event::EventManager>,
    buffer_mgr: buffer::BufferManager,
    ids: SequenceIDs,
}

pub(crate) trait GpuManager: Send + Sync {
    fn as_any(&self) -> &dyn Any;
    fn init(&self) -> Result;
    fn update_globals(&self);
    fn alloc(&self) -> Guard<'_, Mutex<KernelAllocators>>;
    fn new_vm(&self) -> Result<mmu::Vm>;
    fn bind_vm(&self, vm: &mmu::Vm) -> Result<mmu::VmBind>;
    fn new_renderer(
        &self,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
    ) -> Result<Box<dyn render::Renderer>>;
    fn submit_batch(&self, batch: workqueue::WorkQueueBatch<'_>) -> Result;
    fn ids(&self) -> &SequenceIDs;
    fn kick_firmware(&self) -> Result;
    fn invalidate_context(
        &self,
        context: &fw::types::GpuObject<fw::workqueue::GpuContextData>,
    ) -> Result;
    fn flush_fw_cache(&self) -> Result;
    fn handle_timeout(&self, counter: u32, event_slot: u32);
    fn handle_fault(&self);
    fn wait_for_poweroff(&self, timeout: usize) -> Result;
    fn fwctl(&self, msg: FwCtlMsg) -> Result;
    fn get_cfg(&self) -> &'static hw::HwConfig;
    fn get_dyncfg(&self) -> &hw::DynConfig;
}

#[versions(AGX)]
#[vtable]
impl rtkit::Operations for GpuManager::ver {
    type Data = Arc<GpuManager::ver>;
    type Buffer = gem::ObjectRef;

    fn recv_message(data: <Self::Data as PointerWrapper>::Borrowed<'_>, ep: u8, msg: u64) {
        let dev = &data.dev;
        //dev_info!(dev, "RTKit message: {:#x}:{:#x}\n", ep, msg);

        if ep != EP_FIRMWARE || msg != MSG_RX_DOORBELL {
            dev_err!(dev, "Unknown message: {:#x}:{:#x}\n", ep, msg);
            return;
        }

        let mut ch = data.rx_channels.lock();

        ch.fw_log.poll();
        ch.ktrace.poll();
        ch.stats.poll();
        ch.event.poll();
    }

    fn shmem_alloc(
        data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        size: usize,
    ) -> Result<Self::Buffer> {
        let dev = &data.dev;
        mod_dev_dbg!(dev, "shmem_alloc() {:#x} bytes\n", size);

        let mut obj = gem::new_kernel_object(dev, size)?;
        obj.vmap()?;
        let iova = obj.map_into(data.uat.kernel_vm())?;
        mod_dev_dbg!(dev, "shmem_alloc() -> VA {:#x}\n", iova);
        Ok(obj)
    }
}

#[versions(AGX)]
impl GpuManager::ver {
    #[inline(never)]
    pub(crate) fn new(
        dev: &AsahiDevice,
        res: &regs::Resources,
        cfg: &'static hw::HwConfig,
    ) -> Result<Arc<GpuManager::ver>> {
        let uat = Self::make_uat(dev, cfg)?;
        let dyncfg = Self::make_dyncfg(dev, res, cfg, &uat)?;

        let mut alloc = KernelAllocators {
            private: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                IOVA_KERN_PRIV_BASE,
                IOVA_KERN_PRIV_TOP,
                0x80,
                mmu::PROT_FW_PRIV_RW,
                1024 * 1024,
                true,
                fmt!("Kernel Private"),
            )?,
            shared: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                IOVA_KERN_SHARED_BASE,
                IOVA_KERN_SHARED_TOP,
                0x80,
                mmu::PROT_FW_SHARED_RW,
                1024 * 1024,
                true,
                fmt!("Kernel Shared"),
            )?,
            shared_ro: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                IOVA_KERN_SHARED_RO_BASE,
                IOVA_KERN_SHARED_RO_TOP,
                0x80,
                mmu::PROT_FW_SHARED_RO,
                64 * 1024,
                true,
                fmt!("Kernel RO Shared"),
            )?,
            gpu: alloc::DefaultAllocator::new(
                dev,
                uat.kernel_vm(),
                IOVA_KERN_GPU_BASE,
                IOVA_KERN_GPU_TOP,
                0x80,
                mmu::PROT_GPU_FW_SHARED_RW,
                64 * 1024,
                true,
                fmt!("Kernel GPU Shared"),
            )?,
        };

        let event_manager = Self::make_event_manager(&mut alloc)?;
        let initdata = Self::make_initdata(cfg, &dyncfg, &mut alloc)?;
        let mut mgr = Self::make_mgr(dev, cfg, dyncfg, uat, alloc, event_manager, initdata)?;

        {
            let fwctl = mgr.fwctl_channel.lock();
            let p_fwctl = fwctl.to_raw();
            core::mem::drop(fwctl);

            mgr.initdata.fw_status.with_mut(|raw, _inner| {
                raw.fwctl_channel = p_fwctl;
            });
        }

        {
            let txc = mgr.tx_channels.lock();
            let p_device_control = txc.device_control.to_raw();
            core::mem::drop(txc);

            let rxc = mgr.rx_channels.lock();
            let p_event = rxc.event.to_raw();
            let p_fw_log = rxc.fw_log.to_raw();
            let p_ktrace = rxc.ktrace.to_raw();
            let p_stats = rxc.stats.to_raw();
            let p_fwlog_buf = rxc.fw_log.get_buf();
            core::mem::drop(rxc);

            mgr.initdata.runtime_pointers.with_mut(|raw, _inner| {
                raw.device_control = p_device_control;
                raw.event = p_event;
                raw.fw_log = p_fw_log;
                raw.ktrace = p_ktrace;
                raw.stats = p_stats;
                raw.fwlog_buf = Some(p_fwlog_buf);
            });
        }

        let mut p_pipes: Vec<fw::initdata::raw::PipeChannels> = Vec::new();

        for ((v, f), c) in mgr
            .pipes
            .vtx
            .iter()
            .zip(&mgr.pipes.frag)
            .zip(&mgr.pipes.comp)
        {
            p_pipes.try_push(fw::initdata::raw::PipeChannels {
                vtx: v.lock().to_raw(),
                frag: f.lock().to_raw(),
                comp: c.lock().to_raw(),
            })?;
        }

        mgr.initdata.runtime_pointers.with_mut(|raw, _inner| {
            for (i, p) in p_pipes.into_iter().enumerate() {
                raw.pipes[i].vtx = p.vtx;
                raw.pipes[i].frag = p.frag;
                raw.pipes[i].comp = p.comp;
            }
        });

        for (i, map) in cfg.io_mappings.iter().enumerate() {
            if let Some(map) = map.as_ref() {
                mgr.iomap(i, map)?;
            }
        }

        let mgr = Arc::from(mgr);

        let rtkit = Box::try_new(rtkit::RTKit::<GpuManager::ver>::new(
            dev,
            None,
            0,
            mgr.clone(),
        )?)?;

        *mgr.rtkit.lock() = Some(rtkit);

        {
            let mut rxc = mgr.rx_channels.lock();
            rxc.event.set_manager(mgr.clone());
        }

        Ok(mgr)
    }

    fn make_initdata(
        cfg: &'static hw::HwConfig,
        dyncfg: &hw::DynConfig,
        alloc: &mut KernelAllocators,
    ) -> Result<Box<fw::types::GpuObject<fw::initdata::InitData::ver>>> {
        let mut builder = initdata::InitDataBuilder::ver::new(alloc, cfg, dyncfg);
        builder.build()
    }

    #[inline(never)]
    fn make_uat(dev: &AsahiDevice, cfg: &'static hw::HwConfig) -> Result<Box<mmu::Uat>> {
        Ok(Box::try_new(mmu::Uat::new(dev, cfg)?)?)
    }

    #[inline(never)]
    fn make_mgr(
        dev: &AsahiDevice,
        cfg: &'static hw::HwConfig,
        dyncfg: Box<hw::DynConfig>,
        uat: Box<mmu::Uat>,
        mut alloc: KernelAllocators,
        event_manager: Arc<event::EventManager>,
        initdata: Box<fw::types::GpuObject<fw::initdata::InitData::ver>>,
    ) -> Result<UniqueArc<GpuManager::ver>> {
        let mut pipes = PipeChannels {
            vtx: Vec::new(),
            frag: Vec::new(),
            comp: Vec::new(),
        };

        for _i in 0..=NUM_PIPES - 1 {
            pipes
                .vtx
                .try_push(Mutex::new(channel::PipeChannel::new(dev, &mut alloc)?))?;
            pipes
                .frag
                .try_push(Mutex::new(channel::PipeChannel::new(dev, &mut alloc)?))?;
            pipes
                .comp
                .try_push(Mutex::new(channel::PipeChannel::new(dev, &mut alloc)?))?;
        }

        UniqueArc::try_new(GpuManager::ver {
            dev: dev.clone(),
            cfg,
            dyncfg,
            initialized: false,
            initdata,
            uat,
            io_mappings: Vec::new(),
            rtkit: Mutex::new(None),
            rx_channels: Mutex::new(box_in_place!(RxChannels::ver {
                event: channel::EventChannel::new(dev, &mut alloc, event_manager.clone())?,
                fw_log: channel::FwLogChannel::new(dev, &mut alloc)?,
                ktrace: channel::KTraceChannel::new(dev, &mut alloc)?,
                stats: channel::StatsChannel::ver::new(dev, &mut alloc)?,
            })?),
            tx_channels: Mutex::new(Box::try_new(TxChannels {
                device_control: channel::DeviceControlChannel::new(dev, &mut alloc)?,
            })?),
            fwctl_channel: Mutex::new(Box::try_new(channel::FwCtlChannel::new(dev, &mut alloc)?)?),
            pipes,
            event_manager,
            buffer_mgr: buffer::BufferManager::new()?,
            alloc: Mutex::new(alloc),
            ids: Default::default(),
        })
    }

    #[inline(never)]
    fn make_dyncfg(
        dev: &AsahiDevice,
        res: &regs::Resources,
        cfg: &'static hw::HwConfig,
        uat: &mmu::Uat,
    ) -> Result<Box<hw::DynConfig>> {
        let gpu_id = res.get_gpu_id()?;

        dev_info!(dev, "GPU Information:\n");
        dev_info!(
            dev,
            "  Type: {:?}{:?}\n",
            gpu_id.gpu_gen,
            gpu_id.gpu_variant
        );
        dev_info!(dev, "  Max dies: {}\n", gpu_id.max_dies);
        dev_info!(dev, "  Clusters: {}\n", gpu_id.num_clusters);
        dev_info!(
            dev,
            "  Cores: {} ({})\n",
            gpu_id.num_cores,
            gpu_id.num_cores * gpu_id.num_clusters
        );
        dev_info!(
            dev,
            "  Frags: {} ({})\n",
            gpu_id.num_frags,
            gpu_id.num_frags * gpu_id.num_clusters
        );
        dev_info!(
            dev,
            "  GPs: {} ({})\n",
            gpu_id.num_gps,
            gpu_id.num_gps * gpu_id.num_clusters
        );
        dev_info!(dev, "  Core masks: {:#x?}\n", gpu_id.core_masks);
        dev_info!(dev, "  Active cores: {}\n", gpu_id.num_active_cores);

        dev_info!(dev, "Getting configuration from device tree...\n");
        let pwr_cfg = hw::PwrConfig::load(dev, cfg)?;
        dev_info!(dev, "Dynamic configuration fetched\n");

        if gpu_id.gpu_gen != cfg.gpu_gen || gpu_id.gpu_variant != cfg.gpu_variant {
            dev_err!(
                dev,
                "GPU type mismatch (expected {:?}{:?}, found {:?}{:?})\n",
                cfg.gpu_gen,
                cfg.gpu_variant,
                gpu_id.gpu_gen,
                gpu_id.gpu_variant
            );
            return Err(EIO);
        }
        if gpu_id.num_clusters > cfg.max_num_clusters {
            dev_err!(
                dev,
                "Too many clusters ({} > {})\n",
                gpu_id.num_clusters,
                cfg.max_num_clusters
            );
            return Err(EIO);
        }
        if gpu_id.num_cores > cfg.max_num_cores {
            dev_err!(
                dev,
                "Too many cores ({} > {})\n",
                gpu_id.num_cores,
                cfg.max_num_cores
            );
            return Err(EIO);
        }
        if gpu_id.num_frags > cfg.max_num_frags {
            dev_err!(
                dev,
                "Too many frags ({} > {})\n",
                gpu_id.num_frags,
                cfg.max_num_frags
            );
            return Err(EIO);
        }
        if gpu_id.num_gps > cfg.max_num_gps {
            dev_err!(
                dev,
                "Too many GPs ({} > {})\n",
                gpu_id.num_gps,
                cfg.max_num_gps
            );
            return Err(EIO);
        }

        Ok(Box::try_new(hw::DynConfig {
            pwr: pwr_cfg,
            uat_ttb_base: uat.ttb_base(),
            id: gpu_id,
        })?)
    }

    fn make_event_manager(alloc: &mut KernelAllocators) -> Result<Arc<event::EventManager>> {
        Arc::try_new(event::EventManager::new(alloc)?)
    }

    fn iomap(&mut self, index: usize, map: &hw::IOMapping) -> Result {
        let off = map.base & mmu::UAT_PGMSK;
        let base = map.base - off;
        let end = (map.base + map.size + mmu::UAT_PGMSK) & !mmu::UAT_PGMSK;
        let mapping = self
            .uat
            .kernel_vm()
            .map_io(base, end - base, map.writable)?;

        self.initdata.runtime_pointers.hwdata_b.with_mut(|raw, _| {
            raw.io_mappings[index] = fw::initdata::raw::IOMapping {
                phys_addr: map.base as u64,
                virt_addr: (mapping.iova() + off) as u64,
                size: map.size as u32,
                range_size: map.range_size as u32,
                readwrite: map.writable as u64,
            };
        });

        self.io_mappings.try_push(mapping)?;
        Ok(())
    }

    fn mark_pending_events(&self, culprit_slot: Option<u32>, error: workqueue::BatchError) {
        dev_err!(self.dev, "  Pending events:\n");

        self.initdata.globals.with(|raw, _inner| {
            for i in raw.pending_stamps.iter() {
                let info = i.info.load(Ordering::Relaxed);
                let wait_value = i.wait_value.load(Ordering::Relaxed);

                if info & 1 != 0 {
                    let slot = info >> 3;
                    let flags = info & 0x7;
                    dev_err!(
                        self.dev,
                        "    [{}] flags={} value={:#x}\n",
                        slot,
                        flags,
                        wait_value
                    );
                    let error = if culprit_slot.is_some() && culprit_slot != Some(slot) {
                        workqueue::BatchError::Killed
                    } else {
                        error
                    };
                    self.event_manager.mark_error(slot, wait_value, error);
                    i.info.store(0, Ordering::Relaxed);
                    i.wait_value.store(0, Ordering::Relaxed);
                }
            }
        });
    }

    fn get_fault_info(&self) -> Option<regs::FaultInfo> {
        let data = self.dev.data();

        let res = match data.resources() {
            Some(res) => res,
            None => {
                dev_err!(self.dev, "  Failed to acquire resources\n");
                return None;
            }
        };

        let info = res.get_fault_info();
        if info.is_some() {
            dev_err!(self.dev, "  Fault info: {:#x?}\n", info.as_ref().unwrap());
        }
        info
    }

    fn recover(&self) {
        self.initdata.fw_status.with(|raw, _inner| {
            let halt_count = raw.flags.halt_count.load(Ordering::Relaxed);
            let mut halted = raw.flags.halted.load(Ordering::Relaxed);
            dev_err!(self.dev, "  Halt count: {}\n", halt_count);
            dev_err!(self.dev, "  Halted: {}\n", halted);

            if halted == 0 {
                let timeout = time::ktime_get() + Duration::from_millis(HALT_ENTER_TIMEOUT_MS);
                while time::ktime_get() < timeout {
                    halted = raw.flags.halted.load(Ordering::Relaxed);
                    if halted != 0 {
                        break;
                    }
                    mem::sync();
                }
                halted = raw.flags.halted.load(Ordering::Relaxed);
            }

            if debug_enabled(DebugFlags::NoGpuRecovery) {
                dev_crit!(self.dev, "  GPU recovery is disabled, wedging forever!\n");
            } else if halted != 0 {
                dev_err!(self.dev, "  Attempting recovery...\n");
                raw.flags.halted.store(0, Ordering::SeqCst);
                raw.flags.resume.store(1, Ordering::SeqCst);
            } else {
                dev_err!(self.dev, "  Cannot recover.\n");
            }
        });
    }

    pub(crate) fn core_masks_packed(&self) -> &[u32] {
        self.dyncfg.id.core_masks_packed.as_slice()
    }
}

#[versions(AGX)]
impl GpuManager for GpuManager::ver {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn init(&self) -> Result {
        self.tx_channels
            .lock()
            .device_control
            .send(&DeviceControlMsg::Initialize(Default::default()));

        let initdata = self.initdata.gpu_va().get();
        let mut guard = self.rtkit.lock();
        let rtk = guard.as_mut().unwrap();

        rtk.boot()?;
        rtk.start_endpoint(EP_FIRMWARE)?;
        rtk.start_endpoint(EP_DOORBELL)?;
        rtk.send_message(EP_FIRMWARE, MSG_INIT | (initdata & INIT_DATA_MASK))?;
        rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)?;
        Ok(())
    }

    fn update_globals(&self) {
        let mut timeout: u32 = 2;
        if debug_enabled(DebugFlags::WaitForPowerOff) {
            timeout = 0;
        } else if debug_enabled(DebugFlags::KeepGpuPowered) {
            timeout = 5000;
        }

        self.initdata.globals.with(|raw, _inner| {
            raw.idle_off_delay_ms.store(timeout, Ordering::Relaxed);
        });
    }

    fn alloc(&self) -> Guard<'_, Mutex<KernelAllocators>> {
        self.alloc.lock()
    }

    fn new_vm(&self) -> Result<mmu::Vm> {
        self.uat.new_vm(self.ids.vm.next())
    }

    fn bind_vm(&self, vm: &mmu::Vm) -> Result<mmu::VmBind> {
        self.uat.bind(vm)
    }

    fn new_renderer(
        &self,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
    ) -> Result<Box<dyn render::Renderer>> {
        let mut kalloc = self.alloc();
        let id = self.ids.renderer.next();
        Ok(Box::try_new(render::Renderer::ver::new(
            &self.dev,
            &mut *kalloc,
            ualloc,
            ualloc_priv,
            self.event_manager.clone(),
            &self.buffer_mgr,
            id,
        )?)?)
    }

    fn submit_batch(&self, batch: workqueue::WorkQueueBatch<'_>) -> Result {
        let pipe_type = batch.pipe_type();
        let pipes = match pipe_type {
            PipeType::Vertex => &self.pipes.vtx,
            PipeType::Fragment => &self.pipes.frag,
            PipeType::Compute => &self.pipes.comp,
        };

        /* TODO: need try_lock()
        let pipe = 'outer: loop {
            for p in pipes.iter() {
                if let Ok(guard) = p.try_lock() {
                    break 'outer guard;
                }
            }
            break pipes[0].lock();
        };
        */

        let index: usize = 0;
        let mut pipe = pipes[index].lock();

        batch.submit(&mut pipe)?;

        let mut guard = self.rtkit.lock();
        let rtk = guard.as_mut().unwrap();
        rtk.send_message(
            EP_DOORBELL,
            MSG_TX_DOORBELL | pipe_type as u64 | ((index as u64) << 2),
        )?;

        Ok(())
    }

    fn kick_firmware(&self) -> Result {
        let mut guard = self.rtkit.lock();
        let rtk = guard.as_mut().unwrap();
        rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_KICKFW)?;

        Ok(())
    }

    fn invalidate_context(
        &self,
        context: &fw::types::GpuObject<fw::workqueue::GpuContextData>,
    ) -> Result {
        mod_dev_dbg!(
            self.dev,
            "Invalidating GPU context @ {:?}\n",
            context.weak_pointer()
        );

        let dc = context.with(|raw, _inner| DeviceControlMsg::DestroyContext {
            unk_4: 0,
            ctx_23: raw.unk_23,
            __pad0: Default::default(),
            unk_c: 0,
            unk_10: 0,
            ctx_0: raw.unk_0,
            ctx_1: raw.unk_1,
            ctx_4: raw.unk_4,
            __pad1: Default::default(),
            unk_18: 0,
            gpu_context: Some(context.weak_pointer()),
            __pad2: Default::default(),
        });

        mod_dev_dbg!(self.dev, "Context invalidation command: {:?}\n", &dc);

        let mut txch = self.tx_channels.lock();

        let token = txch.device_control.send(&dc);

        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().unwrap();
            rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)?;
        }

        txch.device_control.wait_for(token)?;

        mod_dev_dbg!(
            self.dev,
            "GPU context invalidated: {:?}\n",
            context.weak_pointer()
        );

        Ok(())
    }

    fn flush_fw_cache(&self) -> Result {
        mod_dev_dbg!(self.dev, "Flushing coprocessor data cache\n");

        // ctx_0 == 0xff or ctx_1 == 0xff cause no effect on context,
        // but this command does a full cache flush too, so abuse it
        // for that.

        let dc = DeviceControlMsg::DestroyContext {
            unk_4: 0,
            ctx_23: 0,
            __pad0: Default::default(),
            unk_c: 0,
            unk_10: 0,
            ctx_0: 0xff,
            ctx_1: 0xff,
            ctx_4: 0,
            __pad1: Default::default(),
            unk_18: 0,
            gpu_context: None,
            __pad2: Default::default(),
        };

        let mut txch = self.tx_channels.lock();

        let token = txch.device_control.send(&dc);
        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().unwrap();
            rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)?;
        }

        txch.device_control.wait_for(token)?;
        Ok(())
    }

    fn ids(&self) -> &SequenceIDs {
        &self.ids
    }

    fn handle_timeout(&self, counter: u32, event_slot: u32) {
        dev_err!(self.dev, " (\\________/) \n");
        dev_err!(self.dev, "  |        |  \n");
        dev_err!(self.dev, "'.| \\  , / |.'\n");
        dev_err!(self.dev, "--| / (( \\ |--\n");
        dev_err!(self.dev, ".'|  _-_-  |'.\n");
        dev_err!(self.dev, "  |________|  \n");
        dev_err!(self.dev, "** GPU timeout nya~!!!!! **\n");
        dev_err!(self.dev, "  Event slot: {}\n", event_slot);
        dev_err!(self.dev, "  Timeout count: {}\n", counter);
        self.get_fault_info();
        self.mark_pending_events(Some(event_slot), workqueue::BatchError::Timeout);
        self.recover();
    }

    fn handle_fault(&self) {
        dev_err!(self.dev, " (\\________/) \n");
        dev_err!(self.dev, "  |        |  \n");
        dev_err!(self.dev, "'.| \\  , / |.'\n");
        dev_err!(self.dev, "--| / (( \\ |--\n");
        dev_err!(self.dev, ".'|  _-_-  |'.\n");
        dev_err!(self.dev, "  |________|  \n");
        dev_err!(self.dev, "GPU fault nya~!!!!!\n");
        let error = match self.get_fault_info() {
            Some(info) => workqueue::BatchError::Fault(info),
            None => workqueue::BatchError::Unknown,
        };
        self.mark_pending_events(None, error);
        self.recover();
    }

    fn wait_for_poweroff(&self, timeout: usize) -> Result {
        self.initdata.runtime_pointers.hwdata_a.with(|raw, _inner| {
            for _i in 0..timeout {
                if raw.pwr_status.load(Ordering::Relaxed) == 4 {
                    return Ok(());
                }
                coarse_sleep(Duration::from_millis(1));
            }
            Err(ETIMEDOUT)
        })
    }

    fn fwctl(&self, msg: fw::channels::FwCtlMsg) -> Result {
        let mut fwctl = self.fwctl_channel.lock();
        let token = fwctl.send(&msg);
        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().unwrap();
            rtk.send_message(EP_DOORBELL, MSG_FWCTL)?;
        }
        fwctl.wait_for(token)?;
        Ok(())
    }

    fn get_cfg(&self) -> &'static hw::HwConfig {
        self.cfg
    }
    fn get_dyncfg(&self) -> &hw::DynConfig {
        &*self.dyncfg
    }
}

#[versions(AGX)]
unsafe impl Sync for GpuManager::ver {}

#[versions(AGX)]
unsafe impl Send for GpuManager::ver {}
