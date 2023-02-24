// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Apple AGX UAT (MMU) support

use core::fmt::Debug;
use core::mem::{size_of, ManuallyDrop};
use core::ops::Deref;
use core::ptr::{addr_of_mut, NonNull};
use core::sync::atomic::{fence, AtomicU32, AtomicU64, AtomicU8, Ordering};
use core::time::Duration;

use kernel::{
    bindings, c_str, delay, device,
    drm::mm,
    error::{to_result, Result},
    io_pgtable,
    io_pgtable::{prot, AppleUAT, AppleUATCfg, IOPagetable},
    prelude::*,
    str::CString,
    sync::{smutex::Mutex, Guard},
    sync::{Arc, UniqueArc},
    time, PointerWrapper,
};

use crate::debug::*;
use crate::no_debug;
use crate::{driver, fw, gem, hw, mem, slotalloc};

const DEBUG_CLASS: DebugFlags = DebugFlags::Mmu;

const PPL_MAGIC: u64 = 0x4b1d000000000002;

const UAT_NUM_CTX: usize = 64;
const UAT_USER_CTX_START: usize = 1;
const UAT_USER_CTX: usize = UAT_NUM_CTX - UAT_USER_CTX_START;

pub(crate) const UAT_PGBIT: usize = 14;
pub(crate) const UAT_PGSZ: usize = 1 << UAT_PGBIT;
pub(crate) const UAT_PGMSK: usize = UAT_PGSZ - 1;

type Pte = AtomicU64;
const UAT_NPTE: usize = UAT_PGSZ / size_of::<Pte>();

pub(crate) const UAT_IAS: usize = 39;
pub(crate) const UAT_IAS_KERN: usize = 36;

const IOVA_USER_BASE: usize = UAT_PGSZ;
const IOVA_USER_TOP: usize = (1 << UAT_IAS) - 1;
const IOVA_TTBR1_BASE: usize = 0xffffff8000000000;
const IOVA_KERN_BASE: usize = 0xffffffa000000000;
const IOVA_KERN_TOP: usize = 0xffffffafffffffff;

const TTBR_VALID: u64 = 0x1; // BIT(0)
const TTBR_ASID_SHIFT: usize = 48;

const PTE_TABLE: u64 = 0x3; // BIT(0) | BIT(1)

// Note: prot::CACHE means "cache coherency", which for UAT means *uncached*,
// since uncached mappings from the GFX ASC side are cache coherent with the AP cache.
// Not having that flag means *cached noncoherent*.

pub(crate) const PROT_FW_MMIO_RW: u32 =
    prot::PRIV | prot::READ | prot::WRITE | prot::CACHE | prot::MMIO;
pub(crate) const PROT_FW_MMIO_RO: u32 = prot::PRIV | prot::READ | prot::CACHE | prot::MMIO;
pub(crate) const PROT_FW_SHARED_RW: u32 = prot::PRIV | prot::READ | prot::WRITE | prot::CACHE;
pub(crate) const PROT_FW_SHARED_RO: u32 = prot::PRIV | prot::READ | prot::CACHE;
pub(crate) const PROT_FW_PRIV_RW: u32 = prot::PRIV | prot::READ | prot::WRITE;
pub(crate) const PROT_FW_PRIV_RO: u32 = prot::PRIV | prot::READ;
pub(crate) const PROT_GPU_FW_SHARED_RW: u32 = prot::READ | prot::WRITE | prot::CACHE;
pub(crate) const PROT_GPU_FW_PRIV_RW: u32 = prot::READ | prot::WRITE;
pub(crate) const PROT_GPU_SHARED_RW: u32 = prot::READ | prot::WRITE | prot::CACHE | prot::NOEXEC;
pub(crate) const PROT_GPU_SHARED_RO: u32 = prot::READ | prot::CACHE | prot::NOEXEC;
pub(crate) const PROT_GPU_PRIV_RW: u32 = prot::READ | prot::WRITE | prot::NOEXEC;
pub(crate) const PROT_GPU_PRIV_RO: u32 = prot::READ | prot::NOEXEC;

type PhysAddr = bindings::phys_addr_t;

struct UatRegion {
    base: PhysAddr,
    size: usize,
    map: NonNull<core::ffi::c_void>,
}

#[repr(C)]
struct FlushInfo {
    state: AtomicU64,
    addr: AtomicU64,
    size: AtomicU64,
}

#[repr(C)]
struct Handoff {
    magic_ap: AtomicU64,
    magic_fw: AtomicU64,

    lock_ap: AtomicU8,
    lock_fw: AtomicU8,
    // Implicit padding: 2 bytes
    turn: AtomicU32,
    cur_slot: AtomicU32,
    // Implicit padding: 4 bytes
    flush: [FlushInfo; UAT_NUM_CTX + 1],

    unk2: AtomicU8,
    // Implicit padding: 7 bytes
    unk3: AtomicU64,
}

const HANDOFF_SIZE: usize = size_of::<Handoff>();

#[repr(C)]
struct SlotTTBS {
    ttb0: AtomicU64,
    ttb1: AtomicU64,
}

const SLOTS_SIZE: usize = UAT_NUM_CTX * size_of::<SlotTTBS>();

// We need at least page 0 (ttb0)
const PAGETABLES_SIZE: usize = UAT_PGSZ;

struct VmInner {
    dev: driver::AsahiDevice,
    is_kernel: bool,
    min_va: usize,
    max_va: usize,
    page_table: AppleUAT<Uat>,
    mm: mm::Allocator<(), MappingInner>,
    uat_inner: Arc<UatInner>,
    active_users: usize,
    binding: Option<slotalloc::Guard<SlotInner>>,
    bind_token: Option<slotalloc::SlotToken>,
    id: u64,
}

impl VmInner {
    fn slot(&self) -> Option<u32> {
        if self.is_kernel {
            // The GFX ASC does not care about the ASID. Pick an arbitrary one.
            // TODO: This needs to be a persistently reserved ASID once we integrate
            // with the ARM64 kernel ASID machinery to avoid overlap.
            Some(0)
        } else {
            // We don't check whether we lost the slot, which could cause unnecessary
            // invalidations against another Vm. However, this situation should be very
            // rare (e.g. a Vm lost its slot, which means 63 other Vms bound in the
            // interim, and then it gets killed / drops its mappings without doing any
            // final rendering). Anything doing active maps/unmaps is probably also
            // rendering and therefore likely bound.
            self.bind_token
                .as_ref()
                .map(|token| (token.last_slot() as u32 + UAT_USER_CTX_START as u32))
        }
    }

    fn ttb(&self) -> u64 {
        self.page_table.cfg().ttbr
    }

    fn map_iova(&self, iova: usize, size: usize) -> Result<usize> {
        if iova < self.min_va || (iova + size - 1) > self.max_va {
            Err(EINVAL)
        } else if self.is_kernel {
            Ok(iova - self.min_va)
        } else {
            Ok(iova)
        }
    }

    fn map_pages(
        &mut self,
        mut iova: usize,
        mut paddr: usize,
        pgsize: usize,
        pgcount: usize,
        prot: u32,
    ) -> Result<usize> {
        let mut left = pgcount;
        while left > 0 {
            let mapped_iova = self.map_iova(iova, pgsize * left)?;
            let mapped = self
                .page_table
                .map_pages(mapped_iova, paddr, pgsize, left, prot)?;
            assert!(mapped <= left * pgsize);

            left -= mapped / pgsize;
            paddr += mapped;
            iova += mapped;
        }
        Ok(pgcount * pgsize)
    }

    fn unmap_pages(&mut self, mut iova: usize, pgsize: usize, pgcount: usize) -> Result<usize> {
        let mut left = pgcount;
        while left > 0 {
            let mapped_iova = self.map_iova(iova, pgsize * left)?;
            let unmapped = self.page_table.unmap_pages(mapped_iova, pgsize, left);
            assert!(unmapped <= left * pgsize);

            left -= unmapped / pgsize;
            iova += unmapped;
        }

        Ok(pgcount * pgsize)
    }

    fn map_node(&mut self, node: &mm::Node<(), MappingInner>, prot: u32) -> Result {
        let mut iova = node.start() as usize;
        let sgt = node.sgt.as_ref().ok_or(EINVAL)?;

        for range in sgt.iter() {
            let addr = range.dma_address();
            let len = range.dma_len();

            if (addr | len | iova) & UAT_PGMSK != 0 {
                dev_err!(
                    self.dev,
                    "MMU: Mapping {:#x}:{:#x} -> {:#x} is not page-aligned",
                    addr,
                    len,
                    iova
                );
                return Err(EINVAL);
            }

            mod_dev_dbg!(self.dev, "MMU: map: {:#x}:{:#x} -> {:#x}", addr, len, iova);

            self.map_pages(iova, addr, UAT_PGSZ, len >> UAT_PGBIT, prot)?;

            iova += len;
        }
        Ok(())
    }
}

#[derive(Clone)]
pub(crate) struct Vm {
    id: u64,
    inner: Arc<Mutex<VmInner>>,
}
no_debug!(Vm);

pub(crate) struct SlotInner();

impl slotalloc::SlotItem for SlotInner {
    type Owner = ();
}

#[derive(Debug)]
pub(crate) struct VmBind(Vm, u32);

impl VmBind {
    pub(crate) fn slot(&self) -> u32 {
        self.1
    }
}

impl Drop for VmBind {
    fn drop(&mut self) {
        let mut inner = self.0.inner.lock();

        assert_ne!(inner.active_users, 0);
        inner.active_users -= 1;
        if inner.active_users == 0 {
            inner.binding = None;
        }
    }
}

impl Clone for VmBind {
    fn clone(&self) -> VmBind {
        let mut inner = self.0.inner.lock();

        inner.active_users += 1;
        VmBind(self.0.clone(), self.1)
    }
}

pub(crate) struct MappingInner {
    owner: Arc<Mutex<VmInner>>,
    uat_inner: Arc<UatInner>,
    prot: u32,
    mapped_size: usize,
    sgt: Option<gem::SGTable>,
}

pub(crate) struct Mapping(mm::Node<(), MappingInner>);

impl Mapping {
    pub(crate) fn iova(&self) -> usize {
        self.0.start() as usize
    }

    pub(crate) fn size(&self) -> usize {
        self.0.mapped_size
    }

    fn remap_uncached_and_flush(&mut self) {
        let mut owner = self.0.owner.lock();
        mod_dev_dbg!(
            owner.dev,
            "MMU: remap as uncached {:#x}:{:#x}",
            self.iova(),
            self.size()
        );

        // The IOMMU API does not allow us to remap things in-place...
        // just do an unmap and map again for now.
        // Do not try to unmap guard page (-1)
        if owner
            .unmap_pages(self.iova(), UAT_PGSZ, self.size() >> UAT_PGBIT)
            .is_err()
        {
            dev_err!(
                owner.dev,
                "MMU: unmap for remap {:#x}:{:#x} failed",
                self.iova(),
                self.size()
            );
        }

        let prot = self.0.prot | prot::CACHE;
        if owner.map_node(&self.0, prot).is_err() {
            dev_err!(
                owner.dev,
                "MMU: remap {:#x}:{:#x} failed",
                self.iova(),
                self.size()
            );
        }

        // If we don't have (and have never had) a VM slot, just return
        let slot = match owner.slot() {
            None => return,
            Some(slot) => slot,
        };

        let flush_slot = if owner.is_kernel {
            // If this is a kernel mapping, always flush on index 64
            UAT_NUM_CTX as u32
        } else {
            // Otherwise, check if this slot is the active one, otherwise return
            // Also check that we actually own this slot
            let ttb = owner.ttb() | TTBR_VALID | (slot as u64) << TTBR_ASID_SHIFT;

            let uat_inner = self.0.uat_inner.lock();
            uat_inner.handoff().lock();
            let cur_slot = uat_inner.handoff().current_slot();
            let ttb_cur = uat_inner.ttbs()[slot as usize].ttb0.load(Ordering::Relaxed);
            uat_inner.handoff().unlock();
            if cur_slot == Some(slot) && ttb_cur == ttb {
                slot
            } else {
                return;
            }
        };

        // FIXME: There is a race here, though it'll probably never happen in practice.
        // In theory, it's possible for the ASC to finish using our slot, whatever command
        // it was processing to complete, the slot to be lost to another context, and the ASC
        // to begin using it again with a different page table, thus faulting when it gets a
        // flush request here. In practice, the chance of this happening is probably vanishingly
        // small, as all 62 other slots would have to be recycled or in use before that slot can
        // be reused, and the ASC using user contexts at all is very rare.

        // Still, the locking around UAT/Handoff/TTBs should probably be redesigned to better
        // model the interactions with the firmware and avoid these races.
        // Possibly TTB changes should be tied to slot locks:

        // Flush:
        //  - Can early check handoff here (no need to lock).
        //      If user slot and it doesn't match the active ASC slot,
        //      we can elide the flush as the ASC guarantees it flushes
        //      TLBs/caches when it switches context. We just need a
        //      barrier to ensure ordering.
        //  - Lock TTB slot
        //      - If user ctx:
        //          - Lock handoff AP-side
        //              - Lock handoff dekker
        //                  - Check TTB & handoff cur ctx
        //      - Perform flush if necessary
        //          - This implies taking the fwring lock
        //
        // TTB change:
        //  - lock TTB slot
        //      - lock handoff AP-side
        //          - lock handoff dekker
        //              change TTB

        // Lock this flush slot, and write the range to it
        let flush = self.0.uat_inner.lock_flush(flush_slot);
        flush.begin_flush(self.iova() as u64, self.size() as u64);

        let cmd = fw::channels::FwCtlMsg {
            addr: fw::types::U64(self.iova() as u64),
            unk_8: 0,
            slot: flush_slot,
            unk_10: 1,
            unk_12: 2,
        };

        // Tell the firmware to do a cache flush
        if owner.dev.data().gpu.fwctl(cmd).is_err() {
            dev_err!(
                owner.dev,
                "MMU: ASC cache flush {:#x}:{:#x} timed out",
                self.iova(),
                self.size()
            );
        }

        // Finish the flush
        flush.end_flush();

        // Slot is unlocked here
    }
}

impl Drop for Mapping {
    fn drop(&mut self) {
        // This is the main unmap function for UAT mappings.
        // The sequence of operations here is finicky, due to the interaction
        // between cached GFX ASC mappings and the page tables. These mappings
        // always have to be flushed from the cache before being unmapped.

        // For uncached mappings, just unmapping and flushing the TLB is sufficient.

        // For cached mappings, this is the required sequence:
        // 1. Remap it as uncached
        // 2. Flush the TLB range
        // 3. If kernel VA mapping OR user VA mapping and handoff.current_slot() == slot:
        //    a. Take a lock for this slot
        //    b. Write the flush range to the right context slot in handoff area
        //    c. Issue a cache invalidation request via FwCtl queue
        //    d. Poll for completion via queue
        //    e. Check for completion flag in the handoff area
        //    f. Drop the lock
        // 4. Unmap
        // 5. Flush the TLB range again

        // prot::CACHE means "cache coherent" which means *uncached* here.
        if self.0.prot & prot::CACHE == 0 {
            self.remap_uncached_and_flush();
        }

        let mut owner = self.0.owner.lock();
        mod_dev_dbg!(
            owner.dev,
            "MMU: unmap {:#x}:{:#x}",
            self.iova(),
            self.size()
        );

        if owner
            .unmap_pages(self.iova(), UAT_PGSZ, self.size() >> UAT_PGBIT)
            .is_err()
        {
            dev_err!(
                owner.dev,
                "MMU: unmap {:#x}:{:#x} failed",
                self.iova(),
                self.size()
            );
        }

        if let Some(asid) = owner.slot() {
            mem::tlbi_range(asid as u8, self.iova(), self.size());
            mod_dev_dbg!(
                owner.dev,
                "MMU: flush range: asid={:#x} start={:#x} len={:#x}",
                asid,
                self.iova(),
                self.size()
            );
            mem::sync();
        }
    }
}

struct UatShared {
    handoff_rgn: UatRegion,
    ttbs_rgn: UatRegion,
}

impl UatShared {
    fn handoff(&self) -> &Handoff {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.handoff_rgn.map.as_ptr() as *mut Handoff).as_ref() }.unwrap()
    }

    fn ttbs(&self) -> &[SlotTTBS; UAT_NUM_CTX] {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.ttbs_rgn.map.as_ptr() as *mut [SlotTTBS; UAT_NUM_CTX]).as_ref() }.unwrap()
    }
}

unsafe impl Send for UatShared {}

struct UatInner {
    shared: Mutex<UatShared>,
    handoff_flush: [Mutex<HandoffFlush>; UAT_NUM_CTX + 1],
}

impl UatInner {
    fn lock(&self) -> Guard<'_, Mutex<UatShared>> {
        self.shared.lock()
    }

    fn lock_flush(&self, slot: u32) -> Guard<'_, Mutex<HandoffFlush>> {
        self.handoff_flush[slot as usize].lock()
    }
}

pub(crate) struct Uat {
    dev: driver::AsahiDevice,
    cfg: &'static hw::HwConfig,
    pagetables_rgn: UatRegion,

    inner: Arc<UatInner>,
    slots: slotalloc::SlotAllocator<SlotInner>,

    kernel_vm: Vm,
    kernel_lower_vm: Vm,
}

impl Drop for UatRegion {
    fn drop(&mut self) {
        // SAFETY: the pointer is valid by the type invariant
        unsafe { bindings::memunmap(self.map.as_ptr()) };
    }
}

impl Handoff {
    fn lock(&self) {
        self.lock_ap.store(1, Ordering::Relaxed);
        fence(Ordering::SeqCst);

        while self.lock_fw.load(Ordering::Relaxed) != 0 {
            if self.turn.load(Ordering::Relaxed) != 0 {
                self.lock_ap.store(0, Ordering::Relaxed);
                while self.turn.load(Ordering::Relaxed) != 0 {}
                self.lock_ap.store(1, Ordering::Relaxed);
                fence(Ordering::SeqCst);
            }
        }
        fence(Ordering::Acquire);
    }

    fn unlock(&self) {
        self.turn.store(1, Ordering::Relaxed);
        self.lock_ap.store(0, Ordering::Release);
    }

    fn current_slot(&self) -> Option<u32> {
        let slot = self.cur_slot.load(Ordering::Relaxed);
        if slot == 0 || slot == u32::MAX {
            None
        } else {
            Some(slot)
        }
    }

    fn init(&self) -> Result {
        self.magic_ap.store(PPL_MAGIC, Ordering::Relaxed);
        self.cur_slot.store(0, Ordering::Relaxed);
        self.unk3.store(0, Ordering::Relaxed);
        fence(Ordering::SeqCst);

        let timeout = time::ktime_get() + Duration::from_millis(1000);

        self.lock();
        while time::ktime_get() < timeout {
            if self.magic_fw.load(Ordering::Relaxed) == PPL_MAGIC {
                break;
            } else {
                self.unlock();
                delay::coarse_sleep(Duration::from_millis(10));
                self.lock();
            }
        }

        if self.magic_fw.load(Ordering::Relaxed) != PPL_MAGIC {
            self.unlock();
            pr_err!("Handoff: Failed to initialize (firmware not running?)\n");
            return Err(EIO);
        }

        self.unlock();

        for i in 0..=UAT_NUM_CTX {
            self.flush[i].state.store(0, Ordering::Relaxed);
            self.flush[i].addr.store(0, Ordering::Relaxed);
            self.flush[i].size.store(0, Ordering::Relaxed);
        }
        fence(Ordering::SeqCst);
        Ok(())
    }
}

struct HandoffFlush(*const FlushInfo);

unsafe impl Send for HandoffFlush {}

impl HandoffFlush {
    fn begin_flush(&self, start: u64, size: u64) {
        let flush = unsafe { self.0.as_ref().unwrap() };

        let state = flush.state.load(Ordering::Relaxed);
        if state != 0 {
            pr_err!("Handoff: expected flush state 0, got {}", state);
        }
        flush.addr.store(start, Ordering::Relaxed);
        flush.size.store(size, Ordering::Relaxed);
        flush.state.store(1, Ordering::Relaxed);
    }

    fn end_flush(&self) {
        let flush = unsafe { self.0.as_ref().unwrap() };
        let state = flush.state.load(Ordering::Relaxed);
        if state != 2 {
            pr_err!("Handoff: expected flush state 2, got {}", state);
        }
        flush.state.store(0, Ordering::Relaxed);
    }
}

// We do not implement FlushOps, since we flush manually in this module after
// page table operations. Just provide dummy implementations.
impl io_pgtable::FlushOps for Uat {
    type Data = ();

    fn tlb_flush_all(_data: <Self::Data as PointerWrapper>::Borrowed<'_>) {}
    fn tlb_flush_walk(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _iova: usize,
        _size: usize,
        _granule: usize,
    ) {
    }
    fn tlb_add_page(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _iova: usize,
        _granule: usize,
    ) {
    }
}

impl Vm {
    fn new(
        dev: driver::AsahiDevice,
        uat_inner: Arc<UatInner>,
        cfg: &'static hw::HwConfig,
        is_kernel: bool,
        id: u64,
    ) -> Result<Vm> {
        let page_table = AppleUAT::new(
            &dev,
            io_pgtable::Config {
                pgsize_bitmap: UAT_PGSZ,
                ias: if is_kernel { UAT_IAS_KERN } else { UAT_IAS },
                oas: cfg.uat_oas,
                coherent_walk: true,
                quirks: 0,
            },
            (),
        )?;
        let min_va = if is_kernel {
            IOVA_KERN_BASE
        } else {
            IOVA_USER_BASE
        };
        let max_va = if is_kernel {
            IOVA_KERN_TOP
        } else {
            IOVA_USER_TOP
        };

        let mm = mm::Allocator::new(min_va as u64, (max_va - min_va + 1) as u64, ())?;

        Ok(Vm {
            id,
            inner: Arc::try_new(Mutex::new(VmInner {
                dev,
                min_va,
                max_va,
                is_kernel,
                page_table,
                mm,
                uat_inner,
                binding: None,
                bind_token: None,
                active_users: 0,
                id,
            }))?,
        })
    }

    fn ttb(&self) -> u64 {
        self.inner.lock().ttb()
    }

    pub(crate) fn map(&self, size: usize, sgt: gem::SGTable) -> Result<Mapping> {
        let mut inner = self.inner.lock();

        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.insert_node(
            MappingInner {
                owner: self.inner.clone(),
                uat_inner,
                prot: PROT_FW_SHARED_RW,
                sgt: Some(sgt),
                mapped_size: size,
            },
            (size + UAT_PGSZ) as u64, // Add guard page
        )?;

        inner.map_node(&node, PROT_FW_SHARED_RW)?;
        Ok(Mapping(node))
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn map_in_range(
        &self,
        size: usize,
        sgt: gem::SGTable,
        alignment: u64,
        start: u64,
        end: u64,
        prot: u32,
        guard: bool,
    ) -> Result<Mapping> {
        let mut inner = self.inner.lock();

        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.insert_node_in_range(
            MappingInner {
                owner: self.inner.clone(),
                uat_inner,
                prot,
                sgt: Some(sgt),
                mapped_size: size,
            },
            (size + if guard { UAT_PGSZ } else { 0 }) as u64, // Add guard page
            alignment,
            0,
            start,
            end,
            mm::InsertMode::Best,
        )?;

        inner.map_node(&node, prot)?;
        Ok(Mapping(node))
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn map_at(
        &self,
        addr: u64,
        size: usize,
        sgt: gem::SGTable,
        prot: u32,
        guard: bool,
    ) -> Result<Mapping> {
        let mut inner = self.inner.lock();

        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.reserve_node(
            MappingInner {
                owner: self.inner.clone(),
                uat_inner,
                prot,
                sgt: Some(sgt),
                mapped_size: size,
            },
            addr,
            (size + if guard { UAT_PGSZ } else { 0 }) as u64, // Add guard page
            0,
        )?;

        inner.map_node(&node, prot)?;
        Ok(Mapping(node))
    }
    pub(crate) fn map_io(&self, phys: usize, size: usize, rw: bool) -> Result<Mapping> {
        let prot = if rw { PROT_FW_MMIO_RW } else { PROT_FW_MMIO_RO };
        let mut inner = self.inner.lock();

        let uat_inner = inner.uat_inner.clone();
        let node = inner.mm.insert_node(
            MappingInner {
                owner: self.inner.clone(),
                uat_inner,
                prot,
                sgt: None,
                mapped_size: size,
            },
            (size + UAT_PGSZ) as u64, // Add guard page
        )?;

        let iova = node.start() as usize;

        if (phys | size | iova) & UAT_PGMSK != 0 {
            dev_err!(
                inner.dev,
                "MMU: Mapping {:#x}:{:#x} -> {:#x} is not page-aligned",
                phys,
                size,
                iova
            );
            return Err(EINVAL);
        }

        dev_info!(
            inner.dev,
            "MMU: IO map: {:#x}:{:#x} -> {:#x}",
            phys,
            size,
            iova
        );

        inner.map_pages(iova, phys, UAT_PGSZ, size >> UAT_PGBIT, prot)?;

        Ok(Mapping(node))
    }

    pub(crate) fn id(&self) -> u64 {
        self.id
    }
}

impl Drop for VmInner {
    fn drop(&mut self) {
        assert_eq!(self.active_users, 0);

        mod_pr_debug!(
            "VmInner::Drop [{}]: bind_token={:?}\n",
            self.id,
            self.bind_token
        );

        // Make sure this VM is not mapped to a TTB if it was
        if let Some(token) = self.bind_token.take() {
            let idx = (token.last_slot() as usize) + UAT_USER_CTX_START;
            let ttb = self.ttb() | TTBR_VALID | (idx as u64) << TTBR_ASID_SHIFT;

            let uat_inner = self.uat_inner.lock();
            uat_inner.handoff().lock();
            let handoff_cur = uat_inner.handoff().current_slot();
            let ttb_cur = uat_inner.ttbs()[idx].ttb0.load(Ordering::SeqCst);
            let inval = ttb_cur == ttb;
            if inval {
                if handoff_cur == Some(idx as u32) {
                    pr_err!(
                        "VmInner::drop owning slot {}, but it is currently in use by the ASC?",
                        idx
                    );
                }
                uat_inner.ttbs()[idx].ttb0.store(0, Ordering::SeqCst);
            }
            uat_inner.handoff().unlock();
            core::mem::drop(uat_inner);

            // In principle we dropped all the Mappings already, but we might as
            // well play it safe and invalidate the whole ASID.
            if inval {
                mod_pr_debug!(
                    "VmInner::Drop [{}]: need inval for ASID {:#x}\n",
                    self.id,
                    idx
                );
                mem::tlbi_asid(idx as u8);
                mem::sync();
            }
        }
    }
}

impl Uat {
    fn map_region(
        dev: &dyn device::RawDevice,
        name: &CStr,
        size: usize,
        cached: bool,
    ) -> Result<UatRegion> {
        let rdev = dev.raw_device();

        let mut res = core::mem::MaybeUninit::<bindings::resource>::uninit();

        let res = unsafe {
            let idx = bindings::of_property_match_string(
                (*rdev).of_node,
                c_str!("memory-region-names").as_char_ptr(),
                name.as_char_ptr(),
            );
            to_result(idx)?;

            let np = bindings::of_parse_phandle(
                (*rdev).of_node,
                c_str!("memory-region").as_char_ptr(),
                idx,
            );
            if np.is_null() {
                dev_err!(dev, "Missing {} region\n", name);
                return Err(EINVAL);
            }
            let ret = bindings::of_address_to_resource(np, 0, res.as_mut_ptr());
            bindings::of_node_put(np);

            if ret < 0 {
                dev_err!(dev, "Failed to get {} region\n", name);
                to_result(ret)?
            }

            res.assume_init()
        };

        let rgn_size: usize = unsafe { bindings::resource_size(&res) } as usize;

        if size > rgn_size {
            dev_err!(
                dev,
                "Region {} is too small (expected {}, got {})\n",
                name,
                size,
                rgn_size
            );
            return Err(ENOMEM);
        }

        let flags = if cached {
            bindings::MEMREMAP_WB
        } else {
            bindings::MEMREMAP_WC
        };
        let map = unsafe { bindings::memremap(res.start, rgn_size, flags.into()) };
        let map = NonNull::new(map);

        match map {
            None => {
                dev_err!(dev, "Failed to remap {} region\n", name);
                Err(ENOMEM)
            }
            Some(map) => Ok(UatRegion {
                base: res.start,
                size: rgn_size as usize,
                map,
            }),
        }
    }

    fn kpt0(&self) -> &[Pte; UAT_NPTE] {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.pagetables_rgn.map.as_ptr() as *mut [Pte; UAT_NPTE]).as_ref() }.unwrap()
    }

    pub(crate) fn kernel_vm(&self) -> &Vm {
        &self.kernel_vm
    }

    pub(crate) fn ttb_base(&self) -> u64 {
        let inner = self.inner.lock();

        inner.ttbs_rgn.base as u64
    }

    pub(crate) fn bind(&self, vm: &Vm) -> Result<VmBind> {
        let mut inner = vm.inner.lock();

        if inner.binding.is_none() {
            assert_eq!(inner.active_users, 0);

            let slot = self.slots.get(inner.bind_token)?;
            if slot.changed() {
                mod_pr_debug!("Vm Bind [{}]: bind_token={:?}\n", vm.id, slot.token(),);
                let idx = (slot.slot() as usize) + UAT_USER_CTX_START;
                let ttb = inner.ttb() | TTBR_VALID | (idx as u64) << TTBR_ASID_SHIFT;

                let uat_inner = self.inner.lock();
                let ttbs = uat_inner.ttbs();
                uat_inner.handoff().lock();
                if uat_inner.handoff().current_slot() == Some(idx as u32) {
                    pr_err!(
                        "Vm::bind to slot {}, but it is currently in use by the ASC?",
                        idx
                    );
                }
                ttbs[idx].ttb0.store(ttb, Ordering::Relaxed);
                ttbs[idx].ttb1.store(0, Ordering::Relaxed);
                uat_inner.handoff().unlock();
                core::mem::drop(uat_inner);

                // Make sure all TLB entries from the previous owner of this ASID are gone
                mem::tlbi_asid(idx as u8);
                mem::sync();
            }

            inner.bind_token = Some(slot.token());
            inner.binding = Some(slot);
        }

        inner.active_users += 1;

        Ok(VmBind(
            vm.clone(),
            inner.binding.as_ref().unwrap().slot() + UAT_USER_CTX_START as u32,
        ))
    }

    pub(crate) fn new_vm(&self, id: u64) -> Result<Vm> {
        Vm::new(self.dev.clone(), self.inner.clone(), self.cfg, false, id)
    }

    #[inline(never)]
    fn make_inner(dev: &driver::AsahiDevice) -> Result<Arc<UatInner>> {
        let handoff_rgn = Self::map_region(dev, c_str!("handoff"), HANDOFF_SIZE, false)?;
        let ttbs_rgn = Self::map_region(dev, c_str!("ttbs"), SLOTS_SIZE, false)?;

        dev_info!(dev, "MMU: Initializing kernel page table\n");

        let mut inner = UniqueArc::<UatInner>::try_new_uninit()?;
        let ptr = inner.as_mut_ptr();

        Ok(unsafe {
            let handoff = &(handoff_rgn.map.as_ptr() as *mut Handoff).as_ref().unwrap();

            for i in 0..UAT_NUM_CTX + 1 {
                addr_of_mut!((*ptr).handoff_flush[i])
                    .write(Mutex::new(HandoffFlush(&handoff.flush[i])));
            }

            addr_of_mut!((*ptr).shared).write(Mutex::new(UatShared {
                handoff_rgn,
                ttbs_rgn,
            }));

            inner.assume_init()
        }
        .into())
    }

    #[inline(never)]
    pub(crate) fn new(dev: &driver::AsahiDevice, cfg: &'static hw::HwConfig) -> Result<Self> {
        dev_info!(dev, "MMU: Initializing...\n");

        let inner = Self::make_inner(dev)?;

        let pagetables_rgn = Self::map_region(dev, c_str!("pagetables"), PAGETABLES_SIZE, true)?;

        dev_info!(dev, "MMU: Creating kernel page tables\n");
        let kernel_lower_vm = Vm::new(dev.clone(), inner.clone(), cfg, false, 1)?;
        let kernel_vm = Vm::new(dev.clone(), inner.clone(), cfg, true, 0)?;

        dev_info!(dev, "MMU: Kernel page tables created\n");

        let ttb0 = kernel_lower_vm.ttb();
        let ttb1 = kernel_vm.ttb();

        let uat = Self {
            dev: dev.clone(),
            cfg,
            pagetables_rgn,
            kernel_vm,
            kernel_lower_vm,
            inner,
            slots: slotalloc::SlotAllocator::new(UAT_USER_CTX as u32, (), |_inner, _slot| {
                SlotInner()
            })?,
        };

        let inner = uat.inner.lock();

        inner.handoff().init()?;

        dev_info!(dev, "MMU: Initializing TTBs\n");

        inner.handoff().lock();

        let ttbs = inner.ttbs();

        ttbs[0].ttb0.store(ttb0 | TTBR_VALID, Ordering::Relaxed);
        ttbs[0]
            .ttb1
            .store(uat.pagetables_rgn.base | TTBR_VALID, Ordering::Relaxed);

        for ctx in &ttbs[1..] {
            ctx.ttb0.store(0, Ordering::Relaxed);
            ctx.ttb1.store(0, Ordering::Relaxed);
        }

        inner.handoff().unlock();

        core::mem::drop(inner);

        uat.kpt0()[2].store(ttb1 | PTE_TABLE, Ordering::Relaxed);

        dev_info!(dev, "MMU: initialized\n");

        Ok(uat)
    }
}

impl Drop for Uat {
    fn drop(&mut self) {
        // Unmap what we mapped
        self.kpt0()[2].store(0, Ordering::Relaxed);

        // Make sure we flush the TLBs
        fence(Ordering::SeqCst);
        mem::tlbi_all();
        mem::sync();
    }
}

// SAFETY: All public operations are thread-safe
unsafe impl Send for Uat {}
unsafe impl Sync for Uat {}
