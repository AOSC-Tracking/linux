// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]

//! Asahi ring buffer channels

use crate::debug::*;
use crate::driver::AsahiDevice;
use crate::fw::channels::*;
use crate::fw::initdata::{raw, ChannelRing};
use crate::fw::types::*;
use crate::{event, gpu, mem};
use core::time::Duration;
use kernel::{c_str, dbg, delay::coarse_sleep, prelude::*, sync::Arc, time};

pub(crate) use crate::fw::channels::PipeType;

pub(crate) struct RxChannel<T: RxChannelState, U: Copy + Default>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug + Default,
{
    ring: ChannelRing<T, U>,
    // FIXME: needs feature(generic_const_exprs)
    //rptr: [u32; T::SUB_CHANNELS],
    rptr: [u32; 6],
    count: u32,
}

impl<T: RxChannelState, U: Copy + Default> RxChannel<T, U>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug + Default,
{
    pub(crate) fn new(alloc: &mut gpu::KernelAllocators, count: usize) -> Result<RxChannel<T, U>> {
        Ok(RxChannel {
            ring: ChannelRing {
                state: alloc.shared.new_default()?,
                ring: alloc.shared.array_empty(T::SUB_CHANNELS * count)?,
            },
            rptr: Default::default(),
            count: count as u32,
        })
    }

    fn get_or_peek(&mut self, index: usize, peek: bool) -> Option<U> {
        self.ring.state.with(|raw, _inner| {
            let wptr = T::wptr(raw, index);
            let rptr = &mut self.rptr[index];
            if wptr == *rptr {
                None
            } else {
                let off = self.count as usize * index;
                let msg = self.ring.ring[off + *rptr as usize];
                if !peek {
                    *rptr = (*rptr + 1) % self.count;
                    T::set_rptr(raw, index, *rptr);
                }
                Some(msg)
            }
        })
    }

    pub(crate) fn get(&mut self, index: usize) -> Option<U> {
        self.get_or_peek(index, false)
    }

    pub(crate) fn peek(&mut self, index: usize) -> Option<U> {
        self.get_or_peek(index, true)
    }
}

pub(crate) struct TxChannel<T: TxChannelState, U: Copy + Default>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug + Default,
{
    ring: ChannelRing<T, U>,
    wptr: u32,
    count: u32,
}

impl<T: TxChannelState, U: Copy + Default> TxChannel<T, U>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug + Default,
{
    pub(crate) fn new(alloc: &mut gpu::KernelAllocators, count: usize) -> Result<TxChannel<T, U>> {
        Ok(TxChannel {
            ring: ChannelRing {
                state: alloc.shared.new_default()?,
                ring: alloc.private.array_empty(count)?,
            },
            wptr: 0,
            count: count as u32,
        })
    }

    pub(crate) fn new_uncached(
        alloc: &mut gpu::KernelAllocators,
        count: usize,
    ) -> Result<TxChannel<T, U>> {
        Ok(TxChannel {
            ring: ChannelRing {
                state: alloc.shared.new_default()?,
                ring: alloc.shared.array_empty(count)?,
            },
            wptr: 0,
            count: count as u32,
        })
    }

    pub(crate) fn put(&mut self, msg: &U) -> u32 {
        self.ring.state.with(|raw, _inner| {
            let next_wptr = (self.wptr + 1) % self.count;
            let mut rptr = T::rptr(raw);
            if next_wptr == rptr {
                pr_err!(
                    "TX ring buffer is full! Waiting... ({}, {})",
                    next_wptr,
                    rptr
                );
                // TODO: block properly on incoming messages?
                while next_wptr == rptr {
                    coarse_sleep(Duration::from_millis(8));
                    rptr = T::rptr(raw);
                }
            }
            self.ring.ring[self.wptr as usize] = *msg;
            mem::sync();
            T::set_wptr(raw, next_wptr);
            self.wptr = next_wptr;
        });
        self.wptr
    }

    pub(crate) fn wait_for(&mut self, wptr: u32, timeout_ms: u64) -> Result {
        let timeout = time::ktime_get() + Duration::from_millis(timeout_ms);
        self.ring.state.with(|raw, _inner| {
            while time::ktime_get() < timeout {
                if T::rptr(raw) == wptr {
                    return Ok(());
                }
                mem::sync();
            }
            Err(ETIMEDOUT)
        })
    }
}

pub(crate) struct DeviceControlChannel {
    dev: AsahiDevice,
    ch: TxChannel<ChannelState, DeviceControlMsg>,
}

impl DeviceControlChannel {
    const COMMAND_TIMEOUT_MS: u64 = 100;

    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
    ) -> Result<DeviceControlChannel> {
        Ok(DeviceControlChannel {
            dev: dev.clone(),
            ch: TxChannel::<ChannelState, DeviceControlMsg>::new(alloc, 0x100)?,
        })
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, DeviceControlMsg> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn send(&mut self, msg: &DeviceControlMsg) -> u32 {
        cls_dev_dbg!(DeviceControlCh, self.dev, "DeviceControl: {:?}", msg);
        self.ch.put(msg)
    }

    pub(crate) fn wait_for(&mut self, wptr: u32) -> Result {
        self.ch.wait_for(wptr, Self::COMMAND_TIMEOUT_MS)
    }
}

pub(crate) struct PipeChannel {
    dev: AsahiDevice,
    ch: TxChannel<ChannelState, PipeMsg>,
}

impl PipeChannel {
    pub(crate) fn new(dev: &AsahiDevice, alloc: &mut gpu::KernelAllocators) -> Result<PipeChannel> {
        Ok(PipeChannel {
            dev: dev.clone(),
            ch: TxChannel::<ChannelState, PipeMsg>::new(alloc, 0x100)?,
        })
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, PipeMsg> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn send(&mut self, msg: &PipeMsg) {
        cls_dev_dbg!(PipeCh, self.dev, "Pipe: {:?}", msg);
        self.ch.put(msg);
    }
}

pub(crate) struct FwCtlChannel {
    dev: AsahiDevice,
    ch: TxChannel<FwCtlChannelState, FwCtlMsg>,
}

impl FwCtlChannel {
    const COMMAND_TIMEOUT_MS: u64 = 100;

    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
    ) -> Result<FwCtlChannel> {
        Ok(FwCtlChannel {
            dev: dev.clone(),
            ch: TxChannel::<FwCtlChannelState, FwCtlMsg>::new_uncached(alloc, 0x100)?,
        })
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<FwCtlChannelState, FwCtlMsg> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn send(&mut self, msg: &FwCtlMsg) -> u32 {
        cls_dev_dbg!(FwCtlCh, self.dev, "FwCtl: {:?}", msg);
        self.ch.put(msg)
    }

    pub(crate) fn wait_for(&mut self, wptr: u32) -> Result {
        self.ch.wait_for(wptr, Self::COMMAND_TIMEOUT_MS)
    }
}

pub(crate) struct EventChannel {
    dev: AsahiDevice,
    ch: RxChannel<ChannelState, RawEventMsg>,
    mgr: Arc<event::EventManager>,
    gpu: Option<Arc<dyn gpu::GpuManager>>,
}

impl EventChannel {
    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
        mgr: Arc<event::EventManager>,
    ) -> Result<EventChannel> {
        Ok(EventChannel {
            dev: dev.clone(),
            ch: RxChannel::<ChannelState, RawEventMsg>::new(alloc, 0x100)?,
            mgr,
            gpu: None,
        })
    }

    pub(crate) fn set_manager(&mut self, gpu: Arc<dyn gpu::GpuManager>) {
        self.gpu = Some(gpu);
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, RawEventMsg> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn poll(&mut self) {
        while let Some(msg) = self.ch.get(0) {
            let tag = unsafe { msg.raw.0 };
            match tag {
                0..=EVENT_MAX => {
                    let msg = unsafe { msg.msg };

                    cls_dev_dbg!(EventCh, self.dev, "Event: {:?}", msg);
                    match msg {
                        EventMsg::Fault => match self.gpu.as_ref() {
                            Some(gpu) => gpu.handle_fault(),
                            None => dev_crit!(self.dev, "EventChannel: No GPU manager available!"),
                        },
                        EventMsg::Timeout {
                            counter,
                            event_slot,
                            ..
                        } => match self.gpu.as_ref() {
                            Some(gpu) => gpu.handle_timeout(counter, event_slot),
                            None => dev_crit!(self.dev, "EventChannel: No GPU manager available!"),
                        },
                        EventMsg::Flag { firing, .. } => {
                            for (i, flags) in firing.iter().enumerate() {
                                for j in 0..32 {
                                    if flags & (1u32 << j) != 0 {
                                        self.mgr.signal((i * 32 + j) as u32);
                                    }
                                }
                            }
                        }
                        msg => {
                            dev_crit!(self.dev, "Unknown event message: {:?}", msg);
                        }
                    }
                }
                _ => {
                    dev_warn!(self.dev, "Unknown event message: {:?}", unsafe { msg.raw });
                }
            }
        }
    }
}

pub(crate) struct FwLogChannel {
    dev: AsahiDevice,
    ch: RxChannel<FwLogChannelState, RawFwLogMsg>,
    payload_buf: GpuArray<RawFwLogPayloadMsg>,
}

impl FwLogChannel {
    const RING_SIZE: usize = 0x100;
    const BUF_SIZE: usize = 0x100;

    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
    ) -> Result<FwLogChannel> {
        Ok(FwLogChannel {
            dev: dev.clone(),
            ch: RxChannel::<FwLogChannelState, RawFwLogMsg>::new(alloc, Self::RING_SIZE)?,
            payload_buf: alloc
                .shared
                .array_empty(Self::BUF_SIZE * FwLogChannelState::SUB_CHANNELS)?,
        })
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<FwLogChannelState, RawFwLogMsg> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn get_buf(&self) -> GpuWeakPointer<[RawFwLogPayloadMsg]> {
        self.payload_buf.weak_pointer()
    }

    pub(crate) fn poll(&mut self) {
        for i in 0..=FwLogChannelState::SUB_CHANNELS - 1 {
            while let Some(msg) = self.ch.peek(i) {
                cls_dev_dbg!(FwLogCh, self.dev, "FwLog{}: {:?}", i, msg);
                if msg.msg_type != 2 {
                    dev_warn!(self.dev, "Unknown FWLog{} message: {:?}", i, msg);
                    self.ch.get(i);
                    continue;
                }
                if msg.msg_index.0 as usize >= Self::BUF_SIZE {
                    dev_warn!(
                        self.dev,
                        "FWLog{} message index out of bounds: {:?}",
                        i,
                        msg
                    );
                    self.ch.get(i);
                    continue;
                }
                let index = Self::BUF_SIZE * i + msg.msg_index.0 as usize;
                let payload = &self.payload_buf.as_slice()[index];
                if payload.msg_type != 3 {
                    dev_warn!(self.dev, "Unknown FWLog{} payload: {:?}", i, payload);
                    self.ch.get(i);
                    continue;
                }
                let msg = if let Some(end) = payload.msg.iter().position(|&r| r == 0) {
                    CStr::from_bytes_with_nul(&(*payload.msg)[..end + 1])
                        .unwrap_or(c_str!("cstr_err"))
                } else {
                    dev_warn!(
                        self.dev,
                        "FWLog{} payload not NUL-terminated: {:?}",
                        i,
                        payload
                    );
                    self.ch.get(i);
                    continue;
                };
                match i {
                    0 => dev_dbg!(self.dev, "FWLog: {}", msg),
                    1 => dev_info!(self.dev, "FWLog: {}", msg),
                    2 => dev_notice!(self.dev, "FWLog: {}", msg),
                    3 => dev_warn!(self.dev, "FWLog: {}", msg),
                    4 => dev_err!(self.dev, "FWLog: {}", msg),
                    5 => dev_crit!(self.dev, "FWLog: {}", msg),
                    _ => (),
                };
                self.ch.get(i);
            }
        }
    }
}

pub(crate) struct KTraceChannel {
    dev: AsahiDevice,
    ch: RxChannel<ChannelState, RawKTraceMsg>,
}

impl KTraceChannel {
    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
    ) -> Result<KTraceChannel> {
        Ok(KTraceChannel {
            dev: dev.clone(),
            ch: RxChannel::<ChannelState, RawKTraceMsg>::new(alloc, 0x200)?,
        })
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, RawKTraceMsg> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn poll(&mut self) {
        while let Some(msg) = self.ch.get(0) {
            cls_dev_dbg!(KTraceCh, self.dev, "KTrace: {:?}", msg);
        }
    }
}

#[versions(AGX)]
pub(crate) struct StatsChannel {
    dev: AsahiDevice,
    ch: RxChannel<ChannelState, RawStatsMsg::ver>,
}

#[versions(AGX)]
impl StatsChannel::ver {
    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
    ) -> Result<StatsChannel::ver> {
        Ok(StatsChannel::ver {
            dev: dev.clone(),
            ch: RxChannel::<ChannelState, RawStatsMsg::ver>::new(alloc, 0x100)?,
        })
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, RawStatsMsg::ver> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn poll(&mut self) {
        while let Some(msg) = self.ch.get(0) {
            let tag = unsafe { msg.raw.0 };
            match tag {
                0..=STATS_MAX::ver => {
                    let msg = unsafe { msg.msg };
                    cls_dev_dbg!(StatsCh, self.dev, "Stats: {:?}", msg);
                }
                _ => {
                    pr_warn!("Unknown stats message: {:?}", unsafe { msg.raw });
                }
            }
        }
    }
}
