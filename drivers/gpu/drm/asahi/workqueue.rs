// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Asahi GPU work queues

use crate::debug::*;
use crate::fw::channels::{PipeType, RunWorkQueueMsg};
use crate::fw::event::NotifierList;
use crate::fw::types::*;
use crate::fw::workqueue::*;
use crate::{alloc, channel, event, gpu, object, regs};
use crate::{box_in_place, inner_weak_ptr, place};
use core::mem;
use core::sync::atomic::Ordering;
use core::time::Duration;
use kernel::{
    bindings, dbg,
    prelude::*,
    sync::{smutex, Arc, CondVar, Guard, Mutex, UniqueArc},
    Opaque,
};

const DEBUG_CLASS: DebugFlags = DebugFlags::WorkQueue;

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct WorkToken(u64);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum BatchError {
    Timeout,
    Fault(regs::FaultInfo),
    Unknown,
    Killed,
}

impl From<BatchError> for kernel::error::Error {
    fn from(err: BatchError) -> Self {
        match err {
            BatchError::Timeout => ETIMEDOUT,
            // Not EFAULT because that's for userspace faults
            BatchError::Fault(_) => EIO,
            BatchError::Unknown => ENODATA,
            BatchError::Killed => ECANCELED,
        }
    }
}

pub(crate) struct Batch {
    value: event::EventValue,
    commands: usize,
    // TODO: make abstraction
    completion: Opaque<bindings::completion>,
    wptr: u32,
    vm_slot: u32,
    error: smutex::Mutex<Option<BatchError>>,
}

impl Batch {
    pub(crate) fn value(&self) -> event::EventValue {
        self.value
    }

    pub(crate) fn wait(&self) -> core::result::Result<(), BatchError> {
        unsafe { bindings::wait_for_completion(self.completion.get()) };
        self.error.lock().map_or(Ok(()), Err)
    }
}

struct WorkQueueInner {
    event_manager: Arc<event::EventManager>,
    info: GpuObject<QueueInfo>,
    new: bool,
    pipe_type: PipeType,
    size: u32,
    wptr: u32,
    pending: Vec<Box<dyn object::OpaqueGpuObject>>,
    batches: Vec<Arc<Batch>>,
    last_token: Option<event::Token>,
    event: Option<(event::Event, event::EventValue)>,
}

unsafe impl Send for WorkQueueInner {}

pub(crate) struct WorkQueue {
    info_pointer: GpuWeakPointer<QueueInfo>,
    inner: Mutex<WorkQueueInner>,
    cond: CondVar,
}

const WQ_SIZE: u32 = 0x500;

impl WorkQueueInner {
    fn doneptr(&self) -> u32 {
        self.info
            .state
            .with(|raw, _inner| raw.gpu_doneptr.load(Ordering::Acquire))
    }
}

pub(crate) struct WorkQueueBatch<'a> {
    queue: &'a WorkQueue,
    inner: Guard<'a, Mutex<WorkQueueInner>>,
    commands: usize,
    wptr: u32,
    vm_slot: u32,
}

impl WorkQueue {
    pub(crate) fn new(
        alloc: &mut gpu::KernelAllocators,
        event_manager: Arc<event::EventManager>,
        gpu_context: GpuWeakPointer<GpuContextData>,
        notifier_list: GpuWeakPointer<NotifierList>,
        pipe_type: PipeType,
        id: u64,
    ) -> Result<Arc<WorkQueue>> {
        let mut info = box_in_place!(QueueInfo {
            state: alloc.shared.new_default::<RingState>()?,
            ring: alloc.shared.array_empty(WQ_SIZE as usize)?,
            gpu_buf: alloc.private.array_empty(0x2c18)?,
        })?;

        info.state.with_mut(|raw, _inner| {
            raw.rb_size = WQ_SIZE;
        });

        let inner = WorkQueueInner {
            event_manager,
            info: alloc.private.new_boxed(info, |inner, ptr| {
                Ok(place!(
                    ptr,
                    raw::QueueInfo {
                        state: inner.state.gpu_pointer(),
                        ring: inner.ring.gpu_pointer(),
                        notifier_list: notifier_list,
                        gpu_buf: inner.gpu_buf.gpu_pointer(),
                        gpu_rptr1: Default::default(),
                        gpu_rptr2: Default::default(),
                        gpu_rptr3: Default::default(),
                        event_id: AtomicI32::new(-1),
                        priority: Default::default(),
                        unk_4c: -1,
                        uuid: id as u32,
                        unk_54: -1,
                        unk_58: Default::default(),
                        busy: Default::default(),
                        __pad: Default::default(),
                        unk_84_state: Default::default(),
                        unk_88: Default::default(),
                        unk_8c: Default::default(),
                        unk_90: Default::default(),
                        unk_94: Default::default(),
                        pending: Default::default(),
                        unk_9c: Default::default(),
                        gpu_context: gpu_context,
                        unk_a8: Default::default(),
                    }
                ))
            })?,
            new: true,
            pipe_type,
            size: WQ_SIZE,
            wptr: 0,
            pending: Vec::new(),
            batches: Vec::new(),
            last_token: None,
            event: None,
        };

        let mut queue = Pin::from(UniqueArc::try_new(Self {
            info_pointer: inner.info.weak_pointer(),
            // SAFETY: `condvar_init!` is called below.
            cond: unsafe { CondVar::new() },
            // SAFETY: `mutex_init!` is called below.
            inner: unsafe { Mutex::new(inner) },
        })?);

        // SAFETY: `cond` is pinned when `queue` is.
        let pinned = unsafe { queue.as_mut().map_unchecked_mut(|s| &mut s.cond) };
        match pipe_type {
            PipeType::Vertex => kernel::condvar_init!(pinned, "WorkQueue::cond (Vertex)"),
            PipeType::Fragment => kernel::condvar_init!(pinned, "WorkQueue::cond (Fragment)"),
            PipeType::Compute => kernel::condvar_init!(pinned, "WorkQueue::cond (Compute)"),
        }

        // SAFETY: `inner` is pinned when `queue` is.
        let pinned = unsafe { queue.as_mut().map_unchecked_mut(|s| &mut s.inner) };
        match pipe_type {
            PipeType::Vertex => kernel::mutex_init!(pinned, "WorkQueue::inner (Vertex)"),
            PipeType::Fragment => kernel::mutex_init!(pinned, "WorkQueue::inner (Fragment)"),
            PipeType::Compute => kernel::mutex_init!(pinned, "WorkQueue::inner (Compute)"),
        }

        Ok(queue.into())
    }

    pub(crate) fn info_pointer(&self) -> GpuWeakPointer<QueueInfo> {
        self.info_pointer
    }

    pub(crate) fn begin_batch(this: &Arc<WorkQueue>, vm_slot: u32) -> Result<WorkQueueBatch<'_>> {
        let mut inner = this.inner.lock();

        if inner.event.is_none() {
            let event = inner.event_manager.get(inner.last_token, this.clone())?;
            let cur = event.current();
            inner.last_token = Some(event.token());
            inner.event = Some((event, cur));
        }

        Ok(WorkQueueBatch {
            queue: &*this,
            wptr: inner.wptr,
            inner,
            commands: 0,
            vm_slot,
        })
    }

    pub(crate) fn signal(&self) -> bool {
        let mut inner = self.inner.lock();
        let event = inner.event.as_ref();
        let cur_value = match event {
            None => {
                pr_err!("WorkQueue: signal() called but no event?");
                return true;
            }
            Some(event) => event.0.current(),
        };

        mod_pr_debug!(
            "WorkQueue({:?}): Signaling event {:?} value {:#x?}",
            inner.pipe_type,
            inner.last_token,
            cur_value
        );

        let mut completed_commands: usize = 0;
        let mut batches: usize = 0;

        for batch in inner.batches.iter() {
            if batch.value <= cur_value {
                mod_pr_debug!(
                    "WorkQueue({:?}): Batch at value {:#x?} complete",
                    inner.pipe_type,
                    batch.value
                );
                completed_commands += batch.commands;
                batches += 1;
            } else {
                break;
            }
        }
        mod_pr_debug!(
            "WorkQueue({:?}): Completed {} batches",
            inner.pipe_type,
            batches
        );

        let mut completed = Vec::new();
        for i in inner.batches.drain(..batches) {
            if completed.try_push(i).is_err() {
                pr_err!("Failed to signal completions");
                break;
            }
        }
        if let Some(i) = completed.last() {
            inner
                .info
                .state
                .with(|raw, _inner| raw.cpu_freeptr.store((*i).wptr, Ordering::Release));
        }

        inner.pending.drain(..completed_commands);
        self.cond.notify_all();
        let empty = inner.batches.is_empty();
        if empty {
            inner.event = None;
        }
        core::mem::drop(inner);

        for batch in completed {
            unsafe { bindings::complete_all(batch.completion.get()) };
        }
        empty
    }

    pub(crate) fn mark_error(&self, value: event::EventValue, error: BatchError) {
        // If anything is marked completed, we can consider it successful
        // at this point, even if we didn't get the signal event yet.
        self.signal();

        let inner = self.inner.lock();

        if inner.event.is_none() {
            pr_err!("WorkQueue: signal_fault() called but no event?");
            return;
        }

        mod_pr_debug!(
            "WorkQueue({:?}): Signaling fault for event {:?} at value {:#x?}",
            inner.pipe_type,
            inner.last_token,
            value
        );

        for batch in inner.batches.iter() {
            if batch.value <= value {
                mod_pr_debug!(
                    "WorkQueue({:?}): Batch at value {:#x?} failed ({} commands)",
                    inner.pipe_type,
                    batch.value,
                    batch.commands,
                );
                *(batch.error.lock()) = Some(match error {
                    BatchError::Fault(info) if info.vm_slot != batch.vm_slot => BatchError::Killed,
                    err => err,
                });
            } else {
                break;
            }
        }
    }
}

impl<'a> WorkQueueBatch<'a> {
    pub(crate) fn add<T: Command>(&mut self, command: Box<GpuObject<T>>) -> Result {
        let inner = &mut self.inner;

        let next_wptr = (self.wptr + 1) % inner.size;
        if inner.doneptr() == next_wptr {
            pr_err!("Work queue ring buffer is full! Waiting...");
            while inner.doneptr() == next_wptr {
                if self.queue.cond.wait(inner) {
                    return Err(ERESTARTSYS);
                }
            }
        }
        inner.pending.try_reserve(1)?;

        inner.info.ring[self.wptr as usize] = command.gpu_va().get();

        self.wptr = next_wptr;

        // Cannot fail, since we did a try_reserve(1) above
        inner
            .pending
            .try_push(command)
            .expect("try_push() failed after try_reserve(1)");
        self.commands += 1;
        Ok(())
    }

    pub(crate) fn commit(&mut self) -> Result<Arc<Batch>> {
        let inner = &mut self.inner;
        inner.batches.try_reserve(1)?;

        let event = inner.event.as_mut().expect("WorkQueueBatch lost its event");

        if self.commands == 0 {
            return Err(EINVAL);
        }

        event.1.increment();
        let event_value = event.1;

        inner
            .info
            .state
            .with(|raw, _inner| raw.cpu_wptr.store(self.wptr, Ordering::Release));

        inner.wptr = self.wptr;
        let batch = Arc::try_new(Batch {
            value: event_value,
            commands: self.commands,
            completion: Opaque::uninit(),
            wptr: self.wptr,
            error: smutex::Mutex::new(None),
            vm_slot: self.vm_slot,
        })?;
        unsafe { bindings::init_completion(batch.completion.get()) };
        inner.batches.try_push(batch.clone())?;
        self.commands = 0;
        Ok(batch)
    }

    pub(crate) fn submit(mut self, channel: &mut channel::PipeChannel) -> Result {
        if self.commands != 0 {
            return Err(EINVAL);
        }

        let inner = &mut self.inner;
        let event = inner.event.as_ref().expect("WorkQueueBatch lost its event");
        let msg = RunWorkQueueMsg {
            pipe_type: inner.pipe_type,
            work_queue: Some(inner.info.weak_pointer()),
            wptr: inner.wptr,
            event_slot: event.0.slot(),
            is_new: inner.new,
            __pad: Default::default(),
        };
        channel.send(&msg);
        inner.new = false;
        Ok(())
    }

    pub(crate) fn event(&self) -> &event::Event {
        let event = self
            .inner
            .event
            .as_ref()
            .expect("WorkQueueBatch lost its event");
        &(event.0)
    }

    pub(crate) fn event_value(&self) -> event::EventValue {
        let event = self
            .inner
            .event
            .as_ref()
            .expect("WorkQueueBatch lost its event");
        event.1
    }

    pub(crate) fn pipe_type(&self) -> PipeType {
        self.inner.pipe_type
    }
}

impl<'a> Drop for WorkQueueBatch<'a> {
    fn drop(&mut self) {
        if self.commands != 0 {
            pr_warn!("WorkQueueBatch: rolling back {} commands!", self.commands);

            let inner = &mut self.inner;
            let new_len = inner.pending.len() - self.commands;
            inner.pending.truncate(new_len);
        }
    }
}

unsafe impl Send for WorkQueue {}
unsafe impl Sync for WorkQueue {}
