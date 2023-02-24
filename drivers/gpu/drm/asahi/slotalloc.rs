// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Generic slot allocator

use crate::debug::*;
use core::ops::{Deref, DerefMut};
use kernel::{
    error::Result,
    prelude::*,
    sync::{Arc, CondVar, Mutex, UniqueArc},
};

const DEBUG_CLASS: DebugFlags = DebugFlags::SlotAlloc;

pub(crate) trait SlotItem {
    type Owner;

    fn release(&mut self, _owner: &mut Self::Owner, _slot: u32) {}
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct SlotToken {
    time: u64,
    slot: u32,
}

impl SlotToken {
    pub(crate) fn last_slot(&self) -> u32 {
        self.slot
    }
}

pub(crate) struct Guard<T: SlotItem> {
    item: Option<T>,
    changed: bool,
    token: SlotToken,
    alloc: Arc<SlotAllocatorOuter<T>>,
}

impl<T: SlotItem> Guard<T> {
    pub(crate) fn slot(&self) -> u32 {
        self.token.slot
    }

    pub(crate) fn changed(&self) -> bool {
        self.changed
    }

    pub(crate) fn token(&self) -> SlotToken {
        self.token
    }
}

impl<T: SlotItem> Deref for Guard<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.item.as_ref().expect("SlotItem Guard lost our item!")
    }
}

impl<T: SlotItem> DerefMut for Guard<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.item.as_mut().expect("SlotItem Guard lost our item!")
    }
}

struct Entry<T: SlotItem> {
    item: T,
    get_time: u64,
    drop_time: u64,
}

struct SlotAllocatorInner<T: SlotItem> {
    owner: T::Owner,
    slots: Vec<Option<Entry<T>>>,
    get_count: u64,
    drop_count: u64,
}

pub(crate) struct SlotAllocatorOuter<T: SlotItem> {
    inner: Mutex<SlotAllocatorInner<T>>,
    cond: CondVar,
}

pub(crate) struct SlotAllocator<T: SlotItem>(Arc<SlotAllocatorOuter<T>>);

impl<T: SlotItem> SlotAllocator<T> {
    pub(crate) fn new(
        num_slots: u32,
        mut owner: T::Owner,
        mut constructor: impl FnMut(&mut T::Owner, u32) -> T,
    ) -> Result<SlotAllocator<T>> {
        let mut slots = Vec::try_with_capacity(num_slots as usize)?;

        for i in 0..num_slots {
            slots
                .try_push(Some(Entry {
                    item: constructor(&mut owner, i),
                    get_time: 0,
                    drop_time: 0,
                }))
                .expect("try_push() failed after reservation");
        }

        let inner = SlotAllocatorInner {
            owner,
            slots,
            get_count: 0,
            drop_count: 0,
        };

        let mut alloc = Pin::from(UniqueArc::try_new(SlotAllocatorOuter {
            // SAFETY: `condvar_init!` is called below.
            cond: unsafe { CondVar::new() },
            // SAFETY: `mutex_init!` is called below.
            inner: unsafe { Mutex::new(inner) },
        })?);

        // SAFETY: `cond` is pinned when `alloc` is.
        let pinned = unsafe { alloc.as_mut().map_unchecked_mut(|s| &mut s.cond) };
        kernel::condvar_init!(pinned, "SlotAllocator::cond");

        // SAFETY: `inner` is pinned when `alloc` is.
        let pinned = unsafe { alloc.as_mut().map_unchecked_mut(|s| &mut s.inner) };
        kernel::mutex_init!(pinned, "SlotAllocator::inner");

        Ok(SlotAllocator(alloc.into()))
    }

    pub(crate) fn with_inner<RetVal>(&self, cb: impl FnOnce(&mut T::Owner) -> RetVal) -> RetVal {
        let mut inner = self.0.inner.lock();
        cb(&mut inner.owner)
    }

    pub(crate) fn get(&self, token: Option<SlotToken>) -> Result<Guard<T>> {
        self.get_inner(token, |_a, _b| Ok(()))
    }

    pub(crate) fn get_inner(
        &self,
        token: Option<SlotToken>,
        cb: impl FnOnce(&mut T::Owner, &mut Guard<T>) -> Result<()>,
    ) -> Result<Guard<T>> {
        let mut inner = self.0.inner.lock();

        if let Some(token) = token {
            let slot = &mut inner.slots[token.slot as usize];
            if slot.is_some() {
                let count = slot.as_ref().unwrap().get_time;
                if count == token.time {
                    let mut guard = Guard {
                        item: Some(slot.take().unwrap().item),
                        token,
                        changed: false,
                        alloc: self.0.clone(),
                    };
                    cb(&mut inner.owner, &mut guard)?;
                    return Ok(guard);
                }
            }
        }

        let mut first = true;
        let slot = loop {
            let mut oldest_time = u64::MAX;
            let mut oldest_slot = 0u32;

            for (i, slot) in inner.slots.iter().enumerate() {
                if let Some(slot) = slot.as_ref() {
                    if slot.drop_time < oldest_time {
                        oldest_slot = i as u32;
                        oldest_time = slot.drop_time;
                    }
                }
            }

            if oldest_time == u64::MAX {
                if first {
                    pr_warn!("{}: out of slots, blocking", core::any::type_name::<Self>());
                }
                first = false;
                if self.0.cond.wait(&mut inner) {
                    return Err(ERESTARTSYS);
                }
            } else {
                break oldest_slot;
            }
        };

        inner.get_count += 1;

        let item = inner.slots[slot as usize]
            .take()
            .expect("Someone stole our slot?")
            .item;

        let mut guard = Guard {
            item: Some(item),
            changed: true,
            token: SlotToken {
                time: inner.get_count,
                slot,
            },
            alloc: self.0.clone(),
        };

        cb(&mut inner.owner, &mut guard)?;
        Ok(guard)
    }
}

impl<T: SlotItem> Clone for SlotAllocator<T> {
    fn clone(&self) -> Self {
        SlotAllocator(self.0.clone())
    }
}

impl<T: SlotItem> Drop for Guard<T> {
    fn drop(&mut self) {
        let mut inner = self.alloc.inner.lock();
        if inner.slots[self.token.slot as usize].is_some() {
            pr_crit!(
                "{}: tried to return an item into a full slot ({})",
                core::any::type_name::<Self>(),
                self.token.slot
            );
        } else {
            inner.drop_count += 1;
            let mut item = self.item.take().expect("Guard lost its item");
            item.release(&mut inner.owner, self.token.slot);
            inner.slots[self.token.slot as usize] = Some(Entry {
                item,
                get_time: self.token.time,
                drop_time: inner.drop_count,
            });
            self.alloc.cond.notify_one();
        }
    }
}
