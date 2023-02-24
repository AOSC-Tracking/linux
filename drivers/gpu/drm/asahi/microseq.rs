// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(dead_code)]

//! GPU Micro operation sequence builder

use crate::fw::microseq;
pub(crate) use crate::fw::microseq::*;
use crate::fw::types::*;
use kernel::prelude::*;

pub(crate) type MicroSequence = GpuArray<u8>;

pub(crate) struct Builder {
    ops: Vec<u8>,
}

impl Builder {
    pub(crate) fn new() -> Builder {
        Builder { ops: Vec::new() }
    }

    pub(crate) fn offset_to(&self, target: i32) -> i32 {
        target - self.ops.len() as i32
    }

    pub(crate) fn add<T: microseq::Operation>(&mut self, op: T) -> Result<i32> {
        let off = self.ops.len();
        let p: *const T = &op;
        let p: *const u8 = p as *const u8;
        let s: &[u8] = unsafe { core::slice::from_raw_parts(p, core::mem::size_of::<T>()) };
        self.ops.try_extend_from_slice(s)?;
        Ok(off as i32)
    }

    pub(crate) fn build(self, alloc: &mut Allocator) -> Result<MicroSequence> {
        let mut array = alloc.array_empty::<u8>(self.ops.len())?;

        array.as_mut_slice().clone_from_slice(self.ops.as_slice());
        Ok(array)
    }
}
