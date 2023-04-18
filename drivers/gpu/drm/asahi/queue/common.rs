// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Common queue functionality.
//!
//! Shared helpers used by the submission logic for multiple command types.

use crate::fw::microseq;
use crate::fw::types::*;

use kernel::bindings;
use kernel::io_buffer::IoBufferReader;
use kernel::prelude::*;
use kernel::user_ptr::UserSlicePtr;

use core::mem::MaybeUninit;

pub(super) fn build_attachments(pointer: u64, count: u32) -> Result<microseq::Attachments> {
    if count as usize > microseq::MAX_ATTACHMENTS {
        return Err(EINVAL);
    }

    const STRIDE: usize = core::mem::size_of::<bindings::drm_asahi_attachment>();
    let size = STRIDE * count as usize;

    // SAFETY: We only read this once, so there are no TOCTOU issues.
    let mut reader = unsafe { UserSlicePtr::new(pointer as usize as *mut _, size).reader() };

    let mut attachments: microseq::Attachments = Default::default();

    for i in 0..count {
        let mut att: MaybeUninit<bindings::drm_asahi_attachment> = MaybeUninit::uninit();

        // SAFETY: The size of `att` is STRIDE
        unsafe { reader.read_raw(att.as_mut_ptr() as *mut u8, STRIDE)? };

        // SAFETY: All bit patterns in the struct are valid
        let att = unsafe { att.assume_init() };

        let cache_lines = (att.size + 127) >> 7;
        let order = 1;
        attachments.list[i as usize] = microseq::Attachment {
            address: U64(att.pointer),
            size: cache_lines,
            unk_c: 0x17,
            unk_e: order,
        };

        attachments.count += 1;
    }

    Ok(attachments)
}
