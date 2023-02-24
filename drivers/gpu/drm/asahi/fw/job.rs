// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(dead_code)]

//! Common job structures

use super::types::*;

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub(crate) struct JobMeta {
    pub(crate) unk_4: u32,
    pub(crate) stamp: GpuWeakPointer<Stamp>,
    pub(crate) fw_stamp: GpuWeakPointer<FwStamp>,
    pub(crate) stamp_value: EventValue,
    pub(crate) stamp_slot: u32,
    pub(crate) unk_20: u32,
    pub(crate) unk_24: u32,
    pub(crate) uuid: u32,
    pub(crate) prev_stamp_value: u32,
    pub(crate) unk_30: u32,
    pub(crate) unk_buf_0: U64,
    pub(crate) unk_buf_8: U64,
    pub(crate) unk_buf_10: U64,
    pub(crate) ts1: U64,
    pub(crate) ts2: U64,
    pub(crate) ts3: U64,
}

#[derive(Debug)]
#[repr(C)]
pub(crate) struct EncoderParams<'a> {
    pub(crate) unk_8: u32,
    pub(crate) unk_c: u32,
    pub(crate) unk_10: u32,
    pub(crate) encoder_id: u32,
    pub(crate) unk_18: u32,
    pub(crate) unk_1c: u32,
    pub(crate) seq_buffer: GpuPointer<'a, &'a [u64]>,
    pub(crate) unk_28: U64,
}
