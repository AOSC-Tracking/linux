// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Common GPU job firmware structures

use super::types::*;
use crate::{default_zeroed, trivial_gpustruct};

pub(crate) mod raw {
    use super::*;

    #[derive(Debug, Clone, Copy)]
    #[repr(C)]
    pub(crate) struct JobMeta {
        pub(crate) unk_0: u16,
        pub(crate) unk_2: u8,
        pub(crate) no_preemption: u8,
        pub(crate) stamp: GpuWeakPointer<Stamp>,
        pub(crate) fw_stamp: GpuWeakPointer<FwStamp>,
        pub(crate) stamp_value: EventValue,
        pub(crate) stamp_slot: u32,
        pub(crate) evctl_index: u32,
        pub(crate) flush_stamps: u32,
        pub(crate) uuid: u32,
        pub(crate) event_seq: u32,
    }

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct EncoderParams<'a> {
        pub(crate) unk_8: u32,
        pub(crate) unk_c: u32,
        pub(crate) unk_10: u32,
        pub(crate) encoder_id: u32,
        pub(crate) unk_18: u32,
        pub(crate) iogpu_compute_unk44: u32,
        pub(crate) seq_buffer: GpuPointer<'a, &'a [u64]>,
        pub(crate) unk_28: U64,
    }

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct JobTimestamps {
        pub(crate) start: AtomicU64,
        pub(crate) end: AtomicU64,
    }
    default_zeroed!(JobTimestamps);

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct RenderTimestamps {
        pub(crate) vtx: JobTimestamps,
        pub(crate) frag: JobTimestamps,
    }
    default_zeroed!(RenderTimestamps);
}

trivial_gpustruct!(JobTimestamps);
trivial_gpustruct!(RenderTimestamps);
