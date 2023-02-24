// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![recursion_limit = "1024"]

//! Driver for the Apple AGX GPUs found in Apple Silicon SoCs.

mod alloc;
mod buffer;
mod channel;
mod debug;
mod driver;
mod event;
mod file;
mod float;
mod fw;
mod gem;
mod gpu;
mod hw;
mod initdata;
mod mem;
mod microseq;
mod mmu;
mod object;
mod place;
mod regs;
mod render;
mod slotalloc;
mod util;
mod workqueue;

use kernel::module_platform_driver;

module_platform_driver! {
    type: driver::AsahiDriver,
    name: "asahi",
    license: "Dual MIT/GPL",
    alias: [
        "of:N*T*Capple,agx-t8103C*",
        "of:N*T*Capple,agx-t8103",
        "of:N*T*Capple,agx-t8112C*",
        "of:N*T*Capple,agx-t8112",
        "of:N*T*Capple,agx-t6000C*",
        "of:N*T*Capple,agx-t6000",
        "of:N*T*Capple,agx-t6001C*",
        "of:N*T*Capple,agx-t6001",
        "of:N*T*Capple,agx-t6002C*",
        "of:N*T*Capple,agx-t6002",
    ],
    params: {
        debug_flags: u64 {
            default: 0,
            permissions: 0o644,
            description: "Debug flags",
        },
        fault_control: u32 {
            default: 0,
            permissions: 0,
            description: "Fault control (0x0: hard faults, 0xb: macOS default)",
        },
        initial_tvb_size: usize {
            default: 0x8,
            permissions: 0o644,
            description: "Initial TVB size in blocks",
        },
    },
}
