// SPDX-License-Identifier: GPL-2.0
/*
 * Non-trivial C macros cannot be used in Rust. Similarly, inlined C functions
 * cannot be called either. This file explicitly creates functions ("helpers")
 * that wrap those so that they can be called from Rust.
 *
 * Sorted alphabetically.
 */

#include "blk.c"
#include "bug.c"
#include "build_assert.c"
#include "build_bug.c"
#include "device.c"
#include "dma-fence.c"
#include "dma-mapping.c"
#include "dma-resv.c"
#include "drm.c"
#include "drm_gpuvm.c"
#include "drm_syncobj.c"
#include "err.c"
#include "io.c"
#include "iomem.c"
#include "ioport.c"
#include "jiffies.c"
#include "kunit.c"
#include "lockdep.c"
#include "mutex.c"
#include "of.c"
#include "page.c"
#include "platform.c"
#include "pci.c"
#include "rbtree.c"
#include "rcu.c"
#include "refcount.c"
#include "scatterlist.c"
#include "signal.c"
#include "siphash.c"
#include "slab.c"
#include "spinlock.c"
#include "task.c"
#include "time_namespace.c"
#include "timekeeping.c"
#include "uaccess.c"
#include "vmalloc.c"
#include "wait.c"
#include "workqueue.c"
#include "xarray.c"
