// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple DART page table allocator.
 *
 * Copyright (C) 2022 The Asahi Linux Contributors
 */

/* This will go away on the next iteration of locked DART handling */

#ifndef IO_PGTABLE_DART_H_
#define IO_PGTABLE_DART_H_

int io_pgtable_dart_setup_locked(struct io_pgtable_ops *ops);

#endif /* IO_PGTABLE_DART_H_ */
