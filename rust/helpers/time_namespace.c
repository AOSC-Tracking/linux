// SPDX-License-Identifier: GPL-2.0

#include <linux/time_namespace.h>

void rust_helper_timens_add_monotonic(struct timespec64 *ts) {
	timens_add_monotonic(ts);
}
