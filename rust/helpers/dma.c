// SPDX-License-Identifier: GPL-2.0

#include <linux/dma-mapping.h>

int rust_helper_dma_set_mask_and_coherent(struct device *dev, u64 mask)
{
	return dma_set_mask_and_coherent(dev, mask);
}
