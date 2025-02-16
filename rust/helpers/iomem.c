// SPDX-License-Identifier: GPL-2.0

#include <asm/io.h>

void __iomem *rust_helper_ioremap(resource_size_t offset, unsigned long size)
{
	return ioremap(offset, size);
}

void __iomem *rust_helper_ioremap_np(resource_size_t offset, unsigned long size)
{
	return ioremap_np(offset, size);
}

void rust_helper_memcpy_fromio(void *to, const volatile void __iomem *from, long count)
{
	memcpy_fromio(to, from, count);
}

void rust_helper_memcpy_toio(volatile void __iomem *to, const void *from, size_t count)
{
	memcpy_toio(to, from, count);
}
