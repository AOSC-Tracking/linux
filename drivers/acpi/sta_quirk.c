// SPDX-License-Identifier: GPL-2.0-only
/*
 * ACPI _STA overrides for firmware bugs (non-arch-specific).
 *
 * CIX: unhide SCMI protocol child devices (CIXHA008/CIXHA009) that old
 * firmware leaves at _STA 0 under CIXHA006.
 */

#include <linux/acpi.h>
#include <acpi/acpi_bus.h>

#include "internal.h"

/*
 * Old CIX firmware leaves DVFS/CLKS SCMI protocol devices (children of
 * CIXHA006) at _STA 0 so Linux skips them in acpi_bus_attach() and
 * acpi_get_next_present_subnode(). Treat them as fully present/enabled.
 */
static const struct acpi_device_id cix_scmi_proto_sta_ids[] = {
	{ "CIXHA008", 0 },
	{ "CIXHA009", 0 },
	{ }
};

bool acpi_sta_override_firmware_quirk(struct acpi_device *adev,
				       unsigned long long *status)
{
	if (acpi_match_device_ids(adev, cix_scmi_proto_sta_ids))
		return false;

	if (!acpi_dev_uid_match(adev, 0))
		return false;

	*status = ACPI_STA_DEFAULT;
	return true;
}
