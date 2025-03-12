/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Landlock LSM - Audit helpers
 *
 * Copyright © 2023-2025 Microsoft Corporation
 */

#ifndef _SECURITY_LANDLOCK_AUDIT_H
#define _SECURITY_LANDLOCK_AUDIT_H

#include <linux/audit.h>
#include <linux/lsm_audit.h>

#include "cred.h"
#include "ruleset.h"

enum landlock_request_type {
	LANDLOCK_REQUEST_PTRACE = 1,
	LANDLOCK_REQUEST_FS_CHANGE_LAYOUT,
};

/*
 * We should be careful to only use a variable of this type for
 * landlock_log_denial().  This way, the compiler can remove it entirely if
 * CONFIG_AUDIT is not set.
 */
struct landlock_request {
	/* Mandatory fields. */
	enum landlock_request_type type;
	struct common_audit_data audit;

	/**
	 * layer_plus_one: First layer level that denies the request + 1.  The
	 * extra one is useful to detect uninitialized field.
	 */
	size_t layer_plus_one;
};

#ifdef CONFIG_AUDIT

void landlock_log_drop_domain(const struct landlock_ruleset *const domain);

void landlock_log_denial(const struct landlock_cred_security *const subject,
			 const struct landlock_request *const request);

#else /* CONFIG_AUDIT */

static inline void
landlock_log_drop_domain(const struct landlock_ruleset *const domain)
{
}

static inline void
landlock_log_denial(const struct landlock_cred_security *const subject,
		    const struct landlock_request *const request)
{
}

#endif /* CONFIG_AUDIT */

#endif /* _SECURITY_LANDLOCK_AUDIT_H */
