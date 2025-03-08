/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Landlock LSM - Credential hooks
 *
 * Copyright © 2019-2020 Mickaël Salaün <mic@digikod.net>
 * Copyright © 2019-2020 ANSSI
 */

#ifndef _SECURITY_LANDLOCK_CRED_H
#define _SECURITY_LANDLOCK_CRED_H

#include <linux/cred.h>
#include <linux/init.h>
#include <linux/rcupdate.h>

#include "access.h"
#include "ruleset.h"
#include "setup.h"

struct landlock_cred_security {
	struct landlock_ruleset *domain;
};

static inline struct landlock_cred_security *
landlock_cred(const struct cred *cred)
{
	return cred->security + landlock_blob_sizes.lbs_cred;
}

static inline struct landlock_ruleset *landlock_get_current_domain(void)
{
	return landlock_cred(current_cred())->domain;
}

/*
 * The call needs to come from an RCU read-side critical section.
 */
static inline const struct landlock_ruleset *
landlock_get_task_domain(const struct task_struct *const task)
{
	return landlock_cred(__task_cred(task))->domain;
}

static inline bool landlocked(const struct task_struct *const task)
{
	bool has_dom;

	if (task == current)
		return !!landlock_get_current_domain();

	rcu_read_lock();
	has_dom = !!landlock_get_task_domain(task);
	rcu_read_unlock();
	return has_dom;
}

/**
 * landlock_get_applicable_subject - Return the subject's Landlock credential
 *                                   if its enforced domain applies to (i.e.
 *                                   handles) at least one of the access rights
 *                                   specified in @masks
 *
 * @cred: credential
 * @masks: access masks
 * @handle_layer: returned youngest layer handling a subset of @masks.  Not set
 *                if the function returns NULL.
 *
 * Returns: landlock_cred(@cred) if any access rights specified in @masks is
 * handled, or NULL otherwise.
 */
static inline const struct landlock_cred_security *
landlock_get_applicable_subject(const struct cred *const cred,
				const struct access_masks masks,
				size_t *const handle_layer)
{
	const union access_masks_all masks_all = {
		.masks = masks,
	};
	const struct landlock_ruleset *domain;
	ssize_t layer_level;

	if (!cred)
		return NULL;

	domain = landlock_cred(cred)->domain;
	if (!domain)
		return NULL;

	for (layer_level = domain->num_layers - 1; layer_level >= 0;
	     layer_level--) {
		union access_masks_all layer = {
			.masks = domain->access_masks[layer_level],
		};

		if (layer.all & masks_all.all) {
			if (handle_layer)
				*handle_layer = layer_level;

			return landlock_cred(cred);
		}
	}

	return NULL;
}

__init void landlock_add_cred_hooks(void);

#endif /* _SECURITY_LANDLOCK_CRED_H */
