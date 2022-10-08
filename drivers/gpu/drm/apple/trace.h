// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright (C) The Asahi Linux Contributors */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM dcp

#if !defined(_TRACE_DCP_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_DCP_H

#include "dcp-internal.h"

#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#define show_dcp_endpoint(ep)                                      \
	__print_symbolic(ep, { SYSTEM_ENDPOINT, "system" },        \
			 { TEST_ENDPOINT, "test" },                \
			 { DCP_EXPERT_ENDPOINT, "dcpexpert" },     \
			 { DISP0_ENDPOINT, "disp0" },              \
			 { DPTX_ENDPOINT, "dptxport" },            \
			 { HDCP_ENDPOINT, "hdcp" },                \
			 { REMOTE_ALLOC_ENDPOINT, "remotealloc" }, \
			 { IOMFB_ENDPOINT, "iomfb" })

TRACE_EVENT(dcp_recv_msg,
	    TP_PROTO(struct apple_dcp *dcp, u8 endpoint, u64 message),
	    TP_ARGS(dcp, endpoint, message),

	    TP_STRUCT__entry(__string(devname, dev_name(dcp->dev))
			     __field(u8, endpoint)
			     __field(u64, message)),

	    TP_fast_assign(__assign_str(devname);
			   __entry->endpoint = endpoint;
			   __entry->message = message;),

	    TP_printk("%s: endpoint 0x%x (%s): received message 0x%016llx",
		      __get_str(devname), __entry->endpoint,
		      show_dcp_endpoint(__entry->endpoint), __entry->message));

TRACE_EVENT(dcp_send_msg,
	    TP_PROTO(struct apple_dcp *dcp, u8 endpoint, u64 message),
	    TP_ARGS(dcp, endpoint, message),

	    TP_STRUCT__entry(__string(devname, dev_name(dcp->dev))
			     __field(u8, endpoint)
			     __field(u64, message)),

	    TP_fast_assign(__assign_str(devname);
			   __entry->endpoint = endpoint;
			   __entry->message = message;),

	    TP_printk("%s: endpoint 0x%x (%s): will send message 0x%016llx",
		      __get_str(devname), __entry->endpoint,
		      show_dcp_endpoint(__entry->endpoint), __entry->message));

TRACE_EVENT(iomfb_callback,
	    TP_PROTO(struct apple_dcp *dcp, int tag, const char *name),
	    TP_ARGS(dcp, tag, name),

	    TP_STRUCT__entry(
				__string(devname, dev_name(dcp->dev))
				__field(int, tag)
				__field(const char *, name)
			),

	    TP_fast_assign(
				__assign_str(devname);
				__entry->tag = tag; __entry->name = name;
			),

	    TP_printk("%s: Callback D%03d %s", __get_str(devname), __entry->tag,
		      __entry->name));

TRACE_EVENT(iomfb_push,
	    TP_PROTO(struct apple_dcp *dcp,
		     const struct dcp_method_entry *method, int context,
		     int offset, int depth),
	    TP_ARGS(dcp, method, context, offset, depth),

	    TP_STRUCT__entry(
				__string(devname, dev_name(dcp->dev))
				__string(name, method->name)
				__field(int, context)
				__field(int, offset)
				__field(int, depth)),

	    TP_fast_assign(
				__assign_str(devname);
				__assign_str(name);
				__entry->context = context; __entry->offset = offset;
				__entry->depth = depth;
			),

	    TP_printk("%s: Method %s: context %u, offset %u, depth %u",
		      __get_str(devname), __get_str(name), __entry->context,
		      __entry->offset, __entry->depth));

TRACE_EVENT(iomfb_swap_submit,
	    TP_PROTO(struct apple_dcp *dcp, u32 swap_id),
	    TP_ARGS(dcp, swap_id),
	    TP_STRUCT__entry(
			     __field(u64, dcp)
			     __field(u32, swap_id)
	    ),
	    TP_fast_assign(
			   __entry->dcp = (u64)dcp;
			   __entry->swap_id = swap_id;
	    ),
	    TP_printk("dcp=%llx, swap_id=%d",
		      __entry->dcp,
		      __entry->swap_id)
);

TRACE_EVENT(iomfb_swap_complete,
	    TP_PROTO(struct apple_dcp *dcp, u32 swap_id),
	    TP_ARGS(dcp, swap_id),
	    TP_STRUCT__entry(
			     __field(u64, dcp)
			     __field(u32, swap_id)
	    ),
	    TP_fast_assign(
			   __entry->dcp = (u64)dcp;
			   __entry->swap_id = swap_id;
	    ),
	    TP_printk("dcp=%llx, swap_id=%d",
		      __entry->dcp,
		      __entry->swap_id
	    )
);

TRACE_EVENT(iomfb_swap_complete_intent_gated,
	    TP_PROTO(struct apple_dcp *dcp, u32 swap_id, u32 width, u32 height),
	    TP_ARGS(dcp, swap_id, width, height),
	    TP_STRUCT__entry(
			     __field(u64, dcp)
			     __field(u32, swap_id)
			     __field(u32, width)
			     __field(u32, height)
	    ),
	    TP_fast_assign(
			   __entry->dcp = (u64)dcp;
			   __entry->swap_id = swap_id;
			   __entry->height = height;
			   __entry->width = width;
	    ),
	    TP_printk("dcp=%llx, swap_id=%u %ux%u",
		      __entry->dcp,
		      __entry->swap_id,
		      __entry->width,
		      __entry->height
	    )
);

#endif /* _TRACE_DCP_H */

/* This part must be outside protection */

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#include <trace/define_trace.h>
