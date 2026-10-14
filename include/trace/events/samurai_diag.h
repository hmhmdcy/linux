/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM samurai_diag

#if !defined(_TRACE_SAMURAI_DIAG_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SAMURAI_DIAG_H
#include <linux/tracepoint.h>

TRACE_EVENT(samurai_core_transaction,
    TP_PROTO(unsigned int phase, unsigned int command, unsigned int token,
             unsigned int opcode, int bytes, unsigned int status, int result),
    TP_ARGS(phase, command, token, opcode, bytes, status, result),
    TP_STRUCT__entry(
        __field(unsigned int, phase)
        __field(unsigned int, command)
        __field(unsigned int, token)
        __field(unsigned int, opcode)
        __field(int, bytes)
        __field(unsigned int, status)
        __field(int, result)
    ),
    TP_fast_assign(
        __entry->phase = phase; __entry->command = command;
        __entry->token = token; __entry->opcode = opcode;
        __entry->bytes = bytes; __entry->status = status;
        __entry->result = result;
    ),
    TP_printk("phase=%u command=%#x token=%#x opcode=%#x bytes=%d status=%u result=%d",
              __entry->phase, __entry->command, __entry->token,
              __entry->opcode, __entry->bytes, __entry->status, __entry->result)
);

TRACE_EVENT(samurai_pdm_response,
    TP_PROTO(unsigned int node, unsigned int port, const char *service,
             unsigned int offset, unsigned int total, unsigned int returned, int result),
    TP_ARGS(node, port, service, offset, total, returned, result),
    TP_STRUCT__entry(
        __field(unsigned int, node)
        __field(unsigned int, port)
        __string(service, service)
        __field(unsigned int, offset)
        __field(unsigned int, total)
        __field(unsigned int, returned)
        __field(int, result)
    ),
    TP_fast_assign(
        __entry->node = node; __entry->port = port;
        __assign_str(service);
        __entry->offset = offset; __entry->total = total;
        __entry->returned = returned; __entry->result = result;
    ),
    TP_printk("node=%u port=%u service=%s offset=%u total=%u returned=%u result=%d",
              __entry->node, __entry->port, __get_str(service),
              __entry->offset, __entry->total, __entry->returned, __entry->result)
);

TRACE_EVENT(samurai_qrtr_rx_drop,
    TP_PROTO(int bytes, bool driver_ready, bool endpoint_ready),
    TP_ARGS(bytes, driver_ready, endpoint_ready),
    TP_STRUCT__entry(
        __field(int, bytes)
        __field(bool, driver_ready)
        __field(bool, endpoint_ready)
    ),
    TP_fast_assign(
        __entry->bytes = bytes; __entry->driver_ready = driver_ready;
        __entry->endpoint_ready = endpoint_ready;
    ),
    TP_printk("bytes=%d driver_ready=%u endpoint_ready=%u",
              __entry->bytes, __entry->driver_ready, __entry->endpoint_ready)
);
#endif
#include <trace/define_trace.h>
