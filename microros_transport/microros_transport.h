#ifndef MICROROS_TRANSPORT_H
#define MICROROS_TRANSPORT_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * Transport selection (Makefile C_DEFS):
 *   COMM_USE_VCP  — USB CDC, shared with vision communication
 *   COMM_USE_UART — USART10 DMA, shared with vision communication
 */

/* transport layer API (custom micro-ROS XRCE transport, matching uxr custom transport signatures) */
struct uxrCustomTransport;
bool microros_transport_open(struct uxrCustomTransport *transport);
bool microros_transport_close(struct uxrCustomTransport *transport);
size_t microros_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode);
size_t microros_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout_ms, uint8_t *errcode);

#ifdef MICRO_ROS_ENABLED
#include <rcl/types.h>
rcl_ret_t microros_publish_vision(void);
#endif

#endif
