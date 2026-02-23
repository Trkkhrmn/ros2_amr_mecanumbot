/**
 * protocol.h - UART protocol parser for Jetson <-> STM32 link.
 * Receives ASCII lines: M,w_fl,w_fr,w_rl,w_rr and L,0 or L,1.
 * Copyright (c) 2025 Tarik Kahraman. SPDX-License-Identifier: MIT
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define PROTOCOL_LINE_MAX  128
#define PROTOCOL_NUM_WHEELS 4

void motor_set_speeds(float w_fl, float w_fr, float w_rl, float w_rr);
void lift_set(int up);

void protocol_feed_byte(uint8_t byte);
uint32_t protocol_last_rx_tick(void);
void protocol_set_last_rx_tick(uint32_t tick);

#endif
