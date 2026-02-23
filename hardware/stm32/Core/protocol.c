/**
 * protocol.c
 * Parses ASCII lines from Jetson: M,w_fl,w_fr,w_rl,w_rr and L,0|1.
 * Copyright (c) 2025 Tarik Kahraman. SPDX-License-Identifier: MIT
 */

#include "protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char line_buf[PROTOCOL_LINE_MAX];
static int line_len;
static uint32_t last_rx_tick;

void protocol_feed_byte(uint8_t byte)
{
    /* Call protocol_set_last_rx_tick(HAL_GetTick()) from your UART RX callback for watchdog. */
    if (byte == '\n' || byte == '\r')
    {
        if (line_len > 0)
        {
            line_buf[line_len] = '\0';
            if (line_buf[0] == 'M')
            {
                float w[PROTOCOL_NUM_WHEELS];
                int n = sscanf(line_buf, "M,%f,%f,%f,%f",
                              &w[0], &w[1], &w[2], &w[3]);
                if (n == PROTOCOL_NUM_WHEELS)
                    motor_set_speeds(w[0], w[1], w[2], w[3]);
            }
            else if (line_buf[0] == 'L')
            {
                int up = 0;
                if (sscanf(line_buf, "L,%d", &up) >= 1)
                    lift_set(up ? 1 : 0);
            }
            line_len = 0;
        }
        return;
    }
    if (line_len < (int)(sizeof(line_buf) - 1))
        line_buf[line_len++] = (char)byte;
    else
        line_len = 0; /* overflow: discard */
}

uint32_t protocol_last_rx_tick(void)
{
    return last_rx_tick;
}

/* Optional: call from your code to update tick for watchdog. */
void protocol_set_last_rx_tick(uint32_t tick)
{
    last_rx_tick = tick;
}
