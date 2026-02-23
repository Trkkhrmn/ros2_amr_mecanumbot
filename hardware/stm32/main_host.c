/**
 * Host build entry point: exercises protocol parser (no hardware).
 * Build with: make -C hardware/stm32 build_host
 * Copyright (c) 2025 Tarik Kahraman. SPDX-License-Identifier: MIT
 */

#include "protocol.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t s_ticks = 0;
uint32_t get_tick_ms(void) { return s_ticks++; }

static float last_w[4];
static int last_lift = -1;

void motor_set_speeds(float w_fl, float w_fr, float w_rl, float w_rr)
{
    last_w[0] = w_fl;
    last_w[1] = w_fr;
    last_w[2] = w_rl;
    last_w[3] = w_rr;
}

void lift_set(int up)
{
    last_lift = up;
}

int main(void)
{
    const char *motion = "M,1.0,-0.5,0.5,-0.25\n";
    const char *lift_up = "L,1\n";
    const char *lift_down = "L,0\n";

    protocol_set_last_rx_tick(0);

    for (const char *p = motion; *p; p++)
    {
        protocol_feed_byte((uint8_t)*p);
        protocol_set_last_rx_tick(get_tick_ms());
    }
    if (last_w[0] != 1.0f || last_w[1] != -0.5f)
    {
        fprintf(stderr, "FAIL: motion parse\n");
        return 1;
    }

    for (const char *p = lift_up; *p; p++)
    {
        protocol_feed_byte((uint8_t)*p);
        protocol_set_last_rx_tick(get_tick_ms());
    }
    if (last_lift != 1)
    {
        fprintf(stderr, "FAIL: lift up\n");
        return 1;
    }

    for (const char *p = lift_down; *p; p++)
    {
        protocol_feed_byte((uint8_t)*p);
        protocol_set_last_rx_tick(get_tick_ms());
    }
    if (last_lift != 0)
    {
        fprintf(stderr, "FAIL: lift down\n");
        return 1;
    }

    printf("protocol host test OK\n");
    return 0;
}
