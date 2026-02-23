/**
 * main.c
 * Example main loop: UART RX -> protocol parser, periodic watchdog.
 * Integrate with your HAL (UART, SysTick). This file is a template.
 * Copyright (c) 2025 Tarik Kahraman. SPDX-License-Identifier: MIT
 */

#include "protocol.h"
#include <stdint.h>

#ifndef WATCHDOG_TIMEOUT_MS
#define WATCHDOG_TIMEOUT_MS  500u
#endif

/* Provide a millisecond tick (e.g. from SysTick or HAL_GetTick()). */
extern uint32_t get_tick_ms(void);

/* Called when UART receives one byte; feed it to the protocol. */
extern void uart_rx_byte(uint8_t byte);

/* Optional: platform init (UART, GPIO, PWM). */
static void platform_init(void)
{
    /* TODO: HAL_UART_Init(), GPIO, timer for PWM. */
}

int main(void)
{
    uint32_t last_watchdog_check = 0u;

    platform_init();

    for (;;)
    {
        uint32_t now = get_tick_ms();

        /* Example: in a real system you'd call protocol_feed_byte from your
         * UART RX callback or DMA handler, and protocol_set_last_rx_tick(now)
         * there too. Here we assume that happens elsewhere. */

        /* Watchdog: if no valid frame received within timeout, stop motors. */
        if (now - last_watchdog_check >= 50u)  /* check every 50 ms */
        {
            last_watchdog_check = now;
            uint32_t last_rx = protocol_last_rx_tick();
            if (now - last_rx >= WATCHDOG_TIMEOUT_MS)
                motor_set_speeds(0.0f, 0.0f, 0.0f, 0.0f);
        }
    }

    return 0;
}
