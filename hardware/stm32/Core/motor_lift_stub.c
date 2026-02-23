/**
 * motor_lift_stub.c
 * Stub implementations for motor and lift control.
 * Replace with your HAL: PWM timers for 4 wheels and lift actuator.
 * Copyright (c) 2025 Tarik Kahraman. SPDX-License-Identifier: MIT
 */

#include "protocol.h"
#include <stddef.h>

/* Wheel speeds in rad/s from Jetson. Convert to PWM duty or closed-loop setpoints. */
void motor_set_speeds(float w_fl, float w_fr, float w_rl, float w_rr)
{
    (void)w_fl;
    (void)w_fr;
    (void)w_rl;
    (void)w_rr;
    /* TODO: map to your motor drivers, e.g.:
     * - Set PWM duty for each of 4 wheels (direction + magnitude).
     * - Or set target velocity if using closed-loop drivers.
     */
}

/* up: 0 = lift down, 1 = lift up */
void lift_set(int up)
{
    (void)up;
    /* TODO: drive lift actuator (relay or PWM). */
}
