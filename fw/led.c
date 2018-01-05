/* Megumin LED display firmware
 * Copyright (C) 2018 Sebastian Götte <code@jaseg.net>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "global.h"
#include "led.h"

/* Status LED control */
#define LED_STRETCHING_MS 50
static volatile int error_led_timeout = 0;
static volatile int comm_led_timeout = 0;
static volatile int id_led_timeout = 0;

volatile int led_state = 0;

void trigger_error_led() {
    error_led_timeout = LED_STRETCHING_MS;
}

void trigger_comm_led() {
    comm_led_timeout = LED_STRETCHING_MS;
}

void trigger_id_led() {
    id_led_timeout = LED_STRETCHING_MS;
}

void led_task() {
    static int last_time = 0;
    /* Crude LED logic. The comm, id and error LEDs each have a timeout counter
     * that is reset to the LED_STRETCHING_MS constant on an event (either a
     * frame received correctly or some uart, framing or protocol error). These
     * timeout counters count down in milliseconds and the LEDs are set while
     * they are non-zero. This means a train of several very brief events will
     * make the LED lit permanently.
     */
    int time_now = sys_time; /* Latch sys_time here to avoid race conditions */
    if (last_time != time_now) {
        int diff = (time_now - last_time);

        error_led_timeout -= diff;
        if (error_led_timeout < 0)
            error_led_timeout = 0;

        comm_led_timeout -= diff;
        if (comm_led_timeout < 0)
            comm_led_timeout = 0;

        id_led_timeout -= diff;
        if (id_led_timeout < 0)
            id_led_timeout = 0;

        led_state = (led_state & ~7) | (!!id_led_timeout)<<2 | (!!error_led_timeout)<<1 | (!!comm_led_timeout)<<0;
        last_time = time_now;
    }
}

