/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Sangwoo Shim
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <imxrt.h>
#include <Arduino.h>

#include "t4_gpio.h"


inline void
init_gpio(void)
{
    pinMode(ZWAIT, OUTPUT);
    pinMode(ZINT, OUTPUT);
    pinMode(ZBUSDIR, OUTPUT);
    digitalWriteFast(ZWAIT, HIGH);
    digitalWriteFast(ZINT, HIGH);
    digitalWriteFast(ZBUSDIR, HIGH);

    pinMode(ADDR_HIGH_OE, OUTPUT);
    pinMode(ADDR_LOW_OE, OUTPUT);
    pinMode(DATA_OE, OUTPUT);
    pinMode(DATA_DIR, OUTPUT);
    digitalWriteFast(ADDR_HIGH_OE, HIGH);
    digitalWriteFast(ADDR_LOW_OE, HIGH);
    digitalWriteFast(DATA_OE, HIGH);

    pinMode(ZRD, INPUT_PULLUP);
    pinMode(ZWR, INPUT_PULLUP);
    pinMode(ZM1, INPUT_PULLUP);
    pinMode(ZMREQ, INPUT_PULLUP);
    pinMode(ZIORQ, INPUT_PULLUP);
    pinMode(ZSLTSL, INPUT_PULLUP);
    pinMode(ZRESET, INPUT_PULLUP);
}

inline void
prepare_read_8bit_bus(void)
{
    pinMode(ZBIT0, INPUT_PULLUP);
    pinMode(ZBIT1, INPUT_PULLUP);
    pinMode(ZBIT2, INPUT_PULLUP);
    pinMode(ZBIT3, INPUT_PULLUP);
    pinMode(ZBIT4, INPUT_PULLUP);
    pinMode(ZBIT5, INPUT_PULLUP);
    pinMode(ZBIT6, INPUT_PULLUP);
    pinMode(ZBIT7, INPUT_PULLUP);
}

inline uint8_t
read_8bit_bus(void)
{
    register uint8_t result;

    result = (GPIO6_DR & (0xff << CORE_PIN19_BIT)) >> CORE_PIN19_BIT;
    return result;
}

inline void
prepare_write_8bit_bus(void)
{
    pinMode(ZBIT0, OUTPUT);
    pinMode(ZBIT1, OUTPUT);
    pinMode(ZBIT2, OUTPUT);
    pinMode(ZBIT3, OUTPUT);
    pinMode(ZBIT4, OUTPUT);
    pinMode(ZBIT5, OUTPUT);
    pinMode(ZBIT6, OUTPUT);
    pinMode(ZBIT7, OUTPUT);
}

inline void
write_8bit_bus(uint8_t data)
{
    register uint32_t gpio_reg;
    gpio_reg = GPIO6_DR & 0xff00ffff;
    gpio_reg |= ((uint32_t)data & 0xff) << CORE_PIN19_BIT;
    GPIO6_DR = gpio_reg;
    return;
}