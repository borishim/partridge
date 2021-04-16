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

#include <Arduino.h>

#include "t4_gpio.h"

uint16_t addr;

inline void
on_wait(void)
{
    digitalWriteFast(ZWAIT, LOW);
}

inline void
off_wait(void)
{
    digitalWriteFast(ZWAIT, HIGH);
}

inline uint16_t
pull_addr_from_cpu(void)
{
    uint16_t addr_high;
    uint16_t addr_low;

    digitalWriteFast(ADDR_HIGH_OE, LOW);
    prepare_read_8bit_bus();
    addr_high = read_8bit_bus();
    digitalWriteFast(ADDR_HIGH_OE, HIGH);
    digitalWriteFast(ADDR_LOW_OE, LOW);
    addr_low = read_8bit_bus();
    digitalWriteFast(ADDR_LOW_OE, HIGH);

    return (addr_high << 8) | addr_low;
}

inline void
push_data_to_cpu(uint8_t data)
{
    digitalWriteFast(DATA_OE, LOW);
    digitalWriteFast(DATA_DIR, LOW);
    prepare_write_8bit_bus();
    write_8bit_bus(data);
}

inline uint8_t
pull_data_from_cpu()
{
    uint8_t data;

    digitalWriteFast(DATA_OE, LOW);
    digitalWriteFast(DATA_DIR, HIGH);
    prepare_read_8bit_bus();
    data = read_8bit_bus();
    digitalWriteFast(DATA_OE, HIGH);
    return;
}

void
isr_sltsl_fall(void)
{
    addr = pull_addr_from_cpu();
}

void
isr_rd_fall(void)
{
    uint8_t data;

    on_wait();
    //data = read_mem(addr);
    push_data_to_cpu(data);
    off_wait();
}

void
isr_wr_fall(void)
{
    uint8_t data;

    data = pull_data_from_cpu();
    on_wait();
    //write_mem(addr, data);
    off_wait();
}

void
isr_wr_rise(void)
{
    digitalWriteFast(DATA_OE, HIGH);
}

void setup() {
  // put your setup code here, to run once:
    attachInterrupt(ZSLTSL, isr_sltsl_fall, FALLING);
    attachInterrupt(ZRD, isr_rd_fall, FALLING);
    attachInterrupt(ZWR, isr_wr_fall, FALLING);
    attachInterrupt(ZWR, isr_wr_rise, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
}