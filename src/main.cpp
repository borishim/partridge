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
#include "magtree.h"

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

inline uint8_t
read_rom(uint16_t addr)
{
    uint16_t rom_addr = (addr - 0x4000) % 0x8000;
    return magtree[rom_addr];
}

void
isr_rd_fall(void)
{
    volatile uint16_t addr;
    volatile uint8_t data;


    // read address
    set_input_pinmode_8bit_bus();

    digitalWriteFast(ADDR_HIGH_OE, LOW);
    delayNanoseconds(10);
    addr = read_8bit_bus();
    digitalWriteFast(ADDR_HIGH_OE, HIGH);
    digitalWriteFast(ADDR_LOW_OE, LOW);
    delayNanoseconds(10);
    addr = (addr << 8) | read_8bit_bus();
    digitalWriteFast(ADDR_LOW_OE, HIGH);
    delayNanoseconds(10);

    if (digitalReadFast(ZMREQ) == LOW) {
        // If SLTSL is not active, don't carry out memory read
        if (
          digitalReadFast(ZSLTSL) == HIGH ||
          addr < 0x4000 ||
          addr >= 0x8000
        )
            return;

        on_wait();
        set_output_pinmode_8bit_bus();

        digitalWriteFast(DATA_DIR, HIGH);
        digitalWriteFast(DATA_OE, LOW);
        delayNanoseconds(10);
        data = read_rom(addr);

        off_wait();

        write_8bit_bus(data);
    }
}

//void
//isr_wr_fall(void)
//{
//    volatile uint8_t data;
//
//    prepare_read_8bit_bus();
//    digitalWriteFast(DATA_DIR, LOW);
//    digitalWriteFast(DATA_OE, LOW);
//    delayNanoseconds(10);
//    data = read_8bit_bus();
//    on_wait();
//    write_mem(addr, data);
//    off_wait();
//    digitalWriteFast(DATA_OE, HIGH);
//}

void
isr_rd_rise(void)
{
    digitalWriteFast(DATA_OE, HIGH);
    delayNanoseconds(10);
}

void setup()
{
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;   // enable cycle counter

    init_gpio();
    //attachInterrupt(ZMREQ, isr_mreq_fall, FALLING);
    attachInterrupt(ZRD, isr_rd_fall, FALLING);
    attachInterrupt(ZRD, isr_rd_rise, RISING);
    //attachInterrupt(ZWR, isr_wr_fall, FALLING);
}

void loop() {
    // put your main code here, to run repeatedly:
}