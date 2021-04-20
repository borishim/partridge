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

#ifndef T4_GPIO_H
#define T4_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#include <stdint.h>

#define ZBIT0           19
#define ZBIT1           18
#define ZBIT2           14
#define ZBIT3           15
#define ZBIT4           40
#define ZBIT5           41
#define ZBIT6           17
#define ZBIT7           16

#define TFT_DC          36
#define TFT_CS           0
#define TFT_RST          5
#define TFT_SCK         27
#define TFT_MOSI        26

#define ADDR_HIGH_OE     8
#define ADDR_LOW_OE      9

#define OUT2             2
#define LRCLK2           3
#define BCLK2            4
#define PSG_PWM          33

#define ZRD              31
#define ZWR              30
#define ZIORQ            29
#define ZSLTSL           32
#define ZRESET           28

#define ZBUSDIR          21
#define ZWAIT            22
#define ZINT             20
#define ZM1              24
#define ZMREQ            25

#define PSG_BC1     8
#define PSG_BDIR    30
#define PSG_CLK     9
#define DATA_DIR     7
#define DATA_OE      6

#define MQSL        12

void    init_gpio(void);
void    init_8bit_bus(void);
void    prepare_read_8bit_bus(void);
void    prepare_write_8bit_bus(void);
uint8_t read_8bit_bus(void);
void    write_8bit_bus(uint8_t data);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* T4_GPIO_H */