/**
* Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       MAX1161X_dev.h
* @date       2026-01-29
* @version    v0.1.0
*
*/


#ifndef __MAX1161X_DEFS_H__
#define __MAX1161X_DEFS_H__

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifndef MAX1161X_INTF_RET_TYPE
#define MAX1161X_INTF_RET_TYPE                      int8_t
#endif

#define MAX1161X_STATUS_OK                          0
#define MAX1161X_E_NULL_PTR                         -1
#define MAX1161X_E_COM_FAIL                         -2
#define MAX1161X_E_DEV_NOT_FOUND                    -3
#define MAX1161X_E_NULL_INTF                        -5

#define MAX11614_I2C_ADDR                           0x33
#define MAX11615_I2C_ADDR                           0x33
#define MAX11612_I2C_ADDR                           0x34
#define MAX11613_I2C_ADDR                           0x34
#define MAX11616_I2C_ADDR                           0x35
#define MAX11617_I2C_ADDR                           0x35

#define MAX1161X_INTREF_2V048                       2.048f
#define MAX1161X_INTREF_4V096                       4.096f

#define MAX1161X_REG_SETUP                          0b0
#define MAX1161X_REG_CONFIG                         0b1

#define MAX1161X_SETUP_DEFAULT                      0b10000010
#define MAX1161X_CONFIG_DEFAULT                     0b00000001

#define MAX1161X_SEL_VDDREF_INPUT_NC_IREFOFF        0b000
#define MAX1161X_SEL_EXTREF_REFIN_REFIN_IREFOFF     0b010
#define MAX1161X_SEL_INTREF_AIN_NC_IREFOFF          0b100
#define MAX1161X_SEL_INTREF_AIN_NC_IREFON           0b101
#define MAX1161X_SEL_INTREF_REFOUT_REFOUT_IREFOFF   0b110
#define MAX1161X_SEL_INTREF_REFOUT_REFOUT_IREFON    0b111

#define MAX1161X_CLK_INTERNAL                       0b0
#define MAX1161X_CLK_EXTERNAL                       0b1

#define MAX1161X_POL_UNIPOLAR                       0b0
#define MAX1161X_POL_BIPOLAR                        0b1

#define MAX1161X_RST_RESET                          0b0
#define MAX1161X_RST_NOOP                           0b1

#define MAX1161X_SCAN_TO_CS                         0b00
#define MAX1161X_SCAN_CS_EIGHTTIMES                 0b01
#define MAX1161X_SCAN_SPEC                          0b10
#define MAX1161X_SCAN_CS_SINGLE                     0b11

#define MAX1161X_CS_AIN0                            0b0000
#define MAX1161X_CS_AIN1                            0b0001
#define MAX1161X_CS_AIN2                            0b0010
#define MAX1161X_CS_AIN3                            0b0011
#define MAX1161X_CS_AIN4                            0b0100
#define MAX1161X_CS_AIN5                            0b0101
#define MAX1161X_CS_AIN6                            0b0110
#define MAX1161X_CS_AIN7                            0b0111
#define MAX1161X_CS_AIN8                            0b1000
#define MAX1161X_CS_AIN9                            0b1001
#define MAX1161X_CS_AIN10                           0b1010
#define MAX1161X_CS_AIN11                           0b1011

#define MAX1161X_MODE_DIF                           0b0
#define MAX1161X_MODE_SGL                           0b1

typedef MAX1161X_INTF_RET_TYPE (* max1161x_read_fptr)(uint8_t *buf, uint32_t len, void *intf_ptr);
typedef MAX1161X_INTF_RET_TYPE (* max1161x_write_fptr)(const uint8_t *buf, uint32_t len, void *intf_ptr);
typedef void (* max1161x_delay_fptr)(uint32_t period_us);

union MAX1161X_Setup_u{
    uint8_t byte;
    struct{
        uint8_t nc:1;
        uint8_t rst:1;
        uint8_t pol:1;
        uint8_t clk:1;
        uint8_t sel:3;
        uint8_t reg:1;
    } bits;
};

union MAX1161X_Config_u{
    uint8_t byte;
    struct{
        uint8_t mode:1;
        uint8_t scan:2;
        uint8_t cs:4;
        uint8_t reg:1;
    } bits;
};

struct MAX1161X_Dev_t{
    union MAX1161X_Setup_u setup;
    union MAX1161X_Config_u config;
    uint32_t conversionTime;
    float internalRef;
    float externalRef;
    float VRef;
    void *intf_ptr;
    max1161x_read_fptr read;
    max1161x_write_fptr write;
    max1161x_delay_fptr delay;
};


#endif