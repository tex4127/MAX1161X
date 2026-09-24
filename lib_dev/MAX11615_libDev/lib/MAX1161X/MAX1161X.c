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
* @file       MAX1161X.c
* @date       2026-01-29
* @version    v0.1.0
*
*/


#include "MAX1161X.h"

static int8_t write_regs(const uint8_t *regData, uint32_t len, struct MAX1161X_Dev_t *dev);
static int8_t read_regs(uint8_t *regData, uint32_t len, struct MAX1161X_Dev_t *dev);
static float computeVoltage(uint16_t counts, struct MAX1161X_Dev_t *dev);

MAX1161X_INTF_RET_TYPE max11615_init(struct MAX1161X_Dev_t *dev){
    int8_t res = MAX1161X_STATUS_OK;
    if (NULL == dev) return MAX1161X_E_NULL_PTR;
    if (NULL == dev->write) return MAX1161X_E_NULL_PTR;
    if (NULL == dev->read) return MAX1161X_E_NULL_PTR;
    
    // Set Reference values (external ref should be set in seperate API call)
    dev->internalRef = MAX1161X_INTREF_2V048;
    dev->externalRef = 3.3f; //set with another API Call
    // Force set default register values
    dev->setup.byte = MAX1161X_SETUP_DEFAULT;
    dev->config.byte =  MAX1161X_CONFIG_DEFAULT;
    res = max1161x_softReset(dev);
    if(MAX1161X_STATUS_OK != res) return res;
    // Set setup byte
    res = max1161x_setSetupByte(MAX1161X_GENERATE_SETUP_BYTE(MAX1161X_SEL_INTREF_AIN_NC_IREFON,
                                                             MAX1161X_CLK_INTERNAL,
                                                             MAX1161X_POL_UNIPOLAR,
                                                             MAX1161X_RST_NOOP), dev);
    if (MAX1161X_STATUS_OK != res) return res;
    res = max1161x_setConfigByte(MAX1161X_GENERATE_CONFIG_BYTE(MAX1161X_SCAN_CS_SINGLE,
                                                               MAX1161X_CS_AIN0,
                                                               MAX1161X_MODE_SGL), dev);
    if (MAX1161X_STATUS_OK != res) return res;
    
    dev->conversionTime = (dev->setup.bits.clk == MAX1161X_CLK_INTERNAL) ? 8 : 11; //us
    return res;
}

MAX1161X_INTF_RET_TYPE max1161x_softReset(struct MAX1161X_Dev_t *dev){
    int8_t res = MAX1161X_STATUS_OK;
    uint8_t rw = dev->config.byte | 0x02;
    res = dev->write(&rw, 1, dev->intf_ptr);
    return res;
}

MAX1161X_INTF_RET_TYPE max1161x_setSetupByte(uint8_t setup, struct MAX1161X_Dev_t *dev){
    if (NULL == dev) return MAX1161X_E_NULL_PTR;
    dev->setup.byte = setup;
    dev->VRef = dev->setup.bits.sel & 0b100 ? dev->internalRef : dev->externalRef;
    dev->conversionTime = (dev->setup.bits.clk == MAX1161X_CLK_INTERNAL) ? 8 : 11; //us
    return MAX1161X_STATUS_OK;
}

MAX1161X_INTF_RET_TYPE max1161x_setConfigByte(uint8_t config, struct MAX1161X_Dev_t *dev){
    if (NULL == dev) return MAX1161X_E_NULL_PTR;
    dev->config.byte = config;
    return MAX1161X_STATUS_OK;
}

MAX1161X_INTF_RET_TYPE max1161x_setExternalVRef(float vref, struct MAX1161X_Dev_t *dev){
    if (NULL == dev) return MAX1161X_E_NULL_PTR;
    dev->externalRef = vref;
    dev->VRef = dev->setup.bits.sel & 0b100 ? dev->internalRef : dev->externalRef;
    return MAX1161X_STATUS_OK;
}

MAX1161X_INTF_RET_TYPE max1161x_readADC_singleEnded(uint8_t channel, float *data, struct MAX1161X_Dev_t *dev){
    int8_t res = MAX1161X_STATUS_OK;
    if (NULL == dev) return MAX1161X_E_NULL_PTR;
    dev->config.bits.mode = MAX1161X_MODE_SGL;
    dev->config.bits.scan = MAX1161X_SCAN_CS_SINGLE;
    dev->config.bits.cs = (channel & 0x0f);
    uint8_t dout[2] = {dev->setup.byte, dev->config.byte};
    res = dev->write(dout, 2, dev->intf_ptr);
    dev->delay(dev->conversionTime);
    uint8_t din[2] = {0};
    res = dev->read(din, 2, dev->intf_ptr);
    uint16_t count = ((din[0] << 8) | din[1]) << 4;
    *data = computeVoltage(count, dev);
    return res;
}

MAX1161X_INTF_RET_TYPE max1161x_readADC_differential(uint8_t channel, float *data, struct MAX1161X_Dev_t *dev){
    int8_t res = MAX1161X_STATUS_OK;
    if (NULL == dev) return MAX1161X_E_NULL_PTR;
    dev->config.bits.mode = MAX1161X_MODE_DIF;
    dev->config.bits.scan = MAX1161X_SCAN_CS_SINGLE;
    dev->config.bits.cs = (channel & 0x0c);
    uint8_t dout[2] = {dev->setup.byte, dev->config.byte};
    res = dev->write(dout, 2, dev->intf_ptr);
    dev->delay(dev->conversionTime);
    uint8_t din[2] = {0};
    res = dev->read(din, 2, dev->intf_ptr);
    uint16_t count = ((din[0] << 8) | din[1]) << 4;
    *data = count;
    return res;
}

static int8_t write_regs(const uint8_t *regData, uint32_t len, struct MAX1161X_Dev_t *dev){
    int8_t res = MAX1161X_STATUS_OK;
    res = dev->write(regData, len, dev->intf_ptr);
    return res;
}

static int8_t read_regs(uint8_t *regData, uint32_t len, struct MAX1161X_Dev_t *dev){
    int8_t res = MAX1161X_STATUS_OK;
    res = dev->read(regData, len, dev->intf_ptr);
    return res;
}

static float computeVoltage(uint16_t counts, struct MAX1161X_Dev_t *dev){
    if (NULL == dev) return 0.0f;
    if (MAX1161X_POL_UNIPOLAR == dev->setup.bits.pol){
        //Use unipolar transfer function
        return (dev->VRef * counts)/65535;
    } else{
        //Use bipolar transfer function
        return (dev->VRef * (int16_t)counts)/65535;
    }
    return 0.0f;
}