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
* @file       MAX1161X.h
* @date       2026-01-29
* @version    v0.1.0
*
*/


#ifndef __MAX11615_H__
#define __MAX11615_H__

#include <MAX1161X_defs.h>

#ifdef __cplusplus
extern "C" {
#endif

MAX1161X_INTF_RET_TYPE max11615_init(struct MAX1161X_Dev_t *dev);

MAX1161X_INTF_RET_TYPE max1161x_softReset(struct MAX1161X_Dev_t *dev);

MAX1161X_INTF_RET_TYPE max1161x_setSetupByte(uint8_t setup, struct MAX1161X_Dev_t *dev);

MAX1161X_INTF_RET_TYPE max1161x_setConfigByte(uint8_t config, struct MAX1161X_Dev_t *dev);

MAX1161X_INTF_RET_TYPE max1161x_readADC_singleEnded(uint8_t channel, int16_t *data, struct MAX1161X_Dev_t *dev);

MAX1161X_INTF_RET_TYPE max1161x_readADC_differential(uint8_t channel, int16_t *data, struct MAX1161X_Dev_t *dev);


#ifdef __cplusplus
}
#endif

#endif