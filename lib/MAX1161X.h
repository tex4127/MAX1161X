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

/*!
 * \defgroup max1161X MAX1161X
 * @brief Need to add reference to documents and where this code is stored
 */

#ifndef MAX1161X_H
#define MAX1161X_H

#include "MAX1161X_defs.h"
#include <stdlib.h>

/*Start CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup max1161X
 * \defgroup max1161XApiInit Initialization
 * @brief Initialize sensor and device structure
 */

/*!
 * \ingroup max1161XApiInit
 * \page max1161X_api_max1161X_init max1161X_init
 * \code
 * int8_t max1161X_init(struct MAX1161X_t *dev);
 * \endcode
 * @details This API initializes the chip and code structure to enable communication.
 * This is the API entry point, call this before using other APIs.
 * 
 * @param[in,out] dev : Structure instance of MAX1161X_t
 * 
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
*/
int8_t max1161X_init(struct MAX1161X_t *dev);

/**
 * \ingroup max1161X
 * \defgroup max1161XApiSetupByte SetupByte
 * @brief API for setting Bit groups within the MAX1161X Setup Byte
 */

/*!
 * \ingroup max1161XApiSetupByte
 * \page max1161X_api_set_reference max1161X_set_reference
 * \code
 * int8_t max1161X_set_reference(uint8_t ref, struct MAX1161X_t *dev)
 * \endcode
 * @details This API sets SEL bits, [6-4], of the Setup Byte.
 * 
 * @param[in]     ref : Reference Bit Selection to be applied
 * @param[in,out] dev : Pointer to the MAX1161X Device Structure
 * 
 * @returns Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t max1161X_set_reference(uint8_t ref, struct MAX1161X_t *dev);

/*!
 * \ingroup max1161XApiSetupByte
 * \page max1161X_api_set_reference_voltage max1161X_set_reference_voltage
 * \code 
 * int8_t max1161x_set_reference_voltage(float refV, struct MAX1161X_t *dev)
 * \endcode
 * @details This API sets the Voltage Reference for converting bits to a real value.
 * 
 * @param[in]     ref_voltage : Voltage Reference Value 
 * @param[in,out] dev : Pointer to the MAX1161X Device Structure
 * 
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t max1161x_set_reference_voltage(float refV, struct MAX1161X_t *dev);

/*!
 * \ingroup max1161XApiSetupByte
 * \page max1161X_api_set_clock max1161X_set_clock
 * \code
 * int8_t max1161X_set_clock(uint8_t clk, struct MAX1161X_t *dev)
 * \endcode
 * @details This API sets the CLK bit, [3], of the Setup Byte.
 * 
 * @param[in]     clk : Clock Bit Selection to be applied
 * @param[in,out] dev : Pointer to the MAX1161X Device Structure
 * 
 * @returns Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t max1161X_set_clock(uint8_t clk, struct MAX1161X_t *dev);

/*!
 * \ingroup max1161XApiSetupByte
 * \page max1161X_api_set_polarity max1161X_set_polarity
 * \code
 * int8_t max1161X_set_polarity(uint8_t pol, struct MAX1161X_t *dev)
 * \endcode
 * @details This API sets the POL bit, [2], of the Setup Byte.
 * 
 * @param[in]     pol : Polarity Bit Selection to be applied
 * @param[in,out] dev : Pointer to the MAX1161X Device Structure
 * 
 * @return Result of API execution Status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t max1161X_set_polarity(uint8_t pol, struct MAX1161X_t *dev);

/*!
 * \ingroup max1161XApiSetupByte
 * \page max1161X_api_conf_reset max1161X_conf_reset
 * \code
 * int8_t max1161X_conf_reset(struct MAX1161X_t *dev)
 * \endcode
 * @details This API sends a single setup byte with the RST bit, [1], set to 0 to 
 * force a reset of the device configuration (byte).
 * 
 * @param[in,out] dev : Pointer to the MAX1161X Device Structure
 * 
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t max1161X_conf_reset(struct MAX1161X_t *dev);

/**
 * \ingroup max1161X
 * \defgroup max1161XApiConfigurationByte ConfigurationByte
 * @brief API for setting Bit groups within the MAX1161X Configuration Byte
 */

/*!
 * \ingroup max1161XApiConfigurationByte
 * \page max1161X_api_conf_overwrite max1161X_conf_overwrite
 * \code
 * int8_t max1161X_conf_overwrite(uint8_t conf, struct MAX1161X_t *dev)
 * \endcode
 * @details This API overwrites the Configuration Byte with the passed conf parameters.
 * 
 * @param[in]     conf  : Configuration byte to write to the MAX1161X Device
 * @param[in,out] dev   : Pointer to the MAX1161X Device Structure
 * 
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t max1161X_conf_overwrite(uint8_t conf, struct MAX1161X_t *dev);

/**
 * \ingroup max1161X
 * \defgroup max1161XReadADC ReadADC
 * @brief API for reading various configurations of ADC pins
 */

/*!
 * \ingroup max1161XReadADC
 * \page max1161X_api_readADC_SingleEnded max1161X_readADC_SingleEnded
 * \code
 * int8_t max1161X_readADC_SingleEnded(uint8_t cs, int16_t *count, struct MAX1161X_t *dev)
 * \endcode
 * @details This API is used to perform a single-ended read operation of a selected Input Channel.
 * 
 * @param[in]     cs    : Channel Select Bits to set device Configuration Byte
 * @param[out]    count : Signed 16-bit char to store the conversion result
 * @param[in,out] dev   : Pointer to the MAX1161X Device Structure
 * 
 * @returns Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t max1161X_readADC_SingleEnded(uint8_t cs, int16_t *count, struct MAX1161X_t *dev);

/**
 * \ingroup max1161XReadADC
 * \page max1161X_api_readADC_Differential max1161X_readADC_Differential 
 * \code
 * int8_t max1161X_readADC_Differential(uint8_t cs, int16_t *count, struct MAX1161X_t *dev)
 * \endcode
 * @details This API is used to perform a differential read operation for a pair of analog input channels.
 * 
 * @param[in]     cs    : Channel Select Bits to set the device Configuration Byte
 * @param[out]    count : Signed 16-bit char to store the conversion result
 * @param[in,out] dev   : Pointer to the MAX1161X Device Structure
 * 
 * @returns Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t max1161X_readADC_Differential(uint8_t cs, int16_t *count, struct MAX1161X_t *dev);

/*!
 * \ingroup max1161XReadADC
 * \page max1161X_api_scan_to_channel max1161X_scan_to_channel
 * \code
 * int8_t max1161X_scan_to_channel(uint8_t cs, uint8_t sgldif, int16_t *buf_count, struct MAX1161X_t *dev)
 * \endcode
 * @details This API is used to perform single-ended or differential read operations scanning from AIN0 to a selected channel.
 * 
 * @param[in]     cs          : Channel Select Bits to set device Configureation Byte
 * @param[in]     sgldif      : SGLDIF Bits to set device Configuration Byte 
 * @param[out]    buf_count   : Buffer to store all 12 bit counts for each conversions
 * @param[in,out] dev         : Pointer to the MAX1161X Device Structure
 * 
 * @returns Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t max1161X_scan_to_channel(uint8_t cs, uint8_t sgldif, int16_t *buf_count, struct MAX1161X_t *dev);

/*!
 * \ingroup max1161XReadADC
 * \page max1161X_api_readADC_repeat max1161X_readADC_repeat
 * \code
 * int8_t max1161X_readADC_repeat(uint8_t cs, uint8_t sgldif, int16_t *buf_count, struct MAX1161X_t *dev)
 * \endcode
 * @details This API is used to perform single-ended or differential read operations on a single channel 8 times in succession.
 * 
 * @param[in]     cs          : Channel Select Bits to set device Configureation Byte
 * @param[in]     sgldif      : SGLDIF Bits to set device Configuration Byte 
 * @param[out]    buf_count   : Buffer to store all 12 bit counts for each conversions
 * @param[in,out] dev         : Pointer to the MAX1161X Device Structure
 * 
 * @returns Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t max1161X_readADC_repeat(uint8_t cs, uint8_t sgldif, int16_t *buf_count, struct MAX1161X_t *dev);


/**
 * \ingroup max1161X
 * \defgroup max1161XTransferFunction TransferFunction
 * @brief API for converting ADC counts (in bits) to Voltage
 */

/*!
 * \ingroup max1161XTransferFunction
 * \page max1161X_api_computeVoltage max1161X_computeVoltage
 * \code
 * float max1161X_computeVoltage(int16_t count, struct MAX1161X_t *dev)
 * \endcode
 * @details This 
 * 
 * @param[in]     count : 12-bit ADC count to be converted to a voltage
 * @param[in,out] dev   : Pointer to the MAX1161X_Device
 * 
 * @return Result of API execution status
 * @retval UNIPOLAR voltage between 0 and V_Ref
 * @retval BIPOLAR voltage between Positive and Negative V_Ref/2
 */
float max1161X_computeVoltage(int16_t count, struct MAX1161X_t *dev);

#ifdef __cplusplus
}
#endif 
/*End CPP Guard*/

#endif