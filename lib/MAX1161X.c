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

/*!
 * @brief Initialization API Function to start communications with the MAX1161X Device
 *
 * @param[in,out] dev : Pointer to the MAX1161X Device Structure to initialize
 * 
 * @retval 0 for Success
 * @retval Non-zero for failure
 */
int8_t max1161X_init(struct MAX1161X_t *dev){
   int8_t rslt = MAX1161X_OK;
   dev->conf.value &= 0x7F;
   dev->setup.value |= 0x80;
   rslt = max1161X_conf_reset(dev);
   if(rslt != MAX1161X_OK) return rslt;
   return rslt;
}

/*!
 * @brief Sets the Voltage Reference Bits of the Configuration Byte of the MAX1161X Device
 *
 * @param[in]     ref : Reference Bit Selection to be applied
 * @param[in,out] dev : Pointer to the MAX1161X Device Structure
 * 
 * @retval 0 for Success
 * @retval Non-zero for failure
 */
int8_t max1161X_set_reference(uint8_t ref, struct MAX1161X_t *dev){
   if(dev == NULL) return MAX1161X_E_NULL_PTR;
   dev->setup.SEL = ref;
   return dev->write(&dev->setup.value, 1, dev->intf_ptr);
}

/*!
 * @brief Sets the Voltage Reference value for Bit to Volt Conversions
 *
 * @param[in]     ref_voltage : Voltage Reference Value 
 * @param[in,out] dev : Pointer to the MAX1161X Device Structure
 * 
 * @retval 0 for Success
 * @retval Non-zero for failure
 */
int8_t max1161x_set_reference_voltage(float ref_voltage, struct MAX1161X_t *dev){
   if(dev->setup.SEL < MAX1161X_SEL_BITS_INT_AIN){
      dev->ref_voltage = ref_voltage;
   }else{
      dev->ref_voltage = 2.048f;
   }
   return MAX1161X_OK;
}

/*!
 * @brief Sets the Clock Source Bits of the Configuration Byte of the MAX1161X Device
 *
 * @param[in]     clk : Clock Bit Selection to be applied
 * @param[in,out] dev : Pointer to the MAX1161X Device Structure
 * 
 * @retval 0 for Success
 * @retval Non-zero for failure
 */
int8_t max1161X_set_clock(uint8_t clk, struct MAX1161X_t *dev){
   if(dev == NULL) return MAX1161X_E_NULL_PTR;
   dev->setup.CLK = clk;
   return dev->write(&dev->setup.value, 1, dev->intf_ptr);
}

/*!
 * @brief Sets the Polarity Bits of the Configuration Byte of the MAX1161X Device
 *
 * @param[in]     pol : Polarity Bit Selection to be applied
 * @param[in,out] dev : Pointer to the MAX1161X Device Structure
 * 
 * @retval 0 for Success
 * @retval Non-zero for failure
 */
int8_t max1161X_set_polarity(uint8_t pol, struct MAX1161X_t *dev){
   if(dev == NULL) return MAX1161X_E_NULL_PTR;
   dev->setup.POL = pol;
   return dev->write(&dev->setup.value, 1, dev->intf_ptr);
}

/*!
 * @brief Resets the configuration byte
 *
 * @param[in,out] dev : Pointer to the MAX1161X Device Structure
 * 
 * @retval 0 for Success
 * @retval Non-zero for failure
 */
int8_t max1161X_conf_reset(struct MAX1161X_t *dev){
   if(dev == NULL) return MAX1161X_E_NULL_PTR;
   uint8_t buf = dev->setup.value & MAX1161X_RESET_BIT_MASK;
   return dev->write(&buf, 1, dev->intf_ptr);
};

/*!
 * @brief Writes a new configuration byte to the MAX1161X Device.
 *
 * @param[in]     conf  : Configuration byte to write to the MAX1161X Device
 * @param[in,out] dev   : Pointer to the MAX1161X Device Structure
 * 
 * @retval 0 for Success
 * @retval Non-zero for failure
 */
int8_t max1161X_conf_overwrite(uint8_t conf, struct MAX1161X_t *dev){
   if(dev == NULL) return MAX1161X_E_NULL_PTR;
   dev->conf.value = conf;
   return MAX1161X_E_NULL_PTR;
}

/*!
 * @brief Reads/Converts a Single-Ended measurement
 *
 * @param[in]     cs    : Channel Select Bits to set device Configuration Byte
 * @param[out]    count : Signed 16-bit char to store the conversion result
 * @param[in,out] dev   : Pointer to the MAX1161X Device Structure
 * 
 * @retval 0 for Success
 * @retval Non-zero for failure
 */
int8_t max1161X_readADC_SingleEnded(uint8_t cs, int16_t *count, struct MAX1161X_t *dev){
   if(dev == NULL) return MAX1161X_E_NULL_PTR;
   int8_t rslt = MAX1161X_OK;
   uint8_t meas[2] = {0};
   dev->conf.SGLDIF = MAX1161X_SGLDIF_SINGLEENDED;
   dev->conf.CS = cs;
   dev->conf.SCAN = MAX1161X_SCAN_BITS_SINGLE;
   rslt = dev->write(&dev->conf.value, 1, dev->intf_ptr);
   if(rslt != MAX1161X_OK) return rslt;
   dev->delay_us(1000, dev);
   rslt = dev->read(meas, 2, dev->intf_ptr);
   if(rslt != MAX1161X_OK) return rslt;
   *count = (meas[0] << 8 | meas[1]) & 0x0FFF;
   /*
   if(MAX1161X_POL_BIT_BIPOLAR == dev->setup.POL){
      *count = *count & 0x800 ? *count * -1 : *count;
   }
   */
   return rslt;
}

/*!
 * @brief Reads/Converts Differential measurement
 * CS selects the Cathode for the differential Pair
 * The anode is paired internally, Table 4 in data-sheet outlines pairrings
 *
 * @param[in]     cs    : Channel Select Bits to set device Configureation Byte
 * @param[out]    count : Signed 16-bit char to store the conversion result
 * @param[in,out] dev   : Pointer to the MAX1161X Device Structure
 * 
 * @retval 0 for Success
 * @retval Non-zero for failure
 */
int8_t max1161X_readADC_Differential(uint8_t cs, int16_t *count, struct MAX1161X_t *dev){
   if(dev == NULL) return MAX1161X_E_NULL_PTR;
   int8_t rslt = MAX1161X_OK;
   uint8_t meas[2] = {0};
   dev->conf.SGLDIF = MAX1161X_SGLDIF_DIFFERENTIAL;
   dev->conf.CS = cs;
   dev->conf.SCAN = MAX1161X_SCAN_BITS_SINGLE;
   rslt = dev->write(&dev->conf.value, 1, dev->intf_ptr);
   if(rslt != MAX1161X_OK) return rslt;
   dev->delay_us(10, dev);
   rslt = dev->read(meas, 2, dev->intf_ptr);
   if(rslt != MAX1161X_OK) return rslt;
   *count = (meas[0] << 8 | meas[1]) & 0x0FFF;
   if(MAX1161X_POL_BIT_BIPOLAR == dev->setup.POL){
      *count = *count & 0x800 ? *count * -1 : *count;
   }
   return rslt;
}

/*!
 * @brief Reads Channels between AIN0 and Channel selected by CS parameter
 *
 * @param[in]     cs          : Channel Select Bits to set device Configureation Byte
 * @param[in]     sgldif      : SGLDIF Bits to set device Configuration Byte 
 * @param[out]    buf_count   : Buffer to store all 12 bit counts for each conversions
 * @param[in,out] dev         : Pointer to the MAX1161X Device Structure
 * 
 * @retval 0 for Success
 * @retval Non-zero for failure
 */
int8_t max1161X_scan_to_channel(uint8_t cs, uint8_t sgldif, int16_t *buf_count, struct MAX1161X_t *dev){
   if(dev == NULL) return MAX1161X_E_NULL_PTR;
   int8_t rslt = MAX1161X_OK;
   dev->conf.CS = cs;
   dev->conf.SGLDIF = sgldif;
   dev->conf.SCAN = MAX1161X_SCAN_BITS_DEFAULT;
   rslt = dev->write(&dev->conf.value, 1, dev->intf_ptr);
   if(rslt != MAX1161X_OK) return rslt;
   size_t lb = cs << 1;
   uint8_t *meas = (uint8_t*)malloc(sizeof(uint8_t) * lb);
   dev->delay_us(1000, dev);
   rslt = dev->read(meas, lb, dev->intf_ptr);
   if(rslt != MAX1161X_OK) return rslt;
   for (uint8_t i = 0 ; i < cs; i+=2){
      buf_count[i] = (meas[i] << 8 | meas[i+1]) & 0x0FFF;
      if(MAX1161X_POL_BIT_BIPOLAR == dev->setup.POL){
         buf_count[i] = buf_count[i] & 0x800 ? buf_count[i] * -1 : buf_count[i];
      }
   }
   free(meas);
   return rslt;
}

/*!
 * @brief Reads the same channel of the MAX Device 8 times in succession
 *
 * @param[in]     cs          : Channel Select Bits to set device Configureation Byte
 * @param[in]     sgldif      : SGLDIF Bits to set device Configuration Byte 
 * @param[out]    buf_count   : Buffer to store all 12 bit counts for each conversions
 * @param[in,out] dev         : Pointer to the MAX1161X Device Structure
 * 
 * @retval 0 for Success
 * @retval Non-zero for failure
 */
int8_t max1161X_readADC_repeat(uint8_t cs, uint8_t sgldif, int16_t *buf_count, struct MAX1161X_t * dev){
   if(dev == NULL) return MAX1161X_E_NULL_PTR;
   int8_t rslt = MAX1161X_OK;
   dev->conf.CS = cs;
   dev->conf.SGLDIF = sgldif;
   dev->conf.SCAN = MAX1161X_SCAN_BITS_REPEAT_8;
   rslt = dev->write(&dev->conf.value, 1, dev->intf_ptr);
   if(MAX1161X_OK != rslt) return rslt;
   //uint8_t meas[16] = {0};
   
   return rslt;
}

/*!
 * @brief Converts ADC Counts to Voltage via appropriate transfer function
 *
 * @param[in]     count : 12-bit ADC count to be converted to a voltage
 * @param[in,out] dev   : Pointer to the MAX1161X_Device
 * 
 * @retval UNIPOLAR voltage between 0 and V_Ref
 * @retval BIPOLAR voltage between Positive and Negative V_Ref/2
 */
float max1161X_computeVoltage(int16_t count, struct MAX1161X_t *dev){
   if(MAX1161X_POL_BIT_BIPOLAR == dev->setup.POL){
      return count * dev->ref_voltage / (0x7FF << 1); 
   }else{
      return count * dev->ref_voltage / 0xFFF;
   }
}
