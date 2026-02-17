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
* @file       MAX1161X_defs.h
* @date       2026-01-29
* @version    v0.1.0
*
*/

#include <stdint.h>
#include <stddef.h>

/* I2C Device Address for both MAX11612 and MAX11613 */
#define MAX11612_3_I2C_ADDRESS                      0x34
/* I2C Device Address for both MAX11614 and MAX11615 */
#define MAX11614_5_I2C_ADDRESS                      0x33
/* I2C Device Address for both MAX11616 and MAX11617 */
#define MAX11616_7_I2C_ADDRESS                      0x35

#ifndef MAX11615_INTF_RET_TYPE
#define MAX1161X_INTF_RET_TYPE                      int8_t
#endif

/* Chip Status is OK */
#define MAX1161X_OK                                 INT8_C(0)
/*Error Null Pointer Passed */
#define MAX1161X_E_NULL_PTR                         INT8_C(-1)
/* Error Communication Failure */
#define MAX1161X_E_COM_FAIL                         INT8_C(-2)
/* Error MAX1161X Not Found */
#define MAX1161X_E_DEV_NOT_FOUND                    INT8_C(-3)
/* Error Incorrect Length Parameter */
#define MAX1161X_E_INVALID_LENGTH                   INT8_C(-4)

/* BIT MASK for Register portion of Setup and Config Byte */
#define MAX1161X_REGISTER_BIT_MASK                  ~0xE0
#define MAX1161X_REGISTER_BIT_SETUP                 0x00
#define MAX1161X_REGISTER_BIT_CONFIG                0x80
/* REG bit value to indicate Setup Byte */
#define MAX1161X_REG_BIT_SETUP                      INT8_C(0)
/* REG bit value to indicate Config Byte */
#define MAX1161X_REG_BIT_CONFIG                     INT8_C(1)

/* BIT MASK for Reference Select portion of Setup Byte */
#define MAX1161X_REFERENCE_BIT_MASK                 ~0x70
#define MAX1161X_REFERENCE_BIT_VDD                  0x00
#define MAX1161X_REFERENCE_BIT_EXT                  0x20
#define MAX1161X_REFERENCE_BIT_INT_AIN_REFOFF       0x40
#define MAX1161X_REFERENCE_BIT_INT_AIN_REFON        0x50
#define MAX1161X_REFERENCE_BIT_INT_ROUT_REFOFF      0x60
#define MAX1161X_REFERENCE_BIT_INT_ROUT_REFON       0x70
/* Disables the internal reference state if possible */
#define MAX11615_SEL_BITS_INT_REF_OFF               INT8_C(0)
/* Enables the internal reference state if possible */
#define MAX1161X_SEL_BITS_INT_REF_ON                INT8_C(1)
/* SEL bits value such that:
VREF = VDD
AIN_/REF = Analog Input
Ref Internal = Always Off
*/
#define MAX1161X_SEL_BITS_VDD_AIN                   INT8_C(0)
/* SEL bits value such that
VREF = External Reference
AIN_/REF = Reference Input
Ref Internal = Always Off
*/
#define MAX1161X_SEL_BITS_EXT_REFIN                 INT8_C(2)
/* SEL bits value such that:
VREF = Internal Reference
AIN_/REF = Analog Input
Ref Internal = Always Off
*/
#define MAX1161X_SEL_BITS_INT_AIN                   INT8_C(4)
/* SEL bits value such that:
VREF = Internal Reference
AIN_/REF = Reference Output
Ref Internal = Always Off
*/
#define MAX11615_SEL_BITS_INT_REFOUT                INT8_C(6)


/* BIT MASK for clock portion of Setup Byte */
#define MAX1161X_CLOCK_BIT_MASK                     ~0x08
#define MAX1161X_CLOCK_BIT_INTERNAL                 0x00
#define MAX1161X_CLOCK_BIT_EXTERNAL                 0x08
/* CLK bit value to use internal clock */
#define MAX1161X_CLK_BIT_INTERNAL                   INT8_C(0)
/*CLK bit value to use external clock */
#define MAX1161X_CLK_BIT_EXTERNAL                   INT8_C(1)


/* BIT MASK for polarity portion of Setup Byte */
#define MAX1161X_POLARITY_BIT_MASK                  ~0x04
#define MAX1161X_POLARITY_BIT_UNIPOLAR              0x00
#define MAX1161X_POLARITY_BIT_BIPOLAR               0x04
/* POL bit value to use Unipolar Measurements */
#define MAX1161X_POL_BIT_UNIPOLAR                   INT8_C(0)
/* POL bit value to use Bipolar Measurements */
#define MAX1161X_POL_BIT_BIPOLAR                    INT8_C(1)

/* BIT MASK for Reset section portion of Setup Byte */
#define MAX1161X_RESET_BIT_MASK                     ~0x02
#define MAX1161X_RESET_BIT_ENGAGE                   0x00
#define MAX1161X_RESET_BIT_NOACTION                 0x02
/* RST vit value to engage reset of config byte */
#define MAX1161X_RST_BIT_ENGAGE                     INT8_C(0)
/* RST bit value to bypass config byte reset */
#define MAX1161X_RST_BIT_NOACTION                   INT8_C(1)


/* BTI MASK for Scan selection portion of Config Byte */
#define MAX1161X_SCAN_BITS_MASK                     ~0x60
#define MAX1161X_SCAN_SELECT_DEAFULT                0x00
#define MAX1161X_SCAN_SELECT_REPEAT_8               0x20
#define MAX1161X_SCAN_SELECT_UPPER                  0x40
#define MAX1161X_SCAN_SELECT_SINGLE                 0x60
/* SCAN bits value to convert all pins from AIN0 to pin selected by CS3-CS0 */
#define MAX1161X_SCAN_BITS_DEFAULT                  INT8_C(0)
/* SCAN bits value to convert input selected by CS3-CS0 8 times */
#define MAX1161X_SCAN_BITS_REPEAT_8                 INT8_C(1)
/* SCAN bits value to convert either upper quart or half of channels depending on model */
#define MAX1161X_SCAN_BITS_UPPER                    INT8_C(2)
/* SCAN bits value to convert only channel selected by CS3-CS0 */
#define MAX1161X_SCAN_BITS_SINGLE                   INT8_C(3)


/* BIT MASK for Channel selection portion of Config Byte */
#define MAX1161X_CHANNEL_SEL_BITS_MASK              ~0x1E
#define MAX1161X_CHANNEL_SEL_AIN0P                  0x00
#define MAX1161X_CHANNEL_SEL_AIN1P                  0x02
#define MAX1161X_CHANNEL_SEL_AIN2P                  0x04
#define MAX1161X_CHANNEL_SEL_AIN3P                  0x06
#define MAX1161X_CHANNEL_SEL_AIN4P                  0x08
#define MAX1161X_CHANNEL_SEL_AIN5P                  0x0A
#define MAX1161X_CHANNEL_SEL_AIN6P                  0x0C
#define MAX1161X_CHANNEL_SEL_AIN7P                  0x0E
#define MAX1161X_CHANNEL_SEL_AIN8P                  0x10
#define MAX1161X_CHANNEL_SEL_AIN9P                  0x12
#define MAX1161X_CHANNEL_SEL_AIN10P                 0x14
#define MAX1161X_CHANNEL_SEL_AIN11P                 0x16
/* CS bits value for Positive Pin Being AIN0, Negative Pin dependent on SGL/DIF bit */
#define MAX1161X_CS_BITS_AIN0P                      INT8_C(0)
/* CS bits value for Positive Pin Being AIN1, Negative Pin dependent on SGL/DIF bit */
#define MAX1161X_CS_BITS_AIN1P                      INT8_C(1)
/* CS bits value for Positive Pin Being AIN2, Negative Pin dependent on SGL/DIF bit */
#define MAX1161X_CS_BITS_AIN2P                      INT8_C(2)
/* CS bits value for Positive Pin Being AIN3, Negative Pin dependent on SGL/DIF bit */
#define MAX1161X_CS_BITS_AIN3P                      INT8_C(3)
/* CS bits value for Positive Pin Being AIN4, Negative Pin dependent on SGL/DIF bit */
#define MAX1161X_CS_BITS_AIN4P                      INT8_C(4)
/* CS bits value for Positive Pin Being AIN5, Negative Pin dependent on SGL/DIF bit */
#define MAX1161X_CS_BITS_AIN5P                      INT8_C(5)
/* CS bits value for Positive Pin Being AIN6, Negative Pin dependent on SGL/DIF bit */
#define MAX1161X_CS_BITS_AIN6P                      INT8_C(6)
/* CS bits value for Positive Pin Being AIN7, Negative Pin dependent on SGL/DIF bit */
#define MAX1161X_CS_BITS_AIN7P                      INT8_C(7)
/* CS bits value for Positive Pin Being AIN8, Negative Pin dependent on SGL/DIF bit */
#define MAX1161X_CS_BITS_AIN8P                      INT8_C(8)
/* CS bits value for Positive Pin Being AIN9, Negative Pin dependent on SGL/DIF bit */
#define MAX1161X_CS_BITS_AIN9P                      INT8_C(9)
/* CS bits value for Positive Pin Being AIN10, Negative Pin dependent on SGL/DIF bit */
#define MAX1161X_CS_BITS_AIN10P                     INT8_C(10)
/* CS bits value for Positive Pin Being AIN11, Negative Pin dependent on SGL/DIF bit */
#define MAX1161X_CS_BITS_AIN11P                     INT8_C(11)


/* BIT MASK for SGLDIF portion of Congif Byte */
#define MAX1161X_SGLDIF_BIT_MASK                    ~0x01
#define MAX1161X_SGLDIF_DIFFERENTIAL                0x00
#define MAX1161X_SGLDIF_SINGLEENDED                 0x01
/* SGLDIF bit value for Differential Measurements */
#define MAX1161X_SGLDIF_BIT_DIFFERENTIAL            INT8_C(0)
/* SGLDIF bit value for Single Ended Measurements */
#define MAX1161X_SGLDIF_BIT_SINGLEENDED             INT8_C(1)

/* Conversion Time maximum when using the internal clock source */
#define MAX1161X_CONVERSION_TIME_INT_CLK_MAX_US    UINT32_C(8)

/*! 
 * @brief Write function pointer to be mapped to platform specific method(s)
 *  
 * @param[out]      reg_data    : Pointer to register data to be written to the MAX11615 Device
 * @param[in]       length      : Number of bytes to be written
 * @param[in,out]   intf_ptr    : Pointer to the interface object
 * 
 * @retval 0 for success
 * @retval Non-zero for failure
 */
typedef MAX1161X_INTF_RET_TYPE (*MAX1161X_write_fptr_t)(const uint8_t *reg_data, uint32_t length, void *intf_ptr);
/*!
 * @brief Read function pointer to be mapped to platform specific method(s)
 *
 * @param[out]      reg_data    : Pointer to register data to be read from the MAX1615 Device
 * @param[in]       length      : Number of bytes to be read from the device
 * @param[in,out]   intf_ptr    : Pointer to the interface object
 * 
 * @retval 0 for success
 * @retval Non-zero for failure
 */
typedef MAX1161X_INTF_RET_TYPE (*MAX1161X_read_fptr_t)(uint8_t *reg_data, uint32_t length, void* intf_ptr);
/*!
 * @brief Delay function used to wait the proper time before reading data from the device
 * 
 * @param[in]       period      : Number of microseconds to delay if time based
 * @param[in,out]   intf_ptr    : Pointer to the interface object
 */
typedef void (*MAX1161X_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

/*!
 * @brief Configuration Byte for the MAX1161X Chip
 * Reference Table 2 in Documentation
 */
union MAX1161X_conf{
    /* 8 bit value for the configuration word */
    uint8_t value;
    struct{
        /* Input Selection 
        Deafult = 1
        1 = Single-Ended 
        0 = Differential 
        */
        uint8_t SGLDIF:1;
        /* Channel Select
        Deafult = 0000
        0000 = +AIN0 -(GND/AIN1)
        0001 = +AIN1 -(GND/AIN0)
        0010 = +AIN2 -(GND/AIN3)
        0011 = +AIN3 -(GND/AIN2)
        0100 = +AIN4 -(GND/AIN5)
        0101 = +AIN5 -(GND/AIN4)
        0110 = +AIN6 -(GND/AIN7)
        0111 = +AIN7 -(GND/AIN6)
        1000 = +AIN8 -(GND/AIN9)
        1001 = +AIN9 -(GND/AIN8)
        1010 = +AIN10 -(GND/AIN11)
        1011 = +AIN11 -(GND/AIN10)
        */
        uint8_t CS:4;
        /* Scan Select
        Default = 00
        00 = Scans from AIN0 to CS bit input
        01 = No Scan
        10 = MAX11612/MAX11613 Scan upper half of channels
        10 = MAX11614/MAX11615 Scan upper quartile of channels
        10 = MAX11616/MAX11617 Scan upper half of channels
        11 = No Scan
        */
        uint8_t SCAN:2;
        /* Register Bit
        1 = Register Bit
        0 = Setup Bit
        */
        uint8_t REG:1;
    };
};

/*!
* @brief Setup Byte for the MAX1161X Chip
* Reference Table 1 in Documentation
*/
union MAX1161X_setup{
    /* 8 bit value for the setup word */
    uint8_t value;
    struct{
        /* Dont-Care bit can be 1 or 0 */
        uint8_t nc:1;
        /* Reset 
        Deafult = 0 
        1 = no action
        0 = reset config to default
        */
        uint8_t RST:1;
        /* Polarity 
        Default = 0 
        1 = bipolar 
        0 = unipolar
        */
        uint8_t POL:1;
        /* Clock Select
        Deafult = 0 
        1 = external clock
        0 = internal clock
        */
        uint8_t CLK:1;
        /* Reference Select 
        Default = 000
        00X = VDD Reference         | AIN_/Ref is analog input  | REF not connected | Internal Ref always off
        01X = External Reference    | AIN_/Ref is Ref Input     | Ref is Input      | Internal Ref always off
        100 = Internal Reference    | AIN_/Ref is analog input  | Ref not connected | Internal Ref always off 
        101 = Internal Reference    | AIN_/Ref is analog input  | Ref not connected | Internal Ref always on
        110 = Internal Reference    | AIN_/Ref is Ref Output    | Ref is output     | Internal Ref always off
        111 = Internal Reference    | AIN_/Ref is Ref Output    | Ref is output     | Internal Ref always on
        */
        uint8_t SEL:3;
        /* Register Bit
        1 = Register Bit
        0 = Setup Bit
        */
        uint8_t REG:1;
    };
};

/*!
 * @brief Data from the MAX1161X Chip for each channel
 */
struct MAX1161X_Data_t{
    //Analog Input ADC Counts
    int16_t AIN0_C;
    int16_t AIN1_C;
    int16_t AIN2_C;
    int16_t AIN3_C;
    int16_t AIN4_C;
    int16_t AIN5_C;
    int16_t AIN6_C;
    int16_t AIN7_C;
    int16_t AIN8_C;
    int16_t AIN9_C;
    int16_t AIN10_C;
    int16_t AIN11_C;
    //Analog Input ADC Voltages
    float AIN0_V;
    float AIN1_V;
    float AIN2_V;
    float AIN3_V;
    float AIN4_V;
    float AIN5_V;
    float AIN6_V;
    float AIN7_V;
    float AIN8_V;
    float AIN9_V;
    float AIN10_V;
    float AIN11_V;
};

/*!
 *  @brief MAX11615 Object
 */
struct MAX1161X_t{
    /*! Pointer to the interface device */
    void *intf_ptr;
    /*! Method to read from the MAX1161X Device */
    MAX1161X_read_fptr_t read;
    /*! Method to write to the MAX1161X Device */
    MAX1161X_write_fptr_t write;
    /*! Delay Method to wait while the conversion is completed */
    MAX1161X_delay_us_fptr_t delay_us;
    /*! Configuration Byte for the MAX1161X */
    union MAX1161X_conf conf;
    /*! Setup Byte for the MAX1161X */
    union MAX1161X_setup setup;
    /*! Stores interface pointer error */
    MAX1161X_INTF_RET_TYPE intf_rslt;
    /*! Data for each channel, Raw, Single Measurements */
    struct MAX1161X_Data_t data;
    /*! Reference Voltage for conversion to voltage */
    float ref_voltage;
    /*! Conversion Time required to measure a pin */
    uint32_t conversion_time_us;
};