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
* @file       IIR_Filter.h
* @date       2026-01-29
* @version    v0.1.0
*
*/

/**
 * @defgroup iir_filter IIR_Filter
 * @brief IIR Filter API for simple use of Infinite Impulse Response Filter with set intervals in the time domain.
 */

#ifndef __IIR_FILTER_HH__
#define __IIR_FILTER_HH__

/*Start CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef IIR_ALPHA_DEFAULT
#define IIR_ALPHA_DEFAULT 0.2f
#endif

/**
 * \ingroup iir_filter
 * \defgroup iirFilterApiStructure
 * @brief Structure for using the API
 */

/*!
 * \ingroup iirFilterApiStructure
 * \page iir_filter_api_IIR_Filter_t IIR_Filter_t
 * \code 
 * struct IIR_Filter_t{
 *  float alpha;
 *  float out;
 *  };
 * \endcode
 * @details This Strucutre is the interface structure for using the API.
 */
struct IIR_Filter_t{
    float alpha;
    float out;
};

/**
 * \ingroup iir_filter
 * \defgroup iirFilterApiInit Initialization
 * @brief Initialize Interface Structure prior to using API.
 */

/*!
 * \ingroup iirFilterApiInit
 * \page iir_filter_api_IIR_Filter_Init IIR_Filter_Init
 * \code
 * void IIR_Filter_Init(float alpha, struct IIR_Filter_t *f)
 * \endcode
 * @details Initializes the IIR Filter Structure
 * Sets the alpha coefficient to the alpha parameter
 * Sets the out parameter to 0
 *
 * @param[in]       alpha   : The initial Alpha coefficient to be used 
 * @param[in,out]   f       : The Filter Struct to be initialized
 */
void IIR_Filter_Init(float alpha, struct IIR_Filter_t *f);

/**
 * \ingroup iir_filter
 * \defgroup iirFilterApiUpdate Update
 * @brief Update and Calculation API for applying the filter to data
 */

/*!
 * \ingroup iirFilterApiUpdate
 * \page iir_filter_api_IIR_Filter_Update IIR_Filter_Update
 * \code
 * float IIR_Filter_Update(float in, struct IIR_Filter_t *f)
 * \endcode
 * @details Processes the input data through the filter and updates the filter struct parameters
 *
 * @param[in]       in  : Input data to pass to the filter
 * @param[in,out]   f   : IIR Filter struct to be updated
 * 
 * @return Filtered output stored in the IIR_Filter_t structure passed.
 */
float IIR_Filter_Update(float in, struct IIR_Filter_t *f);

#ifdef __cplusplus
}
#endif 
/*End CPP Guard*/

#endif





