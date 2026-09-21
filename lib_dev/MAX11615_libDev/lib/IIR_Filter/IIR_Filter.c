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
* @file       IIR_Filter.c
* @date       2026-01-29
* @version    v0.1.0
*
*/

#include "IIR_Filter.h"

/*!
 * @brief Initializes the IIR Filter Structure
 * Sets the alpha coefficient to the alpha parameter
 * Sets the out parameter to 0
 *
 * @param[in]       alpha   : The initial Alpha coefficient to be used 
 * @param[in,out]   f       : The Filter Struct to be initialized
 */
void IIR_Filter_Init(float alpha, struct IIR_Filter_t *f){
    if(alpha < 0.0f) f->alpha = 0.0f;
    else if(alpha > 1.0f) f->alpha = 1.0f;
    else f->alpha = alpha;
    f->out = 0.0f;
}

/*!
 * @brief Processes the input data through the filter and updates the filter struct parameters
 *
 * @param[in]       in  : Input data to pass to the filter
 * @param[in,out]   f   : IIR Filter struct to be updated
 * 
 * @retval Filtered ouput
 */
float IIR_Filter_Update(float in, struct IIR_Filter_t *f){
    f->out = (1.0f - f->alpha) * in + f->alpha * f->out;
    return f->out;
}

