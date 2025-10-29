/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014-2023 Vivante Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 ****************************************************************************/

/* VeriSilicon 2022 */

/**
 * @file isi_priv.h
 *
 * @brief Interface description for image sensor specific implementation (iss).
 *
 *****************************************************************************/
/**
 * @page module_name_page Module Name
 * Describe here what this module does.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref module_name
 *
 * @defgroup isi_priv
 * @{
 *
 */
#ifndef __ISI_PRIV_H__
#define __ISI_PRIV_H__

#include <ebase/types.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "isi_iss.h"

/******************************************************************************
* DEFINES
******************************************************************************/
#define MAX_REGISTER_TABLE_ENTRIES 256

#define ISI_I2C_NR_DAT_BYTES_1  (1)                     // sensor has some  8-bit registers
#define ISI_I2C_NR_DAT_BYTES_2  (2)                     // sensor has some 16-bit registers
#define ISI_I2C_NR_DAT_BYTES_4  (4)                     // sensor has some 32-bit registers

#ifndef ARRAY_SIZE
#   define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif


/******************************************************************************
* TYPEDEFS
******************************************************************************/

/*****************************************************************************/
/**
 *          IsiSensorContext_t
 *
 * @brief   abstract class of a sensor context
 *
 */
/*****************************************************************************/
typedef struct IsiSensorContext_s
{
    int            fd;                  /**< /dev/v4l-subdev file description */
    HalHandle_t    HalHandle;           /**< Handle of HAL session to use. */
    IsiSensor_t    *pSensor;            /**< points to the sensor device */
} IsiSensorContext_t;


#ifdef __cplusplus
}
#endif

#endif /* __ISI_PRIV_H__ */
