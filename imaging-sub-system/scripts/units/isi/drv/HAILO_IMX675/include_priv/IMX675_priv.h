/******************************************************************************\
|* Copyright (c) 2023 Hailo Technologies Ltd. (Hailo) All rights reserved.    *|
|*                                                                            *|
|* This proprietary software is the confidential and proprietary information  *|
|* of Hailo Technologies Ltd. and licensed to Hailo by VeriSilicon Holdings   *|
|* Co., Ltd. You may not disclose, copy, distribute, any part of this         *|
|* software without the express written permission of Hailo.                  *|
|* You may use this software only in accordance with the terms of the license *|
|* agreement provided with the software ("End User License Agreement").       *|
|*                                                                            *|
\******************************************************************************/
/******************************************************************************\
|* Copyright (c) 2020 by VeriSilicon Holdings Co., Ltd. ("VeriSilicon")       *|
|* All Rights Reserved.                                                       *|
|*                                                                            *|
|* The material in this file is confidential and contains trade secrets of    *|
|* of VeriSilicon.  This is proprietary information owned or licensed by      *|
|* VeriSilicon.  No part of this work may be disclosed, reproduced, copied,   *|
|* transmitted, or used in any way for any purpose, without the express       *|
|* written permission of VeriSilicon.                                         *|
|*                                                                            *|
\******************************************************************************/
/**
 * @file IMX675_priv.h
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
 * @defgroup IMX675_priv
 * @{
 *
 */
#ifndef __IMX675_PRIV_H__
#define __IMX675_PRIV_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include <hal/hal_api.h>
#include <isi/isi_common.h>
#include "vvsensor.h"

// TODO: Remove this line when the driver is fully implemented (all modes supported, including HDR)
#define BRINGUP_CONFIG

#ifndef NDEBUG
#include "log.h"
#define LOGTAG "HAILO_IMX675"
enum
{
    IMX675_INFO = 0,
    IMX675_WARN = 1,
    IMX675_ERROR = 2,
    IMX675_DEBUG = 3,
    IMX675_REG_INFO = 4,
    IMX675_REG_DEBUG = 5
};
#undef CREATE_TRACER
#define CREATE_TRACER(name, arg_prefix, arg_level, arg_enabled)

#undef TRACE
#define TRACE(level,...) \
if(level == IMX675_DEBUG) ALOGD(__VA_ARGS__); \
if(level == IMX675_REG_DEBUG) ALOGD(__VA_ARGS__); \
if(level == IMX675_INFO) ALOGI(__VA_ARGS__); \
if(level == IMX675_REG_INFO) ALOGI(__VA_ARGS__); \
if(level == IMX675_WARN) ALOGW(__VA_ARGS__); \
if(level == IMX675_ERROR) ALOGE(__VA_ARGS__);
#else
#include <ebase/trace.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct IMX675_Context_s
{
    IsiSensorContext_t  IsiCtx;                 /**< common context of ISI and ISI driver layer; @note: MUST BE FIRST IN DRIVER CONTEXT */

    struct vvsensor_mode_s SensorMode;
    uint32_t            KernelDriverFlag;
    char                SensorRegCfgFile[64];

    uint32_t              HdrMode;
    uint32_t              Resolution;
    uint32_t              MaxFps;
    uint32_t              MinFps;
    uint32_t              CurrFps;
    //// modify below here ////

    IsiCaps_t           CapsConfig;                 /**< sensor configuration */
    bool_t              Configured;             /**< flags that config was applied to sensor */
    bool_t              Streaming;              /**< flags that csensor is streaming data */
    bool_t              TestPattern;            /**< flags that sensor is streaming test-pattern */

    bool_t              isAfpsRun;              /**< if true, just do anything required for Afps parameter calculation, but DON'T access SensorHW! */

    float               one_line_exp_time;
    uint32_t            MaxIntegrationLine;
    uint16_t            MinIntegrationLine;
    uint32_t            gain_accuracy;

    uint32_t            original_vmax;
    uint32_t            cur_rhs1;
    uint32_t            cur_rhs2;
    int                 unlimit_fps;       

    uint16_t            FrameLengthLines;       /**< frame line length */
    uint16_t            CurFrameLengthLines;

    float               AecMinGain;
    float               AecMaxGain;

    float               AecMinIris;
    float               AecMaxIris;

    float               AecMinIntegrationTime;
    float               AecMaxIntegrationTime;

    float               AecIntegrationTimeIncrement; /**< _smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application) */
    float               AecGainIncrement;            /**< _smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application) */

    float               AecCurGainLEF;
    float               AecCurIntegrationTimeLEF;
    float               AecCurGainSEF1;
    float               AecCurIntegrationTimeSEF1;
    float               AecCurGainSEF2;
    float               AecCurIntegrationTimeSEF2;

    bool                GroupHold;
    uint32_t            OldGain;
    float               OldIntegrationTime;
    uint32_t            OldGainSEF1;
    uint32_t            OldIntegrationTimeSEF1;

    int                 subdev;
    bool                enableHdr;
    uint8_t             pattern;
    int                 i2c_fd;
} IMX675_Context_t;



static RESULT IMX675_IsiUnlimitFpsIss(IsiSensorHandle_t handle, float maxIntegrationTime);
static RESULT IMX675_IsiLimitFpsIss(IsiSensorHandle_t handle);
static RESULT IMX675_IsiSetFlickerFpsIss(IsiSensorHandle_t handle, uint32_t flickerMode);


static RESULT IMX675_IsiCreateIss(IsiSensorInstanceConfig_t *pConfig);
static RESULT IMX675_IsiReleaseIss(IsiSensorHandle_t handle);
static RESULT IMX675_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t * pCaps);
static RESULT IMX675_IsiSetupIss(IsiSensorHandle_t handle, 
								const IsiCaps_t *pCaps);
static RESULT IMX675_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on);
static RESULT IMX675_IsiSetPowerIss(IsiSensorHandle_t handle, bool_t on);
static RESULT IMX675_IsiGetRevisionIss(IsiSensorHandle_t handle, 
								uint32_t *pValue);
static RESULT IMX675_IsiGetGainLimitsIss(IsiSensorHandle_t handle,
								float *pMinGain, 
								float *pMaxGain);
static RESULT IMX675_IsiGetIrisLimitsIss(IsiSensorHandle_t handle,
								float *pMinIris, 
								float *pMaxIris);

static RESULT IMX675_IsiSetIrisLimitsIss(IsiSensorHandle_t handle,
								float minIris, 
								float maxIris);


static RESULT IMX675_IsiGetIntegrationTimeLimitsIss(IsiSensorHandle_t handle, 
								float *pMinIntegrationTime,
								float *pMaxIntegrationTime);
static RESULT IMX675_IsiExposureControlIss(IsiSensorHandle_t handle,
								float NewGain, 
								float NewIntegrationTime,
								uint8_t *pNumberOfFramesToSkip,
                                float *pSetGain, 
								float *pSetIntegrationTime,
								float *hdr_ratio);
static RESULT IMX675_IsiExposureControlExpandedIss(IsiSensorHandle_t handle,
								float NewGain, 
								float NewIntegrationTime,
								float NewIris,
								uint8_t *pNumberOfFramesToSkip,
                                float *pSetGain, 
								float *pSetIntegrationTime,
								float *pSetNewIris,
								float *hdr_ratio);

static RESULT IMX675_IsiGetGainIss(IsiSensorHandle_t handle,float *pSetGain);

static RESULT IMX675_IsiGetLEFGainIss(IsiSensorHandle_t handle,float *pSetGain);

#ifndef BRINGUP_CONFIG
static RESULT IMX675_IsiGetSEF1GainIss(IsiSensorHandle_t handle,float *pSetGain);

static RESULT IMX675_IsiGetSEF2GainIss(IsiSensorHandle_t handle,float *pSetGain);
#endif //BRINGUP_CONFIG

static RESULT IMX675_IsiGetGainIncrementIss(IsiSensorHandle_t handle,float *pIncr);

static RESULT IMX675_IsiSetGainIss(IsiSensorHandle_t handle,
								float NewGain, 
								float *pSetGain,
								float *hdr_ratio);

static RESULT IMX675_IsiSetLEFGainIss(IsiSensorHandle_t handle,
								float NewGain, 
								float *pSetGain,
								float *hdr_ratio);

#ifndef BRINGUP_CONFIG
static RESULT IMX675_IsiSetSEF1GainIss(IsiSensorHandle_t handle,
								float NewIntegrationTime,
								float NewGain, 
								float *pSetGain,
								float *hdr_ratio);

static RESULT IMX675_IsiSetSEF2GainIss(IsiSensorHandle_t handle,
								float NewIntegrationTime,
								float NewGain, 
								float *pSetGain,
								float *hdr_ratio);
#endif //BRINGUP_CONFIG

static RESULT IMX675_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
								float *pSetIntegrationTime);

static RESULT IMX675_IsiGetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
								float *pSetIntegrationTime);

#ifndef BRINGUP_CONFIG
static RESULT IMX675_IsiGetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
								float *pSetIntegrationTime);

static RESULT IMX675_IsiGetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
								float *pSetIntegrationTime);
#endif //BRINGUP_CONFIG

static RESULT IMX675_IsiGetIntegrationTimeIncrementIss(IsiSensorHandle_t handle,
								float *pIncr);

static RESULT IMX675_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
								float NewIntegrationTime,
								float *pSetIntegrationTime,
								uint8_t *pNumberOfFramesToSkip,
								float *hdr_ratio);

static RESULT IMX675_IsiSetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
								float NewIntegrationTime, 
								float *pSetIntegrationTime,
								uint8_t *pNumberOfFramesToSkip,
								float *hdr_ratio);

#ifndef BRINGUP_CONFIG
static RESULT IMX675_IsiSetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
								float NewIntegrationTime,
								float *pSetIntegrationTime,
								uint8_t *pNumberOfFramesToSkip,
								float *hdr_ratio);

static RESULT IMX675_IsiSetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
								float NewIntegrationTime,
								float *pSetIntegrationTime,
								uint8_t *pNumberOfFramesToSkip,
								float *hdr_ratio);
#endif //BRINGUP_CONFIG


#ifdef __cplusplus
}
#endif

/* @} IMX675priv */

#endif    /* __IMX675PRIV_H__ */

