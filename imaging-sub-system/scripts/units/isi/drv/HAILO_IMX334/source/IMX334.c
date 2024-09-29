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

#include <common/return_codes.h>
#include <ebase/builtins.h>
#include <ebase/types.h>
#include <common/misc.h>
#include <fcntl.h>
#include <isi/isi.h>
#include <isi/isi_iss.h>
#include <isi/isi_priv.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <sys/ioctl.h>

#include "IMX334_priv.h"
#include "vvsensor.h"

CREATE_TRACER(IMX334_INFO, "IMX334: ", INFO, 1);
CREATE_TRACER(IMX334_WARN, "IMX334: ", WARNING, 1);
CREATE_TRACER(IMX334_ERROR, "IMX334: ", ERROR, 1);
CREATE_TRACER(IMX334_DEBUG, "IMX334: ", INFO, 1);
CREATE_TRACER(IMX334_REG_INFO, "IMX334: ", INFO, 1);
CREATE_TRACER(IMX334_REG_DEBUG, "IMX334: ", INFO, 1);

#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#define IMX334_I2C_ADDR 0x1a

#define IMX334_IRIS_MIN_VAL 1
#define IMX334_IRIS_MAX_VAL 1
#define IMX334_MIN_GAIN_STEP                                          \
    (0.035) /**< min gain step size used by GUI (hardware min = 1/16; \
               1/16..32/16 depending on actual gain ) */
#define IMX334_PLL_PCLK 74250000
#define IMX334_HMAX 0xaec
#define IMX334_VMAX_30FPS 4500
#define IMX334_VMAX_MAX 135000
#define IMX334_MIN_SHR 5
#define IMX334_MAX_GAIN_AEC                                                    \
    (32.0f) /**< max. gain used by the AEC (arbitrarily chosen, hardware limit \
               = 62.0, driver limit = 32.0 ) */
#define IMX334_VS_MAX_INTEGRATION_TIME (0.0018)
#define IMX334_TRANSFER_BUFFER_LENGTH 3

FlickerModePeaksPerSec flickerPeaksPerSecMap[] = {
    { ISI_AE_ANTIBANDING_MODE_OFF, 0 },
    { ISI_AE_ANTIBANDING_MODE_50HZ, 100 },
    { ISI_AE_ANTIBANDING_MODE_60HZ, 120 },
    { ISI_AE_ANTIBANDING_MODE_AUTO, 100 }
};

/*****************************************************************************
 *Sensor Info
 *****************************************************************************/

static struct vvsensor_mode_s pimx334_mode_info[] = {
    {
        .index = 0,
        .size =
            {
                .bounds_width = 3840,
                .bounds_height = 2160,
                .top = 0,
                .left = 0,
                .width = 3840,
                .height = 2160,
            },
        .fps = 30,
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 12,
        .bayer_pattern = BAYER_RGGB,
    },
};

static RESULT IMX334_IsiSetPowerIss(IsiSensorHandle_t handle, bool_t on) {
    RESULT result = RET_SUCCESS;

    return (result);
}

static RESULT IMX334_IsiCreateIss(IsiSensorInstanceConfig_t *pConfig) {
    RESULT result = RET_SUCCESS;
    IMX334_Context_t *pIMX334Ctx;
    char i2c_file_path[PATH_MAX];

    memset(i2c_file_path, 0, PATH_MAX);

    if (!pConfig || !pConfig->pSensor) return (RET_NULL_POINTER);

    pIMX334Ctx = (IMX334_Context_t *)malloc(sizeof(IMX334_Context_t));
    if (!pIMX334Ctx) {
        return (RET_OUTOFMEM);
    }

    MEMSET(pIMX334Ctx, 0, sizeof(IMX334_Context_t));
    result = HalAddRef(pConfig->HalHandle);
    if (result != RET_SUCCESS) {
        free(pIMX334Ctx);
        return (result);
    }

    pIMX334Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pIMX334Ctx->IsiCtx.pSensor = pConfig->pSensor;
    pIMX334Ctx->GroupHold = BOOL_FALSE;
    pIMX334Ctx->OldGain = 1.0;
    pIMX334Ctx->OldIntegrationTime = 0.01;
    pIMX334Ctx->Configured = BOOL_FALSE;
    pIMX334Ctx->Streaming = BOOL_FALSE;
    pIMX334Ctx->TestPattern = BOOL_FALSE;
    pIMX334Ctx->isAfpsRun = BOOL_FALSE;
    pIMX334Ctx->SensorMode.index = pConfig->SensorModeIndex;
    pConfig->hSensor = (IsiSensorHandle_t)pIMX334Ctx;
    pIMX334Ctx->pattern = ISI_BPAT_RGRGGBGB;
    pIMX334Ctx->subdev = HalGetFdHandle(pConfig->HalHandle,
                                        HAL_MODULE_SENSOR);  // two sensors??
    pIMX334Ctx->KernelDriverFlag = 1;
    sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cBusNum);
    pIMX334Ctx->i2c_fd = open(i2c_file_path, O_RDWR);
    if (pIMX334Ctx->i2c_fd < 0) {
        TRACE(IMX334_INFO, "unable to open /dev/i2c-%d\n", pConfig->I2cBusNum);
        return RET_FAILURE;
    }

    if (ioctl(pIMX334Ctx->i2c_fd, I2C_SLAVE_FORCE, IMX334_I2C_ADDR) < 0) {
        TRACE(IMX334_INFO, "unable to set I2C_SLAVE_FORCE on /dev/i2c-%d\n",
              pConfig->I2cBusNum);
        return RET_FAILURE;
    }

    return (result);
}

static RESULT IMX334_IsiReleaseIss(IsiSensorHandle_t handle) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX334Ctx == NULL) return (RET_WRONG_HANDLE);

    (void)IMX334_IsiSetStreamingIss(pIMX334Ctx, BOOL_FALSE);
    (void)IMX334_IsiSetPowerIss(pIMX334Ctx, BOOL_FALSE);
    (void)HalDelRef(pIMX334Ctx->IsiCtx.HalHandle);
    close(pIMX334Ctx->i2c_fd);
    MEMSET(pIMX334Ctx, 0, sizeof(IMX334_Context_t));
    free(pIMX334Ctx);
    return (result);
}

static RESULT IMX334_IsiReadRegIss(IsiSensorHandle_t handle,
                                   const uint32_t Addr, uint32_t *pValue) {
    RESULT result = RET_SUCCESS;
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    char out[IMX334_TRANSFER_BUFFER_LENGTH];

    if (pIMX334Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX334_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    if (write(pIMX334Ctx->i2c_fd, out, sizeof(uint16_t)) != sizeof(uint16_t)) {
        return RET_FAILURE;
    }

    if (read(pIMX334Ctx->i2c_fd, out, 1) != 1) return RET_FAILURE;

    *pValue = out[0];

    return (result);
}

static RESULT IMX334_IsiWriteRegIss(IsiSensorHandle_t handle,
                                    const uint32_t Addr, const uint32_t Value) {
    RESULT result = RET_SUCCESS;
    char out[IMX334_TRANSFER_BUFFER_LENGTH];

    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    if (pIMX334Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX334_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    out[2] = Value;
    if (write(pIMX334Ctx->i2c_fd, out, sizeof(out)) != sizeof(out))
        result = RET_FAILURE;
    return (result);
}

static RESULT IMX334_UpdateFps(IMX334_Context_t *pIMX334Ctx, uint32_t vmax) {
    float frame_time = 0;
    frame_time = (vmax * pIMX334Ctx->one_line_exp_time);
    if (frame_time == 0) return RET_FAILURE;

    pIMX334Ctx->CurrFps = (uint32_t)(ceil(1 / frame_time));
    return RET_SUCCESS;
}

static RESULT IMX334_ReadVmax(IsiSensorHandle_t handle, uint32_t *vmax) {
    uint32_t vmax_low = 0, vmax_mid = 0, vmax_high = 0;
    RESULT result;

    result = IMX334_IsiReadRegIss(handle, 0x3030, &vmax_low);
    result |= IMX334_IsiReadRegIss(handle, 0x3031, &vmax_mid);
    result |= IMX334_IsiReadRegIss(handle, 0x3032, &vmax_high);
    if (result) return RET_FAILURE;
    *vmax = (vmax_high << 16) | (vmax_mid << 8) | vmax_low;
    return result;
}

static RESULT IMX334_WriteVmax(IsiSensorHandle_t handle, uint32_t vmax) {
    RESULT result;

    result = IMX334_IsiWriteRegIss(handle, 0x3030, vmax & 0xff);
    result |= IMX334_IsiWriteRegIss(handle, 0x3031, (vmax >> 8) & 0xff);
    result |= IMX334_IsiWriteRegIss(handle, 0x3032, (vmax >> 16) & 0x0f);
    if (!result) {
        return IMX334_UpdateFps((IMX334_Context_t *)handle, vmax);
    }
    return result;
}

static RESULT IMX334_WriteShr(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX334_IsiWriteRegIss(handle, 0x3058, (shr & 0xff));
    result = IMX334_IsiWriteRegIss(handle, 0x3059, (shr >> 8) & 0xff);
    result = IMX334_IsiWriteRegIss(handle, 0x305a, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX334_LockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX334_IsiWriteRegIss(handle, 0x3001, 0x1);

    return result;
}

static RESULT IMX334_UnlockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX334_IsiWriteRegIss(handle, 0x3001, 0x0);

    return result;
}

static RESULT IMX334_IsiGetModeIss(IsiSensorHandle_t handle, IsiMode_t *pMode) {
    TRACE(IMX334_INFO, "%s (enter)\n", __func__);
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    if (pIMX334Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    memcpy(pMode, &(pIMX334Ctx->SensorMode), sizeof(pIMX334Ctx->SensorMode));

    TRACE(IMX334_INFO, "%s (exit)\n", __func__);
    return (RET_SUCCESS);
}

static RESULT IMX334_IsiSetModeIss(IsiSensorHandle_t handle, IsiMode_t *pMode) {
    int ret = 0;
    TRACE(IMX334_INFO, "%s (enter)\n", __func__);

    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    if (pIMX334Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    HalContext_t *pHalCtx = (HalContext_t *)pIMX334Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, pMode);
    if (ret != 0) {
        return (RET_FAILURE);
    }

    return (RET_SUCCESS);
}

static RESULT IMX334_IsiHalEnumModeIss(HalHandle_t HalHandle,
                                       IsiEnumMode_t *pEnumMode) {
    HalContext_t *pHalCtx = HalHandle;
    if (pHalCtx == NULL) {
        return RET_NULL_POINTER;
    }

    if (pEnumMode->index >=
        (sizeof(pimx334_mode_info) / sizeof(pimx334_mode_info[0])))
        return RET_OUTOFRANGE;

    for (uint32_t i = 0;
         i < (sizeof(pimx334_mode_info) / sizeof(pimx334_mode_info[0])); i++) {
        if (pimx334_mode_info[i].index == pEnumMode->index) {
            memcpy(&pEnumMode->mode, &pimx334_mode_info[i], sizeof(IsiMode_t));
            TRACE(IMX334_ERROR, "%s (exit)\n", __func__);
            return RET_SUCCESS;
        }
    }

    return RET_NOTSUPP;
}

static RESULT IMX334_IsiEnumModeIss(IsiSensorHandle_t handle,
                                    IsiEnumMode_t *pEnumMode) {
    RESULT result = RET_SUCCESS;
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    if (pIMX334Ctx == NULL || pIMX334Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    HalContext_t *pHalCtx = (HalContext_t *)pIMX334Ctx->IsiCtx.HalHandle;
    result = IMX334_IsiHalEnumModeIss(pHalCtx, pEnumMode);
    if (result != RET_SUCCESS) {
        TRACE(IMX334_ERROR, "%s: sensor enum mode error!\n", __func__);
        return (RET_FAILURE);
    }

    return result;
}

static RESULT IMX334_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t *pCaps) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE(IMX334_INFO, "%s (enter)\n", __func__);

    if (pIMX334Ctx == NULL) return (RET_WRONG_HANDLE);

    if (pCaps == NULL) {
        return (RET_NULL_POINTER);
    }

    if (!pIMX334Ctx->Configured) IMX334_IsiSetupIss(handle, pCaps);

    pCaps->BusWidth = pIMX334Ctx->SensorMode.bit_width;
    pCaps->Mode = ISI_MODE_BAYER;
    pCaps->FieldSelection = ISI_FIELDSEL_BOTH;
    pCaps->YCSequence = ISI_YCSEQ_YCBYCR;
    pCaps->Conv422 = ISI_CONV422_COSITED;
    pCaps->BPat = pIMX334Ctx->SensorMode.bayer_pattern;
    pCaps->HPol = ISI_HPOL_REFPOS;
    pCaps->VPol = ISI_VPOL_POS;
    pCaps->Edge = ISI_EDGE_RISING;
    pCaps->Resolution.width = pIMX334Ctx->SensorMode.size.width;
    pCaps->Resolution.height = pIMX334Ctx->SensorMode.size.height;
    pCaps->SmiaMode = ISI_SMIA_OFF;
    pCaps->MipiLanes = ISI_MIPI_4LANES;

    if (pCaps->BusWidth == 10) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_10;
    } else if (pCaps->BusWidth == 12) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_12;
    } else {
        pCaps->MipiMode = ISI_MIPI_OFF;
    }
    TRACE(IMX334_INFO, "got caps - width %d height %d buswidth %d\n",
          pCaps->Resolution.width, pCaps->Resolution.height, pCaps->BusWidth);
    TRACE(IMX334_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX334_AecSetModeParameters(IMX334_Context_t *pIMX334Ctx,
                                          const IsiCaps_t *pConfig) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX334_INFO, "%s%s: (enter)\n", __func__,
          pIMX334Ctx->isAfpsRun ? "(AFPS)" : "");

    pIMX334Ctx->AecIntegrationTimeIncrement = pIMX334Ctx->one_line_exp_time;
    pIMX334Ctx->AecMinIntegrationTime =
        pIMX334Ctx->one_line_exp_time * pIMX334Ctx->MinIntegrationLine;
    pIMX334Ctx->AecMaxIntegrationTime =
        pIMX334Ctx->one_line_exp_time * pIMX334Ctx->MaxIntegrationLine;
    TRACE(IMX334_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __func__,
          pIMX334Ctx->isAfpsRun ? "(AFPS)" : "",
          pIMX334Ctx->AecMaxIntegrationTime);

    pIMX334Ctx->AecGainIncrement = IMX334_MIN_GAIN_STEP;

    // reflects the state of the sensor registers, must equal default settings
    pIMX334Ctx->AecCurGain = pIMX334Ctx->AecMinGain;
    pIMX334Ctx->AecCurIntegrationTime = pIMX334Ctx->AecMaxIntegrationTime;
    pIMX334Ctx->OldGain = 1;
    pIMX334Ctx->OldIntegrationTime = 0.0f;

    TRACE(IMX334_INFO, "%s%s: (exit)\n", __func__,
          pIMX334Ctx->isAfpsRun ? "(AFPS)" : "");

    return (result);
}

static RESULT IMX334_IsiSetupIss(IsiSensorHandle_t handle,
                                 const IsiCaps_t *pCaps) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX334_INFO, "%s (enter)\n", __func__);

    if (!pIMX334Ctx) {
        TRACE(IMX334_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pCaps == NULL) {
        TRACE(IMX334_ERROR,
              "%s: Invalid configuration (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMX334Ctx->Streaming != BOOL_FALSE) {
        return RET_WRONG_STATE;
    }

    struct vvsensor_mode_s *info = &(pIMX334Ctx->SensorMode);

    info->size.width = 3840;
    info->size.height = 2160;
    info->fps = 30;
    info->hdr_mode = 0;      /*linear*/
    info->bit_width = 12;    /* raw12 */
    info->bayer_pattern = 0; /*BAYER_RGGB*/

    info->ae_info.one_line_exp_time_ns = 7407;  // ns

    info->ae_info.max_integration_time = IMX334_VMAX_30FPS - IMX334_MIN_SHR;
    info->ae_info.min_integration_time = 1;
    info->ae_info.integration_accuracy = 1;

    info->ae_info.gain_accuracy = 1;
    info->ae_info.max_gain = 3981 * info->ae_info.gain_accuracy;
    info->ae_info.min_gain = 1;

    info->ae_info.cur_fps = info->fps;

    if (pIMX334Ctx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        pIMX334Ctx->enableHdr = true;
    } else {
        pIMX334Ctx->enableHdr = false;
    }

    pIMX334Ctx->one_line_exp_time =
        (float)(pIMX334Ctx->SensorMode.ae_info.one_line_exp_time_ns) /
        1000000000;
    pIMX334Ctx->MaxIntegrationLine =
        pIMX334Ctx->SensorMode.ae_info.max_integration_time;
    pIMX334Ctx->MinIntegrationLine =
        pIMX334Ctx->SensorMode.ae_info.min_integration_time;
    pIMX334Ctx->gain_accuracy = pIMX334Ctx->SensorMode.ae_info.gain_accuracy;
    pIMX334Ctx->AecMaxGain = (float)(pIMX334Ctx->SensorMode.ae_info.max_gain) /
                             pIMX334Ctx->gain_accuracy;
    pIMX334Ctx->AecMinGain = (float)(pIMX334Ctx->SensorMode.ae_info.min_gain) /
                             pIMX334Ctx->gain_accuracy;
    pIMX334Ctx->AecMinIris = IMX334_IRIS_MIN_VAL;
    pIMX334Ctx->AecMaxIris = IMX334_IRIS_MAX_VAL;
    pIMX334Ctx->original_vmax = 0;
    pIMX334Ctx->unlimit_fps = 0;
    pIMX334Ctx->unlimit_fps_vmax_changed = 0;
    pIMX334Ctx->flicker_fps_mode = ISI_AE_ANTIBANDING_MODE_OFF;

    pIMX334Ctx->MaxFps = pIMX334Ctx->SensorMode.fps;
    pIMX334Ctx->CurrFps = pIMX334Ctx->MaxFps;
    TRACE(IMX334_INFO, "%s - got caps - width %d height %d buswidth %d\n",
          __func__, pIMX334Ctx->SensorMode.size.width,
          pIMX334Ctx->SensorMode.size.height, pIMX334Ctx->SensorMode.bit_width);

    TRACE(IMX334_INFO, "MinGain %f MaxGain %f\n", pIMX334Ctx->AecMinGain,
          pIMX334Ctx->AecMaxGain);

    memcpy(&pIMX334Ctx->CapsConfig, pCaps, sizeof(pIMX334Ctx->CapsConfig));

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful,
     * bits 6..0 are reserved, reset bit is not sticky */
    TRACE(IMX334_DEBUG, "%s: IMX334 System-Reset executed\n", __func__);
    osSleep(100);

    result = IMX334_AecSetModeParameters(pIMX334Ctx, pCaps);
    if (result != RET_SUCCESS) {
        TRACE(IMX334_ERROR, "%s: SetupOutputWindow failed.\n", __func__);
        return (result);
    }

    pIMX334Ctx->Configured = BOOL_TRUE;
    TRACE(IMX334_INFO, "%s: (exit)\n", __func__);
    return 0;
}

static RESULT IMX334_IsiCheckConnectionIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    return (result);
}

static RESULT IMX334_IsiGetRevisionIss(IsiSensorHandle_t handle,
                                       uint32_t *pValue) {
    RESULT result = RET_SUCCESS;
    int ret = 0;
    uint32_t reg_val;
    uint32_t sensor_id;

    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    if (pIMX334Ctx == NULL || pIMX334Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }
    HalContext_t *pHalCtx = (HalContext_t *)pIMX334Ctx->IsiCtx.HalHandle;

    if (!pValue) return (RET_NULL_POINTER);

    if (pIMX334Ctx->KernelDriverFlag) {
        ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, &sensor_id);
        if (ret != 0) {
            TRACE(IMX334_ERROR, "%s: Read Sensor ID Error! \n", __func__);
            return (RET_FAILURE);
        }
    } else {
        reg_val = 0;
        result = IMX334_IsiReadRegIss(handle, 0x3a04, &reg_val);
        sensor_id = (reg_val & 0xff) << 8;

        reg_val = 0;
        result |= IMX334_IsiReadRegIss(handle, 0x3a05, &reg_val);
        sensor_id |= (reg_val & 0xff);
    }

    *pValue = sensor_id;
    TRACE(IMX334_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX334_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX334_INFO, "%s (enter)\n", __func__);

    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;

    pIMX334Ctx->Streaming = on;

    TRACE(IMX334_INFO, "%s (exit)\n", __func__);
    return (result);
}

static inline int IMX334_getFlickerPeaksPerSec(IsiSensorAntibandingMode_t mode) {
    int num_modes = sizeof(flickerPeaksPerSecMap) / sizeof(FlickerModePeaksPerSec);
    for (int i = 0; i < num_modes; i++) {
        if (flickerPeaksPerSecMap[i].mode == mode) {
            return flickerPeaksPerSecMap[i].value;
        }
    }
    return 0; // Default to 0 if mode not found
}
static inline uint32_t IMX334_getNewVmaxAntiFlicker(IMX334_Context_t *pIMX334Ctx, uint32_t requestedVmax) {
    uint32_t closestVmax = requestedVmax;
    int peaks = 0;
    int difference = INT_MAX;
    int minDifference = INT_MAX;
    if (!pIMX334Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (-1);
    }
    peaks = IMX334_getFlickerPeaksPerSec(pIMX334Ctx->flicker_fps_mode);
    if (peaks == 0) {
        // No anti-flicker mode, return requested or original Vmax
        if (pIMX334Ctx->unlimit_fps && pIMX334Ctx->unlimit_fps_vmax_changed)
            return requestedVmax;
        else
            return pIMX334Ctx->original_vmax; 
    }
    for (int i = 1; i < peaks; ++i) {
        uint32_t candidateVmax = i / (peaks * pIMX334Ctx->one_line_exp_time);
        if (candidateVmax < pIMX334Ctx->SensorMode.size.height || candidateVmax < pIMX334Ctx->original_vmax)
            continue;
        difference = abs((int)requestedVmax - (int)candidateVmax);
        if (difference < minDifference) {
            minDifference = difference;
            closestVmax = candidateVmax;
        }
        // Since i / peaks is monotonically increasing, we can break early if the difference starts to increase
        if (candidateVmax > requestedVmax && difference > minDifference) {
            break;
        }
    }
    return closestVmax;
}

static RESULT IMX334_IsiUnlimitFpsIss(IsiSensorHandle_t handle,
                                      float maxIntegrationTime) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX334_INFO, "%s (enter)\n", __func__);

    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    if (!pIMX334Ctx) {
        return RET_NULL_POINTER;
    }
    pIMX334Ctx->unlimit_fps = 1;
    if (maxIntegrationTime < 0)
        pIMX334Ctx->MaxIntegrationLine = IMX334_VMAX_MAX - IMX334_MIN_SHR;
    else
        pIMX334Ctx->MaxIntegrationLine =
            MIN((uint32_t)(maxIntegrationTime / pIMX334Ctx->one_line_exp_time),
                IMX334_VMAX_MAX - IMX334_MIN_SHR);
    pIMX334Ctx->AecMaxIntegrationTime =
        pIMX334Ctx->one_line_exp_time * pIMX334Ctx->MaxIntegrationLine;

    return result;
}

static RESULT IMX334_IsiLimitFpsIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    uint32_t current_vmax = 0;
    uint32_t new_vmax = 0;
    TRACE(IMX334_INFO, "%s (enter)\n", __func__);

    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    if (!pIMX334Ctx) {
        return RET_NULL_POINTER;
    }
    IMX334_ReadVmax(handle, &current_vmax);
    if (current_vmax == 0) {
        TRACE(IMX334_INFO, "%s - exit because current_vmax is 0\n", __func__);
        return result;
    }
    pIMX334Ctx->unlimit_fps = 0;
    pIMX334Ctx->unlimit_fps_vmax_changed = 0;
    if (pIMX334Ctx->original_vmax == 0) {
        pIMX334Ctx->original_vmax = current_vmax;
    }

    new_vmax = pIMX334Ctx->original_vmax;
    if (pIMX334Ctx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
        new_vmax = IMX334_getNewVmaxAntiFlicker(pIMX334Ctx, pIMX334Ctx->original_vmax);
        TRACE(IMX334_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX334Ctx->flicker_fps_mode, new_vmax);
    }
    if (current_vmax != new_vmax) {
        result |= IMX334_LockRegHold(handle);
        result |= IMX334_WriteVmax(handle, new_vmax);
        result |= IMX334_UnlockRegHold(handle);

        int shr = MAX((int)current_vmax - (int)(pIMX334Ctx->AecCurIntegrationTime / pIMX334Ctx->one_line_exp_time), IMX334_MIN_SHR);
        float configuredIntegrationTime = (new_vmax - shr) * pIMX334Ctx->one_line_exp_time;
        pIMX334Ctx->OldIntegrationTime = configuredIntegrationTime;
        pIMX334Ctx->AecCurIntegrationTime = configuredIntegrationTime;
    }

    pIMX334Ctx->MaxIntegrationLine =
        MAX(new_vmax - IMX334_MIN_SHR, 0);
    pIMX334Ctx->AecMaxIntegrationTime =
        pIMX334Ctx->one_line_exp_time * pIMX334Ctx->MaxIntegrationLine;

    return result;
}

static RESULT IMX334_IsiGetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float *pMinIris, float *pMaxIris) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX334_INFO, "%s: (enter)\n", __func__);

    if (pIMX334Ctx == NULL) {
        TRACE(IMX334_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIris == NULL) || (pMaxIris == NULL)) {
        TRACE(IMX334_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIris = pIMX334Ctx->AecMinIris;
    *pMaxIris = pIMX334Ctx->AecMaxIris;

    TRACE(IMX334_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX334_IsiSetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float minIris, float maxIris) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX334_INFO, "%s: (enter)\n", __func__);

    if (pIMX334Ctx == NULL) {
        TRACE(IMX334_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    pIMX334Ctx->AecMinIris = minIris;
    pIMX334Ctx->AecMaxIris = maxIris;

    TRACE(IMX334_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX334_IsiGetGainLimitsIss(IsiSensorHandle_t handle,
                                         float *pMinGain, float *pMaxGain) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX334_INFO, "%s: (enter)\n", __func__);

    if (pIMX334Ctx == NULL) {
        TRACE(IMX334_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinGain == NULL) || (pMaxGain == NULL)) {
        TRACE(IMX334_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinGain = pIMX334Ctx->AecMinGain;
    *pMaxGain = pIMX334Ctx->AecMaxGain;

    TRACE(IMX334_ERROR, "%s: (enter)\n", __func__);
    return (result);
}

static RESULT IMX334_IsiGetIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float *pMinIntegrationTime,
    float *pMaxIntegrationTime) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX334_INFO, "%s: (enter)\n", __func__);
    if (pIMX334Ctx == NULL) {
        TRACE(IMX334_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX334_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }
    *pMinIntegrationTime = pIMX334Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMX334Ctx->AecMaxIntegrationTime;

    TRACE(IMX334_INFO, "%s: (enter)\n", __func__);
    return (result);
}

RESULT IMX334_IsiGetGainIss(IsiSensorHandle_t handle, float *pSetGain) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX334_INFO, "%s: (enter)\n", __func__);

    if (pIMX334Ctx == NULL) {
        TRACE(IMX334_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pSetGain == NULL) {
        return (RET_NULL_POINTER);
    }
    *pSetGain = pIMX334Ctx->AecCurGain;
    TRACE(IMX334_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX334_IsiGetSEF1GainIss(IsiSensorHandle_t handle, float *pSetGain) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX334_INFO, "%s: (enter)\n", __func__);

    if (pIMX334Ctx == NULL) {
        TRACE(IMX334_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pSetGain == NULL) {
        return (RET_NULL_POINTER);
    }

    *pSetGain = pIMX334Ctx->AecCurGainSEF1;

    TRACE(IMX334_INFO, "%s: (exit)\n", __func__);

    return (result);
}

RESULT IMX334_IsiGetIrisIncrementIss(IsiSensorHandle_t handle, float *pIncr) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX334_INFO, "%s: (enter)\n", __func__);
    if (pIMX334Ctx == NULL) {
        TRACE(IMX334_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = 1;

    TRACE(IMX334_INFO, "%s: (exit)\n", __func__);

    return (result);
}

RESULT IMX334_IsiGetGainIncrementIss(IsiSensorHandle_t handle, float *pIncr) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX334_INFO, "%s: (enter)\n", __func__);

    if (pIMX334Ctx == NULL) {
        TRACE(IMX334_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = pIMX334Ctx->AecGainIncrement;

    TRACE(IMX334_INFO, "%s: (exit)\n", __func__);

    return (result);
}

static inline uint32_t _linear2sensorGain(float gain)
{
    uint32_t db = 0;
    float log_gain = log10(gain);
    log_gain = (log_gain * 10 * 20) / 3;
    db = (uint32_t)(log_gain);
    return db;
}

static inline float _sensorGain2linear(uint32_t db)
{
    float gain = ((float)(db) * 3) / 200;
    gain = pow(10, gain);
    return gain;
}

RESULT IMX334_IsiSetGainIss(IsiSensorHandle_t handle, float NewGain,
                            float *pSetGain, float *hdr_ratio) {
    RESULT result = RET_SUCCESS;
    uint32_t old_gain_log;
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    if (pIMX334Ctx == NULL || pIMX334Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    old_gain_log = _linear2sensorGain(pIMX334Ctx->AecCurGain);

    uint32_t Gain = _linear2sensorGain(NewGain);
    if (old_gain_log == Gain) return RET_SUCCESS;
    result = IMX334_IsiWriteRegIss(handle, 0x3001, 0x01);
    result = IMX334_IsiWriteRegIss(handle, 0x30e8, (Gain & 0x00ff));
    result = IMX334_IsiWriteRegIss(handle, 0x30e9, (Gain & 0x0700) >> 8);
    result = IMX334_IsiWriteRegIss(handle, 0x3001, 0x00);
    pIMX334Ctx->OldGain = NewGain;

    if (result != 0) {
        return RET_FAILURE;
    }

    pIMX334Ctx->AecCurGain = _sensorGain2linear(Gain);

    *pSetGain = pIMX334Ctx->AecCurGain;
    TRACE(IMX334_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
    return (result);
}

RESULT IMX334_IsiSetSEF1GainIss(IsiSensorHandle_t handle,
                                float NewIntegrationTime, float NewGain,
                                float *pSetGain, float *hdr_ratio) {
    RESULT result = RET_SUCCESS;

    return (result);
}

RESULT IMX334_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
                                       float *pSetIntegrationTime) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;

    if (!pIMX334Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pSetIntegrationTime) return (RET_NULL_POINTER);
    *pSetIntegrationTime = pIMX334Ctx->AecCurIntegrationTime;
    return (result);
}

RESULT IMX334_IsiGetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
                                           float *pSetIntegrationTime) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;

    if (!pIMX334Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }
    if (!pSetIntegrationTime) return (RET_NULL_POINTER);

    *pSetIntegrationTime = pIMX334Ctx->AecCurIntegrationTimeSEF1;
    return (result);
}

RESULT IMX334_IsiGetIntegrationTimeIncrementIss(IsiSensorHandle_t handle,
                                                float *pIncr) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;

    if (!pIMX334Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pIncr) return (RET_NULL_POINTER);

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders
    // in the application)
    *pIncr = pIMX334Ctx->AecIntegrationTimeIncrement;
    return (result);
}

RESULT IMX334_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
                                       float NewIntegrationTime,
                                       float *pSetIntegrationTime,
                                       uint8_t *pNumberOfFramesToSkip,
                                       float *hdr_ratio) {
    RESULT result = RET_SUCCESS;

    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;

    int shr = 0;
    uint32_t exp = 0;
    uint32_t new_vmax = 0;
    uint32_t old_exp = 0;
    uint32_t vmax_updated = 0;
    uint32_t current_vmax = 0;
    if (!pIMX334Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pSetIntegrationTime || !pNumberOfFramesToSkip) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMX334Ctx->Streaming) {
        return RET_FAILURE;
    }

    exp = NewIntegrationTime / pIMX334Ctx->one_line_exp_time;
    old_exp = pIMX334Ctx->OldIntegrationTime / pIMX334Ctx->one_line_exp_time;

    TRACE(IMX334_DEBUG, "%s: set AEC_PK_EXPO=0x%05x\n", __func__, exp);

    if ((uint32_t)old_exp != (uint32_t)exp) {
        if (exp > pIMX334Ctx->MaxIntegrationLine || exp == 0) {
            TRACE(IMX334_ERROR, "%s: exp %u out of range (%u)\n", __func__,
                    exp, pIMX334Ctx->MaxIntegrationLine);
            return RET_FAILURE;
        }

        if (IMX334_ReadVmax(handle, &current_vmax)) return RET_FAILURE;

        if (pIMX334Ctx->original_vmax == 0)
            pIMX334Ctx->original_vmax = current_vmax;

        if (pIMX334Ctx->original_vmax - IMX334_MIN_SHR >
            pIMX334Ctx->MaxIntegrationLine) {
            pIMX334Ctx->MaxIntegrationLine =
                pIMX334Ctx->original_vmax - IMX334_MIN_SHR;
            pIMX334Ctx->AecMaxIntegrationTime =
                pIMX334Ctx->one_line_exp_time * pIMX334Ctx->MaxIntegrationLine;
        }

        shr = current_vmax - exp;

        if (shr < IMX334_MIN_SHR || current_vmax > pIMX334Ctx->original_vmax) {
            if (shr < IMX334_MIN_SHR) {
                new_vmax = MIN(exp + IMX334_MIN_SHR, pIMX334Ctx->MaxIntegrationLine + IMX334_MIN_SHR);
            } else {
                new_vmax = MAX(current_vmax - shr + IMX334_MIN_SHR, pIMX334Ctx->original_vmax);
            }
            pIMX334Ctx->unlimit_fps_vmax_changed = new_vmax > pIMX334Ctx->original_vmax && pIMX334Ctx->unlimit_fps;
    
            if (pIMX334Ctx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
                new_vmax = IMX334_getNewVmaxAntiFlicker(pIMX334Ctx, new_vmax);
                TRACE(IMX334_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX334Ctx->flicker_fps_mode, new_vmax);
            }
            
            shr = MAX(IMX334_MIN_SHR, (int)new_vmax - (int)exp);
            vmax_updated = 1;

        } else {
            new_vmax = current_vmax;
        }

        result |= IMX334_LockRegHold(handle);
        if (vmax_updated && pIMX334Ctx->unlimit_fps) {
            IMX334_WriteVmax(handle, new_vmax);
        }
        result |= IMX334_WriteShr(handle, shr);
        result |= IMX334_UnlockRegHold(handle);

        float configuredIntegrationTime = (new_vmax - shr) * pIMX334Ctx->one_line_exp_time;
        pIMX334Ctx->OldIntegrationTime = configuredIntegrationTime;
        pIMX334Ctx->AecCurIntegrationTime = configuredIntegrationTime;

        *pNumberOfFramesToSkip = 1U;
    } else {
        *pNumberOfFramesToSkip = 0U;  // no frame skip
    }

    *pSetIntegrationTime = pIMX334Ctx->AecCurIntegrationTime;
    TRACE(IMX334_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTime);
    return (result);
}

RESULT IMX334_IsiSetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
                                           float NewIntegrationTime,
                                           float *pSetIntegrationTimeSEF1,
                                           uint8_t *pNumberOfFramesToSkip,
                                           float *hdr_ratio) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX334_IsiExposureControlExpandedIss(
    IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime,
    float NewIris, uint8_t *pNumberOfFramesToSkip, float *pSetGain,
    float *pSetIntegrationTime, float *pSetIris, float *hdr_ratio) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    if (pIMX334Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pNumberOfFramesToSkip == NULL) || (pSetGain == NULL) ||
        (pSetIntegrationTime == NULL)) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMX334Ctx->enableHdr) {
        result = IMX334_IsiSetSEF1IntegrationTimeIss(
            handle, NewIntegrationTime, pSetIntegrationTime,
            pNumberOfFramesToSkip, hdr_ratio);
        result = IMX334_IsiSetSEF1GainIss(handle, NewIntegrationTime, NewGain,
                                          pSetGain, hdr_ratio);
    }
    result = IMX334_IsiSetIntegrationTimeIss(handle, NewIntegrationTime,
                                             pSetIntegrationTime,
                                             pNumberOfFramesToSkip, hdr_ratio);
    result = IMX334_IsiSetGainIss(handle, NewGain, pSetGain, hdr_ratio);
    return result;
}

RESULT IMX334_IsiExposureControlIss(IsiSensorHandle_t handle, float NewGain,
                                    float NewIntegrationTime,
                                    uint8_t *pNumberOfFramesToSkip,
                                    float *pSetGain, float *pSetIntegrationTime,
                                    float *hdr_ratio) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    if (pIMX334Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pNumberOfFramesToSkip == NULL) || (pSetGain == NULL) ||
        (pSetIntegrationTime == NULL)) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMX334Ctx->enableHdr) {
        result = IMX334_IsiSetSEF1IntegrationTimeIss(
            handle, NewIntegrationTime, pSetIntegrationTime,
            pNumberOfFramesToSkip, hdr_ratio);
        result = IMX334_IsiSetSEF1GainIss(handle, NewIntegrationTime, NewGain,
                                          pSetGain, hdr_ratio);
    }
    result = IMX334_IsiSetIntegrationTimeIss(handle, NewIntegrationTime,
                                             pSetIntegrationTime,
                                             pNumberOfFramesToSkip, hdr_ratio);
    result = IMX334_IsiSetGainIss(handle, NewGain, pSetGain, hdr_ratio);
    return result;
}

RESULT IMX334_IsiGetCurrentExposureIss(IsiSensorHandle_t handle,
                                       float *pSetGain,
                                       float *pSetIntegrationTime) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX334Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pSetGain == NULL) || (pSetIntegrationTime == NULL))
        return (RET_NULL_POINTER);

    *pSetGain = pIMX334Ctx->AecCurGain;
    *pSetIntegrationTime = pIMX334Ctx->AecCurIntegrationTime;

    return (result);
}

RESULT IMX334_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t *pFps) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX334Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *pFps = pIMX334Ctx->CurrFps;

    return (result);
}

RESULT IMX334_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t Fps) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX334_IsiSetFlickerFpsIss(IsiSensorHandle_t handle, uint32_t flickerMode) {
    RESULT result = RET_SUCCESS;
    IMX334_Context_t* pIMX334Ctx = (IMX334_Context_t*)handle;
    uint32_t current_vmax = 0;
    uint32_t requested_vmax = 0;
    uint32_t shr = 0;
    int exp = 0;
    TRACE(IMX334_DEBUG, "%s: set sensor flickerMode = %d\n", __func__, flickerMode);
    if (!pIMX334Ctx) {
        return RET_NULL_POINTER;
    }
    if (pIMX334Ctx->flicker_fps_mode == flickerMode) {
        return RET_SUCCESS;
    }
    if (flickerMode > ISI_AE_ANTIBANDING_MODE_AUTO) {
        TRACE(IMX334_INFO, "%s: Invalid flickerMode (%d), setting ISI_AE_ANTIBANDING_MODE_AUTO instead.\n", __func__, flickerMode);
        flickerMode = ISI_AE_ANTIBANDING_MODE_AUTO;
    }
    pIMX334Ctx->flicker_fps_mode = flickerMode;
    result = IMX334_ReadVmax(handle, &current_vmax);
    if (result != RET_SUCCESS) {
        TRACE(IMX334_ERROR, "%s: Unable to read VMAX\n", __func__);
        return (result);
    }
    if (pIMX334Ctx->original_vmax == 0) {
        pIMX334Ctx->original_vmax = current_vmax;
    }
    exp = pIMX334Ctx->AecCurIntegrationTime / pIMX334Ctx->one_line_exp_time;
    shr = MAX((int)current_vmax - exp, IMX334_MIN_SHR);

    if (current_vmax > pIMX334Ctx->original_vmax) {
        current_vmax = MAX((int)current_vmax - (int)shr + IMX334_MIN_SHR, IMX334_MIN_SHR);
        shr = MAX((int)current_vmax - exp, IMX334_MIN_SHR);
        pIMX334Ctx->unlimit_fps_vmax_changed = current_vmax > pIMX334Ctx->original_vmax && pIMX334Ctx->unlimit_fps;
    }

    requested_vmax = IMX334_getNewVmaxAntiFlicker(pIMX334Ctx, current_vmax);
    requested_vmax = MAX( MIN(requested_vmax, IMX334_VMAX_MAX), 1);
    
    if (current_vmax != requested_vmax) {
        shr = MAX( (int)requested_vmax - (int)current_vmax + (int)shr , IMX334_MIN_SHR);
        result |= IMX334_LockRegHold(handle);
        result |= IMX334_WriteVmax(handle, requested_vmax);
        result |= IMX334_WriteShr(handle, shr);
        result |= IMX334_UnlockRegHold(handle);
        if (result != RET_SUCCESS) {
            TRACE(IMX334_ERROR, "%s: Unable to write VMAX or Shr0\n", __func__);
            return (result);
        }
        TRACE(IMX334_DEBUG, "%s - writing 0x%x to VMAX, writing 0x%x to SHR0\n", __func__, requested_vmax, shr);
        float configuredIntegrationTime = (requested_vmax - shr) * pIMX334Ctx->one_line_exp_time;
        pIMX334Ctx->OldIntegrationTime = configuredIntegrationTime;
        pIMX334Ctx->AecCurIntegrationTime = configuredIntegrationTime;
        TRACE(IMX334_DEBUG, "%s: Ti=%f\n", __func__, configuredIntegrationTime);
    }
    
    pIMX334Ctx->MaxIntegrationLine = MAX( MIN(requested_vmax - IMX334_MIN_SHR, IMX334_VMAX_MAX - IMX334_MIN_SHR), 1);
    pIMX334Ctx->AecMaxIntegrationTime = pIMX334Ctx->one_line_exp_time * pIMX334Ctx->MaxIntegrationLine;
    TRACE(IMX334_INFO, "%s: set sensor fps = %d\n", __func__,
          pIMX334Ctx->CurrFps);
    TRACE(IMX334_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX334_IsiGetAutoFpsInfoIss(IsiSensorHandle_t handle,
                                   IsiAutoFps_t *pAutoFpsInfo) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX334_IsiGetStartEvIss(IsiSensorHandle_t handle, uint64_t *pStartEv) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX334_IsiGetIspStatusIss(IsiSensorHandle_t handle,
                                 IsiIspStatus_t *pIspStatus) {
    IMX334_Context_t *pIMX334Ctx = (IMX334_Context_t *)handle;
    if (pIMX334Ctx == NULL || pIMX334Ctx->IsiCtx.HalHandle == NULL) {
        return RET_WRONG_HANDLE;
    }

    pIspStatus->useSensorAE = false;
    pIspStatus->useSensorBLC = false;
    pIspStatus->useSensorAWB = false;

    return RET_SUCCESS;
}

RESULT IMX334_IsiSetTpgIss(IsiSensorHandle_t handle, IsiTpg_t Tpg) {
    RESULT result = RET_SUCCESS;

    return (result);
}

RESULT IMX334_IsiGetTpgIss(IsiSensorHandle_t handle, IsiTpg_t *Tpg) {
    RESULT result = RET_SUCCESS;

    return (result);
}

RESULT IMX334_IsiFocusCreateIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX334_IsiFocusReleaseIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX334_IsiFocusGetCalibrateIss(IsiSensorHandle_t handle,
                                      IsiFocusCalibAttr_t *pFocusCalib) {
    return RET_SUCCESS;
}

RESULT IMX334_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos) {
    return RET_SUCCESS;
}

RESULT IMX334_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos) {
    return RET_SUCCESS;
}

RESULT IMX334_IsiGetSensorIss(IsiSensor_t *pIsiSensor) {
    RESULT result = RET_SUCCESS;
    static const char SensorName[16] = "IMX334";
    TRACE(IMX334_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor != NULL) {
        pIsiSensor->pszName = SensorName;
        pIsiSensor->pIsiCreateIss = IMX334_IsiCreateIss;
        pIsiSensor->pIsiReleaseIss = IMX334_IsiReleaseIss;
        pIsiSensor->pIsiReadRegIss = IMX334_IsiReadRegIss;
        pIsiSensor->pIsiWriteRegIss = IMX334_IsiWriteRegIss;
        pIsiSensor->pIsiGetModeIss = IMX334_IsiGetModeIss;
        pIsiSensor->pIsiSetModeIss = IMX334_IsiSetModeIss;
        pIsiSensor->pIsiEnumModeIss = IMX334_IsiEnumModeIss;
        pIsiSensor->pIsiGetCapsIss = IMX334_IsiGetCapsIss;
        pIsiSensor->pIsiSetupIss = IMX334_IsiSetupIss;
        pIsiSensor->pIsiCheckConnectionIss = IMX334_IsiCheckConnectionIss;
        pIsiSensor->pIsiGetRevisionIss = IMX334_IsiGetRevisionIss;
        pIsiSensor->pIsiSetStreamingIss = IMX334_IsiSetStreamingIss;

        pIsiSensor->pIsiUnlimitFpsIss = IMX334_IsiUnlimitFpsIss;
        pIsiSensor->pIsiLimitFpsIss = IMX334_IsiLimitFpsIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss = IMX334_IsiExposureControlIss;
        pIsiSensor->pIsiExposureControlExpandedIss =
            IMX334_IsiExposureControlExpandedIss;
        pIsiSensor->pIsiGetGainLimitsIss = IMX334_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss =
            IMX334_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetIrisLimitsIss = IMX334_IsiGetIrisLimitsIss;
        pIsiSensor->pIsiSetIrisLimitsIss = IMX334_IsiSetIrisLimitsIss;
        pIsiSensor->pIsiGetVSGainIss = IMX334_IsiGetSEF1GainIss;
        pIsiSensor->pIsiGetGainIss = IMX334_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss = IMX334_IsiGetGainIncrementIss;
        pIsiSensor->pIsiGetIrisIncrementIss = IMX334_IsiGetIrisIncrementIss;
        pIsiSensor->pIsiSetGainIss = IMX334_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss = IMX334_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetVSIntegrationTimeIss =
            IMX334_IsiGetSEF1IntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss =
            IMX334_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss = IMX334_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetFpsIss = IMX334_IsiGetFpsIss;
        pIsiSensor->pIsiSetFpsIss = IMX334_IsiSetFpsIss;
        pIsiSensor->pIsiSetFlickerFpsIss = IMX334_IsiSetFlickerFpsIss;
        pIsiSensor->pIsiGetAutoFpsInfoIss = IMX334_IsiGetAutoFpsInfoIss;
        pIsiSensor->pIsiGetStartEvIss = IMX334_IsiGetStartEvIss;

        /* SENSOR ISP */
        pIsiSensor->pIsiGetIspStatusIss = IMX334_IsiGetIspStatusIss;

        /* SENSOE OTHER FUNC*/
        pIsiSensor->pIsiSetPowerIss = IMX334_IsiSetPowerIss;
        pIsiSensor->pIsiSetTpgIss = IMX334_IsiSetTpgIss;
        pIsiSensor->pIsiGetTpgIss = IMX334_IsiGetTpgIss;

        /* AF */
        pIsiSensor->pIsiFocusCreateIss = IMX334_IsiFocusCreateIss;
        pIsiSensor->pIsiFocusReleaseIss = IMX334_IsiFocusReleaseIss;
        pIsiSensor->pIsiFocusGetCalibrateIss = IMX334_IsiFocusGetCalibrateIss;
        pIsiSensor->pIsiFocusSetIss = IMX334_IsiFocusSetIss;
        pIsiSensor->pIsiFocusGetIss = IMX334_IsiFocusGetIss;

    } else {
        result = RET_NULL_POINTER;
    }

    TRACE(IMX334_INFO, "%s (exit)\n", __func__);
    return (result);
}

/*****************************************************************************
 * each sensor driver need declare this struct for isi load
 *****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x9012,
    .pIsiHalEnumModeIss = IMX334_IsiHalEnumModeIss,
    .pIsiGetSensorIss = IMX334_IsiGetSensorIss,
};
