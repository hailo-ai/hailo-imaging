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

#include "IMX678_priv.h"
#include "vvsensor.h"

CREATE_TRACER(IMX678_INFO, "IMX678: ", INFO, 1);
CREATE_TRACER(IMX678_WARN, "IMX678: ", WARNING, 1);
CREATE_TRACER(IMX678_ERROR, "IMX678: ", ERROR, 1);
CREATE_TRACER(IMX678_DEBUG, "IMX678: ", INFO, 1);
CREATE_TRACER(IMX678_REG_INFO, "IMX678: ", INFO, 1);
CREATE_TRACER(IMX678_REG_DEBUG, "IMX678: ", INFO, 1);

#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

/** @defgroup group_defines  API definitions
 *  @{
 */

#define IMX678_I2C_ADDR 0x1a
#define IMX678_IRIS_MIN_VAL 1

/** @} */ // end of group_defines

#define IMX678_IRIS_MAX_VAL 1
#define IMX678_MIN_GAIN_STEP                                          \
    (0.035) /**< min gain step size used by GUI (hardware min = 1/16; \
               1/16..32/16 depending on actual gain ) */
#define IMX678_PLL_PCLK 74250000
#define IMX678_HMAX 0xaec
#define IMX678_VMAX_30FPS 4500
#define IMX678_VMAX_3DOL_HDR 6750
#define IMX678_VMAX_2DOL_HDR 6750
#define IMX678_VMAX_MAX 135000
#define IMX678_MIN_SHR 3
#define IMX678_MAX_GAIN_AEC                                                    \
    (32.0f) /**< max. gain used by the AEC (arbitrarily chosen, hardware limit \
               = 62.0, driver limit = 32.0 ) */
#define IMX678_VS_MAX_INTEGRATION_TIME (0.0018)
#define IMX678_TRANSFER_BUFFER_LENGTH 3
#define IMX678_MAX_GAIN 3981
#define IMX678_SHR0_RHS2_GAP 7
#define IMX678_2DOL_SHR0_RHS1_GAP 5
#define IMX678_SHR0_FSC_GAP 3
#define IMX678_2DOL_SHR0_FSC_GAP 2
#define IMX678_3DOL_SHR1_RHS1_GAP 7
#define IMX678_2DOL_SHR1_MIN_GAP 5
#define IMX678_SHR1_RHS1_GAP 3
#define IMX678_2DOL_SHR1_RHS1_GAP 5
#define IMX678_SHR2_RHS1_GAP 7
#define IMX678_SHR2_RHS2_GAP 3
#define IMX678_PIXEL_CLK_RATE 74.25
#define DEFAULT_RHS1 0x91
#define DEFAULT_RHS2 0xaa
#define MICRO_2_NANO 1000

FlickerModePeaksPerSec flickerPeaksPerSecMap[] = {
    { ISI_AE_ANTIBANDING_MODE_OFF, 0 },
    { ISI_AE_ANTIBANDING_MODE_50HZ, 100 },
    { ISI_AE_ANTIBANDING_MODE_60HZ, 120 },
    { ISI_AE_ANTIBANDING_MODE_AUTO, 100 }
};

/*****************************************************************************
 *Forward Declarations
*****************************************************************************/
RESULT IMX678_IsiSetIrisIss( IsiSensorHandle_t handle,
                    const float NewIris);

/*****************************************************************************
 *Sensor Info
*****************************************************************************/

static struct vvsensor_mode_s pimx678_mode_info[] = {
    {
        .index     = 0,
        .size      ={
			.bounds_width  = 3840,
			.bounds_height = 2160,
			.top           = 0,
			.left          = 0,
			.width         = 3840,
			.height        = 2160,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_RGGB,
	.ae_info = {
		.one_line_exp_time_ns = 7407,
		.max_integration_time = IMX678_VMAX_30FPS - IMX678_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX678_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
    {
        .index     = 1,
        .size      ={
			.bounds_width  = 1920,
			.bounds_height = 1080,
			.top           = 0,
			.left          = 0,
			.width         = 1920,
			.height        = 1080,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_RGGB,
	.ae_info = {
		.one_line_exp_time_ns = 7407,
		.max_integration_time = IMX678_VMAX_30FPS - IMX678_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX678_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
    {
        .index     = 2,
        .size      ={
			.bounds_width  = 1920,
			.bounds_height = 1080,
			.top           = 0,
			.left          = 0,
			.width         = 1920,
			.height        = 1080,
		},
		.fps       = 20 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_3DOL,
		.bit_width = 12,
		.bayer_pattern = BAYER_RGGB,
        .ae_info = {
		.one_line_exp_time_ns = 7407,
		.max_integration_time = DEFAULT_RHS1 - IMX678_3DOL_SHR1_RHS1_GAP,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX678_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 20
	}
    },
    {
        .index     = 3,
        .size      ={
			.bounds_width  = 3840,
			.bounds_height = 2160,
			.top           = 0,
			.left          = 0,
			.width         = 3840,
			.height        = 2160,
		},
		.fps       = 20 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_3DOL,
		.bit_width = 12,
		.bayer_pattern = BAYER_RGGB,
        .ae_info = {
		.one_line_exp_time_ns = 7407,
		.max_integration_time = DEFAULT_RHS1 - IMX678_3DOL_SHR1_RHS1_GAP,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX678_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 20
	}
    },
    {
        .index     = 4,
        .size      ={
			.bounds_width  = 3840,
			.bounds_height = 2160,
			.top           = 0,
			.left          = 0,
			.width         = 3840,
			.height        = 2160,
		},
		.fps       = 20 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_L_AND_S,
		.bit_width = 12,
		.bayer_pattern = BAYER_RGGB,
        .ae_info = {
		.one_line_exp_time_ns = 7407,
		.max_integration_time = DEFAULT_RHS1 - IMX678_2DOL_SHR1_RHS1_GAP,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX678_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    }

};

static RESULT IMX678_IsiSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    RESULT result = RET_SUCCESS;

    return (result);
}

static RESULT IMX678_IsiCreateIss(IsiSensorInstanceConfig_t* pConfig) {
    RESULT result = RET_SUCCESS;
    IMX678_Context_t* pIMX678Ctx;
    char i2c_file_path[PATH_MAX];

    memset(i2c_file_path, 0, PATH_MAX);

    if (!pConfig || !pConfig->pSensor) return (RET_NULL_POINTER);

    pIMX678Ctx = (IMX678_Context_t*)malloc(sizeof(IMX678_Context_t));
    if (!pIMX678Ctx) {
        return (RET_OUTOFMEM);
    }

    MEMSET(pIMX678Ctx, 0, sizeof(IMX678_Context_t));
    result = HalAddRef(pConfig->HalHandle);
    if (result != RET_SUCCESS) {
        free(pIMX678Ctx);
        return (result);
    }

    pIMX678Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pIMX678Ctx->IsiCtx.pSensor = pConfig->pSensor;
    pIMX678Ctx->GroupHold = BOOL_FALSE;
    pIMX678Ctx->OldGain = 1.0;
    pIMX678Ctx->OldIntegrationTime = 0.01;
    pIMX678Ctx->Configured = BOOL_FALSE;
    pIMX678Ctx->Streaming = BOOL_FALSE;
    pIMX678Ctx->TestPattern = BOOL_FALSE;
    pIMX678Ctx->isAfpsRun = BOOL_FALSE;
    pIMX678Ctx->SensorMode.index = pConfig->SensorModeIndex;
    pConfig->SensorModeIndex =
	    MAX(MIN((sizeof(pimx678_mode_info) / sizeof(pimx678_mode_info[0])),
		    pConfig->SensorModeIndex),
		0);
    memcpy(&pIMX678Ctx->SensorMode,
	   &pimx678_mode_info[pConfig->SensorModeIndex],
	   sizeof(pIMX678Ctx->SensorMode));
    pConfig->hSensor = (IsiSensorHandle_t)pIMX678Ctx;
    pIMX678Ctx->pattern = ISI_BPAT_RGRGGBGB;
    pIMX678Ctx->subdev = HalGetFdHandle(pConfig->HalHandle,
                                        HAL_MODULE_SENSOR);  // two sensors??
    pIMX678Ctx->KernelDriverFlag = 1;
    sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cBusNum);
    pIMX678Ctx->i2c_fd = open(i2c_file_path, O_RDWR);
    if (pIMX678Ctx->i2c_fd < 0) {
        TRACE(IMX678_INFO, "unable to open /dev/i2c-%d\n", pConfig->I2cBusNum);
        return RET_FAILURE;
    }

    if (ioctl(pIMX678Ctx->i2c_fd, I2C_SLAVE_FORCE, IMX678_I2C_ADDR) < 0) {
        TRACE(IMX678_INFO, "unable to set I2C_SLAVE_FORCE on /dev/i2c-%d\n",
              pConfig->I2cBusNum);
        return RET_FAILURE;
    }

    return (result);
}

static RESULT IMX678_IsiReleaseIss(IsiSensorHandle_t handle) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX678Ctx == NULL) return (RET_WRONG_HANDLE);

    (void)IMX678_IsiSetStreamingIss(pIMX678Ctx, BOOL_FALSE);
    (void)IMX678_IsiSetPowerIss(pIMX678Ctx, BOOL_FALSE);
    (void)HalDelRef(pIMX678Ctx->IsiCtx.HalHandle);
    close(pIMX678Ctx->i2c_fd);
    MEMSET(pIMX678Ctx, 0, sizeof(IMX678_Context_t));
    free(pIMX678Ctx);
    return (result);
}

static RESULT IMX678_IsiReadRegIss(IsiSensorHandle_t handle,
                                   const uint32_t Addr, uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    char out[IMX678_TRANSFER_BUFFER_LENGTH];

    if (pIMX678Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX678_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    if (write(pIMX678Ctx->i2c_fd, out, sizeof(uint16_t)) != sizeof(uint16_t)) {
        return RET_FAILURE;
    }

    if (read(pIMX678Ctx->i2c_fd, out, 1) != 1) return RET_FAILURE;

    *pValue = out[0];

    return (result);
}

static RESULT IMX678_IsiWriteRegIss(IsiSensorHandle_t handle,
                                    const uint32_t Addr, const uint32_t Value) {
    RESULT result = RET_SUCCESS;
    char out[IMX678_TRANSFER_BUFFER_LENGTH];

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX678_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    out[2] = Value;
    if (write(pIMX678Ctx->i2c_fd, out, sizeof(out)) != sizeof(out))
        result = RET_FAILURE;
    return (result);
}

static RESULT IMX678_UpdateFps(IMX678_Context_t *pIMX678Ctx, uint32_t vmax) {
    float frame_time = 0;
    frame_time = (vmax * pIMX678Ctx->one_line_exp_time);
    if (frame_time == 0) return RET_FAILURE;

    pIMX678Ctx->CurrFps = (uint32_t)(ceil(1 / frame_time));
    return RET_SUCCESS;
}

static RESULT IMX678_ReadVmax(IsiSensorHandle_t handle, uint32_t* vmax) {
    uint32_t vmax_low = 0, vmax_mid = 0, vmax_high = 0;
    RESULT result;

    result = IMX678_IsiReadRegIss(handle, 0x3028, &vmax_low);
    result |= IMX678_IsiReadRegIss(handle, 0x3029, &vmax_mid);
    result |= IMX678_IsiReadRegIss(handle, 0x302a, &vmax_high);
    if (result) return RET_FAILURE;

    *vmax = (vmax_high << 16) | (vmax_mid << 8) | vmax_low;
    return result;
}

static RESULT IMX678_ReadHmax(IsiSensorHandle_t handle, uint32_t* hmax) {
    uint32_t hmax_low = 0, hmax_high = 0;
    RESULT result;

    result = IMX678_IsiReadRegIss(handle, 0x302c, &hmax_low);
    result |= IMX678_IsiReadRegIss(handle, 0x302d, &hmax_high);
    if (result) return RET_FAILURE;

    *hmax = (hmax_high << 8) | hmax_low;
    return result;
}

static RESULT IMX678_WriteVmax(IsiSensorHandle_t handle, uint32_t vmax) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3028, vmax & 0xff);
    result |= IMX678_IsiWriteRegIss(handle, 0x3029, (vmax >> 8) & 0xff);
    result |= IMX678_IsiWriteRegIss(handle, 0x302a, (vmax >> 16) & 0x0f);
    if (!result) {
        return IMX678_UpdateFps((IMX678_Context_t *)handle, vmax);
    }

    return result;
}

static RESULT IMX678_ReadRHS1(IsiSensorHandle_t handle, uint32_t* rhs1) {
    uint32_t rhs1_low = 0, rhs1_mid = 0, rhs1_high = 0;
    RESULT result;

    result = IMX678_IsiReadRegIss(handle, 0x3060, &rhs1_low);
    result |= IMX678_IsiReadRegIss(handle, 0x3061, &rhs1_mid);
    result |= IMX678_IsiReadRegIss(handle, 0x3062, &rhs1_high);
    if (result) return RET_FAILURE;

    *rhs1 = (rhs1_high << 16) | (rhs1_mid << 8) | rhs1_low;
    return result;
}

static RESULT IMX678_ReadRHS2(IsiSensorHandle_t handle, uint32_t* rhs2) {
    uint32_t rhs2_low = 0, rhs2_mid = 0, rhs2_high = 0;
    RESULT result;

    result = IMX678_IsiReadRegIss(handle, 0x3064, &rhs2_low);
    result |= IMX678_IsiReadRegIss(handle, 0x3065, &rhs2_mid);
    result |= IMX678_IsiReadRegIss(handle, 0x3066, &rhs2_high);
    if (result) return RET_FAILURE;

    *rhs2 = (rhs2_high << 16) | (rhs2_mid << 8) | rhs2_low;
    return result;
}


static RESULT IMX678_WriteShr0(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3050, shr & 0xff);
    result |= IMX678_IsiWriteRegIss(handle, 0x3051, (shr >> 8) & 0xff);
    result |= IMX678_IsiWriteRegIss(handle, 0x3052, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX678_WriteShr1(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3054, shr & 0xff);
    result |= IMX678_IsiWriteRegIss(handle, 0x3055, (shr >> 8) & 0xff);
    result |= IMX678_IsiWriteRegIss(handle, 0x3056, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX678_WriteShr2(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3058, shr & 0xff);
    result |= IMX678_IsiWriteRegIss(handle, 0x3059, (shr >> 8) & 0xff);
    result |= IMX678_IsiWriteRegIss(handle, 0x305a, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX678_WriteGain(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3070, (gain & 0x00ff));
	result |= IMX678_IsiWriteRegIss(handle, 0x3071, (gain & 0x0700) >> 8);

    return result;
}

static RESULT IMX678_WriteGain1(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3072, (gain & 0x00ff));
	result |= IMX678_IsiWriteRegIss(handle, 0x3073, (gain & 0x0700) >> 8);

    return result;
}

static RESULT IMX678_WriteGain2(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3074, (gain & 0x00ff));
	result |= IMX678_IsiWriteRegIss(handle, 0x3075, (gain & 0x0700) >> 8);

    return result;
}

static RESULT IMX678_LockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3001, 0x1);

    return result;
}

static RESULT IMX678_UnlockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3001, 0x0);

    return result;
}

static RESULT IMX678_IsiGetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    TRACE(IMX678_INFO, "%s (enter)\n", __func__);
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    memcpy(pMode, &(pIMX678Ctx->SensorMode), sizeof(pIMX678Ctx->SensorMode));

    TRACE(IMX678_INFO, "%s (exit)\n", __func__);
    return (RET_SUCCESS);
}

static RESULT IMX678_IsiSetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    int ret = 0;
    TRACE(IMX678_INFO, "%s (enter)\n", __func__);

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX678Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, pMode);
    if (ret != 0) {
        return (RET_FAILURE);
    }

    return (RET_SUCCESS);
}

static RESULT IMX678_IsiHalEnumModeIss(HalHandle_t HalHandle,
                                       IsiEnumMode_t* pEnumMode) {
    HalContext_t* pHalCtx = HalHandle;
    if (pHalCtx == NULL) {
        return RET_NULL_POINTER;
    }

    if (pEnumMode->index >=
        (sizeof(pimx678_mode_info) / sizeof(pimx678_mode_info[0])))
        return RET_OUTOFRANGE;

    for (uint32_t i = 0;
         i < (sizeof(pimx678_mode_info) / sizeof(pimx678_mode_info[0])); i++) {
        if (pimx678_mode_info[i].index == pEnumMode->index) {
            memcpy(&pEnumMode->mode, &pimx678_mode_info[i], sizeof(IsiMode_t));
            TRACE(IMX678_ERROR, "%s (exit)\n", __func__);
            return RET_SUCCESS;
        }
    }

    return RET_NOTSUPP;
}

static RESULT IMX678_IsiEnumModeIss(IsiSensorHandle_t handle,
                                    IsiEnumMode_t* pEnumMode) {
    RESULT result = RET_SUCCESS;
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL || pIMX678Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    HalContext_t* pHalCtx = (HalContext_t*)pIMX678Ctx->IsiCtx.HalHandle;
    result = IMX678_IsiHalEnumModeIss(pHalCtx, pEnumMode);
    if (result != RET_SUCCESS) {
        TRACE(IMX678_ERROR, "%s: sensor enum mode error!\n", __func__);
        return (RET_FAILURE);
    }

    return result;
}

static RESULT IMX678_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t* pCaps) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;

    RESULT result = RET_SUCCESS;

    TRACE(IMX678_INFO, "%s (enter)\n", __func__);

    if (pIMX678Ctx == NULL) return (RET_WRONG_HANDLE);

    if (pCaps == NULL) {
        return (RET_NULL_POINTER);
    }

    if (!pIMX678Ctx->Configured) IMX678_IsiSetupIss(handle, pCaps);

    pCaps->BusWidth = pIMX678Ctx->SensorMode.bit_width;
    pCaps->Mode = ISI_MODE_BAYER;
    pCaps->FieldSelection = ISI_FIELDSEL_BOTH;
    pCaps->YCSequence = ISI_YCSEQ_YCBYCR;
    pCaps->Conv422 = ISI_CONV422_COSITED;
    pCaps->BPat = pIMX678Ctx->SensorMode.bayer_pattern;
    pCaps->HPol = ISI_HPOL_REFPOS;
    pCaps->VPol = ISI_VPOL_POS;
    pCaps->Edge = ISI_EDGE_RISING;
    pCaps->Resolution.width = pIMX678Ctx->SensorMode.size.width;
    pCaps->Resolution.height = pIMX678Ctx->SensorMode.size.height;
    pCaps->SmiaMode = ISI_SMIA_OFF;
    pCaps->MipiLanes = ISI_MIPI_4LANES;

    if (pCaps->BusWidth == 10) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_10;
    } else if (pCaps->BusWidth == 12) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_12;
    } else {
        pCaps->MipiMode = ISI_MIPI_OFF;
    }
    TRACE(IMX678_INFO, "got caps - width %d height %d buswidth %d\n",
          pCaps->Resolution.width, pCaps->Resolution.height, pCaps->BusWidth);
    TRACE(IMX678_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX678_AecSetModeParameters(IMX678_Context_t* pIMX678Ctx,
                                          const IsiCaps_t* pConfig) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX678_INFO, "%s%s: (enter)\n", __func__,
          pIMX678Ctx->isAfpsRun ? "(AFPS)" : "");

    pIMX678Ctx->AecIntegrationTimeIncrement = pIMX678Ctx->one_line_exp_time;
    pIMX678Ctx->AecMinIntegrationTime =
        pIMX678Ctx->one_line_exp_time * pIMX678Ctx->MinIntegrationLine;
    pIMX678Ctx->AecMaxIntegrationTime =
        pIMX678Ctx->one_line_exp_time * pIMX678Ctx->MaxIntegrationLine;

    TRACE(IMX678_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __func__,
          pIMX678Ctx->isAfpsRun ? "(AFPS)" : "",
          pIMX678Ctx->AecMaxIntegrationTime);

    pIMX678Ctx->AecGainIncrement = IMX678_MIN_GAIN_STEP;

    // reflects the state of the sensor registers, must equal default settings
    pIMX678Ctx->AecCurGainLEF = pIMX678Ctx->AecMinGain;
	pIMX678Ctx->AecCurGainSEF1 = pIMX678Ctx->AecMinGain;
	pIMX678Ctx->AecCurGainSEF2 = pIMX678Ctx->AecMinGain;
	pIMX678Ctx->AecCurIntegrationTimeLEF = pIMX678Ctx->AecMaxIntegrationTime;
	pIMX678Ctx->AecCurIntegrationTimeSEF1 = pIMX678Ctx->AecMaxIntegrationTime;
	pIMX678Ctx->AecCurIntegrationTimeSEF2 = pIMX678Ctx->AecMaxIntegrationTime;
    pIMX678Ctx->OldGain = 1;
    pIMX678Ctx->OldIntegrationTime = 0.0f;

    TRACE(IMX678_INFO, "%s%s: (exit)\n", __func__,
          pIMX678Ctx->isAfpsRun ? "(AFPS)" : "");

    return (result);
}

static RESULT IMX678_IsiSetupIss(IsiSensorHandle_t handle,
                                 const IsiCaps_t* pCaps) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX678_INFO, "%s (enter)\n", __func__);

    if (!pIMX678Ctx) {
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pCaps == NULL) {
        TRACE(IMX678_ERROR,
              "%s: Invalid configuration (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMX678Ctx->Streaming != BOOL_FALSE) {
        return RET_WRONG_STATE;
    }

    if (pIMX678Ctx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        pIMX678Ctx->enableHdr = true;
    } else {
        pIMX678Ctx->enableHdr = false;
    }

    pIMX678Ctx->one_line_exp_time =
        (float)(pIMX678Ctx->SensorMode.ae_info.one_line_exp_time_ns) /
        1000000000;
    pIMX678Ctx->MaxIntegrationLine =
        pIMX678Ctx->SensorMode.ae_info.max_integration_time;
    TRACE(IMX678_INFO, "%s: MaxIntegrationLine %u\n", __func__,
          pIMX678Ctx->MaxIntegrationLine);
    pIMX678Ctx->MinIntegrationLine =
        pIMX678Ctx->SensorMode.ae_info.min_integration_time;
    pIMX678Ctx->gain_accuracy = pIMX678Ctx->SensorMode.ae_info.gain_accuracy;
    pIMX678Ctx->AecMaxGain = (float)(pIMX678Ctx->SensorMode.ae_info.max_gain) /
                             pIMX678Ctx->gain_accuracy;
    pIMX678Ctx->AecMinGain = (float)(pIMX678Ctx->SensorMode.ae_info.min_gain) /
                             pIMX678Ctx->gain_accuracy;

    pIMX678Ctx->AecMinIris = IMX678_IRIS_MIN_VAL;
    pIMX678Ctx->AecMaxIris = IMX678_IRIS_MAX_VAL;

    pIMX678Ctx->original_vmax = 0;
    pIMX678Ctx->unlimit_fps = 0;
    pIMX678Ctx->unlimit_fps_vmax_changed = 0;
    pIMX678Ctx->flicker_fps_mode = ISI_AE_ANTIBANDING_MODE_OFF;

    pIMX678Ctx->MaxFps = pIMX678Ctx->SensorMode.fps;
    pIMX678Ctx->CurrFps = pIMX678Ctx->MaxFps;
    TRACE(IMX678_INFO, "%s - got caps - width %d height %d buswidth %d\n",
          __func__, pIMX678Ctx->SensorMode.size.width,
          pIMX678Ctx->SensorMode.size.height, pIMX678Ctx->SensorMode.bit_width);

    TRACE(IMX678_INFO, "%s - MinGain %f MaxGain %f\n", __func__, pIMX678Ctx->AecMinGain,
          pIMX678Ctx->AecMaxGain);

    memcpy(&pIMX678Ctx->CapsConfig, pCaps, sizeof(pIMX678Ctx->CapsConfig));

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful,
     * bits 6..0 are reserved, reset bit is not sticky */
    TRACE(IMX678_DEBUG, "%s: IMX678 System-Reset executed\n", __func__);
    osSleep(100);

    result = IMX678_AecSetModeParameters(pIMX678Ctx, pCaps);
    if (result != RET_SUCCESS) {
        TRACE(IMX678_ERROR, "%s: SetupOutputWindow failed.\n", __func__);
        return (result);
    }

    pIMX678Ctx->Configured = BOOL_TRUE;
    TRACE(IMX678_INFO, "%s: (exit)\n", __func__);
    return 0;
}

static RESULT IMX678_IsiCheckConnectionIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    return (result);
}

static RESULT IMX678_IsiGetRevisionIss(IsiSensorHandle_t handle,
                                       uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    int ret = 0;
    uint32_t reg_val;
    uint32_t sensor_id;

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL || pIMX678Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX678Ctx->IsiCtx.HalHandle;

    if (!pValue) return (RET_NULL_POINTER);

    if (pIMX678Ctx->KernelDriverFlag) {
        ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, &sensor_id);
        if (ret != 0) {
            TRACE(IMX678_ERROR, "%s: Read Sensor ID Error! \n", __func__);
            return (RET_FAILURE);
        }
    } else {
        reg_val = 0;
        result = IMX678_IsiReadRegIss(handle, 0x3a04, &reg_val);
        sensor_id = (reg_val & 0xff) << 8;

        reg_val = 0;
        result |= IMX678_IsiReadRegIss(handle, 0x3a05, &reg_val);
        sensor_id |= (reg_val & 0xff);
    }

    *pValue = sensor_id;
    TRACE(IMX678_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX678_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX678_INFO, "%s (enter)\n", __func__);

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    pIMX678Ctx->Streaming = on;

    if (pIMX678Ctx->enableHdr)
        return result;

    TRACE(IMX678_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX678_IsiGetGainLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinGain, float* pMaxGain) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX678_INFO, "%s: (enter)\n", __func__);

    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinGain == NULL) || (pMaxGain == NULL)) {
        TRACE(IMX678_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinGain = pIMX678Ctx->AecMinGain;
    *pMaxGain = pIMX678Ctx->AecMaxGain;

    TRACE(IMX678_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static inline int IMX678_getFlickerPeaksPerSec(IsiSensorAntibandingMode_t mode) {
    int num_modes = sizeof(flickerPeaksPerSecMap) / sizeof(FlickerModePeaksPerSec);
    for (int i = 0; i < num_modes; i++) {
        if (flickerPeaksPerSecMap[i].mode == mode) {
            return flickerPeaksPerSecMap[i].value;
        }
    }
    return 0; // Default to 0 if mode not found
}

static inline uint32_t IMX678_getNewVmaxAntiFlicker(IMX678_Context_t *pIMX678Ctx, uint32_t requestedVmax) {
    uint32_t closestVmax = requestedVmax;
    int peaks = 0;
    int difference = INT_MAX;
    int minDifference = INT_MAX;
    if (!pIMX678Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (-1);
    }
    peaks = IMX678_getFlickerPeaksPerSec(pIMX678Ctx->flicker_fps_mode);
    if (peaks == 0) {
        // No anti-flicker mode, return requested or original Vmax
        if (pIMX678Ctx->unlimit_fps && pIMX678Ctx->unlimit_fps_vmax_changed)
            return requestedVmax;
        else
            return pIMX678Ctx->original_vmax; 
    }

    for (int i = 1; i < peaks; ++i) {
        uint32_t candidateVmax = i / (peaks * pIMX678Ctx->one_line_exp_time);
        if (candidateVmax < pIMX678Ctx->SensorMode.size.height || candidateVmax < pIMX678Ctx->original_vmax)
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

static RESULT IMX678_IsiUnlimitFpsIss(IsiSensorHandle_t handle,
                                      float maxIntegrationTime) {
    RESULT result = RET_SUCCESS;
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;

    if (!pIMX678Ctx) {
        return RET_NULL_POINTER;
    }

    if (pIMX678Ctx->enableHdr) {
        return result;
    }
    pIMX678Ctx->unlimit_fps = 1;

    if (maxIntegrationTime < 0)
        pIMX678Ctx->MaxIntegrationLine = IMX678_VMAX_MAX - IMX678_MIN_SHR;
    else
        pIMX678Ctx->MaxIntegrationLine =
            MIN((uint32_t)(maxIntegrationTime / pIMX678Ctx->one_line_exp_time),
                IMX678_VMAX_MAX - IMX678_MIN_SHR);
    TRACE(IMX678_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
        pIMX678Ctx->MaxIntegrationLine);
    pIMX678Ctx->AecMaxIntegrationTime =
        pIMX678Ctx->one_line_exp_time * pIMX678Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX678_IsiLimitFpsIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    uint32_t current_vmax = 0;
    uint32_t new_vmax = 0;
    TRACE(IMX678_INFO, "%s (enter)\n", __func__);
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (!pIMX678Ctx) {
        return RET_NULL_POINTER;
    }

    if (pIMX678Ctx->enableHdr)
        return result;

    IMX678_ReadVmax(handle, &current_vmax);
    if (current_vmax == 0) {
        TRACE(IMX678_INFO, "%s - exit because current_vmax is 0\n", __func__);
        return result;
    }

    pIMX678Ctx->unlimit_fps = 0;
    pIMX678Ctx->unlimit_fps_vmax_changed = 0;
    if (pIMX678Ctx->original_vmax == 0) {
        pIMX678Ctx->original_vmax = current_vmax;
    }

    new_vmax = pIMX678Ctx->original_vmax;
    if (pIMX678Ctx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
        new_vmax = IMX678_getNewVmaxAntiFlicker(pIMX678Ctx, pIMX678Ctx->original_vmax);
        TRACE(IMX678_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX678Ctx->flicker_fps_mode, new_vmax);
    }
    if (current_vmax != new_vmax) {
        result |= IMX678_LockRegHold(handle);
        result |= IMX678_WriteVmax(handle, new_vmax);
        result |= IMX678_UnlockRegHold(handle);

        int shr = MAX((int)current_vmax - (int)(pIMX678Ctx->AecCurIntegrationTimeLEF / pIMX678Ctx->one_line_exp_time), IMX678_MIN_SHR);
        float configuredIntegrationTime = (new_vmax - shr) * pIMX678Ctx->one_line_exp_time;
        pIMX678Ctx->OldIntegrationTime = configuredIntegrationTime;
        pIMX678Ctx->AecCurIntegrationTimeLEF = configuredIntegrationTime;
    }

    pIMX678Ctx->MaxIntegrationLine =
        MAX(new_vmax - IMX678_MIN_SHR, 1);
    TRACE(IMX678_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
        pIMX678Ctx->MaxIntegrationLine);
    pIMX678Ctx->AecMaxIntegrationTime =
        pIMX678Ctx->one_line_exp_time * pIMX678Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX678_IsiGetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinIris, float* pMaxIris) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX678_INFO, "%s: (enter)\n", __func__);

    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIris == NULL) || (pMaxIris == NULL)) {
        TRACE(IMX678_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIris = pIMX678Ctx->AecMinIris;
    *pMaxIris = pIMX678Ctx->AecMaxIris;

    TRACE(IMX678_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX678_IsiSetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float minIris, float maxIris) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX678_INFO, "%s: (enter)\n", __func__);

    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    pIMX678Ctx->AecMinIris = minIris;
    pIMX678Ctx->AecMaxIris = maxIris;

    TRACE(IMX678_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX678_IsiGetIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float* pMinIntegrationTime,
    float* pMaxIntegrationTime) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX678_INFO, "%s: (enter)\n", __func__);
    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX678_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIntegrationTime = pIMX678Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMX678Ctx->AecMaxIntegrationTime;

    TRACE(IMX678_INFO, "%s: (exit) %f, %f\n", 
    __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

/* Gain get functions*/

RESULT IMX678_IsiGetGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	TRACE(IMX678_INFO, "%s: (enter)\n", __func__);

	if (pIMX678Ctx == NULL) {
		TRACE(IMX678_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	if (pIMX678Ctx->enableHdr)
		return IMX678_IsiGetSEF1GainIss(handle, pSetGain);

	return IMX678_IsiGetLEFGainIss(handle, pSetGain);
}

RESULT IMX678_IsiGetLEFGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	TRACE(IMX678_INFO, "%s: (enter)\n", __func__);

	if (pIMX678Ctx == NULL) {
		TRACE(IMX678_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}
	*pSetGain = pIMX678Ctx->AecCurGainLEF;
	TRACE(IMX678_DEBUG, "%s - returning %f\n", __func__, pIMX678Ctx->AecCurGainLEF);
	TRACE(IMX678_INFO, "%s: (exit)\n", __func__);
	return (result);
}

RESULT IMX678_IsiGetSEF1GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX678_INFO, "%s: (enter)\n", __func__);

	if (pIMX678Ctx == NULL) {
		TRACE(IMX678_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX678Ctx->AecCurGainSEF1;
	TRACE(IMX678_DEBUG, "%s - returning %f\n", __func__, pIMX678Ctx->AecCurGainSEF1);

	TRACE(IMX678_INFO, "%s: (exit)\n", __func__);

	return (result);
}

RESULT IMX678_IsiGetSEF2GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX678_INFO, "%s: (enter)\n", __func__);

	if (pIMX678Ctx == NULL) {
		TRACE(IMX678_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX678Ctx->AecCurGainSEF2;
	TRACE(IMX678_DEBUG, "%s - returning %f\n", __func__, pIMX678Ctx->AecCurGainSEF2);

	TRACE(IMX678_INFO, "%s: (exit)\n", __func__);

	return (result);
}

RESULT IMX678_IsiGetGainIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX678_INFO, "%s: (enter)\n", __func__);

    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = pIMX678Ctx->AecGainIncrement;

    TRACE(IMX678_INFO, "%s: (exit)\n", __func__);

    return (result);
}

RESULT IMX678_IsiGetIrisIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX678_INFO, "%s: (enter)\n", __func__);
    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = 1;

    TRACE(IMX678_INFO, "%s: (exit)\n", __func__);

    return (result);
}

/* Gain set functions*/

RESULT IMX678_IsiSetGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	if (pIMX678Ctx == NULL || pIMX678Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	if (pIMX678Ctx->enableHdr) {
		result = IMX678_IsiSetSEF1GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);

		result |= IMX678_IsiSetSEF2GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);
	}

	result |= IMX678_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
	return result;
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

RESULT IMX678_IsiSetLEFGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	if (pIMX678Ctx == NULL || pIMX678Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	TRACE(IMX678_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX678_DEBUG, "%s: writting 0x%x to GAIN\n", __func__, Gain);

    result |= IMX678_LockRegHold(handle);
	result |= IMX678_WriteGain(handle, Gain);
	result |= IMX678_UnlockRegHold(handle);

	if (result != 0) {
		return RET_FAILURE;
	}

	pIMX678Ctx->AecCurGainLEF = _sensorGain2linear(Gain);
	*pSetGain = pIMX678Ctx->AecCurGainLEF;
	TRACE(IMX678_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

RESULT IMX678_IsiSetSEF1GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX678Ctx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);

	TRACE(IMX678_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX678_DEBUG, "%s: writting 0x%x to GAIN1\n", __func__, Gain);

    result |= IMX678_LockRegHold(handle);
	result |= IMX678_WriteGain1(handle, Gain);
    result |= IMX678_UnlockRegHold(handle);

	pIMX678Ctx->AecCurGainSEF1 = _sensorGain2linear(Gain);
	*pSetGain = pIMX678Ctx->AecCurGainSEF1;

	TRACE(IMX678_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

RESULT IMX678_IsiSetSEF2GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX678Ctx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);
	
	TRACE(IMX678_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX678_DEBUG, "%s: writting 0x%x to GAIN2\n", __func__, Gain);

    result |= IMX678_LockRegHold(handle);
	result |= IMX678_WriteGain2(handle, Gain);
    result |= IMX678_UnlockRegHold(handle);

	pIMX678Ctx->AecCurGainSEF2 = _sensorGain2linear(Gain);
	*pSetGain = pIMX678Ctx->AecCurGainSEF2;

	TRACE(IMX678_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

/* Integration Time get functions*/

RESULT IMX678_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;

	if (!pIMX678Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	
	TRACE(IMX678_DEBUG, "%s - enter\n", __func__);
	
	if (pIMX678Ctx->enableHdr)
		return IMX678_IsiGetSEF1IntegrationTimeIss(handle, pSetIntegrationTime);
	
	return IMX678_IsiGetLEFIntegrationTimeIss(handle, pSetIntegrationTime);
}

RESULT IMX678_IsiGetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	RESULT result = RET_SUCCESS;


	if (!pIMX678Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX678Ctx->AecCurIntegrationTimeLEF;
	TRACE(IMX678_DEBUG, "%s - returning %f\n", __func__, pIMX678Ctx->AecCurIntegrationTimeLEF);
	return (result);
}

RESULT IMX678_IsiGetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX678Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX678Ctx->AecCurIntegrationTimeSEF1;
	TRACE(IMX678_DEBUG, "%s - returning %f\n", __func__, pIMX678Ctx->AecCurIntegrationTimeSEF1);
	return (result);
}

RESULT IMX678_IsiGetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX678Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);

	*pSetIntegrationTime = pIMX678Ctx->AecCurIntegrationTimeSEF2;
	TRACE(IMX678_DEBUG, "%s - returning %f\n", __func__, pIMX678Ctx->AecCurIntegrationTimeSEF2);
	return (result);
}

RESULT IMX678_IsiGetIntegrationTimeIncrementIss(IsiSensorHandle_t handle,
                                                float* pIncr) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (!pIMX678Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pIncr) return (RET_NULL_POINTER);

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders
    //in
    // the application)
    *pIncr = pIMX678Ctx->AecIntegrationTimeIncrement;
    return (result);
}

/* Integration Time set functions*/

RESULT IMX678_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;

	if (!pIMX678Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX678Ctx->enableHdr) {
		return IMX678_IsiSetSEF1IntegrationTimeIss(
			handle, NewIntegrationTime, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
	}

	return IMX678_IsiSetLEFIntegrationTimeIss(
		handle, NewIntegrationTime, pSetIntegrationTime,
		pNumberOfFramesToSkip, hdr_ratio);
}

RESULT IMX678_IsiSetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;

    int shr = 0;
    uint32_t exp = 0;
    uint32_t new_vmax = 0;
    uint32_t vmax_updated = 0;
    uint32_t current_vmax = 0;
    uint32_t rhs1;
    uint32_t rhs2;
    if (!pIMX678Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pSetIntegrationTime || !pNumberOfFramesToSkip) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMX678Ctx->Streaming) {
        TRACE(IMX678_ERROR, "%s: sensor not streaming\n", __func__);
        return RET_FAILURE;
    }
    exp = NewIntegrationTime / pIMX678Ctx->one_line_exp_time;

    TRACE(IMX678_DEBUG, "%s: set AEC_PK_EXPO=0x%05x\n", __func__, exp);

    if (fabs(NewIntegrationTime - pIMX678Ctx->AecCurIntegrationTimeLEF) > FLT_EPSILON) {

        if (pIMX678Ctx->enableHdr){
            if (pIMX678Ctx->cur_rhs1 == 0 || pIMX678Ctx->cur_rhs2 == 0) {
                TRACE(IMX678_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
                return (RET_WRONG_CONFIG);
            }

            rhs1 = pIMX678Ctx->cur_rhs1;
            rhs2 = pIMX678Ctx->cur_rhs2;
            if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
                new_vmax = IMX678_VMAX_2DOL_HDR;
                exp = new_vmax - exp;
                exp = exp > rhs1 + IMX678_2DOL_SHR0_RHS1_GAP ? exp : rhs1 + IMX678_2DOL_SHR0_RHS1_GAP;
                exp = exp < new_vmax - IMX678_2DOL_SHR0_FSC_GAP? exp : new_vmax - IMX678_2DOL_SHR0_FSC_GAP;
                shr = exp;
            } else {
                new_vmax = IMX678_VMAX_3DOL_HDR;
                exp = new_vmax - exp;
                exp = exp > rhs2 + IMX678_SHR0_RHS2_GAP ? exp : rhs2 + IMX678_SHR0_RHS2_GAP;
                exp = exp < new_vmax - IMX678_SHR0_FSC_GAP? exp : new_vmax - IMX678_SHR0_FSC_GAP;
                shr = exp;
            }
            
		} else {
            if (exp > pIMX678Ctx->MaxIntegrationLine || exp == 0) {
                TRACE(IMX678_ERROR, "%s: Integration time %f (exp %u) out of range (%u)\n", __func__,
                    NewIntegrationTime, exp, pIMX678Ctx->MaxIntegrationLine);
                return RET_FAILURE;
            }
            result = IMX678_ReadVmax(handle, &current_vmax);

            if (pIMX678Ctx->original_vmax == 0) {
                pIMX678Ctx->original_vmax = current_vmax;
            }

            if (pIMX678Ctx->original_vmax - IMX678_MIN_SHR >
                pIMX678Ctx->MaxIntegrationLine) {
                pIMX678Ctx->MaxIntegrationLine =
                    pIMX678Ctx->original_vmax - IMX678_MIN_SHR;
                TRACE(IMX678_DEBUG, "%s: set MaxIntegrationLine to %u\n", __func__,
                      pIMX678Ctx->MaxIntegrationLine);
                pIMX678Ctx->AecMaxIntegrationTime =
                    pIMX678Ctx->one_line_exp_time * pIMX678Ctx->MaxIntegrationLine;
                TRACE(IMX678_DEBUG, "%s: set AecMaxIntegrationTime to %f\n", __func__,
                      pIMX678Ctx->AecMaxIntegrationTime);
            }

            shr = current_vmax - exp;

            if (shr < IMX678_MIN_SHR || current_vmax > pIMX678Ctx->original_vmax) {
                if (shr < IMX678_MIN_SHR) {
                    // user asked for too big integration-line: more than (current_vmax - minimal_shutter)
                    // increase vmax and set new shutter to minimal value
                    new_vmax = MIN(exp + IMX678_MIN_SHR, pIMX678Ctx->MaxIntegrationLine + IMX678_MIN_SHR);
                } else {
                    // make sure that when we unlimit fps, stay below original fps.
                    // also, make sure we are not using high vmax + high shr more than needed (will slow fps for no reason)
                    new_vmax = MAX(current_vmax - shr + IMX678_MIN_SHR, pIMX678Ctx->original_vmax);
                }
                pIMX678Ctx->unlimit_fps_vmax_changed = new_vmax > pIMX678Ctx->original_vmax && pIMX678Ctx->unlimit_fps;
        
                if (pIMX678Ctx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
                    new_vmax = IMX678_getNewVmaxAntiFlicker(pIMX678Ctx, new_vmax);
                    TRACE(IMX678_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX678Ctx->flicker_fps_mode, new_vmax);
                }
                
                shr = MAX(IMX678_MIN_SHR, (int)new_vmax - (int)exp);
                vmax_updated = 1;

            } else {
                new_vmax = current_vmax;
            }
        }

        result |= IMX678_LockRegHold(handle);
        if (vmax_updated && pIMX678Ctx->unlimit_fps && !pIMX678Ctx->enableHdr) {
            result |= IMX678_WriteVmax(handle, new_vmax);
        }

        TRACE(IMX678_DEBUG, "%s - writing 0x%x to SHR0\n", __func__, shr);
        result |= IMX678_WriteShr0(handle, shr);
        result |= IMX678_UnlockRegHold(handle);

        float configuredIntegrationTime =
            (new_vmax - shr) * pIMX678Ctx->one_line_exp_time;
        pIMX678Ctx->OldIntegrationTime = configuredIntegrationTime;
        pIMX678Ctx->AecCurIntegrationTimeLEF = configuredIntegrationTime;

        *pNumberOfFramesToSkip = 1U;
    } else {
        *pNumberOfFramesToSkip = 0U;  // no frame skip
    }

    *pSetIntegrationTime = pIMX678Ctx->AecCurIntegrationTimeLEF;
    TRACE(IMX678_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTime);
    return (result);
}

RESULT IMX678_IsiSetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF1,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;

	if (!pIMX678Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX678Ctx->cur_rhs1 == 0 || pIMX678Ctx->cur_rhs2 == 0) {
		TRACE(IMX678_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX678Ctx->cur_rhs1;

	if (!pSetIntegrationTimeSEF1 || !pNumberOfFramesToSkip) {
		printf("%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX678_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = (NewIntegrationTime / pIMX678Ctx->one_line_exp_time);
	TRACE(IMX678_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX678Ctx->AecCurIntegrationTimeSEF1) > FLT_EPSILON) {
		exp = rhs1 - exp;
		exp = exp > IMX678_3DOL_SHR1_RHS1_GAP ? exp : IMX678_3DOL_SHR1_RHS1_GAP;
		exp = exp < rhs1 - IMX678_SHR1_RHS1_GAP ? exp : rhs1 - IMX678_SHR1_RHS1_GAP;
		TRACE(IMX678_DEBUG, "%s - writing 0x%x to SHR1\n", __func__, exp);

		result |= IMX678_LockRegHold(handle);
		result |= IMX678_WriteShr1(handle, exp);
		result |= IMX678_UnlockRegHold(handle);

		pIMX678Ctx->AecCurIntegrationTimeSEF1 = (rhs1 - exp) * pIMX678Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF1 = pIMX678Ctx->AecCurIntegrationTimeSEF1;

	TRACE(IMX678_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF1);
	return (result);
}

RESULT IMX678_IsiSetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF2,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;
	uint32_t rhs2;

	if (pIMX678Ctx->cur_rhs1 == 0 || pIMX678Ctx->cur_rhs2 == 0) {
		printf("%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX678Ctx->cur_rhs1;
	rhs2 = pIMX678Ctx->cur_rhs2;

	if (!pIMX678Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTimeSEF2 || !pNumberOfFramesToSkip) {
		printf("%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX678_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = (NewIntegrationTime / pIMX678Ctx->one_line_exp_time);
	TRACE(IMX678_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX678Ctx->AecCurIntegrationTimeSEF2) > FLT_EPSILON) {
		exp = rhs2 - exp;
		exp = exp > rhs1 + IMX678_SHR2_RHS1_GAP ? exp : rhs1 + IMX678_SHR2_RHS1_GAP;
		exp = exp < rhs2 - IMX678_SHR2_RHS2_GAP ? exp : rhs2 - IMX678_SHR2_RHS2_GAP;
		TRACE(IMX678_DEBUG, "%s - writing 0x%x to SHR2\n", __func__, exp);

		result |= IMX678_LockRegHold(handle);
		result |= IMX678_WriteShr2(handle, exp);
		result |= IMX678_UnlockRegHold(handle);

		pIMX678Ctx->AecCurIntegrationTimeSEF2 = (rhs2 - exp) * pIMX678Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF2 = pIMX678Ctx->AecCurIntegrationTimeSEF2;

	TRACE(IMX678_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF2);
	return (result);
}

RESULT IMX678_Calculate3DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it, float *o_very_short_it,
                                    float *o_long_gain, float *o_short_gain, float *o_very_short_gain,
                                    float *hdr_ratio) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    float long_it = 0.0;
	float short_it = 0.0;
	float very_short_it = 0.0;
	float long_exp_val = 0.0;
	float short_exp_val = 0.0;
	float very_short_exp_val = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	float very_short_gain = 1;
	bool calculate_gain = false;
	uint32_t rhs1;
	uint32_t rhs2;

    if (pIMX678Ctx == NULL || o_long_it == NULL || o_short_it == NULL ||
        o_very_short_it == NULL || o_long_gain == NULL || o_short_gain == NULL ||
        o_very_short_gain == NULL || hdr_ratio == NULL) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

	if (pIMX678Ctx->cur_rhs1 == 0 || pIMX678Ctx->cur_rhs2 == 0) {
		TRACE(IMX678_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX678Ctx->cur_rhs1;
	rhs2 = pIMX678Ctx->cur_rhs2;

    TRACE(IMX678_DEBUG, "%s: hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n", 
    __func__, hdr_ratio[0], hdr_ratio[1]);
    
    // Sometimes there is no actual input gain. In that case, we will read it from the sensor
    if (NewGain == 0) {
        TRACE(IMX678_DEBUG, "%s: Input NewGain is 0, reading gain from sensor\n", __func__);
        result = IMX678_IsiGetSEF1GainIss(handle, &NewGain);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // Same for integration time
    if (NewIntegrationTime == 0) {
        TRACE(IMX678_DEBUG, "%s: Input NewIntegrationTime is 0, reading integration time from sensor\n", __func__);
        result = IMX678_IsiGetSEF1IntegrationTimeIss(handle, &NewIntegrationTime);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // assume gain is 1 and see if ratio can be achieved with integration time
    long_it 		= NewIntegrationTime * hdr_ratio[0];
    short_it 		= NewIntegrationTime;
    very_short_it 	= NewIntegrationTime / hdr_ratio[1];
    
    TRACE(IMX678_DEBUG, "%s: requested IT long: %f, short: %f, very_short: %f\n", 
    __func__, long_it, short_it, very_short_it);
    long_exp_val 		= long_it / pIMX678Ctx->one_line_exp_time;
    short_exp_val 		= short_it / pIMX678Ctx->one_line_exp_time;
    very_short_exp_val 	= very_short_it / pIMX678Ctx->one_line_exp_time;

    TRACE(IMX678_DEBUG, "%s: requested IT in lines long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    long_exp_val 		= IMX678_VMAX_3DOL_HDR - long_exp_val;
    short_exp_val 		= rhs1 - short_exp_val;
    very_short_exp_val 	= rhs2 - very_short_exp_val;

    TRACE(IMX678_DEBUG, "%s: requested IT in shr long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    if(long_exp_val < rhs2 + IMX678_SHR0_RHS2_GAP) {
        long_exp_val = rhs2 + IMX678_SHR0_RHS2_GAP;
        long_it = (IMX678_VMAX_3DOL_HDR - long_exp_val) * pIMX678Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX678_DEBUG, "%s: long_exp_val is too long, set to %u, new long_it = %f\n",
        __func__, rhs2 + IMX678_SHR0_RHS2_GAP, long_it);
    } else if(long_exp_val > IMX678_VMAX_3DOL_HDR - IMX678_SHR0_FSC_GAP) {
        long_exp_val = IMX678_VMAX_3DOL_HDR - IMX678_SHR0_FSC_GAP;
        long_it = (IMX678_VMAX_3DOL_HDR - long_exp_val) * pIMX678Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX678_DEBUG, "%s: long_exp_val is too short, set to %u, new long_it = %f\n",
        __func__, IMX678_VMAX_3DOL_HDR - IMX678_SHR0_FSC_GAP, long_it);
    }
    if(short_exp_val < IMX678_3DOL_SHR1_RHS1_GAP) {
        short_exp_val = IMX678_3DOL_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX678Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX678_DEBUG, "%s: short_exp_val is too long, set to %u, new short_it = %f\n",
        __func__, IMX678_3DOL_SHR1_RHS1_GAP, short_it);
    } else if(short_exp_val > rhs1 - IMX678_SHR1_RHS1_GAP) {
        short_exp_val = rhs1 - IMX678_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX678Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX678_DEBUG, "%s: short_exp_val is too short, set to %u, new short_it = %f\n",
        __func__, rhs1 - IMX678_SHR1_RHS1_GAP, short_it);
    }
    if(very_short_exp_val < rhs1 + IMX678_SHR2_RHS1_GAP) {
        very_short_exp_val = rhs1 + IMX678_SHR2_RHS1_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX678Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX678_DEBUG, "%s: very_short_exp_val is too long, set to %u, new very_short_it = %f\n",
        __func__, rhs2 + IMX678_SHR2_RHS1_GAP, very_short_it);
    } else if(very_short_exp_val > rhs2 - IMX678_SHR2_RHS2_GAP) {
        very_short_exp_val = rhs2 - IMX678_SHR2_RHS2_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX678Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX678_DEBUG, "%s: very_short_exp_val is too short, set to %u, new very_short_it = %f\n",
        __func__, rhs2 - IMX678_SHR2_RHS2_GAP, very_short_it);
    }

    // need to use gain to achive ratio / requested gain update
    if(calculate_gain || NewGain != pIMX678Ctx->AecCurGainSEF1) {
        long_gain = (short_it * NewGain * hdr_ratio[0]) / long_it;
        short_gain = NewGain;
        very_short_gain = (short_it * NewGain) / (very_short_it * hdr_ratio[1]);
        TRACE(IMX678_DEBUG, "%s: calculated gain: long: %f, short: %f, very_short: %f\n",
        __func__, long_gain, short_gain, very_short_gain);
    }

    *o_long_it = long_it;
    *o_short_it = short_it;
    *o_very_short_it = very_short_it;
    *o_long_gain = long_gain;
    *o_short_gain = short_gain;
    *o_very_short_gain = very_short_gain;

    return RET_SUCCESS;
}


RESULT IMX678_Calculate2DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it,
                                    float *o_long_gain, float *o_short_gain,
                                    float *hdr_ratio) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    float long_it = 0.0;
	float short_it = 0.0;
	float long_exp_val = 0.0;
	float short_exp_val = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	bool calculate_gain = false;
	uint32_t rhs1;

    if (pIMX678Ctx == NULL || o_long_it == NULL ||
        o_long_gain == NULL || o_short_gain == NULL ||
        hdr_ratio == NULL) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

	if (pIMX678Ctx->cur_rhs1 == 0) {
		TRACE(IMX678_ERROR, "%s: Invalid parameter (RHS1 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX678Ctx->cur_rhs1;

    TRACE(IMX678_DEBUG, "%s: hdr_ratio[0] = LS Ratio = %f\n", 
    __func__, hdr_ratio[0]);
    
    // Sometimes there is no actual input gain. In that case, we will read it from the sensor
    if (NewGain == 0) {
        TRACE(IMX678_DEBUG, "%s: Input NewGain is 0, reading gain from sensor\n", __func__);
        result = IMX678_IsiGetSEF1GainIss(handle, &NewGain);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // Same for integration time
    if (NewIntegrationTime == 0) {
        TRACE(IMX678_DEBUG, "%s: Input NewIntegrationTime is 0, reading integration time from sensor\n", __func__);
        result = IMX678_IsiGetSEF1IntegrationTimeIss(handle, &NewIntegrationTime);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // assume gain is 1 and see if ratio can be achieved with integration time
    long_it 		= NewIntegrationTime * hdr_ratio[0];
    short_it 		= NewIntegrationTime;
    
    TRACE(IMX678_DEBUG, "%s: requested IT long: %f, short: %f\n", 
    __func__, long_it, short_it);
    long_exp_val 		= long_it / pIMX678Ctx->one_line_exp_time;
    short_exp_val 		= short_it / pIMX678Ctx->one_line_exp_time;

    TRACE(IMX678_DEBUG, "%s: requested IT in lines long: %f, short: %f\n", 
    __func__, long_exp_val, short_exp_val);
    long_exp_val 		= IMX678_VMAX_2DOL_HDR - long_exp_val;
    short_exp_val 		= rhs1 - short_exp_val;

    TRACE(IMX678_DEBUG, "%s: requested IT in shr long: %f, short: %f\n",
    __func__, long_exp_val, short_exp_val);
    if(long_exp_val < rhs1 + IMX678_2DOL_SHR0_RHS1_GAP) {
        long_exp_val = rhs1 + IMX678_2DOL_SHR0_RHS1_GAP;
        long_it = (IMX678_VMAX_2DOL_HDR - long_exp_val) * pIMX678Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX678_DEBUG, "%s: long_exp_val is too long, set to %u, new long_it = %f\n",
        __func__, rhs1 + IMX678_2DOL_SHR0_RHS1_GAP, long_it);
    } else if(long_exp_val > IMX678_VMAX_2DOL_HDR - IMX678_2DOL_SHR0_FSC_GAP) {
        long_exp_val = IMX678_VMAX_2DOL_HDR - IMX678_2DOL_SHR0_FSC_GAP;
        long_it = (IMX678_VMAX_2DOL_HDR - long_exp_val) * pIMX678Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX678_DEBUG, "%s: long_exp_val is too short, set to %u, new long_it = %f\n",
        __func__, IMX678_VMAX_2DOL_HDR - IMX678_2DOL_SHR0_FSC_GAP, long_it);
    }
    if(short_exp_val < IMX678_2DOL_SHR1_MIN_GAP) {
        short_exp_val = IMX678_2DOL_SHR1_MIN_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX678Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX678_DEBUG, "%s: short_exp_val is too long, set to %u, new short_it = %f\n",
        __func__, IMX678_2DOL_SHR1_MIN_GAP, short_it);
    } else if(short_exp_val > rhs1 - IMX678_2DOL_SHR1_RHS1_GAP) {
        short_exp_val = rhs1 - IMX678_2DOL_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX678Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX678_DEBUG, "%s: short_exp_val is too short, set to %u, new short_it = %f\n",
        __func__, rhs1 - IMX678_2DOL_SHR1_RHS1_GAP, short_it);
    }

    // need to use gain to achieve ratio / requested gain update
    if(calculate_gain || NewGain != pIMX678Ctx->AecCurGainSEF1) {
        long_gain = (short_it * NewGain * hdr_ratio[0]) / long_it;
        short_gain = NewGain;
        TRACE(IMX678_DEBUG, "%s: calculated gain: long: %f, short: %f\n",
        __func__, long_gain, short_gain);
    }

    *o_long_it = long_it;
    *o_short_it = short_it;
    *o_long_gain = long_gain;
    *o_short_gain = short_gain;

    return RET_SUCCESS;
}

RESULT IMX678_IsiExposureControlIss(IsiSensorHandle_t handle, float NewGain,
                                    float NewIntegrationTime,
                                    uint8_t* pNumberOfFramesToSkip,
                                    float* pSetGain, float* pSetIntegrationTime,
                                    float* hdr_ratio) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;

    RESULT result = RET_SUCCESS;
    float long_it = 0.0;
	float short_it = 0.0;
	float very_short_it = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	float very_short_gain = 1;
    uint32_t hmax;

    TRACE(IMX678_INFO, "%s: enter with NewIntegrationTime: %f, NewGain: %f\n",
        __func__, NewIntegrationTime, NewGain);

    if (pIMX678Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pNumberOfFramesToSkip == NULL) || (pSetGain == NULL) ||
        (pSetIntegrationTime == NULL)) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMX678Ctx->enableHdr) {
        result = IMX678_ReadRHS1(handle, &pIMX678Ctx->cur_rhs1);
        result |= IMX678_ReadRHS2(handle, &pIMX678Ctx->cur_rhs2);
        result |= IMX678_ReadHmax(handle, &hmax);
        if (result != RET_SUCCESS) {
            TRACE(IMX678_ERROR, "%s: Read RHS1, RHS2 or HMAX failed\n", __func__);
            return result;
        }

        pIMX678Ctx->SensorMode.ae_info.one_line_exp_time_ns = (uint32_t)(((float)hmax / IMX678_PIXEL_CLK_RATE) * MICRO_2_NANO);
        pIMX678Ctx->one_line_exp_time =
        (float)(pIMX678Ctx->SensorMode.ae_info.one_line_exp_time_ns) / 1000000000;

        if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
            //2exposure
            result = IMX678_Calculate2DOLExposures(handle, NewIntegrationTime, NewGain,
                                    &long_it, &short_it,
                                    &long_gain, &short_gain,
                                    hdr_ratio);
        } else {
            //3exposure
            result = IMX678_Calculate3DOLExposures(handle, NewIntegrationTime, NewGain,
                                        &long_it, &short_it, &very_short_it,
                                        &long_gain, &short_gain, &very_short_gain,
                                        hdr_ratio);
        }
        
        if (result != RET_SUCCESS) {
            TRACE(IMX678_ERROR, "%s: CalculateHDRExposures failed\n", __func__);
            return result;
        }

		result = IMX678_IsiSetLEFIntegrationTimeIss(handle, long_it,
							pSetIntegrationTime,
							pNumberOfFramesToSkip,
							hdr_ratio);
		result |= IMX678_IsiSetLEFGainIss(handle, long_gain, pSetGain, hdr_ratio);
		result |= IMX678_IsiSetSEF1IntegrationTimeIss(
			handle, short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		result |= IMX678_IsiSetSEF1GainIss(handle, NewIntegrationTime,
						  short_gain, pSetGain, hdr_ratio);
        if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            result |= IMX678_IsiSetSEF2IntegrationTimeIss(
			    handle, very_short_it, pSetIntegrationTime,
			    pNumberOfFramesToSkip, hdr_ratio);
            result |= IMX678_IsiSetSEF2GainIss(handle, NewIntegrationTime,
                            very_short_gain, pSetGain, hdr_ratio);
        }

        // Recalculate `io_hdr_ratio` according to the set values
        hdr_ratio[0] = (long_it * long_gain) / (short_it * short_gain);
        if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            hdr_ratio[1] = (short_it * short_gain) / (very_short_it * very_short_gain);
        }

        // Set the output values to SEF1 values
        *pSetGain = short_gain;
        *pSetIntegrationTime = short_it;

        TRACE(IMX678_DEBUG, "%s: actual hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n",
            __func__, hdr_ratio[0], hdr_ratio[1]);
    } else {
        result |= IMX678_IsiSetLEFIntegrationTimeIss(handle, NewIntegrationTime,
                                                pSetIntegrationTime,
                                                pNumberOfFramesToSkip, hdr_ratio);
        result |= IMX678_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
    }
    return result;
}

RESULT IMX678_IsiExposureControlExpandedIss(
    IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime,
    float NewIris, uint8_t* pNumberOfFramesToSkip, float* pSetGain,
    float* pSetIntegrationTime, float* pSetIris, float* hdr_ratio) {

    if (pSetIris) {
        IMX678_IsiSetIrisIss(handle, NewIris);
        *pSetIris = NewIris;
    }

    return IMX678_IsiExposureControlIss(handle, NewGain, NewIntegrationTime,
                                        pNumberOfFramesToSkip, pSetGain,
                                        pSetIntegrationTime, hdr_ratio);
}

RESULT IMX678_IsiGetCurrentExposureIss(IsiSensorHandle_t handle,
                                       float* pSetGain,
                                       float* pSetIntegrationTime) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX678Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pSetGain == NULL) || (pSetIntegrationTime == NULL))
        return (RET_NULL_POINTER);

    if (pIMX678Ctx->enableHdr) {
		*pSetGain = pIMX678Ctx->AecCurGainSEF1;
		*pSetIntegrationTime = pIMX678Ctx->AecCurIntegrationTimeSEF1;
	} else {
		*pSetGain = pIMX678Ctx->AecCurGainLEF;
		*pSetIntegrationTime = pIMX678Ctx->AecCurIntegrationTimeLEF;
	}

    return (result);
}

RESULT IMX678_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t* pFps) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX678Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *pFps = pIMX678Ctx->CurrFps;

    return (result);
}

RESULT IMX678_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t Fps) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX678_IsiSetFlickerFpsIss(IsiSensorHandle_t handle, uint32_t flickerMode) {
    RESULT result = RET_SUCCESS;
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    uint32_t current_vmax = 0;
    uint32_t requested_vmax = 0;
    uint32_t shr = 0;
    int exp = 0;

    TRACE(IMX678_DEBUG, "%s: set sensor flickerMode = %d\n", __func__, flickerMode);

    if (!pIMX678Ctx) {
        return RET_NULL_POINTER;
    }
    if (pIMX678Ctx->flicker_fps_mode == flickerMode) {
        return RET_SUCCESS;
    }
    if (flickerMode > ISI_AE_ANTIBANDING_MODE_AUTO) {
        TRACE(IMX678_INFO, "%s: Invalid flickerMode (%d), setting ISI_AE_ANTIBANDING_MODE_AUTO instead.\n", __func__, flickerMode);
        flickerMode = ISI_AE_ANTIBANDING_MODE_AUTO;
    }
    pIMX678Ctx->flicker_fps_mode = flickerMode;

    result = IMX678_ReadVmax(handle, &current_vmax);
    if (result != RET_SUCCESS) {
        TRACE(IMX678_ERROR, "%s: Unable to read VMAX\n", __func__);
        return (result);
    }
    if (pIMX678Ctx->original_vmax == 0) {
        pIMX678Ctx->original_vmax = current_vmax;
    }

    exp = pIMX678Ctx->AecCurIntegrationTimeLEF / pIMX678Ctx->one_line_exp_time;
    shr = MAX((int)current_vmax - exp, IMX678_MIN_SHR);

    if (current_vmax > pIMX678Ctx->original_vmax) {
        current_vmax = MAX((int)current_vmax - (int)shr + IMX678_MIN_SHR, IMX678_MIN_SHR);
        shr = MAX((int)current_vmax - exp, IMX678_MIN_SHR);
        pIMX678Ctx->unlimit_fps_vmax_changed = current_vmax > pIMX678Ctx->original_vmax && pIMX678Ctx->unlimit_fps;
    }

    requested_vmax = IMX678_getNewVmaxAntiFlicker(pIMX678Ctx, current_vmax);
    requested_vmax = MAX( MIN(requested_vmax, IMX678_VMAX_MAX), 1);
    
    if (current_vmax != requested_vmax) {
        shr = MAX( (int)requested_vmax - (int)current_vmax + (int)shr , IMX678_MIN_SHR);
        result |= IMX678_LockRegHold(handle);
        result |= IMX678_WriteVmax(handle, requested_vmax);
        result |= IMX678_WriteShr0(handle, shr);
        result |= IMX678_UnlockRegHold(handle);
        if (result != RET_SUCCESS) {
            TRACE(IMX678_ERROR, "%s: Unable to write VMAX or Shr0\n", __func__);
            return (result);
        }
        TRACE(IMX678_DEBUG, "%s - writing 0x%x to VMAX, writing 0x%x to SHR0\n", __func__, requested_vmax, shr);

        float configuredIntegrationTime = (requested_vmax - shr) * pIMX678Ctx->one_line_exp_time;
        pIMX678Ctx->OldIntegrationTime = configuredIntegrationTime;
        pIMX678Ctx->AecCurIntegrationTimeLEF = configuredIntegrationTime;
        TRACE(IMX678_DEBUG, "%s: Ti=%f\n", __func__, configuredIntegrationTime);
    }
    
    pIMX678Ctx->MaxIntegrationLine = MAX( MIN(requested_vmax - IMX678_MIN_SHR, IMX678_VMAX_MAX - IMX678_MIN_SHR), 1);
    pIMX678Ctx->AecMaxIntegrationTime = pIMX678Ctx->one_line_exp_time * pIMX678Ctx->MaxIntegrationLine;

    TRACE(IMX678_INFO, "%s: set sensor fps = %d\n", __func__,
          pIMX678Ctx->CurrFps);

    TRACE(IMX678_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX678_IsiGetAutoFpsInfoIss(IsiSensorHandle_t handle,
                                   IsiAutoFps_t* pAutoFpsInfo) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX678_IsiGetStartEvIss(IsiSensorHandle_t handle, uint64_t* pStartEv) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX678_IsiGetIspStatusIss(IsiSensorHandle_t handle,
                                 IsiIspStatus_t* pIspStatus) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL || pIMX678Ctx->IsiCtx.HalHandle == NULL) {
        return RET_WRONG_HANDLE;
    }

    pIspStatus->useSensorAE = false;
    pIspStatus->useSensorBLC = false;
    pIspStatus->useSensorAWB = false;

    return RET_SUCCESS;
}

RESULT IMX678_IsiSetTpgIss(IsiSensorHandle_t handle, IsiTpg_t Tpg) {
    RESULT result = RET_SUCCESS;

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL || pIMX678Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX678Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (Tpg.enable == 0) {
        result = IMX678_IsiWriteRegIss(handle, 0x3253, 0x00);
    } else {
        result = IMX678_IsiWriteRegIss(handle, 0x3253, 0x80);
    }

    pIMX678Ctx->TestPattern = Tpg.enable;

    return (result);
}

RESULT IMX678_IsiGetTpgIss(IsiSensorHandle_t handle, IsiTpg_t* Tpg) {
    RESULT result = RET_SUCCESS;
    uint32_t value = 0;

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL || pIMX678Ctx->IsiCtx.HalHandle == NULL ||
        Tpg == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX678Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (!IMX678_IsiReadRegIss(handle, 0x5081, &value)) {
        Tpg->enable = ((value & 0x80) != 0) ? 1 : 0;
        if (Tpg->enable) {
            Tpg->pattern = (0xff & value);
        }
        pIMX678Ctx->TestPattern = Tpg->enable;
    }

    return (result);
}

RESULT IMX678_IsiFocusCreateIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX678_IsiFocusReleaseIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX678_IsiFocusGetCalibrateIss(IsiSensorHandle_t handle,
                                      IsiFocusCalibAttr_t* pFocusCalib) {
    return RET_SUCCESS;
}

RESULT IMX678_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

RESULT IMX678_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

// Taken from the IMX681 driver
RESULT IMX678_IsiSetAgainDgainIss(IsiSensorHandle_t handle,
                                  IsiUserGain_t Gain) {
    RESULT result = RET_SUCCESS;
    uint32_t Again = 0, Dgain = 0;
    TRACE(IMX678_INFO, "%s: (enter)\n", __func__);

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((Gain.again < 1) | (Gain.again > 16)) {
        TRACE(IMX678_ERROR, "%s: Invalid sensor again\n", __func__);
        return (RET_OUTOFRANGE);
    }
    // Again = (uint32_t)(1024 - (1024/Gain.again));
    Again = (uint32_t)(((2048 * Gain.again) - 2048) / Gain.again);

    result = IMX678_IsiWriteRegIss(handle, 0x309c, (Again & 0x0000FF));
    result = IMX678_IsiWriteRegIss(handle, 0x309d, (Again & 0x00FF00) >> 8);

    if ((Gain.dgain < 1) | (Gain.dgain > 16)) {
        TRACE(IMX678_ERROR, "%s: Invalid sensor dgain\n", __func__);
        return (RET_OUTOFRANGE);
    }
    Dgain = Gain.dgain * 256;

    result = IMX678_IsiWriteRegIss(handle, 0x308c, (Dgain & 0x0000FF));
    result = IMX678_IsiWriteRegIss(handle, 0x308d, (Dgain & 0x00FF00) >> 8);

    // pIMX678Ctx->CurAgain = Gain.again;
    // pIMX678Ctx->CurDgain = Gain.dgain;

    TRACE(IMX678_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX678_IsiGetIrisIss( IsiSensorHandle_t handle,
                                     float *pSetIris ) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX678_INFO, "%s: (enter)\n", __func__);

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    //TODO implement getting iris here!
    *pSetIris = 1.0;

    TRACE(IMX678_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX678_IsiSetIrisIss( IsiSensorHandle_t handle, 
                    const float NewIris) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX678_INFO, "%s: (enter)\n", __func__);

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (NewIris < IMX678_IRIS_MIN_VAL || NewIris > IMX678_IRIS_MAX_VAL) {
        TRACE(IMX678_ERROR, "%s: Invalid Iris %f\n", __func__, NewIris);
    }

    //TODO implement setting iris here!

    TRACE(IMX678_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX678_IsiGetSensorIss(IsiSensor_t* pIsiSensor) {
    RESULT result = RET_SUCCESS;
    static const char SensorName[16] = "IMX678";
    TRACE(IMX678_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor != NULL) {
        pIsiSensor->pszName = SensorName;
        pIsiSensor->pIsiCreateIss = IMX678_IsiCreateIss;
        pIsiSensor->pIsiReleaseIss = IMX678_IsiReleaseIss;
        pIsiSensor->pIsiReadRegIss = IMX678_IsiReadRegIss;
        pIsiSensor->pIsiWriteRegIss = IMX678_IsiWriteRegIss;
        pIsiSensor->pIsiGetModeIss = IMX678_IsiGetModeIss;
        pIsiSensor->pIsiSetModeIss = IMX678_IsiSetModeIss;
        pIsiSensor->pIsiEnumModeIss = IMX678_IsiEnumModeIss;
        pIsiSensor->pIsiGetCapsIss = IMX678_IsiGetCapsIss;
        pIsiSensor->pIsiSetupIss = IMX678_IsiSetupIss;
        pIsiSensor->pIsiCheckConnectionIss = IMX678_IsiCheckConnectionIss;
        pIsiSensor->pIsiGetRevisionIss = IMX678_IsiGetRevisionIss;
        pIsiSensor->pIsiSetStreamingIss = IMX678_IsiSetStreamingIss;

        /* AEC functions */
        pIsiSensor->pIsiGetGainLimitsIss = 					IMX678_IsiGetGainLimitsIss;
		pIsiSensor->pIsiGetIrisLimitsIss = 					IMX678_IsiGetIrisLimitsIss;
		pIsiSensor->pIsiSetIrisLimitsIss = 					IMX678_IsiSetIrisLimitsIss;
		pIsiSensor->pIsiGetIntegrationTimeLimitsIss =		IMX678_IsiGetIntegrationTimeLimitsIss;

		pIsiSensor->pIsiExposureControlIss =				IMX678_IsiExposureControlIss;
		pIsiSensor->pIsiExposureControlExpandedIss =		IMX678_IsiExposureControlExpandedIss;
		pIsiSensor->pIsiSetIntegrationTimeIss =				IMX678_IsiSetIntegrationTimeIss;

		pIsiSensor->pIsiGetLongIntegrationTimeIss =			IMX678_IsiGetLEFIntegrationTimeIss;
		pIsiSensor->pIsiGetIntegrationTimeIss =				IMX678_IsiGetIntegrationTimeIss;
		pIsiSensor->pIsiGetVSIntegrationTimeIss =			IMX678_IsiGetSEF2IntegrationTimeIss;

		pIsiSensor->pIsiGetLongGainIss = 					IMX678_IsiGetLEFGainIss;
		pIsiSensor->pIsiGetGainIss = 						IMX678_IsiGetGainIss;
		pIsiSensor->pIsiGetVSGainIss = 						IMX678_IsiGetSEF2GainIss;

		pIsiSensor->pIsiGetGainIncrementIss =				IMX678_IsiGetGainIncrementIss;
		pIsiSensor->pIsiGetIrisIncrementIss =				IMX678_IsiGetIrisIncrementIss;
		pIsiSensor->pIsiGetIntegrationTimeIncrementIss =	IMX678_IsiGetIntegrationTimeIncrementIss;
		pIsiSensor->pIsiSetGainIss = 						IMX678_IsiSetGainIss;
		pIsiSensor->pIsiGetFpsIss = 						IMX678_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss = 						IMX678_IsiSetFpsIss;
        pIsiSensor->pIsiSetFlickerFpsIss = 					IMX678_IsiSetFlickerFpsIss;
        pIsiSensor->pIsiUnlimitFpsIss =						IMX678_IsiUnlimitFpsIss;
        pIsiSensor->pIsiLimitFpsIss =						IMX678_IsiLimitFpsIss;
		pIsiSensor->pIsiGetAutoFpsInfoIss = 				IMX678_IsiGetAutoFpsInfoIss;
		pIsiSensor->pIsiGetStartEvIss = 					IMX678_IsiGetStartEvIss;
        pIsiSensor->pIsiGetIrisIss =						IMX678_IsiGetIrisIss;
        pIsiSensor->pIsiSetIrisIss =						IMX678_IsiSetIrisIss;

        /* SENSOR ISP */
        pIsiSensor->pIsiGetIspStatusIss = IMX678_IsiGetIspStatusIss;
        // pIsiSensor->pIsiSetBlcIss                       =
        // IMX678_IsiSetBlcIss; pIsiSensor->pIsiSetWBIss = IMX678_IsiSetWBIss;

        /* SENSOE OTHER FUNC*/
        pIsiSensor->pIsiSetPowerIss = IMX678_IsiSetPowerIss;
        pIsiSensor->pIsiSetTpgIss = IMX678_IsiSetTpgIss;
        pIsiSensor->pIsiGetTpgIss = IMX678_IsiGetTpgIss;
        // pIsiSensor->pIsiGetExpandCurveIss               =
        // IMX678_IsiGetExpandCurveIss; pIsiSensor->pIsiGetCompressCurveIss =
        // IMX678_IsiGetCompressCurveIss; pIsiSensor->pIsiExtendFuncIss =
        // IMX678_IsiExtendFuncIss; pIsiSensor->pIsiGetOtpDataIss =
        // IMX678_IsiGetOtpDataIss;

        /* AF */
        pIsiSensor->pIsiFocusCreateIss = IMX678_IsiFocusCreateIss;
        pIsiSensor->pIsiFocusReleaseIss = IMX678_IsiFocusReleaseIss;
        pIsiSensor->pIsiFocusGetCalibrateIss = IMX678_IsiFocusGetCalibrateIss;
        pIsiSensor->pIsiFocusSetIss = IMX678_IsiFocusSetIss;
        pIsiSensor->pIsiFocusGetIss = IMX678_IsiFocusGetIss;
        pIsiSensor->pIsiSetAgainDgainIss = IMX678_IsiSetAgainDgainIss;

    } else {
        result = RET_NULL_POINTER;
    }

    TRACE(IMX678_INFO, "%s (exit)\n", __func__);
    return (result);
}

/*****************************************************************************
 * each sensor driver need declare this struct for isi load
 *****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x9012,
    .pIsiHalEnumModeIss = IMX678_IsiHalEnumModeIss,
    .pIsiGetSensorIss = IMX678_IsiGetSensorIss,
};
