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

#include "IMX675_priv.h"
#include "vvsensor.h"


CREATE_TRACER(IMX675_INFO, "IMX675: ", INFO, 1);
CREATE_TRACER(IMX675_WARN, "IMX675: ", WARNING, 1);
CREATE_TRACER(IMX675_ERROR, "IMX675: ", ERROR, 1);
CREATE_TRACER(IMX675_DEBUG, "IMX675: ", INFO, 1);
CREATE_TRACER(IMX675_REG_INFO, "IMX675: ", INFO, 1);
CREATE_TRACER(IMX675_REG_DEBUG, "IMX675: ", INFO, 1);

#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#define IMX675_I2C_ADDR 0x1a
#define IMX675_IRIS_MIN_VAL 1
#define IMX675_IRIS_MAX_VAL 1
#define IMX675_MIN_GAIN_STEP                                          \
    (0.035) /**< min gain step size used by GUI (hardware min = 1/16; \
               1/16..32/16 depending on actual gain ) */
#define IMX675_VMAX_30FPS 2200 // 0x898
// #define IMX675_VMAX_3DOL_HDR 6750
// #define IMX675_VMAX_2DOL_HDR 6750
#define IMX675_VMAX_MAX ((1 << 20) - 2) // max even value of unsigned 20 bits - 1048574
#define IMX675_MIN_SHR 4 // Min value of SHR0 in the software reference manual
#define IMX675_TRANSFER_BUFFER_LENGTH 3
#define IMX675_MAX_GAIN 3981
// #define IMX675_SHR0_RHS2_GAP 7
// #define IMX675_2DOL_SHR0_RHS1_GAP 5
// #define IMX675_SHR0_FSC_GAP 3
// #define IMX675_2DOL_SHR0_FSC_GAP 2
// #define IMX675_3DOL_SHR1_RHS1_GAP 7
// #define IMX675_2DOL_SHR1_MIN_GAP 5
// #define IMX675_SHR1_RHS1_GAP 3
// #define IMX675_2DOL_SHR1_RHS1_GAP 5
// #define IMX675_SHR2_RHS1_GAP 7
// #define IMX675_SHR2_RHS2_GAP 3
// #define IMX675_PIXEL_CLK_RATE 74.25
// #define DEFAULT_RHS1 0x91
// #define MICRO_2_NANO 1000


FlickerModePeaksPerSec flickerPeaksPerSecMap[] = {
    { ISI_AE_ANTIBANDING_MODE_OFF, 0 },
    { ISI_AE_ANTIBANDING_MODE_50HZ, 100 },
    { ISI_AE_ANTIBANDING_MODE_60HZ, 120 },
    { ISI_AE_ANTIBANDING_MODE_AUTO, 100 }
};

/*****************************************************************************
 *Forward Declarations
*****************************************************************************/
RESULT IMX675_IsiSetIrisIss( IsiSensorHandle_t handle,
                    const float NewIris);

/*****************************************************************************
 *Sensor Info
*****************************************************************************/

static struct vvsensor_mode_s pimx675_mode_info[] = {
    // Recommended resolution
    {
        .index     = 0,
        .size      ={
			.bounds_width  = 2592,
			.bounds_height = 1944,
			.top           = 0,
			.left          = 0,
			.width         = 2592,
			.height        = 1944,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_RGGB,
	.ae_info = {
		.one_line_exp_time_ns = 7410,
		.max_integration_time = IMX675_VMAX_30FPS - IMX675_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX675_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
#ifndef BRINGUP_CONFIG
    // Same as previous, except for the resolution (FHD)
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
		.one_line_exp_time_ns = 7410,
		.max_integration_time = IMX675_VMAX_30FPS - IMX675_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX675_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
    // HDR 3DOL
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
		.one_line_exp_time_ns = 7410,
		.max_integration_time = DEFAULT_RHS1 - IMX675_3DOL_SHR1_RHS1_GAP,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX675_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 20
	}
    },
    // Same as previous, except for the resolution (FHD)
    {
        .index     = 3,
        .size      ={
			.bounds_width  = 2592,
			.bounds_height = 1944,
			.top           = 0,
			.left          = 0,
			.width         = 2592,
			.height        = 1944,
		},
		.fps       = 20 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_3DOL,
		.bit_width = 12,
		.bayer_pattern = BAYER_RGGB,
        .ae_info = {
		.one_line_exp_time_ns = 7410,
		.max_integration_time = DEFAULT_RHS1 - IMX675_3DOL_SHR1_RHS1_GAP,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX675_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 20
	}
    },
    // HDR 2DOL
    {
        .index     = 4,
        .size      ={
			.bounds_width  = 2592,
			.bounds_height = 1944,
			.top           = 0,
			.left          = 0,
			.width         = 2592,
			.height        = 1944,
		},
		.fps       = 20 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_L_AND_S,
		.bit_width = 12,
		.bayer_pattern = BAYER_RGGB,
        .ae_info = {
		.one_line_exp_time_ns = 7410,
		.max_integration_time = DEFAULT_RHS1 - IMX675_2DOL_SHR1_RHS1_GAP,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX675_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    }
#endif // BRINGUP_CONFIG
};

static RESULT IMX675_IsiSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    RESULT result = RET_SUCCESS;
    return (result);
}

static RESULT IMX675_IsiCreateIss(IsiSensorInstanceConfig_t* pConfig) {
    RESULT result = RET_SUCCESS;
    IMX675_Context_t* pIMX675Ctx;
    char i2c_file_path[PATH_MAX];

    memset(i2c_file_path, 0, PATH_MAX);

    if (!pConfig || !pConfig->pSensor) return (RET_NULL_POINTER);

    pIMX675Ctx = (IMX675_Context_t*)malloc(sizeof(IMX675_Context_t));
    if (!pIMX675Ctx) {
        return (RET_OUTOFMEM);
    }

    MEMSET(pIMX675Ctx, 0, sizeof(IMX675_Context_t));
    result = HalAddRef(pConfig->HalHandle);
    if (result != RET_SUCCESS) {
        free(pIMX675Ctx);
        return (result);
    }

    pIMX675Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pIMX675Ctx->IsiCtx.pSensor = pConfig->pSensor;
    pIMX675Ctx->GroupHold = BOOL_FALSE;
    pIMX675Ctx->OldGain = 1.0;
    pIMX675Ctx->OldIntegrationTime = 0.01;
    pIMX675Ctx->Configured = BOOL_FALSE;
    pIMX675Ctx->Streaming = BOOL_FALSE;
    pIMX675Ctx->TestPattern = BOOL_FALSE;
    pIMX675Ctx->isAfpsRun = BOOL_FALSE;
    pIMX675Ctx->SensorMode.index = pConfig->SensorModeIndex;
    pConfig->SensorModeIndex =
	    MAX(MIN((sizeof(pimx675_mode_info) / sizeof(pimx675_mode_info[0])),
		    pConfig->SensorModeIndex),
		0);
    memcpy(&pIMX675Ctx->SensorMode,
	   &pimx675_mode_info[pConfig->SensorModeIndex],
	   sizeof(pIMX675Ctx->SensorMode));
    pConfig->hSensor = (IsiSensorHandle_t)pIMX675Ctx;
    pIMX675Ctx->pattern = ISI_BPAT_RGRGGBGB;
    pIMX675Ctx->subdev = HalGetFdHandle(pConfig->HalHandle,
                                        HAL_MODULE_SENSOR);  // two sensors??
    pIMX675Ctx->KernelDriverFlag = 1;
    sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cBusNum);
    pIMX675Ctx->i2c_fd = open(i2c_file_path, O_RDWR);
    if (pIMX675Ctx->i2c_fd < 0) {
        TRACE(IMX675_INFO, "unable to open /dev/i2c-%d\n", pConfig->I2cBusNum);
        return RET_FAILURE;
    }

    if (ioctl(pIMX675Ctx->i2c_fd, I2C_SLAVE_FORCE, IMX675_I2C_ADDR) < 0) {
        TRACE(IMX675_INFO, "unable to set I2C_SLAVE_FORCE on /dev/i2c-%d\n",
              pConfig->I2cBusNum);
        return RET_FAILURE;
    }

    return (result);
}

static RESULT IMX675_IsiReleaseIss(IsiSensorHandle_t handle) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX675Ctx == NULL) return (RET_WRONG_HANDLE);

    (void)IMX675_IsiSetStreamingIss(pIMX675Ctx, BOOL_FALSE);
    (void)IMX675_IsiSetPowerIss(pIMX675Ctx, BOOL_FALSE);
    (void)HalDelRef(pIMX675Ctx->IsiCtx.HalHandle);
    close(pIMX675Ctx->i2c_fd);
    MEMSET(pIMX675Ctx, 0, sizeof(IMX675_Context_t));
    free(pIMX675Ctx);
    return (result);
}

static RESULT IMX675_IsiReadRegIss(IsiSensorHandle_t handle,
                                   const uint32_t Addr, uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    char out[IMX675_TRANSFER_BUFFER_LENGTH];

    if (pIMX675Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX675_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    if (write(pIMX675Ctx->i2c_fd, out, sizeof(uint16_t)) != sizeof(uint16_t)) {
        return RET_FAILURE;
    }

    if (read(pIMX675Ctx->i2c_fd, out, 1) != 1) return RET_FAILURE;

    *pValue = out[0];

    return (result);
}

static RESULT IMX675_IsiWriteRegIss(IsiSensorHandle_t handle,
                                    const uint32_t Addr, const uint32_t Value) {
    RESULT result = RET_SUCCESS;
    char out[IMX675_TRANSFER_BUFFER_LENGTH];

    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    if (pIMX675Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX675_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    out[2] = Value;
    if (write(pIMX675Ctx->i2c_fd, out, sizeof(out)) != sizeof(out))
        result = RET_FAILURE;
    return (result);
}

static RESULT IMX675_UpdateFps(IMX675_Context_t *pIMX675Ctx, uint32_t vmax) {
    float frame_time = 0;
    frame_time = (vmax * pIMX675Ctx->one_line_exp_time);

    if (frame_time == 0) return RET_FAILURE;

    pIMX675Ctx->CurrFps = (uint32_t)(ceil(1 / frame_time));
    return RET_SUCCESS;
}

static RESULT IMX675_ReadVmax(IsiSensorHandle_t handle, uint32_t* vmax) {
    uint32_t vmax_low = 0, vmax_mid = 0, vmax_high = 0;
    RESULT result;

    result = IMX675_IsiReadRegIss(handle, 0x3028, &vmax_low);
    result |= IMX675_IsiReadRegIss(handle, 0x3029, &vmax_mid);
    result |= IMX675_IsiReadRegIss(handle, 0x302a, &vmax_high);
    if (result) return RET_FAILURE;

    *vmax = (vmax_high << 16) | (vmax_mid << 8) | vmax_low;
    return result;
}

#ifndef BRINGUP_CONFIG
static RESULT IMX675_ReadHmax(IsiSensorHandle_t handle, uint32_t* hmax) {
    uint32_t hmax_low = 0, hmax_high = 0;
    RESULT result;

    result = IMX675_IsiReadRegIss(handle, 0x302c, &hmax_low);
    result |= IMX675_IsiReadRegIss(handle, 0x302d, &hmax_high);
    if (result) return RET_FAILURE;

    *hmax = (hmax_high << 8) | hmax_low;
    return result;
}
#endif // BRINGUP_CONFIG

static RESULT IMX675_WriteVmax(IsiSensorHandle_t handle, uint32_t vmax) {
    RESULT result;

    result = IMX675_IsiWriteRegIss(handle, 0x3028, vmax & 0xff);
    result |= IMX675_IsiWriteRegIss(handle, 0x3029, (vmax >> 8) & 0xff);
    result |= IMX675_IsiWriteRegIss(handle, 0x302a, (vmax >> 16) & 0x0f);
    if (!result) {
        return IMX675_UpdateFps((IMX675_Context_t *)handle, vmax);
    }

    return result;
}

#ifndef BRINGUP_CONFIG
static RESULT IMX675_ReadRHS1(IsiSensorHandle_t handle, uint32_t* rhs1) {
    uint32_t rhs1_low = 0, rhs1_mid = 0, rhs1_high = 0;
    RESULT result;

    result = IMX675_IsiReadRegIss(handle, 0x3060, &rhs1_low);
    result |= IMX675_IsiReadRegIss(handle, 0x3061, &rhs1_mid);
    result |= IMX675_IsiReadRegIss(handle, 0x3062, &rhs1_high);
    if (result) return RET_FAILURE;

    *rhs1 = (rhs1_high << 16) | (rhs1_mid << 8) | rhs1_low;
    return result;
}

static RESULT IMX675_ReadRHS2(IsiSensorHandle_t handle, uint32_t* rhs2) {
    uint32_t rhs2_low = 0, rhs2_mid = 0, rhs2_high = 0;
    RESULT result;

    result = IMX675_IsiReadRegIss(handle, 0x3064, &rhs2_low);
    result |= IMX675_IsiReadRegIss(handle, 0x3065, &rhs2_mid);
    result |= IMX675_IsiReadRegIss(handle, 0x3066, &rhs2_high);
    if (result) return RET_FAILURE;

    *rhs2 = (rhs2_high << 16) | (rhs2_mid << 8) | rhs2_low;
    return result;
}
#endif // BRINGUP_CONFIG

static RESULT IMX675_WriteShr0(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX675_IsiWriteRegIss(handle, 0x3050, shr & 0xff);
    result |= IMX675_IsiWriteRegIss(handle, 0x3051, (shr >> 8) & 0xff);
    result |= IMX675_IsiWriteRegIss(handle, 0x3052, (shr >> 16) & 0x0f);

    return result;
}

#ifndef BRINGUP_CONFIG
static RESULT IMX675_WriteShr1(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX675_IsiWriteRegIss(handle, 0x3054, shr & 0xff);
    result |= IMX675_IsiWriteRegIss(handle, 0x3055, (shr >> 8) & 0xff);
    result |= IMX675_IsiWriteRegIss(handle, 0x3056, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX675_WriteShr2(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX675_IsiWriteRegIss(handle, 0x3058, shr & 0xff);
    result |= IMX675_IsiWriteRegIss(handle, 0x3059, (shr >> 8) & 0xff);
    result |= IMX675_IsiWriteRegIss(handle, 0x305a, (shr >> 16) & 0x0f);

    return result;
}
#endif // BRINGUP_CONFIG

static RESULT IMX675_WriteGain(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX675_IsiWriteRegIss(handle, 0x3070, (gain & 0x00ff));
	result |= IMX675_IsiWriteRegIss(handle, 0x3071, (gain & 0x0700) >> 8);

    return result;
}

#ifndef BRINGUP_CONFIG
static RESULT IMX675_WriteGain1(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX675_IsiWriteRegIss(handle, 0x3072, (gain & 0x00ff));
	result |= IMX675_IsiWriteRegIss(handle, 0x3073, (gain & 0x0700) >> 8);

    return result;
}

static RESULT IMX675_WriteGain2(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX675_IsiWriteRegIss(handle, 0x3074, (gain & 0x00ff));
	result |= IMX675_IsiWriteRegIss(handle, 0x3075, (gain & 0x0700) >> 8);

    return result;
}
#endif // BRINGUP_CONFIG

static RESULT IMX675_LockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX675_IsiWriteRegIss(handle, 0x3001, 0x1);

    return result;
}

static RESULT IMX675_UnlockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX675_IsiWriteRegIss(handle, 0x3001, 0x0);

    return result;
}

static RESULT IMX675_IsiGetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    TRACE(IMX675_INFO, "%s (enter)\n", __func__);
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    if (pIMX675Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    memcpy(pMode, &(pIMX675Ctx->SensorMode), sizeof(pIMX675Ctx->SensorMode));

    TRACE(IMX675_INFO, "%s (exit)\n", __func__);
    return (RET_SUCCESS);
}

static RESULT IMX675_IsiSetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    int ret = 0;
    TRACE(IMX675_INFO, "%s (enter)\n", __func__);

    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    if (pIMX675Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX675Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, pMode);
    if (ret != 0) {
        return (RET_FAILURE);
    }

    return (RET_SUCCESS);
}

static RESULT IMX675_IsiHalEnumModeIss(HalHandle_t HalHandle,
                                       IsiEnumMode_t* pEnumMode) {
    HalContext_t* pHalCtx = HalHandle;
    if (pHalCtx == NULL) {
        return RET_NULL_POINTER;
    }

    if (pEnumMode->index >=
        (sizeof(pimx675_mode_info) / sizeof(pimx675_mode_info[0])))
        return RET_OUTOFRANGE;

    for (uint32_t i = 0;
         i < (sizeof(pimx675_mode_info) / sizeof(pimx675_mode_info[0])); i++) {
        if (pimx675_mode_info[i].index == pEnumMode->index) {
            memcpy(&pEnumMode->mode, &pimx675_mode_info[i], sizeof(IsiMode_t));
            TRACE(IMX675_ERROR, "%s (exit)\n", __func__);
            return RET_SUCCESS;
        }
    }

    return RET_NOTSUPP;
}

static RESULT IMX675_IsiEnumModeIss(IsiSensorHandle_t handle,
                                    IsiEnumMode_t* pEnumMode) {
    RESULT result = RET_SUCCESS;
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    if (pIMX675Ctx == NULL || pIMX675Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    HalContext_t* pHalCtx = (HalContext_t*)pIMX675Ctx->IsiCtx.HalHandle;
    result = IMX675_IsiHalEnumModeIss(pHalCtx, pEnumMode);
    if (result != RET_SUCCESS) {
        TRACE(IMX675_ERROR, "%s: sensor enum mode error!\n", __func__);
        return (RET_FAILURE);
    }

    return result;
}

static RESULT IMX675_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t* pCaps) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;

    RESULT result = RET_SUCCESS;

    TRACE(IMX675_INFO, "%s (enter)\n", __func__);

    if (pIMX675Ctx == NULL) return (RET_WRONG_HANDLE);

    if (pCaps == NULL) {
        return (RET_NULL_POINTER);
    }

    if (!pIMX675Ctx->Configured) IMX675_IsiSetupIss(handle, pCaps);

    pCaps->BusWidth = pIMX675Ctx->SensorMode.bit_width;
    pCaps->Mode = ISI_MODE_BAYER;
    pCaps->FieldSelection = ISI_FIELDSEL_BOTH;
    pCaps->YCSequence = ISI_YCSEQ_YCBYCR;
    pCaps->Conv422 = ISI_CONV422_COSITED;
    pCaps->BPat = pIMX675Ctx->SensorMode.bayer_pattern;
    pCaps->HPol = ISI_HPOL_REFPOS;
    pCaps->VPol = ISI_VPOL_POS;
    pCaps->Edge = ISI_EDGE_RISING;
    pCaps->Resolution.width = pIMX675Ctx->SensorMode.size.width;
    pCaps->Resolution.height = pIMX675Ctx->SensorMode.size.height;
    pCaps->SmiaMode = ISI_SMIA_OFF;
    pCaps->MipiLanes = ISI_MIPI_4LANES;

    if (pCaps->BusWidth == 10) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_10;
    } else if (pCaps->BusWidth == 12) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_12;
    } else {
        pCaps->MipiMode = ISI_MIPI_OFF;
    }
    TRACE(IMX675_INFO, "got caps - width %d height %d buswidth %d\n",
          pCaps->Resolution.width, pCaps->Resolution.height, pCaps->BusWidth);
    TRACE(IMX675_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX675_AecSetModeParameters(IMX675_Context_t* pIMX675Ctx,
                                          const IsiCaps_t* pConfig) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX675_INFO, "%s%s: (enter)\n", __func__,
          pIMX675Ctx->isAfpsRun ? "(AFPS)" : "");

    pIMX675Ctx->AecIntegrationTimeIncrement = pIMX675Ctx->one_line_exp_time;
    pIMX675Ctx->AecMinIntegrationTime =
        pIMX675Ctx->one_line_exp_time * pIMX675Ctx->MinIntegrationLine;
    pIMX675Ctx->AecMaxIntegrationTime =
        pIMX675Ctx->one_line_exp_time * pIMX675Ctx->MaxIntegrationLine;

    TRACE(IMX675_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __func__,
          pIMX675Ctx->isAfpsRun ? "(AFPS)" : "",
          pIMX675Ctx->AecMaxIntegrationTime);

    pIMX675Ctx->AecGainIncrement = IMX675_MIN_GAIN_STEP;

    // reflects the state of the sensor registers, must equal default settings
    pIMX675Ctx->AecCurGainLEF = pIMX675Ctx->AecMinGain;
	pIMX675Ctx->AecCurGainSEF1 = pIMX675Ctx->AecMinGain;
	pIMX675Ctx->AecCurGainSEF2 = pIMX675Ctx->AecMinGain;
	pIMX675Ctx->AecCurIntegrationTimeLEF = pIMX675Ctx->AecMaxIntegrationTime;
	pIMX675Ctx->AecCurIntegrationTimeSEF1 = pIMX675Ctx->AecMaxIntegrationTime;
	pIMX675Ctx->AecCurIntegrationTimeSEF2 = pIMX675Ctx->AecMaxIntegrationTime;
    pIMX675Ctx->OldGain = 1;
    pIMX675Ctx->OldIntegrationTime = 0.0f;

    TRACE(IMX675_INFO, "%s%s: (exit)\n", __func__,
          pIMX675Ctx->isAfpsRun ? "(AFPS)" : "");

    return (result);
}

static RESULT IMX675_IsiSetupIss(IsiSensorHandle_t handle,
                                 const IsiCaps_t* pCaps) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX675_INFO, "%s (enter)\n", __func__);

    if (!pIMX675Ctx) {
        TRACE(IMX675_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pCaps == NULL) {
        TRACE(IMX675_ERROR,
              "%s: Invalid configuration (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMX675Ctx->Streaming != BOOL_FALSE) {
        return RET_WRONG_STATE;
    }

    if (pIMX675Ctx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
#ifndef BRINGUP_CONFIG
        pIMX675Ctx->enableHdr = true;
#else
        printf("%s: HDR mode not supported yet!\n", __func__);
        return -EINVAL;
#endif
    } else {
        pIMX675Ctx->enableHdr = false;
    }

    pIMX675Ctx->one_line_exp_time =
        (float)(pIMX675Ctx->SensorMode.ae_info.one_line_exp_time_ns) /
        1000000000;
    pIMX675Ctx->MaxIntegrationLine =
        pIMX675Ctx->SensorMode.ae_info.max_integration_time;
    TRACE(IMX675_INFO, "%s: MaxIntegrationLine %u\n", __func__,
          pIMX675Ctx->MaxIntegrationLine);
    pIMX675Ctx->MinIntegrationLine =
        pIMX675Ctx->SensorMode.ae_info.min_integration_time;
    pIMX675Ctx->gain_accuracy = pIMX675Ctx->SensorMode.ae_info.gain_accuracy;
    pIMX675Ctx->AecMaxGain = (float)(pIMX675Ctx->SensorMode.ae_info.max_gain) /
                             pIMX675Ctx->gain_accuracy;
    pIMX675Ctx->AecMinGain = (float)(pIMX675Ctx->SensorMode.ae_info.min_gain) /
                             pIMX675Ctx->gain_accuracy;

    pIMX675Ctx->AecMinIris = IMX675_IRIS_MIN_VAL;
    pIMX675Ctx->AecMaxIris = IMX675_IRIS_MAX_VAL;

    pIMX675Ctx->original_vmax = 0;
    pIMX675Ctx->unlimit_fps = 0;

    pIMX675Ctx->MaxFps = pIMX675Ctx->SensorMode.fps;
    pIMX675Ctx->CurrFps = pIMX675Ctx->MaxFps;
    TRACE(IMX675_INFO, "%s - got caps - width %d height %d buswidth %d\n",
          __func__, pIMX675Ctx->SensorMode.size.width,
          pIMX675Ctx->SensorMode.size.height, pIMX675Ctx->SensorMode.bit_width);

    TRACE(IMX675_INFO, "%s - MinGain %f MaxGain %f\n", __func__, pIMX675Ctx->AecMinGain,
          pIMX675Ctx->AecMaxGain);

    memcpy(&pIMX675Ctx->CapsConfig, pCaps, sizeof(pIMX675Ctx->CapsConfig));

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful,
     * bits 6..0 are reserved, reset bit is not sticky */
    TRACE(IMX675_DEBUG, "%s: IMX675 System-Reset executed\n", __func__);
    osSleep(100);

    result = IMX675_AecSetModeParameters(pIMX675Ctx, pCaps);
    if (result != RET_SUCCESS) {
        TRACE(IMX675_ERROR, "%s: SetupOutputWindow failed.\n", __func__);
        return (result);
    }

    pIMX675Ctx->Configured = BOOL_TRUE;
    TRACE(IMX675_INFO, "%s: (exit)\n", __func__);
    return 0;
}

static RESULT IMX675_IsiCheckConnectionIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    return (result);
}

static RESULT IMX675_IsiGetRevisionIss(IsiSensorHandle_t handle,
                                       uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    int ret = 0;
    uint32_t reg_val;
    uint32_t sensor_id;

    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    if (pIMX675Ctx == NULL || pIMX675Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX675Ctx->IsiCtx.HalHandle;

    if (!pValue) return (RET_NULL_POINTER);

    if (pIMX675Ctx->KernelDriverFlag) {
        ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, &sensor_id);
        if (ret != 0) {
            TRACE(IMX675_ERROR, "%s: Read Sensor ID Error! \n", __func__);
            return (RET_FAILURE);
        }
    } else {
        reg_val = 0;
        result = IMX675_IsiReadRegIss(handle, 0x3a04, &reg_val);
        sensor_id = (reg_val & 0xff) << 8;

        reg_val = 0;
        result |= IMX675_IsiReadRegIss(handle, 0x3a05, &reg_val);
        sensor_id |= (reg_val & 0xff);
    }

    *pValue = sensor_id;
    TRACE(IMX675_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX675_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX675_INFO, "%s (enter)\n", __func__);

    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    if (pIMX675Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    pIMX675Ctx->Streaming = on;

    if (pIMX675Ctx->enableHdr)
        return result;

    TRACE(IMX675_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX675_IsiGetGainLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinGain, float* pMaxGain) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX675_INFO, "%s: (enter)\n", __func__);

    if (pIMX675Ctx == NULL) {
        TRACE(IMX675_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinGain == NULL) || (pMaxGain == NULL)) {
        TRACE(IMX675_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinGain = pIMX675Ctx->AecMinGain;
    *pMaxGain = pIMX675Ctx->AecMaxGain;

    TRACE(IMX675_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX675_IsiUnlimitFpsIss(IsiSensorHandle_t handle,
                                      float maxIntegrationTime) {
    RESULT result = RET_SUCCESS;
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;

    if (!pIMX675Ctx) {
        return RET_NULL_POINTER;
    }

    if (pIMX675Ctx->enableHdr) {
        return result;
    }
    pIMX675Ctx->unlimit_fps = 1;

    if (maxIntegrationTime < 0)
        pIMX675Ctx->MaxIntegrationLine = IMX675_VMAX_MAX - IMX675_MIN_SHR;
    else
        pIMX675Ctx->MaxIntegrationLine =
            MIN((uint32_t)(maxIntegrationTime / pIMX675Ctx->one_line_exp_time),
                IMX675_VMAX_MAX - IMX675_MIN_SHR);
    TRACE(IMX675_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
        pIMX675Ctx->MaxIntegrationLine);
    pIMX675Ctx->AecMaxIntegrationTime =
        pIMX675Ctx->one_line_exp_time * pIMX675Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX675_IsiLimitFpsIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    uint32_t current_vmax = 0;
    TRACE(IMX675_INFO, "%s (enter)\n", __func__);
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    if (!pIMX675Ctx) {
        return RET_NULL_POINTER;
    }

    if (pIMX675Ctx->enableHdr)
        return result;

    IMX675_ReadVmax(handle, &current_vmax);

    pIMX675Ctx->unlimit_fps = 0;
    if (pIMX675Ctx->original_vmax == 0) {
        pIMX675Ctx->original_vmax = current_vmax;
    }
    if (current_vmax != pIMX675Ctx->original_vmax) {
        IMX675_WriteVmax(handle, pIMX675Ctx->original_vmax);
    }

    pIMX675Ctx->MaxIntegrationLine =
        MAX(pIMX675Ctx->original_vmax - IMX675_MIN_SHR, 1);
    TRACE(IMX675_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
          pIMX675Ctx->MaxIntegrationLine);
    pIMX675Ctx->AecMaxIntegrationTime =
        pIMX675Ctx->one_line_exp_time * pIMX675Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX675_IsiGetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinIris, float* pMaxIris) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX675_INFO, "%s: (enter)\n", __func__);

    if (pIMX675Ctx == NULL) {
        TRACE(IMX675_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIris == NULL) || (pMaxIris == NULL)) {
        TRACE(IMX675_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIris = pIMX675Ctx->AecMinIris;
    *pMaxIris = pIMX675Ctx->AecMaxIris;

    TRACE(IMX675_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX675_IsiSetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float minIris, float maxIris) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX675_INFO, "%s: (enter)\n", __func__);

    if (pIMX675Ctx == NULL) {
        TRACE(IMX675_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    pIMX675Ctx->AecMinIris = minIris;
    pIMX675Ctx->AecMaxIris = maxIris;

    TRACE(IMX675_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX675_IsiGetIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float* pMinIntegrationTime,
    float* pMaxIntegrationTime) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX675_INFO, "%s: (enter)\n", __func__);
    if (pIMX675Ctx == NULL) {
        TRACE(IMX675_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX675_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIntegrationTime = pIMX675Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMX675Ctx->AecMaxIntegrationTime;

    TRACE(IMX675_INFO, "%s: (exit) %f, %f\n", 
    __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

/* Gain get functions*/

RESULT IMX675_IsiGetGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;
	TRACE(IMX675_INFO, "%s: (enter)\n", __func__);

	if (pIMX675Ctx == NULL) {
		TRACE(IMX675_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

#ifndef BRINGUP_CONFIG
	if (pIMX675Ctx->enableHdr)
		return IMX675_IsiGetSEF1GainIss(handle, pSetGain);
#endif // BRINGUP_CONFIG

	return IMX675_IsiGetLEFGainIss(handle, pSetGain);
}

RESULT IMX675_IsiGetLEFGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	TRACE(IMX675_INFO, "%s: (enter)\n", __func__);

	if (pIMX675Ctx == NULL) {
		TRACE(IMX675_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}
	*pSetGain = pIMX675Ctx->AecCurGainLEF;
	TRACE(IMX675_DEBUG, "%s - returning %f\n", __func__, pIMX675Ctx->AecCurGainLEF);
	TRACE(IMX675_INFO, "%s: (exit)\n", __func__);
	return (result);
}

#ifndef BRINGUP_CONFIG
RESULT IMX675_IsiGetSEF1GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX675_INFO, "%s: (enter)\n", __func__);

	if (pIMX675Ctx == NULL) {
		TRACE(IMX675_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX675Ctx->AecCurGainSEF1;
	TRACE(IMX675_DEBUG, "%s - returning %f\n", __func__, pIMX675Ctx->AecCurGainSEF1);

	TRACE(IMX675_INFO, "%s: (exit)\n", __func__);

	return (result);
}

RESULT IMX675_IsiGetSEF2GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX675_INFO, "%s: (enter)\n", __func__);

	if (pIMX675Ctx == NULL) {
		TRACE(IMX675_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX675Ctx->AecCurGainSEF2;
	TRACE(IMX675_DEBUG, "%s - returning %f\n", __func__, pIMX675Ctx->AecCurGainSEF2);

	TRACE(IMX675_INFO, "%s: (exit)\n", __func__);

	return (result);
}
#endif //BRINGUP_CONFIG

RESULT IMX675_IsiGetGainIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX675_INFO, "%s: (enter)\n", __func__);

    if (pIMX675Ctx == NULL) {
        TRACE(IMX675_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = pIMX675Ctx->AecGainIncrement;

    TRACE(IMX675_INFO, "%s: (exit)\n", __func__);

    return (result);
}

RESULT IMX675_IsiGetIrisIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX675_INFO, "%s: (enter)\n", __func__);
    if (pIMX675Ctx == NULL) {
        TRACE(IMX675_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = 1;

    TRACE(IMX675_INFO, "%s: (exit)\n", __func__);

    return (result);
}

/* Gain set functions*/

RESULT IMX675_IsiSetGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;
	if (pIMX675Ctx == NULL || pIMX675Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

#ifndef BRINGUP_CONFIG
	if (pIMX675Ctx->enableHdr) {
		result = IMX675_IsiSetSEF1GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);

		result |= IMX675_IsiSetSEF2GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);
	}
#endif //BRINGUP_CONFIG

	result |= IMX675_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
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

RESULT IMX675_IsiSetLEFGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;
	if (pIMX675Ctx == NULL || pIMX675Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	TRACE(IMX675_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX675_DEBUG, "%s: writting 0x%x to GAIN\n", __func__, Gain);

    result |= IMX675_LockRegHold(handle);
	result |= IMX675_WriteGain(handle, Gain);
	result |= IMX675_UnlockRegHold(handle);

	if (result != 0) {
		return RET_FAILURE;
	}

	pIMX675Ctx->AecCurGainLEF = _sensorGain2linear(Gain);
	*pSetGain = pIMX675Ctx->AecCurGainLEF;
	TRACE(IMX675_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

#ifndef BRINGUP_CONFIG
RESULT IMX675_IsiSetSEF1GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX675Ctx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);

	TRACE(IMX675_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX675_DEBUG, "%s: writting 0x%x to GAIN1\n", __func__, Gain);

    result |= IMX675_LockRegHold(handle);
	result |= IMX675_WriteGain1(handle, Gain);
    result |= IMX675_UnlockRegHold(handle);

	pIMX675Ctx->AecCurGainSEF1 = _sensorGain2linear(Gain);
	*pSetGain = pIMX675Ctx->AecCurGainSEF1;

	TRACE(IMX675_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

RESULT IMX675_IsiSetSEF2GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX675Ctx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);
	
	TRACE(IMX675_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX675_DEBUG, "%s: writting 0x%x to GAIN2\n", __func__, Gain);

    result |= IMX675_LockRegHold(handle);
	result |= IMX675_WriteGain2(handle, Gain);
    result |= IMX675_UnlockRegHold(handle);

	pIMX675Ctx->AecCurGainSEF2 = _sensorGain2linear(Gain);
	*pSetGain = pIMX675Ctx->AecCurGainSEF2;

	TRACE(IMX675_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}
#endif //BRINGUP_CONFIG

/* Integration Time get functions*/

RESULT IMX675_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;

	if (!pIMX675Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	
	TRACE(IMX675_DEBUG, "%s - enter\n", __func__);
	
#ifndef BRINGUP_CONFIG
	if (pIMX675Ctx->enableHdr)
		return IMX675_IsiGetSEF1IntegrationTimeIss(handle, pSetIntegrationTime);
#endif //BRINGUP_CONFIG
	
	return IMX675_IsiGetLEFIntegrationTimeIss(handle, pSetIntegrationTime);
}

RESULT IMX675_IsiGetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;
	RESULT result = RET_SUCCESS;


	if (!pIMX675Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX675Ctx->AecCurIntegrationTimeLEF;
	TRACE(IMX675_DEBUG, "%s - returning %f\n", __func__, pIMX675Ctx->AecCurIntegrationTimeLEF);
	return (result);
}

#ifndef BRINGUP_CONFIG
RESULT IMX675_IsiGetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX675Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX675Ctx->AecCurIntegrationTimeSEF1;
	TRACE(IMX675_DEBUG, "%s - returning %f\n", __func__, pIMX675Ctx->AecCurIntegrationTimeSEF1);
	return (result);
}

RESULT IMX675_IsiGetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX675Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);

	*pSetIntegrationTime = pIMX675Ctx->AecCurIntegrationTimeSEF2;
	TRACE(IMX675_DEBUG, "%s - returning %f\n", __func__, pIMX675Ctx->AecCurIntegrationTimeSEF2);
	return (result);
}
#endif //BRINGUP_CONFIG

RESULT IMX675_IsiGetIntegrationTimeIncrementIss(IsiSensorHandle_t handle,
                                                float* pIncr) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (!pIMX675Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pIncr) return (RET_NULL_POINTER);

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders
    //in
    // the application)
    *pIncr = pIMX675Ctx->AecIntegrationTimeIncrement;
    return (result);
}

/* Integration Time set functions*/

RESULT IMX675_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;

	if (!pIMX675Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

#ifndef BRINGUP_CONFIG
	if (pIMX675Ctx->enableHdr) {
		return IMX675_IsiSetSEF1IntegrationTimeIss(
			handle, NewIntegrationTime, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
	}
#endif //BRINGUP_CONFIG

	return IMX675_IsiSetLEFIntegrationTimeIss(
		handle, NewIntegrationTime, pSetIntegrationTime,
		pNumberOfFramesToSkip, hdr_ratio);
}

RESULT IMX675_IsiSetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;

    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;

    int shr = 0;
    uint32_t exp = 0;
    uint32_t new_vmax = 0;
    uint32_t vmax_updated = 0;
    uint32_t current_vmax = 0;
#ifndef BRINGUP_CONFIG
    uint32_t rhs1;
    uint32_t rhs2;
#endif //BRINGUP_CONFIG

    if (!pIMX675Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pSetIntegrationTime || !pNumberOfFramesToSkip) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMX675Ctx->Streaming) {
        TRACE(IMX675_ERROR, "%s: sensor not streaming\n", __func__);
        return RET_FAILURE;
    }
    exp = NewIntegrationTime / pIMX675Ctx->one_line_exp_time;

    TRACE(IMX675_DEBUG, "%s: set AEC_PK_EXPO=0x%05x\n", __func__, exp);

    if (fabs(NewIntegrationTime - pIMX675Ctx->AecCurIntegrationTimeLEF) > FLT_EPSILON) {

        if (pIMX675Ctx->enableHdr){
#ifndef BRINGUP_CONFIG
            if (pIMX675Ctx->cur_rhs1 == 0 || pIMX675Ctx->cur_rhs2 == 0) {
                TRACE(IMX675_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
                return (RET_WRONG_CONFIG);
            }

            rhs1 = pIMX675Ctx->cur_rhs1;
            rhs2 = pIMX675Ctx->cur_rhs2;
            if (pIMX675Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
                new_vmax = IMX675_VMAX_2DOL_HDR;
                exp = new_vmax - exp;
                exp = exp > rhs1 + IMX675_2DOL_SHR0_RHS1_GAP ? exp : rhs1 + IMX675_2DOL_SHR0_RHS1_GAP;
                exp = exp < new_vmax - IMX675_2DOL_SHR0_FSC_GAP? exp : new_vmax - IMX675_2DOL_SHR0_FSC_GAP;
                shr = exp;
            } else {
                new_vmax = IMX675_VMAX_3DOL_HDR;
                exp = new_vmax - exp;
                exp = exp > rhs2 + IMX675_SHR0_RHS2_GAP ? exp : rhs2 + IMX675_SHR0_RHS2_GAP;
                exp = exp < new_vmax - IMX675_SHR0_FSC_GAP? exp : new_vmax - IMX675_SHR0_FSC_GAP;
                shr = exp;
            }
#endif //BRINGUP_CONFIG
		} else {
            if (exp > pIMX675Ctx->MaxIntegrationLine || exp == 0) {
                TRACE(IMX675_ERROR, "%s: Integration time %f (exp %u) out of range (%u)\n", __func__,
                    NewIntegrationTime, exp, pIMX675Ctx->MaxIntegrationLine);
                return RET_FAILURE;
            }
            result = IMX675_ReadVmax(handle, &current_vmax);

            if (pIMX675Ctx->original_vmax == 0) {
                pIMX675Ctx->original_vmax = current_vmax;
            }

            if (pIMX675Ctx->original_vmax - IMX675_MIN_SHR >
                pIMX675Ctx->MaxIntegrationLine) {
                pIMX675Ctx->MaxIntegrationLine =
                    pIMX675Ctx->original_vmax - IMX675_MIN_SHR;
                TRACE(IMX675_DEBUG, "%s: set MaxIntegrationLine to %u\n", __func__,
                      pIMX675Ctx->MaxIntegrationLine);
                pIMX675Ctx->AecMaxIntegrationTime =
                    pIMX675Ctx->one_line_exp_time * pIMX675Ctx->MaxIntegrationLine;
                TRACE(IMX675_DEBUG, "%s: set AecMaxIntegrationTime to %f\n", __func__,
                      pIMX675Ctx->AecMaxIntegrationTime);
            }

            shr = current_vmax - exp;

            if (shr < IMX675_MIN_SHR) {
                new_vmax = MIN(exp + IMX675_MIN_SHR,
                            pIMX675Ctx->MaxIntegrationLine + IMX675_MIN_SHR);
                shr = IMX675_MIN_SHR;
                vmax_updated = 1;
            } else if (shr > IMX675_MIN_SHR &&
                    current_vmax > pIMX675Ctx->original_vmax) {
                new_vmax = MAX(current_vmax - shr + IMX675_MIN_SHR,
                            pIMX675Ctx->original_vmax);
                shr = new_vmax - exp;
                vmax_updated = 1;
            } else {
                new_vmax = current_vmax;
            }
        }

        result |= IMX675_LockRegHold(handle);
        if (vmax_updated && pIMX675Ctx->unlimit_fps && !pIMX675Ctx->enableHdr) {
            result = IMX675_WriteVmax(handle, new_vmax);
        }

        TRACE(IMX675_DEBUG, "%s - writing 0x%x to SHR0\n", __func__, shr);
        result |= IMX675_WriteShr0(handle, shr);
        result |= IMX675_UnlockRegHold(handle);

        float configuredIntegrationTime =
            (new_vmax - shr) * pIMX675Ctx->one_line_exp_time;
        pIMX675Ctx->OldIntegrationTime = configuredIntegrationTime;
        pIMX675Ctx->AecCurIntegrationTimeLEF = configuredIntegrationTime;

        *pNumberOfFramesToSkip = 1U;
    } else {
        *pNumberOfFramesToSkip = 0U;  // no frame skip
    }

    *pSetIntegrationTime = pIMX675Ctx->AecCurIntegrationTimeLEF;
    TRACE(IMX675_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTime);
    return (result);
}

#ifndef BRINGUP_CONFIG
RESULT IMX675_IsiSetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF1,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;

	if (!pIMX675Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX675Ctx->cur_rhs1 == 0 || pIMX675Ctx->cur_rhs2 == 0) {
		TRACE(IMX675_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX675Ctx->cur_rhs1;

	if (!pSetIntegrationTimeSEF1 || !pNumberOfFramesToSkip) {
		printf("%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX675_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = (NewIntegrationTime / pIMX675Ctx->one_line_exp_time);
	TRACE(IMX675_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX675Ctx->AecCurIntegrationTimeSEF1) > FLT_EPSILON) {
		exp = rhs1 - exp;
		exp = exp > IMX675_3DOL_SHR1_RHS1_GAP ? exp : IMX675_3DOL_SHR1_RHS1_GAP;
		exp = exp < rhs1 - IMX675_SHR1_RHS1_GAP ? exp : rhs1 - IMX675_SHR1_RHS1_GAP;
		TRACE(IMX675_DEBUG, "%s - writing 0x%x to SHR1\n", __func__, exp);

		result |= IMX675_LockRegHold(handle);
		result |= IMX675_WriteShr1(handle, exp);
		result |= IMX675_UnlockRegHold(handle);

		pIMX675Ctx->AecCurIntegrationTimeSEF1 = (rhs1 - exp) * pIMX675Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF1 = pIMX675Ctx->AecCurIntegrationTimeSEF1;

	TRACE(IMX675_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF1);
	return (result);
}

RESULT IMX675_IsiSetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF2,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX675_Context_t *pIMX675Ctx = (IMX675_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;
	uint32_t rhs2;

	if (pIMX675Ctx->cur_rhs1 == 0 || pIMX675Ctx->cur_rhs2 == 0) {
		printf("%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX675Ctx->cur_rhs1;
	rhs2 = pIMX675Ctx->cur_rhs2;

	if (!pIMX675Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTimeSEF2 || !pNumberOfFramesToSkip) {
		printf("%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX675_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = (NewIntegrationTime / pIMX675Ctx->one_line_exp_time);
	TRACE(IMX675_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX675Ctx->AecCurIntegrationTimeSEF2) > FLT_EPSILON) {
		exp = rhs2 - exp;
		exp = exp > rhs1 + IMX675_SHR2_RHS1_GAP ? exp : rhs1 + IMX675_SHR2_RHS1_GAP;
		exp = exp < rhs2 - IMX675_SHR2_RHS2_GAP ? exp : rhs2 - IMX675_SHR2_RHS2_GAP;
		TRACE(IMX675_DEBUG, "%s - writing 0x%x to SHR2\n", __func__, exp);

		result |= IMX675_LockRegHold(handle);
		result |= IMX675_WriteShr2(handle, exp);
		result |= IMX675_UnlockRegHold(handle);

		pIMX675Ctx->AecCurIntegrationTimeSEF2 = (rhs2 - exp) * pIMX675Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF2 = pIMX675Ctx->AecCurIntegrationTimeSEF2;

	TRACE(IMX675_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF2);
	return (result);
}

RESULT IMX675_Calculate3DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it, float *o_very_short_it,
                                    float *o_long_gain, float *o_short_gain, float *o_very_short_gain,
                                    float *hdr_ratio) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
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

    if (pIMX675Ctx == NULL || o_long_it == NULL || o_short_it == NULL ||
        o_very_short_it == NULL || o_long_gain == NULL || o_short_gain == NULL ||
        o_very_short_gain == NULL || hdr_ratio == NULL) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

	if (pIMX675Ctx->cur_rhs1 == 0 || pIMX675Ctx->cur_rhs2 == 0) {
		TRACE(IMX675_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX675Ctx->cur_rhs1;
	rhs2 = pIMX675Ctx->cur_rhs2;

    TRACE(IMX675_DEBUG, "%s: hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n", 
    __func__, hdr_ratio[0], hdr_ratio[1]);
    
    // Sometimes there is no actual input gain. In that case, we will read it from the sensor
    if (NewGain == 0) {
        TRACE(IMX675_DEBUG, "%s: Input NewGain is 0, reading gain from sensor\n", __func__);
        result = IMX675_IsiGetSEF1GainIss(handle, &NewGain);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // Same for integration time
    if (NewIntegrationTime == 0) {
        TRACE(IMX675_DEBUG, "%s: Input NewIntegrationTime is 0, reading integration time from sensor\n", __func__);
        result = IMX675_IsiGetSEF1IntegrationTimeIss(handle, &NewIntegrationTime);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // assume gain is 1 and see if ratio can be achieved with integration time
    long_it 		= NewIntegrationTime * hdr_ratio[0];
    short_it 		= NewIntegrationTime;
    very_short_it 	= NewIntegrationTime / hdr_ratio[1];
    
    TRACE(IMX675_DEBUG, "%s: requested IT long: %f, short: %f, very_short: %f\n", 
    __func__, long_it, short_it, very_short_it);
    long_exp_val 		= long_it / pIMX675Ctx->one_line_exp_time;
    short_exp_val 		= short_it / pIMX675Ctx->one_line_exp_time;
    very_short_exp_val 	= very_short_it / pIMX675Ctx->one_line_exp_time;

    TRACE(IMX675_DEBUG, "%s: requested IT in lines long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    long_exp_val 		= IMX675_VMAX_3DOL_HDR - long_exp_val;
    short_exp_val 		= rhs1 - short_exp_val;
    very_short_exp_val 	= rhs2 - very_short_exp_val;

    TRACE(IMX675_DEBUG, "%s: requested IT in shr long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    if(long_exp_val < rhs2 + IMX675_SHR0_RHS2_GAP) {
        long_exp_val = rhs2 + IMX675_SHR0_RHS2_GAP;
        long_it = (IMX675_VMAX_3DOL_HDR - long_exp_val) * pIMX675Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX675_DEBUG, "%s: long_exp_val is too long, set to %u, new long_it = %f\n",
        __func__, rhs2 + IMX675_SHR0_RHS2_GAP, long_it);
    } else if(long_exp_val > IMX675_VMAX_3DOL_HDR - IMX675_SHR0_FSC_GAP) {
        long_exp_val = IMX675_VMAX_3DOL_HDR - IMX675_SHR0_FSC_GAP;
        long_it = (IMX675_VMAX_3DOL_HDR - long_exp_val) * pIMX675Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX675_DEBUG, "%s: long_exp_val is too short, set to %u, new long_it = %f\n",
        __func__, IMX675_VMAX_3DOL_HDR - IMX675_SHR0_FSC_GAP, long_it);
    }
    if(short_exp_val < IMX675_3DOL_SHR1_RHS1_GAP) {
        short_exp_val = IMX675_3DOL_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX675Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX675_DEBUG, "%s: short_exp_val is too long, set to %u, new short_it = %f\n",
        __func__, IMX675_3DOL_SHR1_RHS1_GAP, short_it);
    } else if(short_exp_val > rhs1 - IMX675_SHR1_RHS1_GAP) {
        short_exp_val = rhs1 - IMX675_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX675Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX675_DEBUG, "%s: short_exp_val is too short, set to %u, new short_it = %f\n",
        __func__, rhs1 - IMX675_SHR1_RHS1_GAP, short_it);
    }
    if(very_short_exp_val < rhs1 + IMX675_SHR2_RHS1_GAP) {
        very_short_exp_val = rhs1 + IMX675_SHR2_RHS1_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX675Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX675_DEBUG, "%s: very_short_exp_val is too long, set to %u, new very_short_it = %f\n",
        __func__, rhs2 + IMX675_SHR2_RHS1_GAP, very_short_it);
    } else if(very_short_exp_val > rhs2 - IMX675_SHR2_RHS2_GAP) {
        very_short_exp_val = rhs2 - IMX675_SHR2_RHS2_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX675Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX675_DEBUG, "%s: very_short_exp_val is too short, set to %u, new very_short_it = %f\n",
        __func__, rhs2 - IMX675_SHR2_RHS2_GAP, very_short_it);
    }

    // need to use gain to achive ratio / requested gain update
    if(calculate_gain || NewGain != pIMX675Ctx->AecCurGainSEF1) {
        long_gain = (short_it * NewGain * hdr_ratio[0]) / long_it;
        short_gain = NewGain;
        very_short_gain = (short_it * NewGain) / (very_short_it * hdr_ratio[1]);
        TRACE(IMX675_DEBUG, "%s: calculated gain: long: %f, short: %f, very_short: %f\n",
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


RESULT IMX675_Calculate2DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it,
                                    float *o_long_gain, float *o_short_gain,
                                    float *hdr_ratio) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    float long_it = 0.0;
	float short_it = 0.0;
	float long_exp_val = 0.0;
	float short_exp_val = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	bool calculate_gain = false;
	uint32_t rhs1;

    if (pIMX675Ctx == NULL || o_long_it == NULL ||
        o_long_gain == NULL || o_short_gain == NULL ||
        hdr_ratio == NULL) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

	if (pIMX675Ctx->cur_rhs1 == 0) {
		TRACE(IMX675_ERROR, "%s: Invalid parameter (RHS1 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX675Ctx->cur_rhs1;

    TRACE(IMX675_DEBUG, "%s: hdr_ratio[0] = LS Ratio = %f\n", 
    __func__, hdr_ratio[0]);
    
    // Sometimes there is no actual input gain. In that case, we will read it from the sensor
    if (NewGain == 0) {
        TRACE(IMX675_DEBUG, "%s: Input NewGain is 0, reading gain from sensor\n", __func__);
        result = IMX675_IsiGetSEF1GainIss(handle, &NewGain);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // Same for integration time
    if (NewIntegrationTime == 0) {
        TRACE(IMX675_DEBUG, "%s: Input NewIntegrationTime is 0, reading integration time from sensor\n", __func__);
        result = IMX675_IsiGetSEF1IntegrationTimeIss(handle, &NewIntegrationTime);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // assume gain is 1 and see if ratio can be achieved with integration time
    long_it 		= NewIntegrationTime * hdr_ratio[0];
    short_it 		= NewIntegrationTime;
    
    TRACE(IMX675_DEBUG, "%s: requested IT long: %f, short: %f\n", 
    __func__, long_it, short_it);
    long_exp_val 		= long_it / pIMX675Ctx->one_line_exp_time;
    short_exp_val 		= short_it / pIMX675Ctx->one_line_exp_time;

    TRACE(IMX675_DEBUG, "%s: requested IT in lines long: %f, short: %f\n", 
    __func__, long_exp_val, short_exp_val);
    long_exp_val 		= IMX675_VMAX_2DOL_HDR - long_exp_val;
    short_exp_val 		= rhs1 - short_exp_val;

    TRACE(IMX675_DEBUG, "%s: requested IT in shr long: %f, short: %f\n",
    __func__, long_exp_val, short_exp_val);
    if(long_exp_val < rhs1 + IMX675_2DOL_SHR0_RHS1_GAP) {
        long_exp_val = rhs1 + IMX675_2DOL_SHR0_RHS1_GAP;
        long_it = (IMX675_VMAX_2DOL_HDR - long_exp_val) * pIMX675Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX675_DEBUG, "%s: long_exp_val is too long, set to %u, new long_it = %f\n",
        __func__, rhs1 + IMX675_2DOL_SHR0_RHS1_GAP, long_it);
    } else if(long_exp_val > IMX675_VMAX_2DOL_HDR - IMX675_2DOL_SHR0_FSC_GAP) {
        long_exp_val = IMX675_VMAX_2DOL_HDR - IMX675_2DOL_SHR0_FSC_GAP;
        long_it = (IMX675_VMAX_2DOL_HDR - long_exp_val) * pIMX675Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX675_DEBUG, "%s: long_exp_val is too short, set to %u, new long_it = %f\n",
        __func__, IMX675_VMAX_2DOL_HDR - IMX675_2DOL_SHR0_FSC_GAP, long_it);
    }
    if(short_exp_val < IMX675_2DOL_SHR1_MIN_GAP) {
        short_exp_val = IMX675_2DOL_SHR1_MIN_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX675Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX675_DEBUG, "%s: short_exp_val is too long, set to %u, new short_it = %f\n",
        __func__, IMX675_2DOL_SHR1_MIN_GAP, short_it);
    } else if(short_exp_val > rhs1 - IMX675_2DOL_SHR1_RHS1_GAP) {
        short_exp_val = rhs1 - IMX675_2DOL_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX675Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX675_DEBUG, "%s: short_exp_val is too short, set to %u, new short_it = %f\n",
        __func__, rhs1 - IMX675_2DOL_SHR1_RHS1_GAP, short_it);
    }

    // need to use gain to achieve ratio / requested gain update
    if(calculate_gain || NewGain != pIMX675Ctx->AecCurGainSEF1) {
        long_gain = (short_it * NewGain * hdr_ratio[0]) / long_it;
        short_gain = NewGain;
        TRACE(IMX675_DEBUG, "%s: calculated gain: long: %f, short: %f\n",
        __func__, long_gain, short_gain);
    }

    *o_long_it = long_it;
    *o_short_it = short_it;
    *o_long_gain = long_gain;
    *o_short_gain = short_gain;

    return RET_SUCCESS;
}
#endif //BRINGUP_CONFIG

RESULT IMX675_IsiExposureControlIss(IsiSensorHandle_t handle, float NewGain,
                                    float NewIntegrationTime,
                                    uint8_t* pNumberOfFramesToSkip,
                                    float* pSetGain, float* pSetIntegrationTime,
                                    float* hdr_ratio) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;

    RESULT result = RET_SUCCESS;
#ifndef BRINGUP_CONFIG
    float long_it = 0.0;
	float short_it = 0.0;
	float very_short_it = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	float very_short_gain = 1;
    uint32_t hmax;
#endif //BRINGUP_CONFIG

    TRACE(IMX675_INFO, "%s: enter with NewIntegrationTime: %f, NewGain: %f\n",
        __func__, NewIntegrationTime, NewGain);

    if (pIMX675Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pNumberOfFramesToSkip == NULL) || (pSetGain == NULL) ||
        (pSetIntegrationTime == NULL)) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMX675Ctx->enableHdr) {
#ifndef BRINGUP_CONFIG
        result = IMX675_ReadRHS1(handle, &pIMX675Ctx->cur_rhs1);
        result |= IMX675_ReadRHS2(handle, &pIMX675Ctx->cur_rhs2);
        result |= IMX675_ReadHmax(handle, &hmax);
        if (result != RET_SUCCESS) {
            TRACE(IMX675_ERROR, "%s: Read RHS1, RHS2 or HMAX failed\n", __func__);
            return result;
        }

        pIMX675Ctx->SensorMode.ae_info.one_line_exp_time_ns = (uint32_t)(((float)hmax / IMX675_PIXEL_CLK_RATE) * MICRO_2_NANO);
        pIMX675Ctx->one_line_exp_time =
        (float)(pIMX675Ctx->SensorMode.ae_info.one_line_exp_time_ns) / 1000000000;

        if (pIMX675Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
            //2exposure
            result = IMX675_Calculate2DOLExposures(handle, NewIntegrationTime, NewGain,
                                    &long_it, &short_it,
                                    &long_gain, &short_gain,
                                    hdr_ratio);
        } else {
            //3exposure
            result = IMX675_Calculate3DOLExposures(handle, NewIntegrationTime, NewGain,
                                        &long_it, &short_it, &very_short_it,
                                        &long_gain, &short_gain, &very_short_gain,
                                        hdr_ratio);
        }
        
        if (result != RET_SUCCESS) {
            TRACE(IMX675_ERROR, "%s: CalculateHDRExposures failed\n", __func__);
            return result;
        }

		result = IMX675_IsiSetLEFIntegrationTimeIss(handle, long_it,
							pSetIntegrationTime,
							pNumberOfFramesToSkip,
							hdr_ratio);
		result |= IMX675_IsiSetLEFGainIss(handle, long_gain, pSetGain, hdr_ratio);
		result |= IMX675_IsiSetSEF1IntegrationTimeIss(
			handle, short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		result |= IMX675_IsiSetSEF1GainIss(handle, NewIntegrationTime,
						  short_gain, pSetGain, hdr_ratio);
        if (pIMX675Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            result |= IMX675_IsiSetSEF2IntegrationTimeIss(
			    handle, very_short_it, pSetIntegrationTime,
			    pNumberOfFramesToSkip, hdr_ratio);
            result |= IMX675_IsiSetSEF2GainIss(handle, NewIntegrationTime,
                            very_short_gain, pSetGain, hdr_ratio);
        }

        // Recalculate `io_hdr_ratio` according to the set values
        hdr_ratio[0] = (long_it * long_gain) / (short_it * short_gain);
        if (pIMX675Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            hdr_ratio[1] = (short_it * short_gain) / (very_short_it * very_short_gain);
        }

        // Set the output values to SEF1 values
        *pSetGain = short_gain;
        *pSetIntegrationTime = short_it;

        TRACE(IMX675_DEBUG, "%s: actual hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n",
            __func__, hdr_ratio[0], hdr_ratio[1]);
#endif //BRINGUP_CONFIG
    } else {
        result |= IMX675_IsiSetLEFIntegrationTimeIss(handle, NewIntegrationTime,
                                                pSetIntegrationTime,
                                                pNumberOfFramesToSkip, hdr_ratio);
        result |= IMX675_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
    }
    return result;
}

RESULT IMX675_IsiExposureControlExpandedIss(
    IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime,
    float NewIris, uint8_t* pNumberOfFramesToSkip, float* pSetGain,
    float* pSetIntegrationTime, float* pSetIris, float* hdr_ratio) {

    if (pSetIris) {
        IMX675_IsiSetIrisIss(handle, NewIris);
        *pSetIris = NewIris;
    }

    return IMX675_IsiExposureControlIss(handle, NewGain, NewIntegrationTime,
                                        pNumberOfFramesToSkip, pSetGain,
                                        pSetIntegrationTime, hdr_ratio);
}

RESULT IMX675_IsiGetCurrentExposureIss(IsiSensorHandle_t handle,
                                       float* pSetGain,
                                       float* pSetIntegrationTime) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX675Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pSetGain == NULL) || (pSetIntegrationTime == NULL))
        return (RET_NULL_POINTER);

    if (pIMX675Ctx->enableHdr) {
		*pSetGain = pIMX675Ctx->AecCurGainSEF1;
		*pSetIntegrationTime = pIMX675Ctx->AecCurIntegrationTimeSEF1;
	} else {
		*pSetGain = pIMX675Ctx->AecCurGainLEF;
		*pSetIntegrationTime = pIMX675Ctx->AecCurIntegrationTimeLEF;
	}

    return (result);
}

RESULT IMX675_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t* pFps) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX675Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *pFps = pIMX675Ctx->CurrFps;

    return (result);
}

RESULT IMX675_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t Fps) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX675_IsiSetFlickerFpsIss(IsiSensorHandle_t handle, uint32_t flickerMode) {
    RESULT result = RET_SUCCESS;
    // IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    TRACE(IMX675_DEBUG, "%s: set sensor flickerMode = %d\n", __func__, flickerMode);

    // TODO: implement

    TRACE(IMX675_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX675_IsiGetAutoFpsInfoIss(IsiSensorHandle_t handle,
                                   IsiAutoFps_t* pAutoFpsInfo) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX675_IsiGetStartEvIss(IsiSensorHandle_t handle, uint64_t* pStartEv) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX675_IsiGetIspStatusIss(IsiSensorHandle_t handle,
                                 IsiIspStatus_t* pIspStatus) {
    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    if (pIMX675Ctx == NULL || pIMX675Ctx->IsiCtx.HalHandle == NULL) {
        return RET_WRONG_HANDLE;
    }

    pIspStatus->useSensorAE = false;
    pIspStatus->useSensorBLC = false;
    pIspStatus->useSensorAWB = false;

    return RET_SUCCESS;
}

RESULT IMX675_IsiSetTpgIss(IsiSensorHandle_t handle, IsiTpg_t Tpg) {
    RESULT result = RET_SUCCESS;

    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    if (pIMX675Ctx == NULL || pIMX675Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX675Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (Tpg.enable == 0) {
        result = IMX675_IsiWriteRegIss(handle, 0x3253, 0x00);
    } else {
        result = IMX675_IsiWriteRegIss(handle, 0x3253, 0x80);
    }

    pIMX675Ctx->TestPattern = Tpg.enable;

    return (result);
}

RESULT IMX675_IsiGetTpgIss(IsiSensorHandle_t handle, IsiTpg_t* Tpg) {
    RESULT result = RET_SUCCESS;
    uint32_t value = 0;

    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    if (pIMX675Ctx == NULL || pIMX675Ctx->IsiCtx.HalHandle == NULL ||
        Tpg == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX675Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (!IMX675_IsiReadRegIss(handle, 0x5081, &value)) {
        Tpg->enable = ((value & 0x80) != 0) ? 1 : 0;
        if (Tpg->enable) {
            Tpg->pattern = (0xff & value);
        }
        pIMX675Ctx->TestPattern = Tpg->enable;
    }

    return (result);
}

RESULT IMX675_IsiFocusCreateIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX675_IsiFocusReleaseIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX675_IsiFocusGetCalibrateIss(IsiSensorHandle_t handle,
                                      IsiFocusCalibAttr_t* pFocusCalib) {
    return RET_SUCCESS;
}

RESULT IMX675_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

RESULT IMX675_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

// Taken from the IMX681 driver
RESULT IMX675_IsiSetAgainDgainIss(IsiSensorHandle_t handle,
                                  IsiUserGain_t Gain) {
    RESULT result = RET_SUCCESS;
    uint32_t Again = 0, Dgain = 0;
    TRACE(IMX675_INFO, "%s: (enter)\n", __func__);

    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    if (pIMX675Ctx == NULL) {
        TRACE(IMX675_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((Gain.again < 1) | (Gain.again > 16)) {
        TRACE(IMX675_ERROR, "%s: Invalid sensor again\n", __func__);
        return (RET_OUTOFRANGE);
    }
    // Again = (uint32_t)(1024 - (1024/Gain.again));
    Again = (uint32_t)(((2048 * Gain.again) - 2048) / Gain.again);

    result = IMX675_IsiWriteRegIss(handle, 0x309c, (Again & 0x0000FF));
    result = IMX675_IsiWriteRegIss(handle, 0x309d, (Again & 0x00FF00) >> 8);

    if ((Gain.dgain < 1) | (Gain.dgain > 16)) {
        TRACE(IMX675_ERROR, "%s: Invalid sensor dgain\n", __func__);
        return (RET_OUTOFRANGE);
    }
    Dgain = Gain.dgain * 256;

    result = IMX675_IsiWriteRegIss(handle, 0x308c, (Dgain & 0x0000FF));
    result = IMX675_IsiWriteRegIss(handle, 0x308d, (Dgain & 0x00FF00) >> 8);

    // pIMX675Ctx->CurAgain = Gain.again;
    // pIMX675Ctx->CurDgain = Gain.dgain;

    TRACE(IMX675_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX675_IsiGetIrisIss( IsiSensorHandle_t handle,
                                     float *pSetIris ) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX675_INFO, "%s: (enter)\n", __func__);

    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    if (pIMX675Ctx == NULL) {
        TRACE(IMX675_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    //TODO implement getting iris here!
    *pSetIris = 1.0;

    TRACE(IMX675_INFO, "%s: (exit)\n", __func__);
    return (result);
}

// TODO: Should this sensor actually support iris control?
RESULT IMX675_IsiSetIrisIss( IsiSensorHandle_t handle, 
                    const float NewIris) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX675_INFO, "%s: (enter)\n", __func__);

    IMX675_Context_t* pIMX675Ctx = (IMX675_Context_t*)handle;
    if (pIMX675Ctx == NULL) {
        TRACE(IMX675_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (NewIris < IMX675_IRIS_MIN_VAL || NewIris > IMX675_IRIS_MAX_VAL) {
        TRACE(IMX675_ERROR, "%s: Invalid Iris %f\n", __func__, NewIris);
    }

    //TODO implement setting iris here!

    TRACE(IMX675_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX675_IsiGetSensorIss(IsiSensor_t* pIsiSensor) {
    RESULT result = RET_SUCCESS;
    static const char SensorName[16] = "IMX675";
    TRACE(IMX675_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor != NULL) {
        pIsiSensor->pszName = SensorName;
        pIsiSensor->pIsiCreateIss = IMX675_IsiCreateIss;
        pIsiSensor->pIsiReleaseIss = IMX675_IsiReleaseIss;
        pIsiSensor->pIsiReadRegIss = IMX675_IsiReadRegIss;
        pIsiSensor->pIsiWriteRegIss = IMX675_IsiWriteRegIss;
        pIsiSensor->pIsiGetModeIss = IMX675_IsiGetModeIss;
        pIsiSensor->pIsiSetModeIss = IMX675_IsiSetModeIss;
        pIsiSensor->pIsiEnumModeIss = IMX675_IsiEnumModeIss;
        pIsiSensor->pIsiGetCapsIss = IMX675_IsiGetCapsIss;
        pIsiSensor->pIsiSetupIss = IMX675_IsiSetupIss;
        pIsiSensor->pIsiCheckConnectionIss = IMX675_IsiCheckConnectionIss;
        pIsiSensor->pIsiGetRevisionIss = IMX675_IsiGetRevisionIss;
        pIsiSensor->pIsiSetStreamingIss = IMX675_IsiSetStreamingIss;

        /* AEC functions */
        pIsiSensor->pIsiGetGainLimitsIss = 					IMX675_IsiGetGainLimitsIss;
		pIsiSensor->pIsiGetIrisLimitsIss = 					IMX675_IsiGetIrisLimitsIss;
		pIsiSensor->pIsiSetIrisLimitsIss = 					IMX675_IsiSetIrisLimitsIss;
		pIsiSensor->pIsiGetIntegrationTimeLimitsIss =		IMX675_IsiGetIntegrationTimeLimitsIss;

		pIsiSensor->pIsiExposureControlIss =				IMX675_IsiExposureControlIss;
		pIsiSensor->pIsiExposureControlExpandedIss =		IMX675_IsiExposureControlExpandedIss;
		pIsiSensor->pIsiSetIntegrationTimeIss =				IMX675_IsiSetIntegrationTimeIss;

		pIsiSensor->pIsiGetLongIntegrationTimeIss =			IMX675_IsiGetLEFIntegrationTimeIss;
		pIsiSensor->pIsiGetIntegrationTimeIss =				IMX675_IsiGetIntegrationTimeIss;
#ifndef BRINGUP_CONFIG
		pIsiSensor->pIsiGetVSIntegrationTimeIss =			IMX675_IsiGetSEF2IntegrationTimeIss;
#endif //BRINGUP_CONFIG

		pIsiSensor->pIsiGetLongGainIss = 					IMX675_IsiGetLEFGainIss;
		pIsiSensor->pIsiGetGainIss = 						IMX675_IsiGetGainIss;
#ifndef BRINGUP_CONFIG
		pIsiSensor->pIsiGetVSGainIss = 						IMX675_IsiGetSEF2GainIss;
#endif //BRINGUP_CONFIG

		pIsiSensor->pIsiGetGainIncrementIss =				IMX675_IsiGetGainIncrementIss;
		pIsiSensor->pIsiGetIrisIncrementIss =				IMX675_IsiGetIrisIncrementIss;
		pIsiSensor->pIsiGetIntegrationTimeIncrementIss =	IMX675_IsiGetIntegrationTimeIncrementIss;
		pIsiSensor->pIsiSetGainIss = 						IMX675_IsiSetGainIss;
		pIsiSensor->pIsiGetFpsIss = 						IMX675_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss = 						IMX675_IsiSetFpsIss;
        pIsiSensor->pIsiSetFlickerFpsIss = 					IMX675_IsiSetFlickerFpsIss;
        pIsiSensor->pIsiUnlimitFpsIss =						IMX675_IsiUnlimitFpsIss;
        pIsiSensor->pIsiLimitFpsIss =						IMX675_IsiLimitFpsIss;
		pIsiSensor->pIsiGetAutoFpsInfoIss = 				IMX675_IsiGetAutoFpsInfoIss;
		pIsiSensor->pIsiGetStartEvIss = 					IMX675_IsiGetStartEvIss;
        pIsiSensor->pIsiGetIrisIss =						IMX675_IsiGetIrisIss;
        pIsiSensor->pIsiSetIrisIss =						IMX675_IsiSetIrisIss;

        /* SENSOR ISP */
        pIsiSensor->pIsiGetIspStatusIss = IMX675_IsiGetIspStatusIss;
        // pIsiSensor->pIsiSetBlcIss                       =
        // IMX675_IsiSetBlcIss; pIsiSensor->pIsiSetWBIss = IMX675_IsiSetWBIss;

        /* SENSOE OTHER FUNC*/
        pIsiSensor->pIsiSetPowerIss = IMX675_IsiSetPowerIss;
        pIsiSensor->pIsiSetTpgIss = IMX675_IsiSetTpgIss;
        pIsiSensor->pIsiGetTpgIss = IMX675_IsiGetTpgIss;
        // pIsiSensor->pIsiGetExpandCurveIss               =
        // IMX675_IsiGetExpandCurveIss; pIsiSensor->pIsiGetCompressCurveIss =
        // IMX675_IsiGetCompressCurveIss; pIsiSensor->pIsiExtendFuncIss =
        // IMX675_IsiExtendFuncIss; pIsiSensor->pIsiGetOtpDataIss =
        // IMX675_IsiGetOtpDataIss;

        /* AF */
        pIsiSensor->pIsiFocusCreateIss = IMX675_IsiFocusCreateIss;
        pIsiSensor->pIsiFocusReleaseIss = IMX675_IsiFocusReleaseIss;
        pIsiSensor->pIsiFocusGetCalibrateIss = IMX675_IsiFocusGetCalibrateIss;
        pIsiSensor->pIsiFocusSetIss = IMX675_IsiFocusSetIss;
        pIsiSensor->pIsiFocusGetIss = IMX675_IsiFocusGetIss;
        pIsiSensor->pIsiSetAgainDgainIss = IMX675_IsiSetAgainDgainIss;

    } else {
        result = RET_NULL_POINTER;
    }

    TRACE(IMX675_INFO, "%s (exit)\n", __func__);
    return (result);
}

/*****************************************************************************
 * each sensor driver need declare this struct for isi load
 *****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x9012,
    .pIsiHalEnumModeIss = IMX675_IsiHalEnumModeIss,
    .pIsiGetSensorIss = IMX675_IsiGetSensorIss,
};
