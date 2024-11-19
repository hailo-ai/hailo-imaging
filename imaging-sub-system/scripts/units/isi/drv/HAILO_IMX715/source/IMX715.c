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

#include "IMX715_priv.h"
#include "vvsensor.h"

CREATE_TRACER(IMX715_INFO, "IMX715: ", INFO, 1);
CREATE_TRACER(IMX715_WARN, "IMX715: ", WARNING, 1);
CREATE_TRACER(IMX715_ERROR, "IMX715: ", ERROR, 1);
CREATE_TRACER(IMX715_DEBUG, "IMX715: ", INFO, 1);
CREATE_TRACER(IMX715_REG_INFO, "IMX715: ", INFO, 1);
CREATE_TRACER(IMX715_REG_DEBUG, "IMX715: ", INFO, 1);

#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#define IMX715_I2C_ADDR 0x1a
#define IMX715_IRIS_MIN_VAL 1
#define IMX715_IRIS_MAX_VAL 1
#define IMX715_MIN_GAIN_STEP                                          \
    (0.035) /**< min gain step size used by GUI (hardware min = 1/16; \
               1/16..32/16 depending on actual gain ) */
#define IMX715_PLL_PCLK 74250000
#define IMX715_HMAX 0xaec
#define IMX715_VMAX_30FPS 4500
#define IMX715_VMAX_HDR 6750
#define IMX715_VMAX_MAX ((1 << 20) - 2)
#define IMX715_MIN_SHR 3
#define IMX715_MAX_GAIN_AEC                                                    \
    (32.0f) /**< max. gain used by the AEC (arbitrarily chosen, hardware limit \
               = 62.0, driver limit = 32.0 ) */
#define IMX715_VS_MAX_INTEGRATION_TIME (0.0018)
#define IMX715_TRANSFER_BUFFER_LENGTH 3
#define IMX715_MAX_GAIN 3981
#define IMX715_TRANSFER_BUFFER_LENGTH 3
#define IMX715_SHR0_RHS2_GAP 7
#define IMX715_SHR0_FSC_GAP 3
#define IMX715_SHR1_MIN_GAP 7
#define IMX715_SHR1_RHS1_GAP 3
#define IMX715_SHR2_RHS1_GAP 7
#define IMX715_SHR2_RHS2_GAP 3
#define IMX715_PIXEL_CLK_RATE 74.25
#define DEFAULT_RHS1 0x91
#define DEFAULT_RHS2 0xaa
#define MICRO_2_NANO 1000

#define SPI_IOC_MAGIC   'k'
#define HAILO15_IOC_GET_IRIS    _IOR(SPI_IOC_MAGIC, 1, int)
#define HAILO15_IOC_SET_IRIS    _IOW(SPI_IOC_MAGIC, 2, int)


/*****************************************************************************
 *Forward Declarations
*****************************************************************************/
RESULT IMX715_IsiSetIrisIss( IsiSensorHandle_t handle,
                    const float NewIris);

/*****************************************************************************
 *Sensor Info
*****************************************************************************/

static struct vvsensor_mode_s pimx715_mode_info[] = {
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
	.bayer_pattern = BAYER_GBRG,
	.ae_info = {
		.one_line_exp_time_ns = 7410,
		.max_integration_time = IMX715_VMAX_30FPS - IMX715_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX715_MAX_GAIN,
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
		.one_line_exp_time_ns = 7410,
		.max_integration_time = IMX715_VMAX_30FPS - IMX715_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX715_MAX_GAIN,
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
		.bayer_pattern = BAYER_GBRG,
        .ae_info = {
		.one_line_exp_time_ns = 7410,
		.max_integration_time = DEFAULT_RHS1 - IMX715_SHR1_MIN_GAP,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX715_MAX_GAIN,
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
		.bayer_pattern = BAYER_GBRG,
        .ae_info = {
		.one_line_exp_time_ns = 7410,
		.max_integration_time = DEFAULT_RHS1 - IMX715_SHR1_MIN_GAP,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX715_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 20
	}
    }

};

static RESULT IMX715_IsiSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    RESULT result = RET_SUCCESS;

    return (result);
}

static RESULT IMX715_IsiCreateIss(IsiSensorInstanceConfig_t* pConfig) {
    RESULT result = RET_SUCCESS;
    IMX715_Context_t* pIMX715Ctx;
    char i2c_file_path[PATH_MAX];    
    memset(i2c_file_path, 0, PATH_MAX);

    if (!pConfig || !pConfig->pSensor) return (RET_NULL_POINTER);

    pIMX715Ctx = (IMX715_Context_t*)malloc(sizeof(IMX715_Context_t));
    if (!pIMX715Ctx) {
        return (RET_OUTOFMEM);
    }

    MEMSET(pIMX715Ctx, 0, sizeof(IMX715_Context_t));
    result = HalAddRef(pConfig->HalHandle);
    if (result != RET_SUCCESS) {
        free(pIMX715Ctx);
        return (result);
    }

    pIMX715Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pIMX715Ctx->IsiCtx.pSensor = pConfig->pSensor;
    pIMX715Ctx->GroupHold = BOOL_FALSE;
    pIMX715Ctx->OldGain = 1.0;
    pIMX715Ctx->OldIntegrationTime = 0.01;
    pIMX715Ctx->Configured = BOOL_FALSE;
    pIMX715Ctx->Streaming = BOOL_FALSE;
    pIMX715Ctx->TestPattern = BOOL_FALSE;
    pIMX715Ctx->isAfpsRun = BOOL_FALSE;
    pIMX715Ctx->SensorMode.index = pConfig->SensorModeIndex;
    pConfig->SensorModeIndex =
	    MAX(MIN((sizeof(pimx715_mode_info) / sizeof(pimx715_mode_info[0])),
		    pConfig->SensorModeIndex),
		0);
    memcpy(&pIMX715Ctx->SensorMode,
	   &pimx715_mode_info[pConfig->SensorModeIndex],
	   sizeof(pIMX715Ctx->SensorMode));
    pConfig->hSensor = (IsiSensorHandle_t)pIMX715Ctx;
    pIMX715Ctx->pattern = ISI_BPAT_RGRGGBGB;
    pIMX715Ctx->subdev = HalGetFdHandle(pConfig->HalHandle,
                                        HAL_MODULE_SENSOR);  // two sensors??
    pIMX715Ctx->KernelDriverFlag = 1;
    sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cBusNum);
    pIMX715Ctx->i2c_fd = open(i2c_file_path, O_RDWR);
    if (pIMX715Ctx->i2c_fd < 0) {
        TRACE(IMX715_INFO, "unable to open /dev/i2c-%d\n", pConfig->I2cBusNum);
        return RET_FAILURE;
    }

    if (ioctl(pIMX715Ctx->i2c_fd, I2C_SLAVE_FORCE, IMX715_I2C_ADDR) < 0) {
        TRACE(IMX715_INFO, "unable to set I2C_SLAVE_FORCE on /dev/i2c-%d\n",
              pConfig->I2cBusNum);
        return RET_FAILURE;
    }
    return (result);
}

static RESULT IMX715_IsiReleaseIss(IsiSensorHandle_t handle) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX715Ctx == NULL) return (RET_WRONG_HANDLE);

    (void)IMX715_IsiSetStreamingIss(pIMX715Ctx, BOOL_FALSE);
    (void)IMX715_IsiSetPowerIss(pIMX715Ctx, BOOL_FALSE);
    (void)HalDelRef(pIMX715Ctx->IsiCtx.HalHandle);
    close(pIMX715Ctx->i2c_fd);
    MEMSET(pIMX715Ctx, 0, sizeof(IMX715_Context_t));
    free(pIMX715Ctx);
    return (result);
}

static RESULT IMX715_IsiReadRegIss(IsiSensorHandle_t handle,
                                   const uint32_t Addr, uint32_t* pValue) {
    RESULT result = RET_SUCCESS;    
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    char out[IMX715_TRANSFER_BUFFER_LENGTH];

    if (pIMX715Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX715_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    if (write(pIMX715Ctx->i2c_fd, out, sizeof(uint16_t)) != sizeof(uint16_t)) {
        return RET_FAILURE;
    }

    if (read(pIMX715Ctx->i2c_fd, out, 1) != 1) return RET_FAILURE;

    *pValue = out[0];

    return (result);
}

static RESULT IMX715_IsiWriteRegIss(IsiSensorHandle_t handle,
                                    const uint32_t Addr, const uint32_t Value) {
    RESULT result = RET_SUCCESS;
    char out[IMX715_TRANSFER_BUFFER_LENGTH];

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX715_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    out[2] = Value;
    if (write(pIMX715Ctx->i2c_fd, out, sizeof(out)) != sizeof(out))
        result = RET_FAILURE;
    return (result);
}

static RESULT IMX715_UpdateFps(IMX715_Context_t *pIMX715Ctx, uint32_t vmax) {
    float frame_time = 0;
    frame_time = (vmax * pIMX715Ctx->one_line_exp_time);
    if (frame_time == 0) return RET_FAILURE;

    pIMX715Ctx->CurrFps = (uint32_t)(ceil(1 / frame_time));
    return RET_SUCCESS;
}

static RESULT IMX715_ReadVmax(IsiSensorHandle_t handle, uint32_t* vmax) {
    uint32_t vmax_low = 0, vmax_mid = 0, vmax_high = 0;
    RESULT result;

    result = IMX715_IsiReadRegIss(handle, 0x3024, &vmax_low);
    result |= IMX715_IsiReadRegIss(handle, 0x3025, &vmax_mid);
    result |= IMX715_IsiReadRegIss(handle, 0x3026, &vmax_high);
    if (result) return RET_FAILURE;

    *vmax = (vmax_high << 16) | (vmax_mid << 8) | vmax_low;
    return result;
}

static RESULT IMX715_ReadHmax(IsiSensorHandle_t handle, uint32_t* hmax) {
    uint32_t hmax_low = 0, hmax_high = 0;
    RESULT result;

    result = IMX715_IsiReadRegIss(handle, 0x3028, &hmax_low);
    result |= IMX715_IsiReadRegIss(handle, 0x3029, &hmax_high);
    if (result) return RET_FAILURE;

    *hmax = (hmax_high << 8) | hmax_low;
    return result;
}

static RESULT IMX715_WriteVmax(IsiSensorHandle_t handle, uint32_t vmax) {
    RESULT result;
    result = IMX715_IsiWriteRegIss(handle, 0x3024, vmax & 0xff);
    result |= IMX715_IsiWriteRegIss(handle, 0x3025, (vmax >> 8) & 0xff);
    result |= IMX715_IsiWriteRegIss(handle, 0x3026, (vmax >> 16) & 0x0f);
    if (!result) {
        return IMX715_UpdateFps((IMX715_Context_t *)handle, vmax);
    }

    return result;
}

static RESULT IMX715_ReadRHS1(IsiSensorHandle_t handle, uint32_t* rhs1) {
    uint32_t rhs1_low = 0, rhs1_mid = 0, rhs1_high = 0;
    RESULT result;

    result = IMX715_IsiReadRegIss(handle, 0x3060, &rhs1_low);
    result |= IMX715_IsiReadRegIss(handle, 0x3061, &rhs1_mid);
    result |= IMX715_IsiReadRegIss(handle, 0x3062, &rhs1_high);
    if (result) return RET_FAILURE;

    *rhs1 = (rhs1_high << 16) | (rhs1_mid << 8) | rhs1_low;
    return result;
}

static RESULT IMX715_ReadRHS2(IsiSensorHandle_t handle, uint32_t* rhs2) {
    uint32_t rhs2_low = 0, rhs2_mid = 0, rhs2_high = 0;
    RESULT result;

    result = IMX715_IsiReadRegIss(handle, 0x3064, &rhs2_low);
    result |= IMX715_IsiReadRegIss(handle, 0x3065, &rhs2_mid);
    result |= IMX715_IsiReadRegIss(handle, 0x3066, &rhs2_high);
    if (result) return RET_FAILURE;

    *rhs2 = (rhs2_high << 16) | (rhs2_mid << 8) | rhs2_low;
    return result;
}

static RESULT IMX715_WriteShr0(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;
    result = IMX715_IsiWriteRegIss(handle, 0x3050, shr & 0xff);
    result |= IMX715_IsiWriteRegIss(handle, 0x3051, (shr >> 8) & 0xff);
    result |= IMX715_IsiWriteRegIss(handle, 0x3052, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX715_WriteShr1(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3054, shr & 0xff);
    result |= IMX715_IsiWriteRegIss(handle, 0x3055, (shr >> 8) & 0xff);
    result |= IMX715_IsiWriteRegIss(handle, 0x3056, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX715_WriteShr2(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3058, shr & 0xff);
    result |= IMX715_IsiWriteRegIss(handle, 0x3059, (shr >> 8) & 0xff);
    result |= IMX715_IsiWriteRegIss(handle, 0x305a, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX715_WriteGain(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3090, (gain & 0x00ff));
	result |= IMX715_IsiWriteRegIss(handle, 0x3091, (gain & 0x0700) >> 8);

    return result;
}

static RESULT IMX715_WriteGain1(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3092, (gain & 0x00ff));
	result |= IMX715_IsiWriteRegIss(handle, 0x3093, (gain & 0x0700) >> 8);

    return result;
}

static RESULT IMX715_WriteGain2(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3094, (gain & 0x00ff));
	result |= IMX715_IsiWriteRegIss(handle, 0x3095, (gain & 0x0700) >> 8);

    return result;
}

static RESULT IMX715_LockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3001, 0x1);

    return result;
}

static RESULT IMX715_UnlockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3001, 0x0);

    return result;
}

static RESULT IMX715_IsiGetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    TRACE(IMX715_INFO, "%s (enter)\n", __func__);
    
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    memcpy(pMode, &(pIMX715Ctx->SensorMode), sizeof(pIMX715Ctx->SensorMode));

    TRACE(IMX715_INFO, "%s (exit)\n", __func__);
    return (RET_SUCCESS);
}

static RESULT IMX715_IsiSetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    int ret = 0;
    TRACE(IMX715_INFO, "%s (enter)\n", __func__);

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX715Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, pMode);
    if (ret != 0) {
        return (RET_FAILURE);
    }

    return (RET_SUCCESS);
}

static RESULT IMX715_IsiHalEnumModeIss(HalHandle_t HalHandle,
                                       IsiEnumMode_t* pEnumMode) {
    HalContext_t* pHalCtx = HalHandle;
    if (pHalCtx == NULL) {
        return RET_NULL_POINTER;
    }

    if (pEnumMode->index >=
        (sizeof(pimx715_mode_info) / sizeof(pimx715_mode_info[0])))
        return RET_OUTOFRANGE;

    for (uint32_t i = 0;
         i < (sizeof(pimx715_mode_info) / sizeof(pimx715_mode_info[0])); i++) {
        if (pimx715_mode_info[i].index == pEnumMode->index) {
            memcpy(&pEnumMode->mode, &pimx715_mode_info[i], sizeof(IsiMode_t));            
            TRACE(IMX715_INFO, "%s (exit)\n", __func__);
            return RET_SUCCESS;
        }
    }

    return RET_NOTSUPP;
}

static RESULT IMX715_IsiEnumModeIss(IsiSensorHandle_t handle,
                                    IsiEnumMode_t* pEnumMode) {
    RESULT result = RET_SUCCESS;
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL || pIMX715Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    HalContext_t* pHalCtx = (HalContext_t*)pIMX715Ctx->IsiCtx.HalHandle;
    result = IMX715_IsiHalEnumModeIss(pHalCtx, pEnumMode);
    if (result != RET_SUCCESS) {
        TRACE(IMX715_ERROR, "%s: sensor enum mode error!\n", __func__);
        return (RET_FAILURE);
    }

    return result;
}

static RESULT IMX715_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t* pCaps) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;

    RESULT result = RET_SUCCESS;

    TRACE(IMX715_INFO, "%s (enter)\n", __func__);

    if (pIMX715Ctx == NULL) return (RET_WRONG_HANDLE);

    if (pCaps == NULL) {
        return (RET_NULL_POINTER);
    }

    if (!pIMX715Ctx->Configured) IMX715_IsiSetupIss(handle, pCaps);

    pCaps->BusWidth = pIMX715Ctx->SensorMode.bit_width;
    pCaps->Mode = ISI_MODE_BAYER;
    pCaps->FieldSelection = ISI_FIELDSEL_BOTH;
    pCaps->YCSequence = ISI_YCSEQ_YCBYCR;
    pCaps->Conv422 = ISI_CONV422_COSITED;
    pCaps->BPat = pIMX715Ctx->SensorMode.bayer_pattern;
    pCaps->HPol = ISI_HPOL_REFPOS;
    pCaps->VPol = ISI_VPOL_POS;
    pCaps->Edge = ISI_EDGE_RISING;
    pCaps->Resolution.width = pIMX715Ctx->SensorMode.size.width;
    pCaps->Resolution.height = pIMX715Ctx->SensorMode.size.height;
    pCaps->SmiaMode = ISI_SMIA_OFF;
    pCaps->MipiLanes = ISI_MIPI_4LANES;

    if (pCaps->BusWidth == 10) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_10;
    } else if (pCaps->BusWidth == 12) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_12;
    } else {
        pCaps->MipiMode = ISI_MIPI_OFF;
    }
    TRACE(IMX715_INFO, "got caps - width %d height %d buswidth %d\n",
          pCaps->Resolution.width, pCaps->Resolution.height, pCaps->BusWidth);
    TRACE(IMX715_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX715_AecSetModeParameters(IMX715_Context_t* pIMX715Ctx,
                                          const IsiCaps_t* pConfig) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX715_INFO, "%s%s: (enter)\n", __func__,
          pIMX715Ctx->isAfpsRun ? "(AFPS)" : "");

    pIMX715Ctx->AecIntegrationTimeIncrement = pIMX715Ctx->one_line_exp_time;
    pIMX715Ctx->AecMinIntegrationTime =
        pIMX715Ctx->one_line_exp_time * pIMX715Ctx->MinIntegrationLine;
    pIMX715Ctx->AecMaxIntegrationTime =
        pIMX715Ctx->one_line_exp_time * pIMX715Ctx->MaxIntegrationLine;

    TRACE(IMX715_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __func__,
          pIMX715Ctx->isAfpsRun ? "(AFPS)" : "",
          pIMX715Ctx->AecMaxIntegrationTime);

    pIMX715Ctx->AecGainIncrement = IMX715_MIN_GAIN_STEP;

    // reflects the state of the sensor registers, must equal default settings
    pIMX715Ctx->AecCurGainLEF = pIMX715Ctx->AecMinGain;
	pIMX715Ctx->AecCurGainSEF1 = pIMX715Ctx->AecMinGain;
	pIMX715Ctx->AecCurGainSEF2 = pIMX715Ctx->AecMinGain;
	pIMX715Ctx->AecCurIntegrationTimeLEF = pIMX715Ctx->AecMaxIntegrationTime;
	pIMX715Ctx->AecCurIntegrationTimeSEF1 = pIMX715Ctx->AecMaxIntegrationTime;
	pIMX715Ctx->AecCurIntegrationTimeSEF2 = pIMX715Ctx->AecMaxIntegrationTime;
    pIMX715Ctx->OldGain = 1;
    pIMX715Ctx->OldIntegrationTime = 0.0f;

    TRACE(IMX715_INFO, "%s%s: (exit)\n", __func__,
          pIMX715Ctx->isAfpsRun ? "(AFPS)" : "");

    return (result);
}

static RESULT IMX715_IsiSetupIss(IsiSensorHandle_t handle,
                                 const IsiCaps_t* pCaps) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX715_INFO, "%s (enter)\n", __func__);

    if (!pIMX715Ctx) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pCaps == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid configuration (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMX715Ctx->Streaming != BOOL_FALSE) {
        return RET_WRONG_STATE;
    }

    if (pIMX715Ctx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        pIMX715Ctx->enableHdr = true;
    } else {
        pIMX715Ctx->enableHdr = false;
    }

    pIMX715Ctx->one_line_exp_time =
        (float)(pIMX715Ctx->SensorMode.ae_info.one_line_exp_time_ns) /
        1000000000;
    pIMX715Ctx->MaxIntegrationLine =
        pIMX715Ctx->SensorMode.ae_info.max_integration_time;
    TRACE(IMX715_INFO, "%s: MaxIntegrationLine %u\n", __func__,
          pIMX715Ctx->MaxIntegrationLine);
    pIMX715Ctx->MinIntegrationLine =
        pIMX715Ctx->SensorMode.ae_info.min_integration_time;
    pIMX715Ctx->gain_accuracy = pIMX715Ctx->SensorMode.ae_info.gain_accuracy;
    pIMX715Ctx->AecMaxGain = (float)(pIMX715Ctx->SensorMode.ae_info.max_gain) /
                             pIMX715Ctx->gain_accuracy;
    pIMX715Ctx->AecMinGain = (float)(pIMX715Ctx->SensorMode.ae_info.min_gain) /
                             pIMX715Ctx->gain_accuracy;

    pIMX715Ctx->AecMinIris = IMX715_IRIS_MIN_VAL;
    pIMX715Ctx->AecMaxIris = IMX715_IRIS_MAX_VAL;

    pIMX715Ctx->original_vmax = 0;
    pIMX715Ctx->unlimit_fps = 0;

    pIMX715Ctx->MaxFps = pIMX715Ctx->SensorMode.fps;
    pIMX715Ctx->CurrFps = pIMX715Ctx->MaxFps;
    TRACE(IMX715_INFO, "%s - got caps - width %d height %d buswidth %d\n",
          __func__, pIMX715Ctx->SensorMode.size.width,
          pIMX715Ctx->SensorMode.size.height, pIMX715Ctx->SensorMode.bit_width);

    TRACE(IMX715_INFO, "%s - MinGain %f MaxGain %f\n", __func__, pIMX715Ctx->AecMinGain,
          pIMX715Ctx->AecMaxGain);

    memcpy(&pIMX715Ctx->CapsConfig, pCaps, sizeof(pIMX715Ctx->CapsConfig));

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful,
     * bits 6..0 are reserved, reset bit is not sticky */
    TRACE(IMX715_DEBUG, "%s: IMX715 System-Reset executed\n", __func__);
    osSleep(100);

    result = IMX715_AecSetModeParameters(pIMX715Ctx, pCaps);
    if (result != RET_SUCCESS) {
        TRACE(IMX715_ERROR, "%s: SetupOutputWindow failed.\n", __func__);
        return (result);
    }

    pIMX715Ctx->Configured = BOOL_TRUE;
    TRACE(IMX715_INFO, "%s: (exit)\n", __func__);
    return 0;
}

static RESULT IMX715_IsiCheckConnectionIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    return (result);
}

// Not tested
static RESULT IMX715_IsiGetRevisionIss(IsiSensorHandle_t handle,
                                       uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    int ret = 0;
    uint32_t reg_val;
    uint32_t sensor_id;

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL || pIMX715Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX715Ctx->IsiCtx.HalHandle;

    if (!pValue) return (RET_NULL_POINTER);

    if (pIMX715Ctx->KernelDriverFlag) {
        ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, &sensor_id);
        if (ret != 0) {
            TRACE(IMX715_ERROR, "%s: Read Sensor ID Error! \n", __func__);
            return (RET_FAILURE);
        }
    } else {
        reg_val = 0;
        result = IMX715_IsiReadRegIss(handle, 0x3a04, &reg_val);
        sensor_id = (reg_val & 0xff) << 8;

        reg_val = 0;
        result |= IMX715_IsiReadRegIss(handle, 0x3a05, &reg_val);
        sensor_id |= (reg_val & 0xff);
    }

    *pValue = sensor_id;
    TRACE(IMX715_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX715_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX715_INFO, "%s (enter)\n", __func__);

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    pIMX715Ctx->Streaming = on;

    if (pIMX715Ctx->enableHdr)
        return result;

    TRACE(IMX715_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX715_IsiGetGainLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinGain, float* pMaxGain) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX715_INFO, "%s: (enter)\n", __func__);    

    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinGain == NULL) || (pMaxGain == NULL)) {
        TRACE(IMX715_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinGain = pIMX715Ctx->AecMinGain;
    *pMaxGain = pIMX715Ctx->AecMaxGain;

    TRACE(IMX715_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX715_IsiUnlimitFpsIss(IsiSensorHandle_t handle,
                                      float maxIntegrationTime) {
    RESULT result = RET_SUCCESS;
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;

    if (!pIMX715Ctx) {
        return RET_NULL_POINTER;
    }

    if (pIMX715Ctx->enableHdr) {
        return result;
    }
    pIMX715Ctx->unlimit_fps = 1;

    if (maxIntegrationTime < 0)
        pIMX715Ctx->MaxIntegrationLine = IMX715_VMAX_MAX - IMX715_MIN_SHR;
    else
        pIMX715Ctx->MaxIntegrationLine =
            MIN((uint32_t)(maxIntegrationTime / pIMX715Ctx->one_line_exp_time),
                IMX715_VMAX_MAX - IMX715_MIN_SHR);
    TRACE(IMX715_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
        pIMX715Ctx->MaxIntegrationLine);
    pIMX715Ctx->AecMaxIntegrationTime =
        pIMX715Ctx->one_line_exp_time * pIMX715Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX715_IsiLimitFpsIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    uint32_t current_vmax = 0;
    
    TRACE(IMX715_INFO, "%s (enter)\n", __func__);

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (!pIMX715Ctx) 
        return RET_NULL_POINTER;

    if (pIMX715Ctx->enableHdr)
        return result;
    
    // Working around overriding VMAX issue with reset(?) value
    if (pIMX715Ctx->Streaming == BOOL_FALSE)
        return result;

    IMX715_ReadVmax(handle, &current_vmax);

    pIMX715Ctx->unlimit_fps = 0;
    if (pIMX715Ctx->original_vmax == 0) {
        pIMX715Ctx->original_vmax = current_vmax;
    }
    if (current_vmax != pIMX715Ctx->original_vmax) {
        IMX715_WriteVmax(handle, pIMX715Ctx->original_vmax);
    }

    pIMX715Ctx->MaxIntegrationLine =
        MAX(pIMX715Ctx->original_vmax - IMX715_MIN_SHR, 1);
    TRACE(IMX715_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
          pIMX715Ctx->MaxIntegrationLine);
    pIMX715Ctx->AecMaxIntegrationTime =
        pIMX715Ctx->one_line_exp_time * pIMX715Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX715_IsiGetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinIris, float* pMaxIris) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX715_INFO, "%s: (enter)\n", __func__);

    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIris == NULL) || (pMaxIris == NULL)) {
        TRACE(IMX715_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIris = pIMX715Ctx->AecMinIris;
    *pMaxIris = pIMX715Ctx->AecMaxIris;

    TRACE(IMX715_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX715_IsiSetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float minIris, float maxIris) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX715_INFO, "%s: (enter)\n", __func__);

    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    pIMX715Ctx->AecMinIris = minIris;
    pIMX715Ctx->AecMaxIris = maxIris;

    TRACE(IMX715_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX715_IsiGetIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float* pMinIntegrationTime,
    float* pMaxIntegrationTime) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX715_INFO, "%s: (enter)\n", __func__);
    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX715_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIntegrationTime = pIMX715Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMX715Ctx->AecMaxIntegrationTime;
    TRACE(IMX715_INFO, "%s: (exit) %f, %f\n", 
    __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

/* Gain get functions*/

RESULT IMX715_IsiGetGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	TRACE(IMX715_INFO, "%s: (enter)\n", __func__);

	if (pIMX715Ctx == NULL) {
		TRACE(IMX715_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	if (pIMX715Ctx->enableHdr)
		return IMX715_IsiGetSEF1GainIss(handle, pSetGain);

	return IMX715_IsiGetLEFGainIss(handle, pSetGain);
}

RESULT IMX715_IsiGetLEFGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	TRACE(IMX715_INFO, "%s: (enter)\n", __func__);

	if (pIMX715Ctx == NULL) {
		TRACE(IMX715_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}
	*pSetGain = pIMX715Ctx->AecCurGainLEF;
	TRACE(IMX715_DEBUG, "%s - returning %f\n", __func__, pIMX715Ctx->AecCurGainLEF);
	TRACE(IMX715_INFO, "%s: (exit)\n", __func__);
	return (result);
}

// HDR has not been tested yet
RESULT IMX715_IsiGetSEF1GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX715_INFO, "%s: (enter)\n", __func__);

	if (pIMX715Ctx == NULL) {
		TRACE(IMX715_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX715Ctx->AecCurGainSEF1;
	TRACE(IMX715_DEBUG, "%s - returning %f\n", __func__, pIMX715Ctx->AecCurGainSEF1);

	TRACE(IMX715_INFO, "%s: (exit)\n", __func__);

	return (result);
}

// HDR has not been tested yet
RESULT IMX715_IsiGetSEF2GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX715_INFO, "%s: (enter)\n", __func__);

	if (pIMX715Ctx == NULL) {
		TRACE(IMX715_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX715Ctx->AecCurGainSEF2;
	TRACE(IMX715_DEBUG, "%s - returning %f\n", __func__, pIMX715Ctx->AecCurGainSEF2);

	TRACE(IMX715_INFO, "%s: (exit)\n", __func__);

	return (result);
}

RESULT IMX715_IsiGetGainIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX715_INFO, "%s: (enter)\n", __func__);

    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = pIMX715Ctx->AecGainIncrement;

    TRACE(IMX715_INFO, "%s: (exit)\n", __func__);

    return (result);
}

RESULT IMX715_IsiGetIrisIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX715_INFO, "%s: (enter)\n", __func__);
    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = 0.0001;

    TRACE(IMX715_INFO, "%s: (exit)\n", __func__);

    return (result);
}

/* Gain set functions*/

RESULT IMX715_IsiSetGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;    
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	if (pIMX715Ctx == NULL || pIMX715Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	if (pIMX715Ctx->enableHdr) {
		result = IMX715_IsiSetSEF1GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);

		result |= IMX715_IsiSetSEF2GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);
	}
	result |= IMX715_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
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

RESULT IMX715_IsiSetLEFGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	if (pIMX715Ctx == NULL || pIMX715Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}    
    
	TRACE(IMX715_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX715_DEBUG, "%s: writting 0x%x to GAIN\n", __func__, Gain);

    result |= IMX715_LockRegHold(handle);
	result |= IMX715_WriteGain(handle, Gain);
	result |= IMX715_UnlockRegHold(handle);

	if (result != 0) {
		return RET_FAILURE;
	}

	pIMX715Ctx->AecCurGainLEF = _sensorGain2linear(Gain);
	*pSetGain = pIMX715Ctx->AecCurGainLEF;
	TRACE(IMX715_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

RESULT IMX715_IsiSetSEF1GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	if (!pIMX715Ctx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);

	TRACE(IMX715_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX715_DEBUG, "%s: writting 0x%x to GAIN1\n", __func__, Gain);

    result |= IMX715_LockRegHold(handle);
	result |= IMX715_WriteGain1(handle, Gain);
    result |= IMX715_UnlockRegHold(handle);

	pIMX715Ctx->AecCurGainSEF1 = _sensorGain2linear(Gain);
	*pSetGain = pIMX715Ctx->AecCurGainSEF1;

	TRACE(IMX715_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

RESULT IMX715_IsiSetSEF2GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX715Ctx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);
	
	TRACE(IMX715_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX715_DEBUG, "%s: writting 0x%x to GAIN2\n", __func__, Gain);

    result |= IMX715_LockRegHold(handle);
	result |= IMX715_WriteGain2(handle, Gain);
    result |= IMX715_UnlockRegHold(handle);

	pIMX715Ctx->AecCurGainSEF2 = _sensorGain2linear(Gain);
	*pSetGain = pIMX715Ctx->AecCurGainSEF2;

	TRACE(IMX715_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

/* Integration Time get functions*/

RESULT IMX715_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;

	if (!pIMX715Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	
	TRACE(IMX715_DEBUG, "%s - enter\n", __func__);
	
	if (pIMX715Ctx->enableHdr)
		return IMX715_IsiGetSEF1IntegrationTimeIss(handle, pSetIntegrationTime);
	
	return IMX715_IsiGetLEFIntegrationTimeIss(handle, pSetIntegrationTime);
}

RESULT IMX715_IsiGetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;


	if (!pIMX715Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX715Ctx->AecCurIntegrationTimeLEF;
	TRACE(IMX715_DEBUG, "%s - returning %f\n", __func__, pIMX715Ctx->AecCurIntegrationTimeLEF);
	return (result);
}

// HDR has not been tested yet
RESULT IMX715_IsiGetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX715Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX715Ctx->AecCurIntegrationTimeSEF1;
	TRACE(IMX715_DEBUG, "%s - returning %f\n", __func__, pIMX715Ctx->AecCurIntegrationTimeSEF1);
	return (result);
}

// HDR has not been tested yet
RESULT IMX715_IsiGetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX715Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);

	*pSetIntegrationTime = pIMX715Ctx->AecCurIntegrationTimeSEF2;
	TRACE(IMX715_DEBUG, "%s - returning %f\n", __func__, pIMX715Ctx->AecCurIntegrationTimeSEF2);
	return (result);
}

RESULT IMX715_IsiGetIntegrationTimeIncrementIss(IsiSensorHandle_t handle,
                                                float* pIncr) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (!pIMX715Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pIncr) return (RET_NULL_POINTER);

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders
    //in
    // the application)
    *pIncr = pIMX715Ctx->AecIntegrationTimeIncrement;
    return (result);
}

/* Integration Time set functions*/

RESULT IMX715_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;

	if (!pIMX715Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX715Ctx->enableHdr) {
		return IMX715_IsiSetSEF1IntegrationTimeIss(
			handle, NewIntegrationTime, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
	}

	return IMX715_IsiSetLEFIntegrationTimeIss(
		handle, NewIntegrationTime, pSetIntegrationTime,
		pNumberOfFramesToSkip, hdr_ratio);
}

RESULT IMX715_IsiSetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;

    int shr = 0;
    uint32_t exp = 0;
    uint32_t new_vmax = 0;
    uint32_t vmax_updated = 0;
    uint32_t current_vmax = 0;
    uint32_t rhs2;
    if (!pIMX715Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pSetIntegrationTime || !pNumberOfFramesToSkip) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMX715Ctx->Streaming) {
        TRACE(IMX715_ERROR, "%s: sensor not streaming\n", __func__);
        return RET_FAILURE;
    }
    exp = NewIntegrationTime / pIMX715Ctx->one_line_exp_time;

    TRACE(IMX715_DEBUG, "%s: set AEC_PK_EXPO=0x%05x\n", __func__, exp);

    if (fabs(NewIntegrationTime - pIMX715Ctx->AecCurIntegrationTimeLEF) > FLT_EPSILON) {

        if (pIMX715Ctx->enableHdr){
            if (pIMX715Ctx->cur_rhs1 == 0 || pIMX715Ctx->cur_rhs2 == 0) {
                TRACE(IMX715_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
                return (RET_WRONG_CONFIG);
            }

            rhs2 = pIMX715Ctx->cur_rhs2;
            new_vmax = IMX715_VMAX_HDR;
            exp = new_vmax - exp;
            exp = exp > rhs2 + IMX715_SHR0_RHS2_GAP ? exp : rhs2 + IMX715_SHR0_RHS2_GAP;
            exp = exp < new_vmax - IMX715_SHR0_FSC_GAP? exp : new_vmax - IMX715_SHR0_FSC_GAP;
            shr = exp;
		} else {
            if (exp > pIMX715Ctx->MaxIntegrationLine || exp == 0) {
                TRACE(IMX715_ERROR, "%s: Integration time %f (exp %u) out of range (%u)\n", __func__,
                    NewIntegrationTime, exp, pIMX715Ctx->MaxIntegrationLine);
                return RET_FAILURE;
            }
            result = IMX715_ReadVmax(handle, &current_vmax);

            if (pIMX715Ctx->original_vmax == 0) {                
                pIMX715Ctx->original_vmax = current_vmax;
            }

            if (pIMX715Ctx->original_vmax - IMX715_MIN_SHR >
                pIMX715Ctx->MaxIntegrationLine) {
                pIMX715Ctx->MaxIntegrationLine =
                    pIMX715Ctx->original_vmax - IMX715_MIN_SHR;
                TRACE(IMX715_DEBUG, "%s: set MaxIntegrationLine to %u\n", __func__,
                      pIMX715Ctx->MaxIntegrationLine);
                pIMX715Ctx->AecMaxIntegrationTime =
                    pIMX715Ctx->one_line_exp_time * pIMX715Ctx->MaxIntegrationLine;
                TRACE(IMX715_DEBUG, "%s: set AecMaxIntegrationTime to %f\n", __func__,
                      pIMX715Ctx->AecMaxIntegrationTime);
            }

            shr = current_vmax - exp;

            if (shr < IMX715_MIN_SHR) {
                new_vmax = MIN(exp + IMX715_MIN_SHR,
                            pIMX715Ctx->MaxIntegrationLine + IMX715_MIN_SHR);
                shr = IMX715_MIN_SHR;
                vmax_updated = 1;
            } else if (shr > IMX715_MIN_SHR &&
                    current_vmax > pIMX715Ctx->original_vmax) {
                new_vmax = MAX(current_vmax - shr + IMX715_MIN_SHR,
                            pIMX715Ctx->original_vmax);
                shr = new_vmax - exp;
                vmax_updated = 1;
            } else {
                new_vmax = current_vmax;
            }
        }

        result |= IMX715_LockRegHold(handle);
        if (vmax_updated && pIMX715Ctx->unlimit_fps && !pIMX715Ctx->enableHdr) {
            result = IMX715_WriteVmax(handle, new_vmax);
        }

        TRACE(IMX715_DEBUG, "%s - writing 0x%x to SHR0\n", __func__, shr);
        result |= IMX715_WriteShr0(handle, shr);
        result |= IMX715_UnlockRegHold(handle);

        float configuredIntegrationTime =
            (new_vmax - shr) * pIMX715Ctx->one_line_exp_time;
        pIMX715Ctx->OldIntegrationTime = configuredIntegrationTime;
        pIMX715Ctx->AecCurIntegrationTimeLEF = configuredIntegrationTime;

        *pNumberOfFramesToSkip = 1U;
    } else {
        *pNumberOfFramesToSkip = 0U;  // no frame skip
    }

    *pSetIntegrationTime = pIMX715Ctx->AecCurIntegrationTimeLEF;
    TRACE(IMX715_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTime);
    return (result);
}

RESULT IMX715_IsiSetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF1,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;

	if (!pIMX715Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX715Ctx->cur_rhs1 == 0 || pIMX715Ctx->cur_rhs2 == 0) {
		TRACE(IMX715_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX715Ctx->cur_rhs1;

	if (!pSetIntegrationTimeSEF1 || !pNumberOfFramesToSkip) {
		printf("%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX715_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = (NewIntegrationTime / pIMX715Ctx->one_line_exp_time);
	TRACE(IMX715_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX715Ctx->AecCurIntegrationTimeSEF1) > FLT_EPSILON) {
		exp = rhs1 - exp;
		exp = exp > IMX715_SHR1_MIN_GAP ? exp : IMX715_SHR1_MIN_GAP;
		exp = exp < rhs1 - IMX715_SHR1_RHS1_GAP ? exp : rhs1 - IMX715_SHR1_RHS1_GAP;
		TRACE(IMX715_DEBUG, "%s - writing 0x%x to SHR1\n", __func__, exp);

		result |= IMX715_LockRegHold(handle);
		result |= IMX715_WriteShr1(handle, exp);
		result |= IMX715_UnlockRegHold(handle);

		pIMX715Ctx->AecCurIntegrationTimeSEF1 = (rhs1 - exp) * pIMX715Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF1 = pIMX715Ctx->AecCurIntegrationTimeSEF1;

	TRACE(IMX715_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF1);
	return (result);
}

RESULT IMX715_IsiSetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF2,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;
	uint32_t rhs2;

	if (pIMX715Ctx->cur_rhs1 == 0 || pIMX715Ctx->cur_rhs2 == 0) {
		printf("%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX715Ctx->cur_rhs1;
	rhs2 = pIMX715Ctx->cur_rhs2;

	if (!pIMX715Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTimeSEF2 || !pNumberOfFramesToSkip) {
		printf("%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX715_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = (NewIntegrationTime / pIMX715Ctx->one_line_exp_time);
	TRACE(IMX715_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX715Ctx->AecCurIntegrationTimeSEF2) > FLT_EPSILON) {
		exp = rhs2 - exp;
		exp = exp > rhs1 + IMX715_SHR2_RHS1_GAP ? exp : rhs1 + IMX715_SHR2_RHS1_GAP;
		exp = exp < rhs2 - IMX715_SHR2_RHS2_GAP ? exp : rhs2 - IMX715_SHR2_RHS2_GAP;
		TRACE(IMX715_DEBUG, "%s - writing 0x%x to SHR2\n", __func__, exp);

		result |= IMX715_LockRegHold(handle);
		result |= IMX715_WriteShr2(handle, exp);
		result |= IMX715_UnlockRegHold(handle);

		pIMX715Ctx->AecCurIntegrationTimeSEF2 = (rhs2 - exp) * pIMX715Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF2 = pIMX715Ctx->AecCurIntegrationTimeSEF2;

	TRACE(IMX715_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF2);
	return (result);
}

RESULT IMX715_CalculateHDRExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it, float *o_very_short_it,
                                    float *o_long_gain, float *o_short_gain, float *o_very_short_gain,
                                    float *hdr_ratio) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
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

    if (pIMX715Ctx == NULL || o_long_it == NULL || o_short_it == NULL ||
        o_very_short_it == NULL || o_long_gain == NULL || o_short_gain == NULL ||
        o_very_short_gain == NULL || hdr_ratio == NULL) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

	if (pIMX715Ctx->cur_rhs1 == 0 || pIMX715Ctx->cur_rhs2 == 0) {
		TRACE(IMX715_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX715Ctx->cur_rhs1;
	rhs2 = pIMX715Ctx->cur_rhs2;

    TRACE(IMX715_DEBUG, "%s: hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n", 
    __func__, hdr_ratio[0], hdr_ratio[1]);
    
    // Sometimes there is no actual input gain. In that case, we will read it from the sensor
    if (NewGain == 0) {
        TRACE(IMX715_DEBUG, "%s: Input NewGain is 0, reading gain from sensor\n", __func__);
        result = IMX715_IsiGetSEF1GainIss(handle, &NewGain);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // Same for integration time
    if (NewIntegrationTime == 0) {
        TRACE(IMX715_DEBUG, "%s: Input NewIntegrationTime is 0, reading integration time from sensor\n", __func__);
        result = IMX715_IsiGetSEF1IntegrationTimeIss(handle, &NewIntegrationTime);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // assume gain is 1 and see if ratio can be achieved with integration time
    long_it 		= NewIntegrationTime * hdr_ratio[0];
    short_it 		= NewIntegrationTime;
    very_short_it 	= NewIntegrationTime / hdr_ratio[1];
    
    TRACE(IMX715_DEBUG, "%s: requested IT long: %f, short: %f, very_short: %f\n", 
    __func__, long_it, short_it, very_short_it);
    long_exp_val 		= long_it / pIMX715Ctx->one_line_exp_time;
    short_exp_val 		= short_it / pIMX715Ctx->one_line_exp_time;
    very_short_exp_val 	= very_short_it / pIMX715Ctx->one_line_exp_time;

    TRACE(IMX715_DEBUG, "%s: requested IT in lines long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    long_exp_val 		= IMX715_VMAX_HDR - long_exp_val;
    short_exp_val 		= rhs1 - short_exp_val;
    very_short_exp_val 	= rhs2 - very_short_exp_val;

    TRACE(IMX715_DEBUG, "%s: requested IT in shr long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    if(long_exp_val < rhs2 + IMX715_SHR0_RHS2_GAP) {
        long_exp_val = rhs2 + IMX715_SHR0_RHS2_GAP;
        long_it = (IMX715_VMAX_HDR - long_exp_val) * pIMX715Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX715_DEBUG, "%s: long_exp_val is too long, set to %u, new long_it = %f\n",
        __func__, rhs2 + IMX715_SHR0_RHS2_GAP, long_it);
    } else if(long_exp_val > IMX715_VMAX_HDR - IMX715_SHR0_FSC_GAP) {
        long_exp_val = IMX715_VMAX_HDR - IMX715_SHR0_FSC_GAP;
        long_it = (IMX715_VMAX_HDR - long_exp_val) * pIMX715Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX715_DEBUG, "%s: long_exp_val is too short, set to %u, new long_it = %f\n",
        __func__, IMX715_VMAX_HDR - IMX715_SHR0_FSC_GAP, long_it);
    }
    if(short_exp_val < IMX715_SHR1_MIN_GAP) {
        short_exp_val = IMX715_SHR1_MIN_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX715Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX715_DEBUG, "%s: short_exp_val is too long, set to %u, new short_it = %f\n",
        __func__, IMX715_SHR1_MIN_GAP, short_it);
    } else if(short_exp_val > rhs1 - IMX715_SHR1_RHS1_GAP) {
        short_exp_val = rhs1 - IMX715_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX715Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX715_DEBUG, "%s: short_exp_val is too short, set to %u, new short_it = %f\n",
        __func__, rhs1 - IMX715_SHR1_RHS1_GAP, short_it);
    }
    if(very_short_exp_val < rhs1 + IMX715_SHR2_RHS1_GAP) {
        very_short_exp_val = rhs1 + IMX715_SHR2_RHS1_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX715Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX715_DEBUG, "%s: very_short_exp_val is too long, set to %u, new very_short_it = %f\n",
        __func__, rhs2 + IMX715_SHR2_RHS1_GAP, very_short_it);
    } else if(very_short_exp_val > rhs2 - IMX715_SHR2_RHS2_GAP) {
        very_short_exp_val = rhs2 - IMX715_SHR2_RHS2_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX715Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX715_DEBUG, "%s: very_short_exp_val is too short, set to %u, new very_short_it = %f\n",
        __func__, rhs2 - IMX715_SHR2_RHS2_GAP, very_short_it);
    }

    // need to use gain to achive ratio / requested gain update
    if(calculate_gain || NewGain != pIMX715Ctx->AecCurGainSEF1) {
        long_gain = (short_it * NewGain * hdr_ratio[0]) / long_it;
        short_gain = NewGain;
        very_short_gain = (short_it * NewGain) / (very_short_it * hdr_ratio[1]);
        TRACE(IMX715_DEBUG, "%s: calculated gain: long: %f, short: %f, very_short: %f\n",
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

RESULT IMX715_IsiExposureControlIss(IsiSensorHandle_t handle, float NewGain,
                                    float NewIntegrationTime,
                                    uint8_t* pNumberOfFramesToSkip,
                                    float* pSetGain, float* pSetIntegrationTime,
                                    float* hdr_ratio) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;    
    RESULT result = RET_SUCCESS;
    float long_it = 0.0;
	float short_it = 0.0;
	float very_short_it = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	float very_short_gain = 1;
    uint32_t hmax;
    TRACE(IMX715_INFO, "%s: enter with NewIntegrationTime: %f, NewGain: %f\n",
        __func__, NewIntegrationTime, NewGain);
    if (pIMX715Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }        
    if ((pNumberOfFramesToSkip == NULL) || (pSetGain == NULL) ||
        (pSetIntegrationTime == NULL)) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    // HDR has not been tested yet
    if (pIMX715Ctx->enableHdr) {
        result = IMX715_ReadRHS1(handle, &pIMX715Ctx->cur_rhs1);
        result |= IMX715_ReadRHS2(handle, &pIMX715Ctx->cur_rhs2);
        result |= IMX715_ReadHmax(handle, &hmax);
        if (result != RET_SUCCESS) {
            TRACE(IMX715_ERROR, "%s: Read RHS1, RHS2 or HMAX failed\n", __func__);
            return result;
        }

        pIMX715Ctx->SensorMode.ae_info.max_integration_time = pIMX715Ctx->cur_rhs1 - IMX715_SHR1_MIN_GAP;
        pIMX715Ctx->SensorMode.ae_info.one_line_exp_time_ns = (uint32_t)(((float)hmax / IMX715_PIXEL_CLK_RATE) * MICRO_2_NANO);
        pIMX715Ctx->one_line_exp_time =
        (float)(pIMX715Ctx->SensorMode.ae_info.one_line_exp_time_ns) / 1000000000;

        result = IMX715_CalculateHDRExposures(handle, NewIntegrationTime, NewGain,
                                    &long_it, &short_it, &very_short_it,
                                    &long_gain, &short_gain, &very_short_gain,
                                    hdr_ratio);
        if (result != RET_SUCCESS) {
            TRACE(IMX715_ERROR, "%s: CalculateHDRExposures failed\n", __func__);
            return result;
        }

		result = IMX715_IsiSetLEFIntegrationTimeIss(handle, long_it,
							pSetIntegrationTime,
							pNumberOfFramesToSkip,
							hdr_ratio);
		result |= IMX715_IsiSetLEFGainIss(handle, long_gain, pSetGain, hdr_ratio);        
		result |= IMX715_IsiSetSEF1IntegrationTimeIss(
			handle, short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);

		result |= IMX715_IsiSetSEF1GainIss(handle, NewIntegrationTime,
						  short_gain, pSetGain, hdr_ratio);
		result |= IMX715_IsiSetSEF2IntegrationTimeIss(
			handle, very_short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		result |= IMX715_IsiSetSEF2GainIss(handle, NewIntegrationTime,
						  very_short_gain, pSetGain, hdr_ratio);

        // Recalculate `io_hdr_ratio` according to the set values
        hdr_ratio[0] = (long_it * long_gain) / (short_it * short_gain);
        hdr_ratio[1] = (short_it * short_gain) / (very_short_it * very_short_gain);

        // Set the output values to SEF1 values
        *pSetGain = short_gain;
        *pSetIntegrationTime = short_it;

        TRACE(IMX715_DEBUG, "%s: actual hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n",
        __func__, hdr_ratio[0], hdr_ratio[1]);
    } else {
        result |= IMX715_IsiSetLEFIntegrationTimeIss(handle, NewIntegrationTime,
                                                pSetIntegrationTime,
                                                pNumberOfFramesToSkip, hdr_ratio);
        result |= IMX715_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
    }
    return result;
}

RESULT IMX715_IsiExposureControlExpandedIss(
    IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime,
    float NewIris, uint8_t* pNumberOfFramesToSkip, float* pSetGain,
    float* pSetIntegrationTime, float* pSetIris, float* hdr_ratio) {

    if (pSetIris) {
        IMX715_IsiSetIrisIss(handle, NewIris);
        *pSetIris = NewIris;
    }

    return IMX715_IsiExposureControlIss(handle, NewGain, NewIntegrationTime,
                                        pNumberOfFramesToSkip, pSetGain,
                                        pSetIntegrationTime, hdr_ratio);
}

RESULT IMX715_IsiGetCurrentExposureIss(IsiSensorHandle_t handle,
                                       float* pSetGain,
                                       float* pSetIntegrationTime) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX715Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pSetGain == NULL) || (pSetIntegrationTime == NULL))
        return (RET_NULL_POINTER);

    if (pIMX715Ctx->enableHdr) {
		*pSetGain = pIMX715Ctx->AecCurGainSEF1;
		*pSetIntegrationTime = pIMX715Ctx->AecCurIntegrationTimeSEF1;
	} else {
		*pSetGain = pIMX715Ctx->AecCurGainLEF;
		*pSetIntegrationTime = pIMX715Ctx->AecCurIntegrationTimeLEF;
	}

    return (result);
}

RESULT IMX715_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t* pFps) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX715Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *pFps = pIMX715Ctx->CurrFps;

    return (result);
}

RESULT IMX715_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t Fps) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX715_IsiSetFlickerFpsIss(IsiSensorHandle_t handle, uint32_t flickerMode) {
    RESULT result = RET_SUCCESS;
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;

    TRACE(IMX715_DEBUG, "%s: set sensor flickerMode = %d\n", __func__, flickerMode);

    if (!pIMX715Ctx) {
        return RET_NULL_POINTER;
    }
    return result;
}

RESULT IMX715_IsiGetAutoFpsInfoIss(IsiSensorHandle_t handle,
                                   IsiAutoFps_t* pAutoFpsInfo) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX715_IsiGetStartEvIss(IsiSensorHandle_t handle, uint64_t* pStartEv) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX715_IsiGetIspStatusIss(IsiSensorHandle_t handle,
                                 IsiIspStatus_t* pIspStatus) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL || pIMX715Ctx->IsiCtx.HalHandle == NULL) {
        return RET_WRONG_HANDLE;
    }

    pIspStatus->useSensorAE = false;
    pIspStatus->useSensorBLC = false;
    pIspStatus->useSensorAWB = false;

    return RET_SUCCESS;
}

RESULT IMX715_IsiSetTpgIss(IsiSensorHandle_t handle, IsiTpg_t Tpg) {
    RESULT result = RET_SUCCESS;

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL || pIMX715Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX715Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (Tpg.enable == 0) {
        result = IMX715_IsiWriteRegIss(handle, 0x3253, 0x00);
    } else {
        result = IMX715_IsiWriteRegIss(handle, 0x3253, 0x80);
    }

    pIMX715Ctx->TestPattern = Tpg.enable;

    return (result);
}

RESULT IMX715_IsiGetTpgIss(IsiSensorHandle_t handle, IsiTpg_t* Tpg) {
    RESULT result = RET_SUCCESS;
    uint32_t value = 0;

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL || pIMX715Ctx->IsiCtx.HalHandle == NULL ||
        Tpg == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX715Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (!IMX715_IsiReadRegIss(handle, 0x5081, &value)) {
        Tpg->enable = ((value & 0x80) != 0) ? 1 : 0;
        if (Tpg->enable) {
            Tpg->pattern = (0xff & value);
        }
        pIMX715Ctx->TestPattern = Tpg->enable;
    }

    return (result);
}

RESULT IMX715_IsiFocusCreateIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX715_IsiFocusReleaseIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX715_IsiFocusGetCalibrateIss(IsiSensorHandle_t handle,
                                      IsiFocusCalibAttr_t* pFocusCalib) {
    return RET_SUCCESS;
}

RESULT IMX715_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

RESULT IMX715_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

// Taken from the IMX681 driver
RESULT IMX715_IsiSetAgainDgainIss(IsiSensorHandle_t handle,
                                  IsiUserGain_t Gain) {
    RESULT result = RET_SUCCESS;
    uint32_t Again = 0, Dgain = 0;
    TRACE(IMX715_INFO, "%s: (enter)\n", __func__);

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((Gain.again < 1) | (Gain.again > 16)) {
        TRACE(IMX715_ERROR, "%s: Invalid sensor again\n", __func__);
        return (RET_OUTOFRANGE);
    }
    // Again = (uint32_t)(1024 - (1024/Gain.again));
    Again = (uint32_t)(((2048 * Gain.again) - 2048) / Gain.again);

    result = IMX715_IsiWriteRegIss(handle, 0x309c, (Again & 0x0000FF));
    result = IMX715_IsiWriteRegIss(handle, 0x309d, (Again & 0x00FF00) >> 8);

    if ((Gain.dgain < 1) | (Gain.dgain > 16)) {
        TRACE(IMX715_ERROR, "%s: Invalid sensor dgain\n", __func__);
        return (RET_OUTOFRANGE);
    }
    Dgain = Gain.dgain * 256;

    result = IMX715_IsiWriteRegIss(handle, 0x308c, (Dgain & 0x0000FF));
    result = IMX715_IsiWriteRegIss(handle, 0x308d, (Dgain & 0x00FF00) >> 8);

    TRACE(IMX715_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX715_IsiGetIrisIss( IsiSensorHandle_t handle, float *pSetIris ) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX715_INFO, "%s: (enter)\n", __func__);

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    //TODO implement getting iris here!
    *pSetIris = 1.0;

    TRACE(IMX715_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX715_IsiSetIrisIss( IsiSensorHandle_t handle, float NewIris) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX715_INFO, "%s: (enter)\n", __func__);

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (NewIris < IMX715_IRIS_MIN_VAL || NewIris > IMX715_IRIS_MAX_VAL) {
        TRACE(IMX715_ERROR, "%s: Invalid Iris %f\n", __func__, NewIris);
    }

    //TODO implement setting iris here!

    TRACE(IMX715_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX715_IsiGetSensorIss(IsiSensor_t* pIsiSensor) {
    RESULT result = RET_SUCCESS;
    static const char SensorName[16] = "IMX715";
    TRACE(IMX715_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor != NULL) {
        pIsiSensor->pszName = SensorName;
        pIsiSensor->pIsiCreateIss = IMX715_IsiCreateIss;
        pIsiSensor->pIsiReleaseIss = IMX715_IsiReleaseIss;
        pIsiSensor->pIsiReadRegIss = IMX715_IsiReadRegIss;
        pIsiSensor->pIsiWriteRegIss = IMX715_IsiWriteRegIss;
        pIsiSensor->pIsiGetModeIss = IMX715_IsiGetModeIss;
        pIsiSensor->pIsiSetModeIss = IMX715_IsiSetModeIss;
        pIsiSensor->pIsiEnumModeIss = IMX715_IsiEnumModeIss;
        pIsiSensor->pIsiGetCapsIss = IMX715_IsiGetCapsIss;
        pIsiSensor->pIsiSetupIss = IMX715_IsiSetupIss;
        pIsiSensor->pIsiCheckConnectionIss = IMX715_IsiCheckConnectionIss;
        pIsiSensor->pIsiGetRevisionIss = IMX715_IsiGetRevisionIss;
        pIsiSensor->pIsiSetStreamingIss = IMX715_IsiSetStreamingIss;

        /* AEC functions */
        pIsiSensor->pIsiGetGainLimitsIss = 					IMX715_IsiGetGainLimitsIss;
		pIsiSensor->pIsiGetIrisLimitsIss = 					IMX715_IsiGetIrisLimitsIss;
		pIsiSensor->pIsiSetIrisLimitsIss = 					IMX715_IsiSetIrisLimitsIss;
		pIsiSensor->pIsiGetIntegrationTimeLimitsIss =		IMX715_IsiGetIntegrationTimeLimitsIss;

		pIsiSensor->pIsiExposureControlIss =				IMX715_IsiExposureControlIss;
		pIsiSensor->pIsiExposureControlExpandedIss =		IMX715_IsiExposureControlExpandedIss;
		pIsiSensor->pIsiSetIntegrationTimeIss =				IMX715_IsiSetIntegrationTimeIss;

		pIsiSensor->pIsiGetLongIntegrationTimeIss =			IMX715_IsiGetLEFIntegrationTimeIss;
		pIsiSensor->pIsiGetIntegrationTimeIss =				IMX715_IsiGetIntegrationTimeIss;
		pIsiSensor->pIsiGetVSIntegrationTimeIss =			IMX715_IsiGetSEF2IntegrationTimeIss;

		pIsiSensor->pIsiGetLongGainIss = 					IMX715_IsiGetLEFGainIss;
		pIsiSensor->pIsiGetGainIss = 						IMX715_IsiGetGainIss;
		pIsiSensor->pIsiGetVSGainIss = 						IMX715_IsiGetSEF2GainIss;

		pIsiSensor->pIsiGetGainIncrementIss =				IMX715_IsiGetGainIncrementIss;
		pIsiSensor->pIsiGetIrisIncrementIss =				IMX715_IsiGetIrisIncrementIss;
		pIsiSensor->pIsiGetIntegrationTimeIncrementIss =	IMX715_IsiGetIntegrationTimeIncrementIss;
		pIsiSensor->pIsiSetGainIss = 						IMX715_IsiSetGainIss;
		pIsiSensor->pIsiGetFpsIss = 						IMX715_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss = 						IMX715_IsiSetFpsIss;
        pIsiSensor->pIsiSetFlickerFpsIss = 					IMX715_IsiSetFlickerFpsIss;
        pIsiSensor->pIsiUnlimitFpsIss =						IMX715_IsiUnlimitFpsIss;
        pIsiSensor->pIsiLimitFpsIss =						IMX715_IsiLimitFpsIss;
		pIsiSensor->pIsiGetAutoFpsInfoIss = 				IMX715_IsiGetAutoFpsInfoIss;
		pIsiSensor->pIsiGetStartEvIss = 					IMX715_IsiGetStartEvIss;
        pIsiSensor->pIsiGetIrisIss =						IMX715_IsiGetIrisIss;
        pIsiSensor->pIsiSetIrisIss =						IMX715_IsiSetIrisIss;

        /* SENSOR ISP */
        pIsiSensor->pIsiGetIspStatusIss = IMX715_IsiGetIspStatusIss;
        // pIsiSensor->pIsiSetBlcIss                       =
        // IMX715_IsiSetBlcIss; pIsiSensor->pIsiSetWBIss = IMX715_IsiSetWBIss;

        /* SENSOR OTHER FUNC*/
        pIsiSensor->pIsiSetPowerIss = IMX715_IsiSetPowerIss;
        pIsiSensor->pIsiSetTpgIss = IMX715_IsiSetTpgIss;
        pIsiSensor->pIsiGetTpgIss = IMX715_IsiGetTpgIss;
        // pIsiSensor->pIsiGetExpandCurveIss               =
        // IMX715_IsiGetExpandCurveIss; pIsiSensor->pIsiGetCompressCurveIss =
        // IMX715_IsiGetCompressCurveIss; pIsiSensor->pIsiExtendFuncIss =
        // IMX715_IsiExtendFuncIss; pIsiSensor->pIsiGetOtpDataIss =
        // IMX715_IsiGetOtpDataIss;

        /* AF */
        pIsiSensor->pIsiFocusCreateIss = IMX715_IsiFocusCreateIss;
        pIsiSensor->pIsiFocusReleaseIss = IMX715_IsiFocusReleaseIss;
        pIsiSensor->pIsiFocusGetCalibrateIss = IMX715_IsiFocusGetCalibrateIss;
        pIsiSensor->pIsiFocusSetIss = IMX715_IsiFocusSetIss;
        pIsiSensor->pIsiFocusGetIss = IMX715_IsiFocusGetIss;
        pIsiSensor->pIsiSetAgainDgainIss = IMX715_IsiSetAgainDgainIss;

    } else {
        result = RET_NULL_POINTER;
    }

    TRACE(IMX715_INFO, "%s (exit)\n", __func__);
    return (result);
}

/*****************************************************************************
 * each sensor driver need declare this struct for isi load
 *****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x9012,
    .pIsiHalEnumModeIss = IMX715_IsiHalEnumModeIss,
    .pIsiGetSensorIss = IMX715_IsiGetSensorIss,
};
