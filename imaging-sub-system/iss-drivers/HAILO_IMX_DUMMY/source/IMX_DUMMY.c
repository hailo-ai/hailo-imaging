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

#include "IMX_dummy_priv.h"
#include "vvsensor.h"

CREATE_TRACER(IMX_DUMMY_INFO, "IMX_DUMMY: ", INFO, 1)
CREATE_TRACER(IMX_DUMMY_WARN, "IMX_DUMMY: ", WARNING, 1)
CREATE_TRACER(IMX_DUMMY_ERROR, "IMX_DUMMY: ", ERROR, 1)
CREATE_TRACER(IMX_DUMMY_DEBUG, "IMX_DUMMY: ", INFO, 1)
CREATE_TRACER(IMX_DUMMY_REG_INFO, "IMX_DUMMY: ", INFO, 1)
CREATE_TRACER(IMX_DUMMY_REG_DEBUG, "IMX_DUMMY: ", INFO, 1)

#define LOG_ERROR(...) TRACE(IMX_DUMMY_ERROR, __VA_ARGS__)

#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

/** @defgroup group_defines  API definitions
 *  @{
 */

#define IMX_DUMMY_I2C_ADDR 0x1a
#define IMX_DUMMY_IRIS_MIN_VAL 1

/** @} */ // end of group_defines

#define IMX_DUMMY_IRIS_MAX_VAL 1
#define IMX_DUMMY_MIN_GAIN_STEP                                          \
    (0.035) /**< min gain step size used by GUI (hardware min = 1/16; \
               1/16..32/16 depending on actual gain ) */
#define IMX_DUMMY_PLL_PCLK 74250000
#define IMX_DUMMY_HMAX 0xaec
#define IMX_DUMMY_VMAX_30FPS 4500
#define IMX_DUMMY_VMAX_3DOL_HDR 6750
#define IMX_DUMMY_VMAX_2DOL_HDR 4500
#define IMX_DUMMY_VMAX_MAX 135000
#define IMX_DUMMY_MIN_SHR 3
#define IMX_DUMMY_MAX_GAIN_AEC                                                    \
    (32.0f) /**< max. gain used by the AEC (arbitrarily chosen, hardware limit \
               = 62.0, driver limit = 32.0 ) */
#define IMX_DUMMY_VS_MAX_INTEGRATION_TIME (0.0018)
#define IMX_DUMMY_TRANSFER_BUFFER_LENGTH 3
#define IMX_DUMMY_MAX_GAIN 3981
#define IMX_DUMMY_SHR0_RHS2_GAP 7
#define IMX_DUMMY_2DOL_SHR0_RHS1_GAP 5
#define IMX_DUMMY_SHR0_FSC_GAP 3
#define IMX_DUMMY_2DOL_SHR0_FSC_GAP 2
#define IMX_DUMMY_3DOL_SHR1_RHS1_GAP 7
#define IMX_DUMMY_2DOL_SHR1_MIN_GAP 5
#define IMX_DUMMY_SHR1_RHS1_GAP 3
#define IMX_DUMMY_2DOL_SHR1_RHS1_GAP 5
#define IMX_DUMMY_SHR2_RHS1_GAP 7
#define IMX_DUMMY_SHR2_RHS2_GAP 3
#define IMX_DUMMY_PIXEL_CLK_RATE 74.25
#define DEFAULT_RHS1_3DOL 0x1F3
#define DEFAULT_RHS2_3DOL 0x230
#define DEFAULT_RHS1_2DOL 0xa3
#define MICRO_2_NANO 1000
#define IMX_DUMMY_2DOL_NUM_EXP 2
#define IMX_DUMMY_3DOL_NUM_EXP 3
/* When required long EV is within this many lines of max_long_it, use unity gain to avoid quantization noise. */
#define IMX_DUMMY_2DOL_MAX_IT_NEAR_LINES 2

FlickerModePeaksPerSec flickerPeaksPerSecMap[] = {
    { ISI_AE_ANTIBANDING_MODE_OFF, 0 },
    { ISI_AE_ANTIBANDING_MODE_50HZ, 100 },
    { ISI_AE_ANTIBANDING_MODE_60HZ, 120 },
    { ISI_AE_ANTIBANDING_MODE_AUTO, 100 }
};

/*****************************************************************************
 *Forward Declarations
*****************************************************************************/
RESULT IMX_Dummy_IsiSetIrisIss( IsiSensorHandle_t handle,
                    const float NewIris);

/*****************************************************************************
 *Sensor Info
*****************************************************************************/

static struct vvsensor_mode_s pimx_dummy_mode_info[] = {
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
		.max_integration_time = IMX_DUMMY_VMAX_30FPS - IMX_DUMMY_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX_DUMMY_MAX_GAIN,
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
		.max_integration_time = IMX_DUMMY_VMAX_30FPS - IMX_DUMMY_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX_DUMMY_MAX_GAIN,
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
		.max_integration_time = DEFAULT_RHS1_3DOL - IMX_DUMMY_3DOL_SHR1_RHS1_GAP,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX_DUMMY_MAX_GAIN,
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
		.max_integration_time = DEFAULT_RHS1_3DOL - IMX_DUMMY_3DOL_SHR1_RHS1_GAP,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX_DUMMY_MAX_GAIN,
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
		.fps       = 30 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_L_AND_S,
		.bit_width = 12,
		.bayer_pattern = BAYER_RGGB,
        .ae_info = {
		.one_line_exp_time_ns = 7407,
		.max_integration_time = DEFAULT_RHS1_2DOL - IMX_DUMMY_2DOL_SHR1_RHS1_GAP,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX_DUMMY_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
    {
        .index     = 5,
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
		.one_line_exp_time_ns = 7407,
		.max_integration_time = IMX_DUMMY_VMAX_30FPS - IMX_DUMMY_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX_DUMMY_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
    {
        .index     = 6,
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
		.max_integration_time = IMX_DUMMY_VMAX_30FPS - IMX_DUMMY_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX_DUMMY_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
    {
        .index     = 7,
        .size      ={
			.bounds_width  = 2688,
			.bounds_height = 1520,
			.top           = 0,
			.left          = 0,
			.width         = 2688   ,
			.height        = 1520,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_RGGB,
	.ae_info = {
		.one_line_exp_time_ns = 7410,
		.max_integration_time = IMX_DUMMY_VMAX_30FPS - IMX_DUMMY_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX_DUMMY_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    }

};

static RESULT IMX_Dummy_IsiSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    RESULT result = RET_SUCCESS;

    return (result);
}

static RESULT IMX_Dummy_IsiCreateIss(IsiSensorInstanceConfig_t* pConfig) {
    RESULT result = RET_SUCCESS;
    IMX_dummy_Context_t* pIMXDummyCtx;
    char i2c_file_path[PATH_MAX];

    memset(i2c_file_path, 0, PATH_MAX);

    if (!pConfig || !pConfig->pSensor) return (RET_NULL_POINTER);

    pIMXDummyCtx = (IMX_dummy_Context_t*)malloc(sizeof(IMX_dummy_Context_t));
    if (!pIMXDummyCtx) {
        return (RET_OUTOFMEM);
    }

    MEMSET(pIMXDummyCtx, 0, sizeof(IMX_dummy_Context_t));
    pIMXDummyCtx->i2c_fd = -1;
    pIMXDummyCtx->i2c_af_fd = -1;

    result = HalAddRef(pConfig->HalHandle);
    if (result != RET_SUCCESS) {
        goto error_free_ctx;
    }

    pIMXDummyCtx->IsiCtx.HalHandle = pConfig->HalHandle;
    pIMXDummyCtx->IsiCtx.pSensor = pConfig->pSensor;
    pIMXDummyCtx->GroupHold = BOOL_FALSE;
    pIMXDummyCtx->OldGain = 1.0;
    pIMXDummyCtx->OldIntegrationTime = 0.01;
    pIMXDummyCtx->Configured = BOOL_FALSE;
    pIMXDummyCtx->Streaming = BOOL_FALSE;
    pIMXDummyCtx->TestPattern = BOOL_FALSE;
    pIMXDummyCtx->isAfpsRun = BOOL_FALSE;
    pIMXDummyCtx->hcg_factor = 1.0f;
    pIMXDummyCtx->SensorMode.index = pConfig->SensorModeIndex;
    pConfig->SensorModeIndex =
	    MAX(MIN((sizeof(pimx_dummy_mode_info) / sizeof(pimx_dummy_mode_info[0])),
		    pConfig->SensorModeIndex),
		0);
    memcpy(&pIMXDummyCtx->SensorMode,
	   &pimx_dummy_mode_info[pConfig->SensorModeIndex],
	   sizeof(pIMXDummyCtx->SensorMode));
    pConfig->hSensor = (IsiSensorHandle_t)pIMXDummyCtx;
    pIMXDummyCtx->pattern = ISI_BPAT_RGRGGBGB;
    pIMXDummyCtx->subdev = HalGetFdHandle(pConfig->HalHandle,
                                        HAL_MODULE_SENSOR);  // two sensors??
    pIMXDummyCtx->KernelDriverFlag = 1;
    TRACE(IMX_DUMMY_INFO, "%s - sensor i2c bus: %d, af i2c bus: %d, sensor i2c addr: 0x%x, af i2c addr: 0x%x\n", __func__,
           pConfig->I2cBusNum, pConfig->I2cAfBusNum, pConfig->SlaveAddr, pConfig->SlaveAfAddr);
    pIMXDummyCtx->i2c_addr = pConfig->SlaveAddr;
    pIMXDummyCtx->i2c_af_addr = pConfig->SlaveAfAddr;

    sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cBusNum);
    pIMXDummyCtx->i2c_fd = open(i2c_file_path, O_RDWR);
    if (pIMXDummyCtx->i2c_fd < 0) {
        TRACE(IMX_DUMMY_ERROR, "%s: unable to open /dev/i2c-%d, errno=%d (%s)\n",
              __func__, pConfig->I2cBusNum, errno, strerror(errno));
        goto error_hal_del_ref;
    }

    if (ioctl(pIMXDummyCtx->i2c_fd, I2C_SLAVE_FORCE, pIMXDummyCtx->i2c_addr) < 0) {
        TRACE(IMX_DUMMY_ERROR, "%s: unable to set I2C_SLAVE_FORCE on /dev/i2c-%d, errno=%d (%s)\n",
              __func__, pConfig->I2cBusNum, errno, strerror(errno));
        goto error_close_i2c_fd;
    }

    if (pConfig->I2cAfBusNum < 0) {
        TRACE(IMX_DUMMY_INFO, "no af i2c bus\n");
        pIMXDummyCtx->SensorMode.af_mode = ISI_SENSOR_AF_MODE_NOTSUPP;
    } else {
        sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cAfBusNum);
        pIMXDummyCtx->i2c_af_fd = open(i2c_file_path, O_RDWR);
        if (pIMXDummyCtx->i2c_af_fd < 0) {
            TRACE(IMX_DUMMY_ERROR, "%s: unable to open /dev/i2c-%d, errno=%d (%s)\n",
                  __func__, pConfig->I2cAfBusNum, errno, strerror(errno));
            goto error_close_i2c_fd;
        }
        pIMXDummyCtx->SensorMode.af_mode = ISI_SENSOR_AF_MODE_CDAF;
    }

    return (result);

error_close_i2c_fd:
    close(pIMXDummyCtx->i2c_fd);
error_hal_del_ref:
    (void)HalDelRef(pIMXDummyCtx->IsiCtx.HalHandle);
error_free_ctx:
    free(pIMXDummyCtx);
    pIMXDummyCtx = NULL;
    return RET_FAILURE;
}

static RESULT IMX_Dummy_IsiReleaseIss(IsiSensorHandle_t handle) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMXDummyCtx == NULL) return (RET_WRONG_HANDLE);

    result = IMX_Dummy_IsiSetStreamingIss(pIMXDummyCtx, BOOL_FALSE);
    CHECK_RESULT_RET(result, "SetStreaming off");
    result = IMX_Dummy_IsiSetPowerIss(pIMXDummyCtx, BOOL_FALSE);
    CHECK_RESULT_RET(result, "SetPower off");
    (void)HalDelRef(pIMXDummyCtx->IsiCtx.HalHandle);
    if (pIMXDummyCtx->i2c_af_fd >= 0) {
        close(pIMXDummyCtx->i2c_af_fd);
    }
    close(pIMXDummyCtx->i2c_fd);
    MEMSET(pIMXDummyCtx, 0, sizeof(IMX_dummy_Context_t));
    free(pIMXDummyCtx);
    pIMXDummyCtx = NULL;
    return (result);
}

static RESULT IMX_Dummy_IsiReadRegIss(IsiSensorHandle_t handle,
                                   const uint32_t Addr, uint32_t* pValue) {

    if (pValue)
        *pValue = 0;
    return RET_SUCCESS;
}

static RESULT IMX_Dummy_IsiWriteRegIss(IsiSensorHandle_t handle,
                                    const uint32_t Addr, const uint32_t Value) {
    return RET_SUCCESS;
}

static RESULT IMX_Dummy_UpdateFps(IMX_dummy_Context_t *pIMXDummyCtx, uint32_t vmax) {
    float frame_time = 0;
    frame_time = (vmax * pIMXDummyCtx->one_line_exp_time);
    if (frame_time == 0) {
        TRACE(IMX_DUMMY_ERROR, "%s: frame_time is 0, cannot update FPS\n", __func__);
        return RET_FAILURE;
    }

    pIMXDummyCtx->CurrFps = (uint32_t)(ceil(1 / frame_time));
    return RET_SUCCESS;
}

static RESULT IMX_Dummy_ReadVmax(IsiSensorHandle_t handle, uint32_t* vmax) {

    *vmax = IMX_DUMMY_VMAX_30FPS;
    return RET_SUCCESS;
}

static RESULT IMX_Dummy_ReadHmax(IsiSensorHandle_t handle, uint32_t* hmax) {

    *hmax = IMX_DUMMY_HMAX;
    return RET_SUCCESS;
}

static RESULT IMX_Dummy_WriteVmax(IsiSensorHandle_t handle, uint32_t vmax) {
    RESULT result;

    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3028, vmax & 0xff);
    CHECK_RESULT_RET(result, "WriteVmax low");
    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3029, (vmax >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteVmax mid");
    result = IMX_Dummy_IsiWriteRegIss(handle, 0x302a, (vmax >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteVmax high");

    result = IMX_Dummy_UpdateFps((IMX_dummy_Context_t *)handle, vmax);
    CHECK_RESULT_RET(result, "UpdateFps");
    return result;
}

static RESULT IMX_Dummy_ReadRHS1(IsiSensorHandle_t handle, uint32_t* rhs1) {

    *rhs1 = 0;
    return RET_SUCCESS;
}

static RESULT IMX_Dummy_ReadRHS2(IsiSensorHandle_t handle, uint32_t* rhs2) {

    *rhs2 = 0;
    return RET_SUCCESS;
}


static RESULT IMX_Dummy_WriteShr0(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3050, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr0 low");
    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3051, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr0 mid");
    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3052, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr0 high");

    return RET_SUCCESS;
}

static RESULT IMX_Dummy_WriteShr1(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3054, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr1 low");
    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3055, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr1 mid");
    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3056, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr1 high");

    return RET_SUCCESS;
}

static RESULT IMX_Dummy_WriteShr2(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3058, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr2 low");
    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3059, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr2 mid");
    result = IMX_Dummy_IsiWriteRegIss(handle, 0x305a, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr2 high");

    return RET_SUCCESS;
}

static RESULT IMX_Dummy_WriteGain(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3070, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain low");
    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3071, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain high");

    return RET_SUCCESS;
}

static RESULT IMX_Dummy_WriteGain1(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3072, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain1 low");
    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3073, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain1 high");

    return RET_SUCCESS;
}

static RESULT IMX_Dummy_WriteGain2(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3074, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain2 low");
    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3075, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain2 high");

    return RET_SUCCESS;
}

static RESULT IMX_Dummy_LockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3001, 0x1);
    CHECK_RESULT_RET(result, "LockRegHold");

    return RET_SUCCESS;
}

static RESULT IMX_Dummy_UnlockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3001, 0x0);
    CHECK_RESULT_RET(result, "UnlockRegHold");

    return RET_SUCCESS;
}

static RESULT IMX_Dummy_IsiGetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    TRACE(IMX_DUMMY_INFO, "%s (enter)\n", __func__);
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    if (pIMXDummyCtx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    memcpy(pMode, &(pIMXDummyCtx->SensorMode), sizeof(pIMXDummyCtx->SensorMode));

    TRACE(IMX_DUMMY_INFO, "%s (exit)\n", __func__);
    return (RET_SUCCESS);
}

static RESULT IMX_Dummy_IsiSetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    int ret = 0;
    TRACE(IMX_DUMMY_INFO, "%s (enter)\n", __func__);

    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMXDummyCtx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, pMode);
    if (ret != 0) {
        TRACE(IMX_DUMMY_ERROR, "%s: ioctl VVSENSORIOC_S_SENSOR_MODE failed, errno=%d (%s)\n",
              __func__, errno, strerror(errno));
        return (RET_FAILURE);
    }

    return (RET_SUCCESS);
}

static RESULT IMX_Dummy_IsiHalEnumModeIss(HalHandle_t HalHandle,
                                       IsiEnumMode_t* pEnumMode) {
    HalContext_t* pHalCtx = HalHandle;
    if (pHalCtx == NULL) {
        return RET_NULL_POINTER;
    }

    if (pEnumMode->index >=
        (sizeof(pimx_dummy_mode_info) / sizeof(pimx_dummy_mode_info[0])))
        return RET_OUTOFRANGE;

    for (uint32_t i = 0;
         i < (sizeof(pimx_dummy_mode_info) / sizeof(pimx_dummy_mode_info[0])); i++) {
        if (pimx_dummy_mode_info[i].index == pEnumMode->index) {
            memcpy(&pEnumMode->mode, &pimx_dummy_mode_info[i], sizeof(IsiMode_t));
            TRACE(IMX_DUMMY_INFO, "%s (exit)\n", __func__);
            return RET_SUCCESS;
        }
    }

    return RET_NOTSUPP;
}

static RESULT IMX_Dummy_IsiEnumModeIss(IsiSensorHandle_t handle,
                                    IsiEnumMode_t* pEnumMode) {
    RESULT result = RET_SUCCESS;
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    if (pIMXDummyCtx == NULL || pIMXDummyCtx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    HalContext_t* pHalCtx = (HalContext_t*)pIMXDummyCtx->IsiCtx.HalHandle;
    result = IMX_Dummy_IsiHalEnumModeIss(pHalCtx, pEnumMode);
    CHECK_RESULT_RET(result, "sensor enum mode");

    return result;
}

static RESULT IMX_Dummy_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t* pCaps) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;

    RESULT result = RET_SUCCESS;

    TRACE(IMX_DUMMY_INFO, "%s (enter)\n", __func__);

    if (pIMXDummyCtx == NULL) return (RET_WRONG_HANDLE);

    if (pCaps == NULL) {
        return (RET_NULL_POINTER);
    }

    if (!pIMXDummyCtx->Configured) {
        result = IMX_Dummy_IsiSetupIss(handle, pCaps);
        CHECK_RESULT_RET(result, "Setup");
    }

    pCaps->BusWidth = pIMXDummyCtx->SensorMode.bit_width;
    pCaps->Mode = ISI_MODE_BAYER;
    pCaps->FieldSelection = ISI_FIELDSEL_BOTH;
    pCaps->YCSequence = ISI_YCSEQ_YCBYCR;
    pCaps->Conv422 = ISI_CONV422_COSITED;
    pCaps->BPat = pIMXDummyCtx->SensorMode.bayer_pattern;
    pCaps->HPol = ISI_HPOL_REFPOS;
    pCaps->VPol = ISI_VPOL_POS;
    pCaps->Edge = ISI_EDGE_RISING;
    pCaps->Resolution.width = pIMXDummyCtx->SensorMode.size.width;
    pCaps->Resolution.height = pIMXDummyCtx->SensorMode.size.height;
    pCaps->SmiaMode = ISI_SMIA_OFF;
    pCaps->MipiLanes = ISI_MIPI_4LANES;

    if (pCaps->BusWidth == 10) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_10;
    } else if (pCaps->BusWidth == 12) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_12;
    } else {
        pCaps->MipiMode = ISI_MIPI_OFF;
    }
    TRACE(IMX_DUMMY_INFO, "got caps - width %d height %d buswidth %d\n",
          pCaps->Resolution.width, pCaps->Resolution.height, pCaps->BusWidth);
    TRACE(IMX_DUMMY_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX_Dummy_AecSetModeParameters(IMX_dummy_Context_t* pIMXDummyCtx,
                                          const IsiCaps_t* pConfig) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX_DUMMY_INFO, "%s%s: (enter)\n", __func__,
          pIMXDummyCtx->isAfpsRun ? "(AFPS)" : "");

    pIMXDummyCtx->AecIntegrationTimeIncrement = pIMXDummyCtx->one_line_exp_time;
    pIMXDummyCtx->AecMinIntegrationTime =
        pIMXDummyCtx->one_line_exp_time * pIMXDummyCtx->MinIntegrationLine;
    pIMXDummyCtx->AecMaxIntegrationTime =
        pIMXDummyCtx->one_line_exp_time * pIMXDummyCtx->MaxIntegrationLine;

    TRACE(IMX_DUMMY_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __func__,
          pIMXDummyCtx->isAfpsRun ? "(AFPS)" : "",
          pIMXDummyCtx->AecMaxIntegrationTime);

    pIMXDummyCtx->AecGainIncrement = IMX_DUMMY_MIN_GAIN_STEP;

    // reflects the state of the sensor registers, must equal default settings
    pIMXDummyCtx->AecCurGainLEF = pIMXDummyCtx->AecMinGain;
	pIMXDummyCtx->AecCurGainSEF1 = pIMXDummyCtx->AecMinGain;
	pIMXDummyCtx->AecCurGainSEF2 = pIMXDummyCtx->AecMinGain;
	pIMXDummyCtx->AecCurIntegrationTimeLEF = pIMXDummyCtx->AecMaxIntegrationTime;
	pIMXDummyCtx->AecCurIntegrationTimeSEF1 = pIMXDummyCtx->AecMaxIntegrationTime;
	pIMXDummyCtx->AecCurIntegrationTimeSEF2 = pIMXDummyCtx->AecMaxIntegrationTime;
    pIMXDummyCtx->OldGain = 1;
    pIMXDummyCtx->OldIntegrationTime = 0.0f;

    TRACE(IMX_DUMMY_INFO, "%s%s: (exit)\n", __func__,
          pIMXDummyCtx->isAfpsRun ? "(AFPS)" : "");

    return (result);
}

static RESULT IMX_Dummy_IsiSetupIss(IsiSensorHandle_t handle,
                                 const IsiCaps_t* pCaps) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX_DUMMY_INFO, "%s (enter)\n", __func__);

    if (!pIMXDummyCtx) {
        TRACE(IMX_DUMMY_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pCaps == NULL) {
        TRACE(IMX_DUMMY_ERROR,
              "%s: Invalid configuration (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMXDummyCtx->Streaming != BOOL_FALSE) {
        return RET_WRONG_STATE;
    }

    if (pIMXDummyCtx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        pIMXDummyCtx->enableHdr = true;
    } else {
        pIMXDummyCtx->enableHdr = false;
    }

    pIMXDummyCtx->one_line_exp_time =
        (float)(pIMXDummyCtx->SensorMode.ae_info.one_line_exp_time_ns) /
        1000000000;
    pIMXDummyCtx->MaxIntegrationLine =
        pIMXDummyCtx->SensorMode.ae_info.max_integration_time;
    TRACE(IMX_DUMMY_INFO, "%s: MaxIntegrationLine %u\n", __func__,
          pIMXDummyCtx->MaxIntegrationLine);
    pIMXDummyCtx->MinIntegrationLine =
        pIMXDummyCtx->SensorMode.ae_info.min_integration_time;
    pIMXDummyCtx->gain_accuracy = pIMXDummyCtx->SensorMode.ae_info.gain_accuracy;
    pIMXDummyCtx->AecMaxGain = (float)(pIMXDummyCtx->SensorMode.ae_info.max_gain) /
                             pIMXDummyCtx->gain_accuracy;
    pIMXDummyCtx->AecMinGain = (float)(pIMXDummyCtx->SensorMode.ae_info.min_gain) /
                             pIMXDummyCtx->gain_accuracy;

    pIMXDummyCtx->AecMinIris = IMX_DUMMY_IRIS_MIN_VAL;
    pIMXDummyCtx->AecMaxIris = IMX_DUMMY_IRIS_MAX_VAL;

    pIMXDummyCtx->original_vmax = 0;
    pIMXDummyCtx->unlimit_fps = 0;
    pIMXDummyCtx->unlimit_fps_vmax_changed = 0;
    pIMXDummyCtx->flicker_fps_mode = ISI_AE_ANTIBANDING_MODE_OFF;

    pIMXDummyCtx->MaxFps = pIMXDummyCtx->SensorMode.fps;
    pIMXDummyCtx->CurrFps = pIMXDummyCtx->MaxFps;
    TRACE(IMX_DUMMY_INFO, "%s - got caps - width %d height %d buswidth %d\n",
          __func__, pIMXDummyCtx->SensorMode.size.width,
          pIMXDummyCtx->SensorMode.size.height, pIMXDummyCtx->SensorMode.bit_width);

    TRACE(IMX_DUMMY_INFO, "%s - MinGain %f MaxGain %f\n", __func__, pIMXDummyCtx->AecMinGain,
          pIMXDummyCtx->AecMaxGain);

    memcpy(&pIMXDummyCtx->CapsConfig, pCaps, sizeof(pIMXDummyCtx->CapsConfig));

    /* SW reset comment preserved - no actual reset code was present,
     * and the 100ms sleep was unnecessary (no hardware access in this function). */

    result = IMX_Dummy_AecSetModeParameters(pIMXDummyCtx, pCaps);
    CHECK_RESULT_RET(result, "AecSetModeParameters");

    pIMXDummyCtx->Configured = BOOL_TRUE;
    TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);
    return 0;
}

static RESULT IMX_Dummy_IsiCheckConnectionIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    return (result);
}

static RESULT IMX_Dummy_IsiGetRevisionIss(IsiSensorHandle_t handle,
                                       uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    int ret = 0;
    uint32_t reg_val;
    uint32_t sensor_id;

    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    if (pIMXDummyCtx == NULL || pIMXDummyCtx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMXDummyCtx->IsiCtx.HalHandle;

    if (!pValue) return (RET_NULL_POINTER);

    if (pIMXDummyCtx->KernelDriverFlag) {
        ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, &sensor_id);
        if (ret != 0) {
            TRACE(IMX_DUMMY_ERROR, "%s: Read Sensor ID Error! \n", __func__);
            return (RET_FAILURE);
        }
    } else {
        reg_val = 0;
        result = IMX_Dummy_IsiReadRegIss(handle, 0x3a04, &reg_val);
        CHECK_RESULT_RET(result, "read sensor ID high");
        sensor_id = (reg_val & 0xff) << 8;

        reg_val = 0;
        result = IMX_Dummy_IsiReadRegIss(handle, 0x3a05, &reg_val);
        CHECK_RESULT_RET(result, "read sensor ID low");
        sensor_id |= (reg_val & 0xff);
    }

    *pValue = sensor_id;
    TRACE(IMX_DUMMY_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX_Dummy_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t is_on) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX_DUMMY_INFO, "%s (enter)\n", __func__);

    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    if (pIMXDummyCtx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    pIMXDummyCtx->Streaming = is_on;

    if (pIMXDummyCtx->enableHdr)
        return result;

    TRACE(IMX_DUMMY_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX_Dummy_IsiGetGainLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinGain, float* pMaxGain) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);

    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinGain == NULL) || (pMaxGain == NULL)) {
        TRACE(IMX_DUMMY_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinGain = pIMXDummyCtx->AecMinGain;
    *pMaxGain = pIMXDummyCtx->AecMaxGain;

    TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX_Dummy_GetNumExposures(IMX_dummy_Context_t* pIMXDummyCtx, size_t* num_exposures) {
    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    if (num_exposures == NULL) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid output parameter\n", __func__);
        return RET_NULL_POINTER;
    }

    if (pIMXDummyCtx->SensorMode.hdr_mode == SENSOR_MODE_LINEAR) {
        *num_exposures = 1; // SDR
    } else if (pIMXDummyCtx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        *num_exposures = IMX_DUMMY_2DOL_NUM_EXP;
    } else if (pIMXDummyCtx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        *num_exposures = IMX_DUMMY_3DOL_NUM_EXP;
    } else {
        TRACE(IMX_DUMMY_ERROR, "%s: Unsupported HDR mode %d with stitching mode %d\n",
            __func__, pIMXDummyCtx->SensorMode.hdr_mode, pIMXDummyCtx->SensorMode.stitching_mode);
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

static RESULT IMX_Dummy_UpdateCurrLEFIntegrationTimeFromFsc(IMX_dummy_Context_t* pIMXDummyCtx, uint32_t fsc, uint32_t shr0) {
    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    float configuredIntegrationTime = (fsc - shr0) * pIMXDummyCtx->one_line_exp_time;

    pIMXDummyCtx->OldIntegrationTime = configuredIntegrationTime;
    pIMXDummyCtx->AecCurIntegrationTimeLEF = configuredIntegrationTime;

    TRACE(IMX_DUMMY_DEBUG, "%s: Updated LEF Integration Time = (fsc[%u] - shr0[%u]) * one_line_exp_time[%f] = %f\n",
          __func__, fsc, shr0, pIMXDummyCtx->one_line_exp_time, configuredIntegrationTime);
    return RET_SUCCESS;
}

static RESULT IMX_Dummy_UpdateCurrLEFIntegrationTimeFromVmax(IMX_dummy_Context_t* pIMXDummyCtx, uint32_t vmax, uint32_t shr0) {
    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    size_t dol = 0;
    RESULT result = IMX_Dummy_GetNumExposures(pIMXDummyCtx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");
    uint32_t fsc = vmax * dol;
    TRACE(IMX_DUMMY_DEBUG, "%s: fsc = vmax[%u] * dol[%zu] = %u\n", __func__, vmax, dol, fsc);
    result = IMX_Dummy_UpdateCurrLEFIntegrationTimeFromFsc(pIMXDummyCtx, fsc, shr0);
    CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromFsc");
    return RET_SUCCESS;
}

static inline int IMX_Dummy_getFlickerPeaksPerSec(IsiSensorAntibandingMode_t mode) {
    int num_modes = sizeof(flickerPeaksPerSecMap) / sizeof(FlickerModePeaksPerSec);
    for (int i = 0; i < num_modes; i++) {
        if (flickerPeaksPerSecMap[i].mode == mode) {
            return flickerPeaksPerSecMap[i].value;
        }
    }
    return 0; // Default to 0 if mode not found
}

static inline RESULT IMX_Dummy_getNewVmaxAntiFlicker(IMX_dummy_Context_t *pIMXDummyCtx, uint32_t requestedVmax, uint32_t *outClosestVmax) {
    uint32_t closestVmax = requestedVmax;
    int peaks = 0;
    int difference = INT_MAX;
    int minDifference = INT_MAX;
    if (!pIMXDummyCtx) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return RET_FAILURE;
    }
    peaks = IMX_Dummy_getFlickerPeaksPerSec(pIMXDummyCtx->flicker_fps_mode);
    if (peaks == 0) {
        // No anti-flicker mode, return requested or original Vmax
        if (pIMXDummyCtx->unlimit_fps && pIMXDummyCtx->unlimit_fps_vmax_changed)
            *outClosestVmax = requestedVmax;
        else
            *outClosestVmax = pIMXDummyCtx->original_vmax;
        return RET_SUCCESS;
    }

    for (int i = 1; i < peaks; ++i) {
        uint32_t candidateVmax = i / (peaks * pIMXDummyCtx->one_line_exp_time);
        if (candidateVmax < pIMXDummyCtx->SensorMode.size.height || candidateVmax < pIMXDummyCtx->original_vmax)
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
    *outClosestVmax = closestVmax;
    return RET_SUCCESS;
}

static RESULT IMX_Dummy_IsiUnlimitFpsIss(IsiSensorHandle_t handle,
                                      float maxIntegrationTime) {
    RESULT result = RET_SUCCESS;
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;

    if (!pIMXDummyCtx) {
        return RET_NULL_POINTER;
    }

    if (pIMXDummyCtx->enableHdr) {
        return result;
    }
    pIMXDummyCtx->unlimit_fps = 1;

    if (maxIntegrationTime < 0)
        pIMXDummyCtx->MaxIntegrationLine = IMX_DUMMY_VMAX_MAX - IMX_DUMMY_MIN_SHR;
    else
        pIMXDummyCtx->MaxIntegrationLine =
            MIN((uint32_t)(maxIntegrationTime / pIMXDummyCtx->one_line_exp_time),
                IMX_DUMMY_VMAX_MAX - IMX_DUMMY_MIN_SHR);
    TRACE(IMX_DUMMY_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
        pIMXDummyCtx->MaxIntegrationLine);
    pIMXDummyCtx->AecMaxIntegrationTime =
        pIMXDummyCtx->one_line_exp_time * pIMXDummyCtx->MaxIntegrationLine;
    return result;
}

static RESULT IMX_Dummy_IsiLimitFpsIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    uint32_t current_vmax = 0;
    uint32_t new_vmax = 0;
    TRACE(IMX_DUMMY_INFO, "%s (enter)\n", __func__);
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    if (!pIMXDummyCtx) {
        return RET_NULL_POINTER;
    }

    if (pIMXDummyCtx->enableHdr)
        return result;

    result = IMX_Dummy_ReadVmax(handle, &current_vmax);
    CHECK_RESULT_RET(result, "ReadVmax");
    if (current_vmax == 0) {
        TRACE(IMX_DUMMY_INFO, "%s - exit because current_vmax is 0\n", __func__);
        return result;
    }

    pIMXDummyCtx->unlimit_fps = 0;
    pIMXDummyCtx->unlimit_fps_vmax_changed = 0;
    if (pIMXDummyCtx->original_vmax == 0) {
        pIMXDummyCtx->original_vmax = current_vmax;
    }

    new_vmax = pIMXDummyCtx->original_vmax;
    if (pIMXDummyCtx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
        result = IMX_Dummy_getNewVmaxAntiFlicker(pIMXDummyCtx, pIMXDummyCtx->original_vmax, &new_vmax);
        CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
        TRACE(IMX_DUMMY_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMXDummyCtx->flicker_fps_mode, new_vmax);
    }
    if (current_vmax != new_vmax) {
        result = IMX_Dummy_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        result = IMX_Dummy_WriteVmax(handle, new_vmax);
        CHECK_RESULT_RET(result, "WriteVmax");
        result = IMX_Dummy_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");

        int shr = MAX((int)current_vmax - (int)(pIMXDummyCtx->AecCurIntegrationTimeLEF / pIMXDummyCtx->one_line_exp_time), IMX_DUMMY_MIN_SHR);
        float configuredIntegrationTime = (new_vmax - shr) * pIMXDummyCtx->one_line_exp_time;
        pIMXDummyCtx->OldIntegrationTime = configuredIntegrationTime;
        pIMXDummyCtx->AecCurIntegrationTimeLEF = configuredIntegrationTime;
    }

    pIMXDummyCtx->MaxIntegrationLine =
        MAX(new_vmax - IMX_DUMMY_MIN_SHR, 1);
    TRACE(IMX_DUMMY_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
        pIMXDummyCtx->MaxIntegrationLine);
    pIMXDummyCtx->AecMaxIntegrationTime =
        pIMXDummyCtx->one_line_exp_time * pIMXDummyCtx->MaxIntegrationLine;
    return result;
}

static RESULT IMX_Dummy_IsiGetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinIris, float* pMaxIris) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);

    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIris == NULL) || (pMaxIris == NULL)) {
        TRACE(IMX_DUMMY_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIris = pIMXDummyCtx->AecMinIris;
    *pMaxIris = pIMXDummyCtx->AecMaxIris;

    TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX_Dummy_IsiSetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float minIris, float maxIris) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);

    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    pIMXDummyCtx->AecMinIris = minIris;
    pIMXDummyCtx->AecMaxIris = maxIris;

    TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX_Dummy_IsiGetIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float* pMinIntegrationTime,
    float* pMaxIntegrationTime) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);
    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX_DUMMY_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIntegrationTime = pIMXDummyCtx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMXDummyCtx->AecMaxIntegrationTime;

    TRACE(IMX_DUMMY_INFO, "%s: (exit) %f, %f\n", 
    __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

/* Gain get functions*/

RESULT IMX_Dummy_IsiGetGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;
	TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);

	if (pIMXDummyCtx == NULL) {
		TRACE(IMX_DUMMY_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	if (pIMXDummyCtx->enableHdr) {
		RESULT result = IMX_Dummy_IsiGetSEF1GainIss(handle, pSetGain);
		CHECK_RESULT_RET(result, "GetSEF1Gain");
		return result;
	}

	RESULT result = IMX_Dummy_IsiGetLEFGainIss(handle, pSetGain);
	CHECK_RESULT_RET(result, "GetLEFGain");
	return result;
}

RESULT IMX_Dummy_IsiGetLEFGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);

	if (pIMXDummyCtx == NULL) {
		TRACE(IMX_DUMMY_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}
	*pSetGain = pIMXDummyCtx->AecCurGainLEF;
	TRACE(IMX_DUMMY_DEBUG, "%s - returning %f\n", __func__, pIMXDummyCtx->AecCurGainLEF);
	TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);
	return (result);
}

RESULT IMX_Dummy_IsiGetSEF1GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);

	if (pIMXDummyCtx == NULL) {
		TRACE(IMX_DUMMY_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMXDummyCtx->AecCurGainSEF1;
	TRACE(IMX_DUMMY_DEBUG, "%s - returning %f\n", __func__, pIMXDummyCtx->AecCurGainSEF1);

	TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);

	return (result);
}

RESULT IMX_Dummy_IsiGetSEF2GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);

	if (pIMXDummyCtx == NULL) {
		TRACE(IMX_DUMMY_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMXDummyCtx->AecCurGainSEF2;
	TRACE(IMX_DUMMY_DEBUG, "%s - returning %f\n", __func__, pIMXDummyCtx->AecCurGainSEF2);

	TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);

	return (result);
}

RESULT IMX_Dummy_IsiGetGainIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);

    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = pIMXDummyCtx->AecGainIncrement;

    TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);

    return (result);
}

RESULT IMX_Dummy_IsiGetIrisIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);
    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = 1;

    TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);

    return (result);
}

/* Gain set functions*/

RESULT IMX_Dummy_IsiSetGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;
	if (pIMXDummyCtx == NULL || pIMXDummyCtx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	if (pIMXDummyCtx->enableHdr) {
		result = IMX_Dummy_IsiSetSEF1GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1Gain");

		result = IMX_Dummy_IsiSetSEF2GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF2Gain");
	}

	result = IMX_Dummy_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
	CHECK_RESULT_RET(result, "SetLEFGain");
	return RET_SUCCESS;
}

static inline uint32_t _linear2sensorGain(float gain)
{
    uint32_t db = 0;
    float log_gain = log10(gain);
    log_gain = (log_gain * 10 * 20) / 3;
    db = roundf(log_gain);
    return db;
}

static inline uint32_t _linear2sensorGainCeil(float gain)
{
    const float epsilon = 0.1;

    uint32_t db = 0;
    float log_gain = log10(gain);
    log_gain = (log_gain * 10 * 20) / 3;

    // We can assume that due to rounding/quantization, given gain is not exactly accurate.
    // And if it's lower than it's original value, this function might eventually round down the value
    // This will break the hdr ratios for this extreme case.
    // To prevent this, we add epsilon to our calculated gain.
    // This way, we are only allowed to make mistakes that increase gain, and not decrease it.
    db = ceil(log_gain + epsilon);
    return db;
}

static inline float _sensorGain2linear(uint32_t db)
{
    float gain = ((float)(db) * 3) / 200;
    gain = pow(10, gain);
    return gain;
}

RESULT IMX_Dummy_IsiSetLEFGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;
	if (pIMXDummyCtx == NULL || pIMXDummyCtx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	TRACE(IMX_DUMMY_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX_DUMMY_DEBUG, "%s: writting 0x%x to GAIN\n", __func__, Gain);

	result = IMX_Dummy_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX_Dummy_WriteGain(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain");
	result = IMX_Dummy_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pIMXDummyCtx->AecCurGainLEF = _sensorGain2linear(Gain);
	*pSetGain = pIMXDummyCtx->AecCurGainLEF;
	TRACE(IMX_DUMMY_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return RET_SUCCESS;
}

RESULT IMX_Dummy_IsiSetSEF1GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMXDummyCtx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);

	TRACE(IMX_DUMMY_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX_DUMMY_DEBUG, "%s: writting 0x%x to GAIN1\n", __func__, Gain);

	result = IMX_Dummy_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX_Dummy_WriteGain1(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain1");
	result = IMX_Dummy_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pIMXDummyCtx->AecCurGainSEF1 = _sensorGain2linear(Gain);
	*pSetGain = pIMXDummyCtx->AecCurGainSEF1;

	TRACE(IMX_DUMMY_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return RET_SUCCESS;
}

RESULT IMX_Dummy_IsiSetSEF2GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMXDummyCtx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);
	
	TRACE(IMX_DUMMY_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX_DUMMY_DEBUG, "%s: writting 0x%x to GAIN2\n", __func__, Gain);

	result = IMX_Dummy_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX_Dummy_WriteGain2(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain2");
	result = IMX_Dummy_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pIMXDummyCtx->AecCurGainSEF2 = _sensorGain2linear(Gain);
	*pSetGain = pIMXDummyCtx->AecCurGainSEF2;

	TRACE(IMX_DUMMY_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return RET_SUCCESS;
}

/* Integration Time get functions*/

RESULT IMX_Dummy_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;

	if (!pIMXDummyCtx) {
		TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	
	TRACE(IMX_DUMMY_DEBUG, "%s - enter\n", __func__);
	
	if (pIMXDummyCtx->enableHdr) {
		RESULT result = IMX_Dummy_IsiGetSEF1IntegrationTimeIss(handle, pSetIntegrationTime);
		CHECK_RESULT_RET(result, "GetSEF1IntegrationTime");
		return result;
	}
	
	RESULT result = IMX_Dummy_IsiGetLEFIntegrationTimeIss(handle, pSetIntegrationTime);
	CHECK_RESULT_RET(result, "GetLEFIntegrationTime");
	return result;
}

RESULT IMX_Dummy_IsiGetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;
	RESULT result = RET_SUCCESS;


	if (!pIMXDummyCtx) {
		TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMXDummyCtx->AecCurIntegrationTimeLEF;
	TRACE(IMX_DUMMY_DEBUG, "%s - returning %f\n", __func__, pIMXDummyCtx->AecCurIntegrationTimeLEF);
	return (result);
}

RESULT IMX_Dummy_IsiGetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMXDummyCtx) {
		TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMXDummyCtx->AecCurIntegrationTimeSEF1;
	TRACE(IMX_DUMMY_DEBUG, "%s - returning %f\n", __func__, pIMXDummyCtx->AecCurIntegrationTimeSEF1);
	return (result);
}

RESULT IMX_Dummy_IsiGetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMXDummyCtx) {
		TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);

	*pSetIntegrationTime = pIMXDummyCtx->AecCurIntegrationTimeSEF2;
	TRACE(IMX_DUMMY_DEBUG, "%s - returning %f\n", __func__, pIMXDummyCtx->AecCurIntegrationTimeSEF2);
	return (result);
}

RESULT IMX_Dummy_IsiGetIntegrationTimeIncrementIss(IsiSensorHandle_t handle,
                                                float* pIncr) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (!pIMXDummyCtx) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pIncr) return (RET_NULL_POINTER);

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders
    //in
    // the application)
    *pIncr = pIMXDummyCtx->AecIntegrationTimeIncrement;
    return (result);
}

/* Integration Time set functions*/

RESULT IMX_Dummy_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;

	if (!pIMXDummyCtx) {
		TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMXDummyCtx->enableHdr) {
		RESULT result = IMX_Dummy_IsiSetSEF1IntegrationTimeIss(
			handle, NewIntegrationTime, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1IntegrationTime");
		return result;
	}

	RESULT result = IMX_Dummy_IsiSetLEFIntegrationTimeIss(
		handle, NewIntegrationTime, pSetIntegrationTime,
		pNumberOfFramesToSkip, hdr_ratio);
	CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
	return result;
}

RESULT IMX_Dummy_IsiSetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;

    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;

    int shr = 0;
    uint32_t exp = 0;
    uint32_t new_vmax = 0;
    uint32_t vmax_updated = 0;
    uint32_t current_vmax = 0;
    uint32_t rhs1;
    uint32_t rhs2;
    if (!pIMXDummyCtx) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pSetIntegrationTime || !pNumberOfFramesToSkip) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMXDummyCtx->Streaming) {
        TRACE(IMX_DUMMY_ERROR, "%s: sensor not streaming\n", __func__);
        return RET_FAILURE;
    }
    exp = roundf(NewIntegrationTime / pIMXDummyCtx->one_line_exp_time);

    TRACE(IMX_DUMMY_DEBUG, "%s: set AEC_PK_EXPO=0x%05x\n", __func__, exp);

    if (fabs(NewIntegrationTime - pIMXDummyCtx->AecCurIntegrationTimeLEF) > FLT_EPSILON) {

        if (pIMXDummyCtx->enableHdr){
            if (pIMXDummyCtx->cur_rhs1 == 0 || pIMXDummyCtx->cur_rhs2 == 0) {
                TRACE(IMX_DUMMY_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
                return (RET_WRONG_CONFIG);
            }
	

            rhs1 = pIMXDummyCtx->cur_rhs1;
            rhs2 = pIMXDummyCtx->cur_rhs2;
            if (pIMXDummyCtx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
                result = IMX_Dummy_ReadVmax(pIMXDummyCtx, &new_vmax);
                CHECK_RESULT_RET(result, "ReadVmax");
                new_vmax *= IMX_DUMMY_2DOL_NUM_EXP;

                exp = new_vmax - exp;
                exp = exp > rhs1 + IMX_DUMMY_2DOL_SHR0_RHS1_GAP ? exp : rhs1 + IMX_DUMMY_2DOL_SHR0_RHS1_GAP;
                exp = exp < new_vmax - IMX_DUMMY_2DOL_SHR0_FSC_GAP? exp : new_vmax - IMX_DUMMY_2DOL_SHR0_FSC_GAP;
                shr = exp;
            } else {
                new_vmax = IMX_DUMMY_VMAX_3DOL_HDR;
                exp = new_vmax - exp;
                exp = exp > rhs2 + IMX_DUMMY_SHR0_RHS2_GAP ? exp : rhs2 + IMX_DUMMY_SHR0_RHS2_GAP;
                exp = exp < new_vmax - IMX_DUMMY_SHR0_FSC_GAP? exp : new_vmax - IMX_DUMMY_SHR0_FSC_GAP;
                shr = exp;
            }
            
		} else {
            if (exp > pIMXDummyCtx->MaxIntegrationLine || exp == 0) {
                TRACE(IMX_DUMMY_ERROR, "%s: Integration time %f (exp %u) out of range (%u)\n", __func__,
                    NewIntegrationTime, exp, pIMXDummyCtx->MaxIntegrationLine);
                return RET_FAILURE;
            }
            result = IMX_Dummy_ReadVmax(handle, &current_vmax);
            CHECK_RESULT_RET(result, "ReadVmax");

            if (pIMXDummyCtx->original_vmax == 0) {
                pIMXDummyCtx->original_vmax = current_vmax;
            }

            if (pIMXDummyCtx->original_vmax - IMX_DUMMY_MIN_SHR >
                pIMXDummyCtx->MaxIntegrationLine) {
                pIMXDummyCtx->MaxIntegrationLine =
                    pIMXDummyCtx->original_vmax - IMX_DUMMY_MIN_SHR;
                TRACE(IMX_DUMMY_DEBUG, "%s: set MaxIntegrationLine to %u\n", __func__,
                      pIMXDummyCtx->MaxIntegrationLine);
                pIMXDummyCtx->AecMaxIntegrationTime =
                    pIMXDummyCtx->one_line_exp_time * pIMXDummyCtx->MaxIntegrationLine;
                TRACE(IMX_DUMMY_DEBUG, "%s: set AecMaxIntegrationTime to %f\n", __func__,
                      pIMXDummyCtx->AecMaxIntegrationTime);
            }

            shr = current_vmax - exp;

            if (shr < IMX_DUMMY_MIN_SHR || current_vmax > pIMXDummyCtx->original_vmax) {
                if (shr < IMX_DUMMY_MIN_SHR) {
                    // user asked for too big integration-line: more than (current_vmax - minimal_shutter)
                    // increase vmax and set new shutter to minimal value
                    new_vmax = MIN(exp + IMX_DUMMY_MIN_SHR, pIMXDummyCtx->MaxIntegrationLine + IMX_DUMMY_MIN_SHR);
                } else {
                    // make sure that when we unlimit fps, stay below original fps.
                    // also, make sure we are not using high vmax + high shr more than needed (will slow fps for no reason)
                    new_vmax = MAX(current_vmax - shr + IMX_DUMMY_MIN_SHR, pIMXDummyCtx->original_vmax);
                }
                pIMXDummyCtx->unlimit_fps_vmax_changed = new_vmax > pIMXDummyCtx->original_vmax && pIMXDummyCtx->unlimit_fps;
        
                if (pIMXDummyCtx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
                    result = IMX_Dummy_getNewVmaxAntiFlicker(pIMXDummyCtx, new_vmax, &new_vmax);
                    CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
                    TRACE(IMX_DUMMY_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMXDummyCtx->flicker_fps_mode, new_vmax);
                }
                
                shr = MAX(IMX_DUMMY_MIN_SHR, (int)new_vmax - (int)exp);
                vmax_updated = 1;

            } else {
                new_vmax = current_vmax;
            }
        }

        result = IMX_Dummy_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        if (vmax_updated && pIMXDummyCtx->unlimit_fps && !pIMXDummyCtx->enableHdr) {
            result = IMX_Dummy_WriteVmax(handle, new_vmax);
            CHECK_RESULT_RET(result, "WriteVmax");
        }

        TRACE(IMX_DUMMY_DEBUG, "%s - writing 0x%x to SHR0\n", __func__, shr);
        result = IMX_Dummy_WriteShr0(handle, shr);
        CHECK_RESULT_RET(result, "WriteShr0");
        result = IMX_Dummy_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");

        float configuredIntegrationTime =
            (new_vmax - shr) * pIMXDummyCtx->one_line_exp_time;
        pIMXDummyCtx->OldIntegrationTime = configuredIntegrationTime;
        pIMXDummyCtx->AecCurIntegrationTimeLEF = configuredIntegrationTime;

        *pNumberOfFramesToSkip = 1U;
    } else {
        *pNumberOfFramesToSkip = 0U;  // no frame skip
    }

    *pSetIntegrationTime = pIMXDummyCtx->AecCurIntegrationTimeLEF;
    TRACE(IMX_DUMMY_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTime);
    return (result);
}

RESULT IMX_Dummy_IsiSetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF1,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;

	if (!pIMXDummyCtx) {
		TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMXDummyCtx->cur_rhs1 == 0 || pIMXDummyCtx->cur_rhs2 == 0) {
		TRACE(IMX_DUMMY_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMXDummyCtx->cur_rhs1;

	if (!pSetIntegrationTimeSEF1 || !pNumberOfFramesToSkip) {
		TRACE(IMX_DUMMY_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX_DUMMY_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = roundf(NewIntegrationTime / pIMXDummyCtx->one_line_exp_time);

	if (fabs(NewIntegrationTime - pIMXDummyCtx->AecCurIntegrationTimeSEF1) > FLT_EPSILON) {
		if (pIMXDummyCtx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
			exp = rhs1 - exp;
			exp = exp > IMX_DUMMY_2DOL_SHR1_RHS1_GAP ? exp : IMX_DUMMY_2DOL_SHR1_RHS1_GAP;
			exp = exp < rhs1 - IMX_DUMMY_SHR1_RHS1_GAP ? exp : rhs1 - IMX_DUMMY_SHR1_RHS1_GAP;
		} else {
			exp = rhs1 - exp;
			exp = exp > IMX_DUMMY_3DOL_SHR1_RHS1_GAP ? exp : IMX_DUMMY_3DOL_SHR1_RHS1_GAP;
			exp = exp < rhs1 - IMX_DUMMY_SHR1_RHS1_GAP ? exp : rhs1 - IMX_DUMMY_SHR1_RHS1_GAP;
		}

		TRACE(IMX_DUMMY_DEBUG, "%s - writing 0x%x to SHR1\n", __func__, exp);

		result = IMX_Dummy_LockRegHold(handle);
		CHECK_RESULT_RET(result, "LockRegHold");
		result = IMX_Dummy_WriteShr1(handle, exp);
		CHECK_RESULT_RET(result, "WriteShr1");
		result = IMX_Dummy_UnlockRegHold(handle);
		CHECK_RESULT_RET(result, "UnlockRegHold");

		pIMXDummyCtx->AecCurIntegrationTimeSEF1 = (rhs1 - exp) * pIMXDummyCtx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF1 = pIMXDummyCtx->AecCurIntegrationTimeSEF1;

	TRACE(IMX_DUMMY_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF1);
	return (result);
}

RESULT IMX_Dummy_IsiSetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF2,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX_dummy_Context_t *pIMXDummyCtx = (IMX_dummy_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;
	uint32_t rhs2;

	if (pIMXDummyCtx->cur_rhs1 == 0 || pIMXDummyCtx->cur_rhs2 == 0) {
		TRACE(IMX_DUMMY_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMXDummyCtx->cur_rhs1;
	rhs2 = pIMXDummyCtx->cur_rhs2;

	if (!pIMXDummyCtx) {
		TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTimeSEF2 || !pNumberOfFramesToSkip) {
		TRACE(IMX_DUMMY_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX_DUMMY_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = roundf(NewIntegrationTime / pIMXDummyCtx->one_line_exp_time);
	TRACE(IMX_DUMMY_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMXDummyCtx->AecCurIntegrationTimeSEF2) > FLT_EPSILON) {
		exp = rhs2 - exp;
		exp = exp > rhs1 + IMX_DUMMY_SHR2_RHS1_GAP ? exp : rhs1 + IMX_DUMMY_SHR2_RHS1_GAP;
		exp = exp < rhs2 - IMX_DUMMY_SHR2_RHS2_GAP ? exp : rhs2 - IMX_DUMMY_SHR2_RHS2_GAP;
		TRACE(IMX_DUMMY_DEBUG, "%s - writing 0x%x to SHR2\n", __func__, exp);

		result = IMX_Dummy_LockRegHold(handle);
		CHECK_RESULT_RET(result, "LockRegHold");
		result = IMX_Dummy_WriteShr2(handle, exp);
		CHECK_RESULT_RET(result, "WriteShr2");
		result = IMX_Dummy_UnlockRegHold(handle);
		CHECK_RESULT_RET(result, "UnlockRegHold");

		pIMXDummyCtx->AecCurIntegrationTimeSEF2 = (rhs2 - exp) * pIMXDummyCtx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF2 = pIMXDummyCtx->AecCurIntegrationTimeSEF2;

	TRACE(IMX_DUMMY_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF2);
	return (result);
}

RESULT IMX_Dummy_Calculate3DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it, float *o_very_short_it,
                                    float *o_long_gain, float *o_short_gain, float *o_very_short_gain,
                                    float *hdr_ratio) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
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
	bool optimize_long_gain = false;
	bool optimize_short_gain = false;

    if (pIMXDummyCtx == NULL || o_long_it == NULL || o_short_it == NULL ||
        o_very_short_it == NULL || o_long_gain == NULL || o_short_gain == NULL ||
        o_very_short_gain == NULL || hdr_ratio == NULL) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

	if (pIMXDummyCtx->cur_rhs1 == 0 || pIMXDummyCtx->cur_rhs2 == 0) {
		TRACE(IMX_DUMMY_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMXDummyCtx->cur_rhs1;
	rhs2 = pIMXDummyCtx->cur_rhs2;

    TRACE(IMX_DUMMY_DEBUG, "%s: hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n", 
    __func__, hdr_ratio[0], hdr_ratio[1]);
    
    // Sometimes there is no actual input gain. In that case, we will read it from the sensor
    if (NewGain == 0) {
        TRACE(IMX_DUMMY_DEBUG, "%s: Input NewGain is 0, reading gain from sensor\n", __func__);
        result = IMX_Dummy_IsiGetSEF1GainIss(handle, &NewGain);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // Same for integration time
    if (NewIntegrationTime == 0) {
        TRACE(IMX_DUMMY_DEBUG, "%s: Input NewIntegrationTime is 0, reading integration time from sensor\n", __func__);
        result = IMX_Dummy_IsiGetSEF1IntegrationTimeIss(handle, &NewIntegrationTime);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    if(NewIntegrationTime < IMX_DUMMY_SHR2_RHS2_GAP * pIMXDummyCtx->one_line_exp_time * hdr_ratio[1]){
	    pIMXDummyCtx->MinIntegrationLine =  IMX_DUMMY_SHR2_RHS2_GAP * hdr_ratio[1];
	    pIMXDummyCtx->AecMinIntegrationTime = pIMXDummyCtx->MinIntegrationLine * pIMXDummyCtx->one_line_exp_time;
    }

    // assume gain is 1 and see if ratio can be achieved with integration time
    long_it 		= NewIntegrationTime * hdr_ratio[0];
    short_it 		= NewIntegrationTime;
    very_short_it 	= NewIntegrationTime / hdr_ratio[1];
    
    TRACE(IMX_DUMMY_DEBUG, "%s: requested IT long: %f, short: %f, very_short: %f\n", 
    __func__, long_it, short_it, very_short_it);
    long_exp_val 		= long_it / pIMXDummyCtx->one_line_exp_time;
    short_exp_val 		= short_it / pIMXDummyCtx->one_line_exp_time;
    very_short_exp_val 	= very_short_it / pIMXDummyCtx->one_line_exp_time;

    TRACE(IMX_DUMMY_DEBUG, "%s: requested IT in lines long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    long_exp_val 		= IMX_DUMMY_VMAX_3DOL_HDR - long_exp_val;
    short_exp_val 		= rhs1 - short_exp_val;
    very_short_exp_val 	= rhs2 - very_short_exp_val;

    TRACE(IMX_DUMMY_DEBUG, "%s: requested IT in shr long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    if(long_exp_val < rhs2 + IMX_DUMMY_SHR0_RHS2_GAP) {
        long_exp_val = rhs2 + IMX_DUMMY_SHR0_RHS2_GAP;
        long_it = (IMX_DUMMY_VMAX_3DOL_HDR - long_exp_val) * pIMXDummyCtx->one_line_exp_time;
        calculate_gain = true;
        optimize_long_gain = true;
        TRACE(IMX_DUMMY_DEBUG, "%s: long_exp_val is too long, set to %u, new long_it = %f\n",
        __func__, rhs2 + IMX_DUMMY_SHR0_RHS2_GAP, long_it);
    } else if(long_exp_val > IMX_DUMMY_VMAX_3DOL_HDR - IMX_DUMMY_SHR0_FSC_GAP) {
        long_exp_val = IMX_DUMMY_VMAX_3DOL_HDR - IMX_DUMMY_SHR0_FSC_GAP;
        long_it = (IMX_DUMMY_VMAX_3DOL_HDR - long_exp_val) * pIMXDummyCtx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX_DUMMY_DEBUG, "%s: long_exp_val is too short, set to %u, new long_it = %f\n",
        __func__, IMX_DUMMY_VMAX_3DOL_HDR - IMX_DUMMY_SHR0_FSC_GAP, long_it);
    }
    if(short_exp_val < IMX_DUMMY_3DOL_SHR1_RHS1_GAP) {
        short_exp_val = IMX_DUMMY_3DOL_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pIMXDummyCtx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX_DUMMY_DEBUG, "%s: short_exp_val is too long, set to %u, new short_it = %f\n",
        __func__, IMX_DUMMY_3DOL_SHR1_RHS1_GAP, short_it);
    } else if(short_exp_val > rhs1 - IMX_DUMMY_SHR1_RHS1_GAP) {
        short_exp_val = rhs1 - IMX_DUMMY_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pIMXDummyCtx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX_DUMMY_DEBUG, "%s: short_exp_val is too short, set to %u, new short_it = %f\n",
        __func__, rhs1 - IMX_DUMMY_SHR1_RHS1_GAP, short_it);
    }
    if(very_short_exp_val < rhs1 + IMX_DUMMY_SHR2_RHS1_GAP) {
        very_short_exp_val = rhs1 + IMX_DUMMY_SHR2_RHS1_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMXDummyCtx->one_line_exp_time;
        calculate_gain = true;
        optimize_short_gain = true;
        TRACE(IMX_DUMMY_DEBUG, "%s: very_short_exp_val is too long, set to %u, new very_short_it = %f\n",
        __func__, rhs2 + IMX_DUMMY_SHR2_RHS1_GAP, very_short_it);
    } else if(very_short_exp_val > rhs2 - IMX_DUMMY_SHR2_RHS2_GAP) {
        very_short_exp_val = rhs2 - IMX_DUMMY_SHR2_RHS2_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMXDummyCtx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX_DUMMY_DEBUG, "%s: very_short_exp_val is too short, set to %u, new very_short_it = %f\n",
        __func__, rhs2 - IMX_DUMMY_SHR2_RHS2_GAP, very_short_it);
    }

    // need to use gain to achive ratio / requested gain update
    if(calculate_gain || NewGain != pIMXDummyCtx->AecCurGainSEF1) {
        float real_short_gain = _sensorGain2linear(_linear2sensorGain(NewGain));
        long_gain = (short_it * NewGain * hdr_ratio[0]) / long_it;
        if(optimize_long_gain){
             long_gain = _sensorGain2linear(_linear2sensorGain(long_gain) + 1);
             long_it = (short_it * real_short_gain * hdr_ratio[0]) / long_gain;
        }
        short_gain = NewGain;
        very_short_gain = (short_it * NewGain) / (very_short_it * hdr_ratio[1]);
        if(optimize_short_gain){
             very_short_gain = _sensorGain2linear(_linear2sensorGain(very_short_gain) + 1);
             very_short_it = (short_it * real_short_gain) / (very_short_gain * hdr_ratio[1]);
        }
        TRACE(IMX_DUMMY_DEBUG, "%s: calculated gain: long: %f, short: %f, very_short: %f\n",
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

RESULT IMX_Dummy_Calculate2DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it,
                                    float *o_long_gain, float *o_short_gain,
                                    float *hdr_ratio) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    uint32_t vmax;
    size_t dol;
    uint32_t fsc, rhs1;
    float one_line;
    uint32_t max_long_it_lines, min_long_it_lines;
    float short_gain, required_long_ev;
    uint32_t ideal_long_lines, long_it_lines;
    float long_it, long_gain_needed, long_gain = 1.0f;
    uint32_t long_gain_db, max_gain_db;
    float adjusted_long_it, lower_gain, required_lines;

    TRACE(IMX_DUMMY_DEBUG, "%s: enter with NewIntegrationTime=%.6f NewGain=%.4f ratio=%.1f\n",
        __func__, NewIntegrationTime, NewGain, hdr_ratio[0]);

    result = IMX_Dummy_ReadVmax(handle, &vmax);
    CHECK_RESULT_RET(result, "ReadVmax");

    result = IMX_Dummy_GetNumExposures(pIMXDummyCtx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");

    fsc = vmax * (uint32_t)dol;
    rhs1 = pIMXDummyCtx->cur_rhs1;
    one_line = pIMXDummyCtx->one_line_exp_time;

    if (one_line <= 0.0f) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid one_line_exp_time (%.9f)\n", __func__, one_line);
        return (RET_WRONG_CONFIG);
    }

    if (fsc <= rhs1 + IMX_DUMMY_2DOL_SHR0_RHS1_GAP) {
        TRACE(IMX_DUMMY_ERROR, "%s: fsc(%u) <= rhs1(%u) + gap, invalid config\n",
              __func__, fsc, rhs1);
        return (RET_WRONG_CONFIG);
    }

    /* Hardware limits: SHR0 must satisfy  rhs1 + gap <= SHR0 <= fsc - gap. */
    max_long_it_lines = fsc - rhs1 - IMX_DUMMY_2DOL_SHR0_RHS1_GAP;
    min_long_it_lines = IMX_DUMMY_2DOL_SHR0_FSC_GAP;

    /* Quantize short gain to sensor dB steps and compute the total long EV target. */
    short_gain = _sensorGain2linear(_linear2sensorGain(NewGain));
    required_long_ev = NewIntegrationTime * short_gain * hdr_ratio[0];

    /* Step 1: Try to cover the required long EV with integration time alone (gain = 1). */
    ideal_long_lines = (uint32_t)roundf(required_long_ev / one_line);
    long_it_lines = MIN(ideal_long_lines, max_long_it_lines);
    long_it_lines = MAX(long_it_lines, min_long_it_lines);
    long_it = long_it_lines * one_line;

    long_gain_needed = required_long_ev / long_it;

    /* Step 1a: Prefer one extra IT line over applying gain when roundf rounded down. */
    if (long_gain_needed > 1.0f && long_it_lines < max_long_it_lines) {
        uint32_t ceil_lines = long_it_lines + 1;
        float ceil_it = ceil_lines * one_line;
        if (required_long_ev / ceil_it <= 1.0f) {
            long_it_lines = ceil_lines;
            long_it = ceil_it;
            long_gain_needed = required_long_ev / long_it;
        }
    }

    /*
     * Step 1b: When the required EV is within IMX_DUMMY_2DOL_MAX_IT_NEAR_LINES of
     * max_long_it, the shortfall is a float-rounding artifact — accept unity
     * gain and the negligible ratio deviation instead of bumping a gain step.
     */
    if (long_gain_needed > 1.0f && long_it_lines == max_long_it_lines) {
        required_lines = required_long_ev / one_line;
        if (required_lines - (float)max_long_it_lines < (float)IMX_DUMMY_2DOL_MAX_IT_NEAR_LINES) {
            long_gain_needed = 1.0f;
        }
    }

    if (long_gain_needed <= 1.0f) {
        long_gain = 1.0f;
    } else {
        /*
         * Step 2: IT at maximum is not enough — apply the minimum gain that
         * covers the remaining EV.  Quantize up (ceil) to ensure we meet the
         * target, then check whether one step lower still suffices with max IT.
         */
        long_gain_db = _linear2sensorGainCeil(long_gain_needed);
        long_gain = _sensorGain2linear(long_gain_db);

        if (long_gain_db >= 1) {
            lower_gain = _sensorGain2linear(long_gain_db - 1);
            if ((float)max_long_it_lines * one_line * lower_gain >= required_long_ev) {
                long_gain_db--;
                long_gain = lower_gain;
            }
        }

        if (long_gain_db == 0)
            long_gain = 1.0f;

        max_gain_db = _linear2sensorGain(IMX_DUMMY_MAX_GAIN_AEC);
        if (long_gain_db > max_gain_db) {
            long_gain_db = max_gain_db;
            long_gain = _sensorGain2linear(long_gain_db);
        }

        /* Step 3: Re-adjust long IT to match the quantized gain, getting closer to the exact ratio. */
        adjusted_long_it = required_long_ev / long_gain;
        long_it_lines = (uint32_t)roundf(adjusted_long_it / one_line);
        long_it_lines = MIN(long_it_lines, max_long_it_lines);
        long_it_lines = MAX(long_it_lines, min_long_it_lines);
        long_it = long_it_lines * one_line;
    }

    *o_long_it = long_it;
    *o_long_gain = long_gain;
    *o_short_it = NewIntegrationTime;
    *o_short_gain = short_gain;

    TRACE(IMX_DUMMY_DEBUG, "%s: ratio=%.1f vmax=%u fsc=%u rhs1=%u max_long_it_lines=%u\n",
        __func__, hdr_ratio[0], vmax, fsc, rhs1, max_long_it_lines);
    TRACE(IMX_DUMMY_DEBUG, "%s: required_long_ev=%.6f long_it=%.6f(%u lines) long_gain=%.4f\n",
        __func__, required_long_ev, long_it, long_it_lines, long_gain);
    TRACE(IMX_DUMMY_DEBUG, "%s: short_it=%.6f short_gain=%.4f\n",
        __func__, *o_short_it, *o_short_gain);

    return RET_SUCCESS;
}

RESULT IMX_Dummy_IsiExposureControlIss(IsiSensorHandle_t handle, float NewGain,
                                    float NewIntegrationTime,
                                    uint8_t* pNumberOfFramesToSkip,
                                    float* pSetGain, float* pSetIntegrationTime,
                                    float* hdr_ratio) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;

    RESULT result = RET_SUCCESS;
    float long_it = 0.0;
	float short_it = 0.0;
	float very_short_it = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	float very_short_gain = 1;
    uint32_t hmax;

    TRACE(IMX_DUMMY_INFO, "%s: enter with NewIntegrationTime: %f, NewGain: %f\n",
        __func__, NewIntegrationTime, NewGain);

    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pNumberOfFramesToSkip == NULL) || (pSetGain == NULL) ||
        (pSetIntegrationTime == NULL)) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMXDummyCtx->enableHdr) {
        result = IMX_Dummy_ReadRHS1(handle, &pIMXDummyCtx->cur_rhs1);
        CHECK_RESULT_RET(result, "ReadRHS1");
        result = IMX_Dummy_ReadRHS2(handle, &pIMXDummyCtx->cur_rhs2);
        CHECK_RESULT_RET(result, "ReadRHS2");
        result = IMX_Dummy_ReadHmax(handle, &hmax);
        CHECK_RESULT_RET(result, "ReadHmax");

        pIMXDummyCtx->SensorMode.ae_info.one_line_exp_time_ns = (uint32_t)(((float)hmax / IMX_DUMMY_PIXEL_CLK_RATE) * MICRO_2_NANO);
        pIMXDummyCtx->one_line_exp_time =
        (float)(pIMXDummyCtx->SensorMode.ae_info.one_line_exp_time_ns) / 1000000000;

        if (pIMXDummyCtx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
            //2exposure
            result = IMX_Dummy_Calculate2DOLExposures(handle, NewIntegrationTime, NewGain,
                                    &long_it, &short_it,
                                    &long_gain, &short_gain,
                                    hdr_ratio);
            CHECK_RESULT_RET(result, "Calculate2DOLExposures");
        } else {
            //3exposure
            result = IMX_Dummy_Calculate3DOLExposures(handle, NewIntegrationTime, NewGain,
                                        &long_it, &short_it, &very_short_it,
                                        &long_gain, &short_gain, &very_short_gain,
                                        hdr_ratio);
            CHECK_RESULT_RET(result, "Calculate3DOLExposures");
        }

		result = IMX_Dummy_IsiSetLEFIntegrationTimeIss(handle, long_it,
							pSetIntegrationTime,
							pNumberOfFramesToSkip,
							hdr_ratio);
		CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
		result = IMX_Dummy_IsiSetLEFGainIss(handle, long_gain, pSetGain, hdr_ratio);
		CHECK_RESULT_RET(result, "SetLEFGain");
		result = IMX_Dummy_IsiSetSEF1IntegrationTimeIss(
			handle, short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1IntegrationTime");
		result = IMX_Dummy_IsiSetSEF1GainIss(handle, NewIntegrationTime,
						  short_gain, pSetGain, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1Gain");
		if (pIMXDummyCtx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
			result = IMX_Dummy_IsiSetSEF2IntegrationTimeIss(
			    handle, very_short_it, pSetIntegrationTime,
			    pNumberOfFramesToSkip, hdr_ratio);
			CHECK_RESULT_RET(result, "SetSEF2IntegrationTime");
            result = IMX_Dummy_IsiSetSEF2GainIss(handle, NewIntegrationTime,
                            very_short_gain, pSetGain, hdr_ratio);
            CHECK_RESULT_RET(result, "SetSEF2Gain");
        }

        // Recalculate `io_hdr_ratio` according to the set values
        hdr_ratio[0] = (long_it * long_gain) / (short_it * short_gain);
        if (pIMXDummyCtx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            hdr_ratio[1] = (short_it * short_gain) / (very_short_it * very_short_gain);
        }

        // Set the output values to SEF1 values
        *pSetGain = short_gain;
        *pSetIntegrationTime = short_it;

        TRACE(IMX_DUMMY_DEBUG, "%s: actual hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n",
            __func__, hdr_ratio[0], hdr_ratio[1]);
    } else {
        result = IMX_Dummy_IsiSetLEFIntegrationTimeIss(handle, NewIntegrationTime,
                                                pSetIntegrationTime,
                                                pNumberOfFramesToSkip, hdr_ratio);
        CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
        result = IMX_Dummy_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
        CHECK_RESULT_RET(result, "SetLEFGain");
    }
    return RET_SUCCESS;
}

RESULT IMX_Dummy_IsiExposureControlExpandedIss(
    IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime,
    float NewIris, uint8_t* pNumberOfFramesToSkip, float* pSetGain,
    float* pSetIntegrationTime, float* pSetIris, float* hdr_ratio) {

    RESULT result = RET_SUCCESS;
    if (pSetIris) {
        result = IMX_Dummy_IsiSetIrisIss(handle, NewIris);
        CHECK_RESULT_RET(result, "SetIris");
        *pSetIris = NewIris;
    }

    result = IMX_Dummy_IsiExposureControlIss(handle, NewGain, NewIntegrationTime,
                                        pNumberOfFramesToSkip, pSetGain,
                                        pSetIntegrationTime, hdr_ratio);
    CHECK_RESULT_RET(result, "ExposureControl");
    return result;
}

RESULT IMX_Dummy_IsiGetCurrentExposureIss(IsiSensorHandle_t handle,
                                       float* pSetGain,
                                       float* pSetIntegrationTime) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pSetGain == NULL) || (pSetIntegrationTime == NULL))
        return (RET_NULL_POINTER);

    if (pIMXDummyCtx->enableHdr) {
		*pSetGain = pIMXDummyCtx->AecCurGainSEF1;
		*pSetIntegrationTime = pIMXDummyCtx->AecCurIntegrationTimeSEF1;
	} else {
		*pSetGain = pIMXDummyCtx->AecCurGainLEF;
		*pSetIntegrationTime = pIMXDummyCtx->AecCurIntegrationTimeLEF;
	}

    return (result);
}

RESULT IMX_Dummy_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t* pFps) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *pFps = pIMXDummyCtx->CurrFps;

    return (result);
}

RESULT IMX_Dummy_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t Fps) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX_Dummy_IsiSetFlickerFpsIss(IsiSensorHandle_t handle, uint32_t flickerMode) {
    RESULT result = RET_SUCCESS;
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    uint32_t current_vmax = 0;
    uint32_t requested_vmax = 0;
    uint32_t requested_fsc = 0;
    uint32_t shr = 0;
    size_t dol = 0;
    uint32_t fsc = 0;
    uint32_t min_shr0 = (dol == 1) ? IMX_DUMMY_MIN_SHR : IMX_DUMMY_2DOL_SHR0_RHS1_GAP + pIMXDummyCtx->cur_rhs1;
    int exp = 0;

    TRACE(IMX_DUMMY_DEBUG, "%s: set sensor flickerMode = %d\n", __func__, flickerMode);

    if (!pIMXDummyCtx) {
        return RET_NULL_POINTER;
    }
    if (pIMXDummyCtx->flicker_fps_mode == flickerMode) {
        return RET_SUCCESS;
    }
    if (pIMXDummyCtx->enableHdr && (pIMXDummyCtx->SensorMode.stitching_mode != SENSOR_STITCHING_L_AND_S)) {
        return RET_SUCCESS;
    }

    result = IMX_Dummy_GetNumExposures(pIMXDummyCtx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");

    if (flickerMode > ISI_AE_ANTIBANDING_MODE_AUTO) {
        TRACE(IMX_DUMMY_INFO, "%s: Invalid flickerMode (%d), setting ISI_AE_ANTIBANDING_MODE_AUTO instead.\n", __func__, flickerMode);
        flickerMode = ISI_AE_ANTIBANDING_MODE_AUTO;
    }
    pIMXDummyCtx->flicker_fps_mode = flickerMode;

    result = IMX_Dummy_ReadVmax(handle, &current_vmax);
    CHECK_RESULT_RET(result, "ReadVmax");
    if (pIMXDummyCtx->original_vmax == 0) {
        pIMXDummyCtx->original_vmax = current_vmax;
    }

    fsc = current_vmax * dol;

    exp = pIMXDummyCtx->AecCurIntegrationTimeLEF / pIMXDummyCtx->one_line_exp_time;
    shr = MAX((int)(fsc) - exp, min_shr0);

    if (current_vmax > pIMXDummyCtx->original_vmax) {
        current_vmax = MAX((int)fsc - (int)shr + min_shr0, min_shr0);
        fsc = current_vmax * dol;
        shr = MAX((int)fsc - exp, min_shr0);
        pIMXDummyCtx->unlimit_fps_vmax_changed = current_vmax > pIMXDummyCtx->original_vmax && pIMXDummyCtx->unlimit_fps;
    }

    result = IMX_Dummy_getNewVmaxAntiFlicker(pIMXDummyCtx, current_vmax, &requested_vmax);
    CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
    requested_vmax = MAX( MIN(requested_vmax, IMX_DUMMY_VMAX_MAX), 1);
    requested_fsc = requested_vmax * dol;
    
    if (current_vmax != requested_vmax) {
        shr = MAX( (int)requested_fsc - (int)fsc + (int)shr, min_shr0);
        TRACE(IMX_DUMMY_DEBUG, "%s - writing 0x%x to VMAX, writing 0x%x to SHR0\n", __func__, requested_vmax, shr);
        
        result = IMX_Dummy_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        result = IMX_Dummy_WriteVmax(handle, requested_vmax);
        CHECK_RESULT_RET(result, "WriteVmax");
        result = IMX_Dummy_WriteShr0(handle, shr);
        CHECK_RESULT_RET(result, "WriteShr0");
        result = IMX_Dummy_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");
        result = IMX_Dummy_UpdateCurrLEFIntegrationTimeFromVmax(pIMXDummyCtx, requested_vmax, shr);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromVmax");
    }
    
    // these 2 are being used only in SDR
    pIMXDummyCtx->MaxIntegrationLine = MAX( MIN(requested_vmax - IMX_DUMMY_MIN_SHR, IMX_DUMMY_VMAX_MAX - IMX_DUMMY_MIN_SHR), 1);
    pIMXDummyCtx->AecMaxIntegrationTime = pIMXDummyCtx->one_line_exp_time * pIMXDummyCtx->MaxIntegrationLine;

    TRACE(IMX_DUMMY_INFO, "%s: set sensor fps = %d\n", __func__,
          pIMXDummyCtx->CurrFps);

    TRACE(IMX_DUMMY_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX_Dummy_IsiGetAutoFpsInfoIss(IsiSensorHandle_t handle,
                                   IsiAutoFps_t* pAutoFpsInfo) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX_Dummy_IsiGetStartEvIss(IsiSensorHandle_t handle, uint64_t* pStartEv) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX_Dummy_IsiGetIspStatusIss(IsiSensorHandle_t handle,
                                 IsiIspStatus_t* pIspStatus) {
    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    if (pIMXDummyCtx == NULL || pIMXDummyCtx->IsiCtx.HalHandle == NULL) {
        return RET_WRONG_HANDLE;
    }

    pIspStatus->useSensorAE = false;
    pIspStatus->useSensorBLC = false;
    pIspStatus->useSensorAWB = false;

    return RET_SUCCESS;
}

RESULT IMX_Dummy_IsiSetTpgIss(IsiSensorHandle_t handle, IsiTpg_t Tpg) {
    RESULT result = RET_SUCCESS;

    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    if (pIMXDummyCtx == NULL || pIMXDummyCtx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMXDummyCtx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (Tpg.enable == 0) {
        result = IMX_Dummy_IsiWriteRegIss(handle, 0x3253, 0x00);
        CHECK_RESULT_RET(result, "disable test pattern");
    } else {
        result = IMX_Dummy_IsiWriteRegIss(handle, 0x3253, 0x80);
        CHECK_RESULT_RET(result, "enable test pattern");
    }

    pIMXDummyCtx->TestPattern = Tpg.enable;

    return (result);
}

RESULT IMX_Dummy_IsiGetTpgIss(IsiSensorHandle_t handle, IsiTpg_t* Tpg) {
    RESULT result = RET_SUCCESS;
    uint32_t value = 0;

    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    if (pIMXDummyCtx == NULL || pIMXDummyCtx->IsiCtx.HalHandle == NULL ||
        Tpg == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMXDummyCtx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    result = IMX_Dummy_IsiReadRegIss(handle, 0x5081, &value);
    CHECK_RESULT_RET(result, "read TPG register");

    Tpg->enable = ((value & 0x80) != 0) ? 1 : 0;
    if (Tpg->enable) {
        Tpg->pattern = (0xff & value);
    }
    pIMXDummyCtx->TestPattern = Tpg->enable;

    return (result);
}

RESULT IMX_Dummy_IsiFocusCreateIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX_Dummy_IsiFocusReleaseIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX_Dummy_IsiFocusGetCalibrateIss(IsiSensorHandle_t handle,
                                      IsiFocusCalibAttr_t* pFocusCalib) {
    return RET_SUCCESS;
}

RESULT IMX_Dummy_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

RESULT IMX_Dummy_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

// Taken from the IMX681 driver
RESULT IMX_Dummy_IsiSetAgainDgainIss(IsiSensorHandle_t handle,
                                  IsiUserGain_t Gain) {
    RESULT result = RET_SUCCESS;
    uint32_t Again = 0, Dgain = 0;
    TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);

    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((Gain.again < 1) | (Gain.again > 16)) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor again (%f)\n", __func__, Gain.again);
        return (RET_OUTOFRANGE);
    }
    // Again = (uint32_t)(1024 - (1024/Gain.again));
    Again = (uint32_t)(((2048 * Gain.again) - 2048) / Gain.again);

    result = IMX_Dummy_IsiWriteRegIss(handle, 0x309c, (Again & 0x0000FF));
    CHECK_RESULT_RET(result, "write again low");
    result = IMX_Dummy_IsiWriteRegIss(handle, 0x309d, (Again & 0x00FF00) >> 8);
    CHECK_RESULT_RET(result, "write again high");

    if ((Gain.dgain < 1) | (Gain.dgain > 16)) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid sensor dgain (%f)\n", __func__, Gain.dgain);
        return (RET_OUTOFRANGE);
    }
    Dgain = Gain.dgain * 256;

    result = IMX_Dummy_IsiWriteRegIss(handle, 0x308c, (Dgain & 0x0000FF));
    CHECK_RESULT_RET(result, "write dgain low");
    result = IMX_Dummy_IsiWriteRegIss(handle, 0x308d, (Dgain & 0x00FF00) >> 8);
    CHECK_RESULT_RET(result, "write dgain high");

    // pIMXDummyCtx->CurAgain = Gain.again;
    // pIMXDummyCtx->CurDgain = Gain.dgain;

    TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX_Dummy_IsiGetIrisIss( IsiSensorHandle_t handle,
                                     float *pSetIris ) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);

    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    //TODO implement getting iris here!
    *pSetIris = 1.0;

    TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX_Dummy_IsiSetIrisIss( IsiSensorHandle_t handle, 
                    const float NewIris) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);

    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (NewIris < IMX_DUMMY_IRIS_MIN_VAL || NewIris > IMX_DUMMY_IRIS_MAX_VAL) {
        TRACE(IMX_DUMMY_ERROR, "%s: Invalid Iris %f\n", __func__, NewIris);
    }

    //TODO implement setting iris here!

    TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX_Dummy_IsiGetHCGIss( IsiSensorHandle_t handle,
                                     bool *phcg_lef, bool *phcg_sef1, bool *phcg_sef2 ) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);

    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;
    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *phcg_lef = pIMXDummyCtx->hcg_lef;
    *phcg_sef1 = pIMXDummyCtx->hcg_sef1;
    *phcg_sef2 = pIMXDummyCtx->hcg_sef2;

    TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX_Dummy_IsiSetHCGIss(IsiSensorHandle_t handle, bool hcg_lef, bool hcg_sef1, bool hcg_sef2) {

    RESULT result = RET_SUCCESS;

    TRACE(IMX_DUMMY_INFO, "%s: (enter)\n", __func__);

    IMX_dummy_Context_t* pIMXDummyCtx = (IMX_dummy_Context_t*)handle;

    if (pIMXDummyCtx == NULL) {
        TRACE(IMX_DUMMY_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    result = IMX_Dummy_IsiWriteRegIss(handle, 0x3030 , hcg_lef);
    CHECK_RESULT_RET(result, "write HCG LEF");
    pIMXDummyCtx->hcg_lef = hcg_lef;

    TRACE(IMX_DUMMY_INFO, "%s: (exit)\n", __func__);
    return result;

}



RESULT IMX_Dummy_IsiGetSensorIss(IsiSensor_t* pIsiSensor) {
    RESULT result = RET_SUCCESS;
    static const char SensorName[16] = "IMX_Dummy";
    TRACE(IMX_DUMMY_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor != NULL) {
        pIsiSensor->pszName = SensorName;
        pIsiSensor->pIsiCreateIss = IMX_Dummy_IsiCreateIss;
        pIsiSensor->pIsiReleaseIss = IMX_Dummy_IsiReleaseIss;
        pIsiSensor->pIsiReadRegIss = IMX_Dummy_IsiReadRegIss;
        pIsiSensor->pIsiWriteRegIss = IMX_Dummy_IsiWriteRegIss;
        pIsiSensor->pIsiGetModeIss = IMX_Dummy_IsiGetModeIss;
        pIsiSensor->pIsiSetModeIss = IMX_Dummy_IsiSetModeIss;
        pIsiSensor->pIsiEnumModeIss = IMX_Dummy_IsiEnumModeIss;
        pIsiSensor->pIsiGetCapsIss = IMX_Dummy_IsiGetCapsIss;
        pIsiSensor->pIsiSetupIss = IMX_Dummy_IsiSetupIss;
        pIsiSensor->pIsiCheckConnectionIss = IMX_Dummy_IsiCheckConnectionIss;
        pIsiSensor->pIsiGetRevisionIss = IMX_Dummy_IsiGetRevisionIss;
        pIsiSensor->pIsiSetStreamingIss = IMX_Dummy_IsiSetStreamingIss;

        /* AEC functions */
        pIsiSensor->pIsiGetGainLimitsIss = 					IMX_Dummy_IsiGetGainLimitsIss;
		pIsiSensor->pIsiGetIrisLimitsIss = 					IMX_Dummy_IsiGetIrisLimitsIss;
		pIsiSensor->pIsiSetIrisLimitsIss = 					IMX_Dummy_IsiSetIrisLimitsIss;
		pIsiSensor->pIsiGetIntegrationTimeLimitsIss =		IMX_Dummy_IsiGetIntegrationTimeLimitsIss;

		pIsiSensor->pIsiExposureControlIss =				IMX_Dummy_IsiExposureControlIss;
		pIsiSensor->pIsiExposureControlExpandedIss =		IMX_Dummy_IsiExposureControlExpandedIss;
		pIsiSensor->pIsiSetIntegrationTimeIss =				IMX_Dummy_IsiSetIntegrationTimeIss;

		pIsiSensor->pIsiGetLongIntegrationTimeIss =			IMX_Dummy_IsiGetLEFIntegrationTimeIss;
		pIsiSensor->pIsiGetIntegrationTimeIss =				IMX_Dummy_IsiGetIntegrationTimeIss;
		pIsiSensor->pIsiGetVSIntegrationTimeIss =			IMX_Dummy_IsiGetSEF2IntegrationTimeIss;

		pIsiSensor->pIsiGetLongGainIss = 					IMX_Dummy_IsiGetLEFGainIss;
		pIsiSensor->pIsiGetGainIss = 						IMX_Dummy_IsiGetGainIss;
		pIsiSensor->pIsiGetVSGainIss = 						IMX_Dummy_IsiGetSEF2GainIss;

		pIsiSensor->pIsiGetGainIncrementIss =				IMX_Dummy_IsiGetGainIncrementIss;
		pIsiSensor->pIsiGetIrisIncrementIss =				IMX_Dummy_IsiGetIrisIncrementIss;
		pIsiSensor->pIsiGetIntegrationTimeIncrementIss =	IMX_Dummy_IsiGetIntegrationTimeIncrementIss;
		pIsiSensor->pIsiSetGainIss = 						IMX_Dummy_IsiSetGainIss;
		pIsiSensor->pIsiGetFpsIss = 						IMX_Dummy_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss = 						IMX_Dummy_IsiSetFpsIss;
        pIsiSensor->pIsiSetFlickerFpsIss = 					IMX_Dummy_IsiSetFlickerFpsIss;
        pIsiSensor->pIsiUnlimitFpsIss =						IMX_Dummy_IsiUnlimitFpsIss;
        pIsiSensor->pIsiLimitFpsIss =						IMX_Dummy_IsiLimitFpsIss;
		pIsiSensor->pIsiGetAutoFpsInfoIss = 				IMX_Dummy_IsiGetAutoFpsInfoIss;
		pIsiSensor->pIsiGetStartEvIss = 					IMX_Dummy_IsiGetStartEvIss;
        pIsiSensor->pIsiGetIrisIss =						IMX_Dummy_IsiGetIrisIss;
        pIsiSensor->pIsiSetIrisIss =						IMX_Dummy_IsiSetIrisIss;
        pIsiSensor->pIsiGetHCGIss =                         IMX_Dummy_IsiGetHCGIss;
        pIsiSensor->pIsiSetHCGIss =                         IMX_Dummy_IsiSetHCGIss;

        /* SENSOR ISP */
        pIsiSensor->pIsiGetIspStatusIss = IMX_Dummy_IsiGetIspStatusIss;
        // pIsiSensor->pIsiSetBlcIss                       =
        // IMX_Dummy_IsiSetBlcIss; pIsiSensor->pIsiSetWBIss = IMX_Dummy_IsiSetWBIss;

        /* SENSOE OTHER FUNC*/
        pIsiSensor->pIsiSetPowerIss = IMX_Dummy_IsiSetPowerIss;
        pIsiSensor->pIsiSetTpgIss = IMX_Dummy_IsiSetTpgIss;
        pIsiSensor->pIsiGetTpgIss = IMX_Dummy_IsiGetTpgIss;
        // pIsiSensor->pIsiGetExpandCurveIss               =
        // IMX_Dummy_IsiGetExpandCurveIss; pIsiSensor->pIsiGetCompressCurveIss =
        // IMX_Dummy_IsiGetCompressCurveIss; pIsiSensor->pIsiExtendFuncIss =
        // IMX_Dummy_IsiExtendFuncIss; pIsiSensor->pIsiGetOtpDataIss =
        // IMX_Dummy_IsiGetOtpDataIss;

        /* AF */
        pIsiSensor->pIsiFocusCreateIss = IMX_Dummy_IsiFocusCreateIss;
        pIsiSensor->pIsiFocusReleaseIss = IMX_Dummy_IsiFocusReleaseIss;
        pIsiSensor->pIsiFocusGetCalibrateIss = IMX_Dummy_IsiFocusGetCalibrateIss;
        pIsiSensor->pIsiFocusSetIss = IMX_Dummy_IsiFocusSetIss;
        pIsiSensor->pIsiFocusGetIss = IMX_Dummy_IsiFocusGetIss;
        pIsiSensor->pIsiSetAgainDgainIss = IMX_Dummy_IsiSetAgainDgainIss;

    } else {
        result = RET_NULL_POINTER;
    }

    TRACE(IMX_DUMMY_INFO, "%s (exit)\n", __func__);
    return (result);
}

/*****************************************************************************
 * each sensor driver need declare this struct for isi load
 *****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x9012,
    .pIsiHalEnumModeIss = IMX_Dummy_IsiHalEnumModeIss,
    .pIsiGetSensorIss = IMX_Dummy_IsiGetSensorIss,
};
