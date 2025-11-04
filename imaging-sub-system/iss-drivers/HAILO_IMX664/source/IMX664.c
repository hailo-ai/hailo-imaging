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

#include "IMX664_priv.h"
#include "vvsensor.h"
#include <linux/i2c.h>

CREATE_TRACER(IMX664_INFO, "IMX664: ", INFO, 1)
CREATE_TRACER(IMX664_WARN, "IMX664: ", WARNING, 1)
CREATE_TRACER(IMX664_ERROR, "IMX664: ", ERROR, 1)
CREATE_TRACER(IMX664_DEBUG, "IMX664: ", INFO, 1)
CREATE_TRACER(IMX664_REG_INFO, "IMX664: ", INFO, 1)
CREATE_TRACER(IMX664_REG_DEBUG, "IMX664: ", INFO, 1)

#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

/** @defgroup group_defines  API definitions
 *  @{
 */

#define IMX664_I2C_ADDR 0x1a
#define IMX664_IRIS_MIN_VAL 1

/** @} */ // end of group_defines

#define IMX664_IRIS_MAX_VAL 1
#define IMX664_MIN_GAIN_STEP                                          \
    (0.035) /**< min gain step size used by GUI (hardware min = 1/16; \
               1/16..32/16 depending on actual gain ) */
#define IMX664_PLL_PCLK 74250000
#define IMX664_VMAX_30FPS 3300
#define IMX664_VMAX_3DOL_HDR 6750
#define IMX664_VMAX_2DOL_HDR 3300 /* 1650 * 2 */
#define IMX664_VMAX_MAX 135000
#define IMX664_MIN_SHR 8
#define IMX664_MAX_GAIN_AEC                                                    \
    (32.0f) /**< max. gain used by the AEC (arbitrarily chosen, hardware limit \
               = 62.0, driver limit = 32.0 ) */
#define IMX664_VS_MAX_INTEGRATION_TIME (0.0018)
#define IMX664_TRANSFER_BUFFER_LENGTH 3
#define IMX664_MAX_GAIN 3981
#define IMX664_SHR0_RHS2_GAP 7
#define IMX664_2DOL_SHR0_RHS1_GAP 10
#define IMX664_SHR0_FSC_GAP 3
#define IMX664_2DOL_SHR0_FSC_GAP 4
#define IMX664_3DOL_SHR1_RHS1_GAP 4
#define IMX664_2DOL_SHR1_MIN_GAP 10
#define IMX664_3DOL_SHR1_MIN_GAP 7
#define IMX664_SHR1_RHS1_GAP 3
#define IMX664_2DOL_SHR1_RHS1_GAP 2
#define IMX664_SHR2_RHS1_GAP 7
#define IMX664_SHR2_RHS2_GAP 3
#define IMX664_PIXEL_CLK_RATE 74.25
#define MICRO_2_NANO 1000
#define IMX664_2DOL_NUM_EXP 2
#define IMX664_3DOL_NUM_EXP 3

/* resolutions */
#define IMX664_RES_4MP_HEIGHT 1520
#define IMX664_RES_4MP_WIDTH 2688
#define IMX664_RES_FHD_HEIGHT 1080
#define IMX664_RES_FHD_WIDTH 1920

FlickerModePeaksPerSec flickerPeaksPerSecMap[] = {
    { ISI_AE_ANTIBANDING_MODE_OFF, 0 },
    { ISI_AE_ANTIBANDING_MODE_50HZ, 100 },
    { ISI_AE_ANTIBANDING_MODE_60HZ, 120 },
    { ISI_AE_ANTIBANDING_MODE_AUTO, 100 }
};

/*****************************************************************************
 *Forward Declarations
*****************************************************************************/
RESULT IMX664_IsiSetIrisIss( IsiSensorHandle_t handle,
                    const float NewIris);

/*****************************************************************************
 *Sensor Info
*****************************************************************************/

static struct vvsensor_mode_s pimx664_mode_info[] = {
    {
        .index     = 0,
        .size      ={
			.bounds_width  = IMX664_RES_4MP_WIDTH,
			.bounds_height = IMX664_RES_4MP_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX664_RES_4MP_WIDTH,
			.height        = IMX664_RES_4MP_HEIGHT,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_RGGB,
	.ae_info = {
		.one_line_exp_time_ns = 10101,
		.min_integration_time = 8,
		.integration_accuracy = 1,
		.max_gain = IMX664_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
    {
        .index     = 1,
        .size      ={
			.bounds_width  = IMX664_RES_FHD_WIDTH,
			.bounds_height = IMX664_RES_FHD_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX664_RES_FHD_WIDTH,
			.height        = IMX664_RES_FHD_HEIGHT,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_RGGB,
	.ae_info = {
		.one_line_exp_time_ns = 7407,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX664_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
    {
        .index     = 2,
        .size      ={
			.bounds_width  = IMX664_RES_FHD_WIDTH,
			.bounds_height = IMX664_RES_FHD_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX664_RES_FHD_WIDTH,
			.height        = IMX664_RES_FHD_HEIGHT,
		},
		.fps       = 20 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_3DOL,
		.bit_width = 12,
		.bayer_pattern = BAYER_RGGB,
        .ae_info = {
		.one_line_exp_time_ns = 7407,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX664_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 20
	}
    },
    {
        .index     = 3,
        .size      ={
			.bounds_width  = IMX664_RES_4MP_WIDTH,
			.bounds_height = IMX664_RES_4MP_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX664_RES_4MP_WIDTH,
			.height        = IMX664_RES_4MP_HEIGHT,
		},
		.fps       = 20 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_3DOL,
		.bit_width = 12,
		.bayer_pattern = BAYER_RGGB,
        .ae_info = {
		.one_line_exp_time_ns = 7407,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX664_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 20
	}
    },
    {
        .index     = 4,
        .size      ={
			.bounds_width  = IMX664_RES_4MP_WIDTH,
			.bounds_height = IMX664_RES_4MP_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX664_RES_4MP_WIDTH,
			.height        = IMX664_RES_4MP_HEIGHT,
		},
		.fps       = 30 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_L_AND_S,
		.bit_width = 12,
		.bayer_pattern = BAYER_RGGB,
        .ae_info = {
		.one_line_exp_time_ns = 10101,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX664_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    }

};

static RESULT IMX664_IsiSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    RESULT result = RET_SUCCESS;

    return (result);
}

static int IMX664_GetCtrl(int sensor_fd, unsigned int ctrl_cid)
{
	struct v4l2_control control;
	memset(&control, 0, sizeof(control));
	control.id = ctrl_cid;

	int ret = ioctl(sensor_fd, VIDIOC_G_CTRL, &control);
	if (ret < 0) {
		TRACE(IMX664_ERROR, "%s: ioctl failed with error %d (%s). ret= %d\n",
			__func__, errno, strerror(errno), ret);
		return -1;
	}

	return control.value;
}

static uint32_t IMX664_CalculateModeMaxIntegrationTime(
        uint32_t hdr_mode, uint32_t stitching_mode, uint32_t rhs1) {
	if (hdr_mode == SENSOR_MODE_LINEAR) {
		return IMX664_VMAX_30FPS - IMX664_MIN_SHR; // SDR
	} else {
        uint32_t rhs1_gap = (stitching_mode == SENSOR_STITCHING_3DOL) ?
            IMX664_3DOL_SHR1_RHS1_GAP : IMX664_2DOL_SHR1_RHS1_GAP;
        return rhs1 - rhs1_gap; // HDR
	}

	TRACE(IMX664_ERROR, "%s: Unsupported HDR mode %d\n", __func__, hdr_mode);
	return 0;
}

static RESULT IMX664_SetSensorModeData(IMX664_Context_t* pIMX664Ctx, size_t index) {
    int sensor_fd = ((HalContext_t*)pIMX664Ctx->IsiCtx.HalHandle)->sensor_fd;
    uint32_t rhs1 = 0;

    if (pIMX664Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    if (index >= ARRAY_SIZE(pimx664_mode_info)) {
        return RET_OUTOFRANGE;
    }

    pIMX664Ctx->SensorMode.index = index;
    memcpy(&pIMX664Ctx->SensorMode, &pimx664_mode_info[index], sizeof(pIMX664Ctx->SensorMode));
    if (pIMX664Ctx->SensorMode.index != index) {
        TRACE(IMX664_ERROR, "%s: Sensor mode index mismatch: expected %zu, got %u\n",
              __func__, index, pIMX664Ctx->SensorMode.index);
        return RET_FAILURE;
    }

    // If the mode is HDR, we need to get RHS1 value from the sensor.
    if (pIMX664Ctx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        /* Get the RHS1 value via a control rather then reading the register,
         * because the register is not available until a stream is running. */
        rhs1 = IMX664_GetCtrl(sensor_fd, IMX664_CID_RHS1);
        if (rhs1 == 0 || rhs1 == -1) {
            TRACE(IMX664_ERROR, "%s: Failed to get RHS1 value from sensor (%d)\n",
                __func__, rhs1);
            return RET_FAILURE;
        }
    }

    pIMX664Ctx->SensorMode.ae_info.max_integration_time =
        IMX664_CalculateModeMaxIntegrationTime(
            pIMX664Ctx->SensorMode.hdr_mode, pIMX664Ctx->SensorMode.stitching_mode, rhs1);

    if (pIMX664Ctx->SensorMode.ae_info.max_integration_time == 0) {
        TRACE(IMX664_ERROR, "%s: Invalid max integration time calculated (%u) for HDR mode %d, index %zu\n",
              __func__, pIMX664Ctx->SensorMode.ae_info.max_integration_time,
              pIMX664Ctx->SensorMode.hdr_mode, index);
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}


static RESULT IMX664_IsiCreateIss(IsiSensorInstanceConfig_t* pConfig) {
    RESULT result = RET_SUCCESS;
    IMX664_Context_t* pIMX664Ctx;
    char i2c_file_path[PATH_MAX];

    memset(i2c_file_path, 0, PATH_MAX);

    if (!pConfig || !pConfig->pSensor) return (RET_NULL_POINTER);

    pIMX664Ctx = (IMX664_Context_t*)malloc(sizeof(IMX664_Context_t));
    if (!pIMX664Ctx) {
        return (RET_OUTOFMEM);
    }

    MEMSET(pIMX664Ctx, 0, sizeof(IMX664_Context_t));
    result = HalAddRef(pConfig->HalHandle);
    if (result != RET_SUCCESS) {
        free(pIMX664Ctx);
        pIMX664Ctx = NULL;
        return (result);
    }

    pIMX664Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pIMX664Ctx->IsiCtx.pSensor = pConfig->pSensor;
    pIMX664Ctx->GroupHold = BOOL_FALSE;
    pIMX664Ctx->OldGain = 1.0;
    pIMX664Ctx->OldIntegrationTime = 0.01;
    pIMX664Ctx->Configured = BOOL_FALSE;
    pIMX664Ctx->Streaming = BOOL_FALSE;
    pIMX664Ctx->TestPattern = BOOL_FALSE;
    pIMX664Ctx->isAfpsRun = BOOL_FALSE;
    
    result = IMX664_SetSensorModeData(pIMX664Ctx, pConfig->SensorModeIndex);
    if (result != RET_SUCCESS) {
        TRACE(IMX664_ERROR, "%s: Set sensor mode data failed! (%d)\n", __func__, result);
        free(pIMX664Ctx);
        pIMX664Ctx = NULL;
        return result;
    }

    pConfig->hSensor = (IsiSensorHandle_t)pIMX664Ctx;
    pIMX664Ctx->pattern = ISI_BPAT_RGRGGBGB;
    pIMX664Ctx->subdev = HalGetFdHandle(pConfig->HalHandle,
                                        HAL_MODULE_SENSOR);  // two sensors??
    pIMX664Ctx->KernelDriverFlag = 1;
    TRACE(IMX664_INFO, "%s - sensor i2c bus: %d, af i2c bus: %d, sensor i2c addr: 0x%x, af i2c addr: 0x%x\n", __func__,
           pConfig->I2cBusNum, pConfig->I2cAfBusNum, pConfig->SlaveAddr, pConfig->SlaveAfAddr);
    pIMX664Ctx->i2c_addr = pConfig->SlaveAddr;
    pIMX664Ctx->i2c_af_addr = pConfig->SlaveAfAddr;

    sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cBusNum);
    pIMX664Ctx->i2c_fd = open(i2c_file_path, O_RDWR);
    if (pIMX664Ctx->i2c_fd < 0) {
        TRACE(IMX664_INFO, "unable to open /dev/i2c-%d\n", pConfig->I2cBusNum);
        return RET_FAILURE;
    }

    if (ioctl(pIMX664Ctx->i2c_fd, I2C_SLAVE_FORCE, pIMX664Ctx->i2c_addr) < 0) {
        TRACE(IMX664_INFO, "unable to set I2C_SLAVE_FORCE on /dev/i2c-%d\n",
              pConfig->I2cBusNum);
        return RET_FAILURE;
    }

    if (pConfig->I2cAfBusNum < 0) {
        TRACE(IMX664_INFO, "no af i2c bus\n");
        pIMX664Ctx->SensorMode.af_mode = ISI_SENSOR_AF_MODE_NOTSUPP;
    } else {
        sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cAfBusNum);
        pIMX664Ctx->i2c_af_fd = open(i2c_file_path, O_RDWR);
        if (pIMX664Ctx->i2c_af_fd < 0) {
            TRACE(IMX664_INFO, "unable to open /dev/i2c-%d\n",
                  pConfig->I2cAfBusNum);
            return RET_FAILURE;
        }
        pIMX664Ctx->SensorMode.af_mode = ISI_SENSOR_AF_MODE_CDAF;
    }

    return (result);
}

static RESULT IMX664_IsiReleaseIss(IsiSensorHandle_t handle) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX664Ctx == NULL) return (RET_WRONG_HANDLE);

    (void)IMX664_IsiSetStreamingIss(pIMX664Ctx, BOOL_FALSE);
    (void)IMX664_IsiSetPowerIss(pIMX664Ctx, BOOL_FALSE);
    (void)HalDelRef(pIMX664Ctx->IsiCtx.HalHandle);
    close(pIMX664Ctx->i2c_fd);
    MEMSET(pIMX664Ctx, 0, sizeof(IMX664_Context_t));
    free(pIMX664Ctx);
    pIMX664Ctx = NULL;
    return (result);
}

static RESULT IMX664_IsiReadRegIss(IsiSensorHandle_t handle,
                                   const uint32_t Addr, uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    struct i2c_rdwr_ioctl_data ioctl_data;
    unsigned char out[IMX664_TRANSFER_BUFFER_LENGTH];
    struct i2c_msg msgs[2];
    uint8_t addr_buf[2] = { (Addr >> 8) & 0xff, Addr & 0xff };

    if (pIMX664Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    msgs[0].addr = pIMX664Ctx->i2c_addr;
    msgs[0].flags = 0; // Write
    msgs[0].len = sizeof(addr_buf);
    msgs[0].buf = addr_buf;

    msgs[1].addr = pIMX664Ctx->i2c_addr;
    msgs[1].flags = I2C_M_RD; // Read
    msgs[1].len = 1;
    msgs[1].buf = out;

    ioctl_data.msgs = msgs;
    ioctl_data.nmsgs = 2;

    if (ioctl(pIMX664Ctx->i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        return RET_FAILURE;
    }

    *pValue = out[0];

    return (result);
}

static RESULT IMX664_IsiWriteRegIss(IsiSensorHandle_t handle,
                                    const uint32_t Addr, const uint32_t Value) {
    RESULT result = RET_SUCCESS;
    char out[IMX664_TRANSFER_BUFFER_LENGTH];

    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (pIMX664Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX664_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    out[2] = Value;
    if (write(pIMX664Ctx->i2c_fd, out, sizeof(out)) != sizeof(out))
        result = RET_FAILURE;
    return (result);
}

static RESULT IMX664_UpdateFps(IMX664_Context_t *pIMX664Ctx, uint32_t vmax) {
    float frame_time = 0;
    frame_time = (vmax * pIMX664Ctx->one_line_exp_time);
    if (frame_time == 0) return RET_FAILURE;

    pIMX664Ctx->CurrFps = (uint32_t)(ceil(1 / frame_time)) * ISI_FPS_ACCURACY;
    return RET_SUCCESS;
}

static RESULT IMX664_ReadVmax(IsiSensorHandle_t handle, uint32_t* vmax) {
    uint32_t vmax_low = 0, vmax_mid = 0, vmax_high = 0;
    RESULT result;

    result = IMX664_IsiReadRegIss(handle, 0x3028, &vmax_low);
    result |= IMX664_IsiReadRegIss(handle, 0x3029, &vmax_mid);
    result |= IMX664_IsiReadRegIss(handle, 0x302a, &vmax_high);
    if (result) return RET_FAILURE;

    *vmax = (vmax_high << 16) | (vmax_mid << 8) | vmax_low;
    return result;
}

static RESULT IMX664_ReadHmax(IsiSensorHandle_t handle, uint32_t* hmax) {
    uint32_t hmax_low = 0, hmax_high = 0;
    RESULT result;

    result = IMX664_IsiReadRegIss(handle, 0x302c, &hmax_low);
    result |= IMX664_IsiReadRegIss(handle, 0x302d, &hmax_high);
    if (result) return RET_FAILURE;

    *hmax = (hmax_high << 8) | hmax_low;
    return result;
}

static RESULT IMX664_WriteVmax(IsiSensorHandle_t handle, uint32_t vmax) {
    RESULT result;

    result = IMX664_IsiWriteRegIss(handle, 0x3028, vmax & 0xff);
    result |= IMX664_IsiWriteRegIss(handle, 0x3029, (vmax >> 8) & 0xff);
    result |= IMX664_IsiWriteRegIss(handle, 0x302a, (vmax >> 16) & 0x0f);
    if (!result) {
        return IMX664_UpdateFps((IMX664_Context_t *)handle, vmax);
    }

    return result;
}

static RESULT IMX664_ReadRHS1(IsiSensorHandle_t handle, uint32_t* rhs1) {
    uint32_t rhs1_low = 0, rhs1_mid = 0, rhs1_high = 0;
    RESULT result;

    result = IMX664_IsiReadRegIss(handle, 0x3060, &rhs1_low);
    result |= IMX664_IsiReadRegIss(handle, 0x3061, &rhs1_mid);
    result |= IMX664_IsiReadRegIss(handle, 0x3062, &rhs1_high);
    if (result) return RET_FAILURE;

    *rhs1 = (rhs1_high << 16) | (rhs1_mid << 8) | rhs1_low;
    return result;
}

static RESULT IMX664_ReadRHS2(IsiSensorHandle_t handle, uint32_t* rhs2) {
    uint32_t rhs2_low = 0, rhs2_mid = 0, rhs2_high = 0;
    RESULT result;

    result = IMX664_IsiReadRegIss(handle, 0x3064, &rhs2_low);
    result |= IMX664_IsiReadRegIss(handle, 0x3065, &rhs2_mid);
    result |= IMX664_IsiReadRegIss(handle, 0x3066, &rhs2_high);
    if (result) return RET_FAILURE;

    *rhs2 = (rhs2_high << 16) | (rhs2_mid << 8) | rhs2_low;
    return result;
}


static RESULT IMX664_WriteShr0(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX664_IsiWriteRegIss(handle, 0x3050, shr & 0xff);
    result |= IMX664_IsiWriteRegIss(handle, 0x3051, (shr >> 8) & 0xff);
    result |= IMX664_IsiWriteRegIss(handle, 0x3052, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX664_WriteShr1(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX664_IsiWriteRegIss(handle, 0x3054, shr & 0xff);
    result |= IMX664_IsiWriteRegIss(handle, 0x3055, (shr >> 8) & 0xff);
    result |= IMX664_IsiWriteRegIss(handle, 0x3056, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX664_WriteShr2(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX664_IsiWriteRegIss(handle, 0x3058, shr & 0xff);
    result |= IMX664_IsiWriteRegIss(handle, 0x3059, (shr >> 8) & 0xff);
    result |= IMX664_IsiWriteRegIss(handle, 0x305a, (shr >> 16) & 0x0f);

    return result;
}

static RESULT IMX664_WriteGain(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX664_IsiWriteRegIss(handle, 0x3070, (gain & 0x00ff));
	result |= IMX664_IsiWriteRegIss(handle, 0x3071, (gain & 0x0700) >> 8);

    return result;
}

static RESULT IMX664_WriteGain1(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX664_IsiWriteRegIss(handle, 0x3072, (gain & 0x00ff));
	result |= IMX664_IsiWriteRegIss(handle, 0x3073, (gain & 0x0700) >> 8);

    return result;
}

static RESULT IMX664_WriteGain2(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX664_IsiWriteRegIss(handle, 0x3074, (gain & 0x00ff));
	result |= IMX664_IsiWriteRegIss(handle, 0x3075, (gain & 0x0700) >> 8);

    return result;
}

static RESULT IMX664_LockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX664_IsiWriteRegIss(handle, 0x3001, 0x1);

    return result;
}

static RESULT IMX664_UnlockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX664_IsiWriteRegIss(handle, 0x3001, 0x0);

    return result;
}

static RESULT IMX664_IsiGetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    TRACE(IMX664_INFO, "%s (enter)\n", __func__);
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (pIMX664Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    memcpy(pMode, &(pIMX664Ctx->SensorMode), sizeof(pIMX664Ctx->SensorMode));

    TRACE(IMX664_INFO, "%s (exit)\n", __func__);
    return (RET_SUCCESS);
}

static RESULT IMX664_IsiSetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    int ret = 0;
    TRACE(IMX664_INFO, "%s (enter)\n", __func__);

    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (pIMX664Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX664Ctx->IsiCtx.HalHandle;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, pMode);
    if (ret != 0) {
        return (RET_FAILURE);
    }

    return (RET_SUCCESS);
}

static RESULT IMX664_IsiHalEnumModeIss(HalHandle_t HalHandle,
                                       IsiEnumMode_t* pEnumMode) {
    HalContext_t* pHalCtx = HalHandle;
    if (pHalCtx == NULL) {
        return RET_NULL_POINTER;
    }

    if (pEnumMode->index >=
        (sizeof(pimx664_mode_info) / sizeof(pimx664_mode_info[0])))
        return RET_OUTOFRANGE;

    for (uint32_t i = 0;
         i < (sizeof(pimx664_mode_info) / sizeof(pimx664_mode_info[0])); i++) {
        if (pimx664_mode_info[i].index == pEnumMode->index) {
            memcpy(&pEnumMode->mode, &pimx664_mode_info[i], sizeof(IsiMode_t));
            TRACE(IMX664_INFO, "%s (exit)\n", __func__);
            return RET_SUCCESS;
        }
    }

    return RET_NOTSUPP;
}

static RESULT IMX664_IsiEnumModeIss(IsiSensorHandle_t handle,
                                    IsiEnumMode_t* pEnumMode) {
    RESULT result = RET_SUCCESS;
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (pIMX664Ctx == NULL || pIMX664Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX664Ctx->IsiCtx.HalHandle;
    result = IMX664_IsiHalEnumModeIss(pHalCtx, pEnumMode);
    if (result != RET_SUCCESS) {
        TRACE(IMX664_ERROR, "%s: sensor enum mode error!\n", __func__);
        return (RET_FAILURE);
    }

    return result;
}

static RESULT IMX664_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t* pCaps) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;

    RESULT result = RET_SUCCESS;

    TRACE(IMX664_INFO, "%s (enter)\n", __func__);

    if (pIMX664Ctx == NULL) return (RET_WRONG_HANDLE);

    if (pCaps == NULL) {
        return (RET_NULL_POINTER);
    }

    if (!pIMX664Ctx->Configured) IMX664_IsiSetupIss(handle, pCaps);

    pCaps->BusWidth = pIMX664Ctx->SensorMode.bit_width;
    pCaps->Mode = ISI_MODE_BAYER;
    pCaps->FieldSelection = ISI_FIELDSEL_BOTH;
    pCaps->YCSequence = ISI_YCSEQ_YCBYCR;
    pCaps->Conv422 = ISI_CONV422_COSITED;
    pCaps->BPat = pIMX664Ctx->SensorMode.bayer_pattern;
    pCaps->HPol = ISI_HPOL_REFPOS;
    pCaps->VPol = ISI_VPOL_POS;
    pCaps->Edge = ISI_EDGE_RISING;
    pCaps->Resolution.width = pIMX664Ctx->SensorMode.size.width;
    pCaps->Resolution.height = pIMX664Ctx->SensorMode.size.height;
    pCaps->SmiaMode = ISI_SMIA_OFF;
    pCaps->MipiLanes = ISI_MIPI_4LANES;

    if (pCaps->BusWidth == 10) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_10;
    } else if (pCaps->BusWidth == 12) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_12;
    } else {
        pCaps->MipiMode = ISI_MIPI_OFF;
    }
    TRACE(IMX664_INFO, "got caps - width %d height %d buswidth %d\n",
          pCaps->Resolution.width, pCaps->Resolution.height, pCaps->BusWidth);
    TRACE(IMX664_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX664_AecSetModeParameters(IMX664_Context_t* pIMX664Ctx,
                                          const IsiCaps_t* pConfig) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX664_INFO, "%s%s: (enter)\n", __func__,
          pIMX664Ctx->isAfpsRun ? "(AFPS)" : "");

    pIMX664Ctx->AecIntegrationTimeIncrement = pIMX664Ctx->one_line_exp_time;
    pIMX664Ctx->AecMinIntegrationTime =
        pIMX664Ctx->one_line_exp_time * pIMX664Ctx->MinIntegrationLine;
    pIMX664Ctx->AecMaxIntegrationTime =
        pIMX664Ctx->one_line_exp_time * pIMX664Ctx->MaxIntegrationLine;

    TRACE(IMX664_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __func__,
          pIMX664Ctx->isAfpsRun ? "(AFPS)" : "",
          pIMX664Ctx->AecMaxIntegrationTime);

    pIMX664Ctx->AecGainIncrement = IMX664_MIN_GAIN_STEP;

    // reflects the state of the sensor registers, must equal default settings
    pIMX664Ctx->AecCurGainLEF = pIMX664Ctx->AecMinGain;
	pIMX664Ctx->AecCurGainSEF1 = pIMX664Ctx->AecMinGain;
	pIMX664Ctx->AecCurGainSEF2 = pIMX664Ctx->AecMinGain;
	pIMX664Ctx->AecCurIntegrationTimeLEF = pIMX664Ctx->AecMaxIntegrationTime;
	pIMX664Ctx->AecCurIntegrationTimeSEF1 = pIMX664Ctx->AecMaxIntegrationTime;
	pIMX664Ctx->AecCurIntegrationTimeSEF2 = pIMX664Ctx->AecMaxIntegrationTime;
    pIMX664Ctx->OldGain = 1;
    pIMX664Ctx->OldIntegrationTime = 0.0f;

    TRACE(IMX664_INFO, "%s%s: (exit)\n", __func__,
          pIMX664Ctx->isAfpsRun ? "(AFPS)" : "");

    return (result);
}

static RESULT IMX664_IsiSetupIss(IsiSensorHandle_t handle,
                                 const IsiCaps_t* pCaps) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX664_INFO, "%s (enter)\n", __func__);

    if (!pIMX664Ctx) {
        TRACE(IMX664_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pCaps == NULL) {
        TRACE(IMX664_ERROR,
              "%s: Invalid configuration (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMX664Ctx->Streaming != BOOL_FALSE) {
        return RET_WRONG_STATE;
    }

    if (pIMX664Ctx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        pIMX664Ctx->enableHdr = true;
    } else {
        pIMX664Ctx->enableHdr = false;
    }

    pIMX664Ctx->one_line_exp_time =
        (float)(pIMX664Ctx->SensorMode.ae_info.one_line_exp_time_ns) /
        1000000000;
    pIMX664Ctx->MaxIntegrationLine =
        pIMX664Ctx->SensorMode.ae_info.max_integration_time;
    TRACE(IMX664_INFO, "%s: MaxIntegrationLine %u\n", __func__,
          pIMX664Ctx->MaxIntegrationLine);
    pIMX664Ctx->MinIntegrationLine =
        pIMX664Ctx->SensorMode.ae_info.min_integration_time;
    pIMX664Ctx->gain_accuracy = pIMX664Ctx->SensorMode.ae_info.gain_accuracy;
    pIMX664Ctx->AecMaxGain = (float)(pIMX664Ctx->SensorMode.ae_info.max_gain) /
                             pIMX664Ctx->gain_accuracy;
    pIMX664Ctx->AecMinGain = (float)(pIMX664Ctx->SensorMode.ae_info.min_gain) /
                             pIMX664Ctx->gain_accuracy;

    pIMX664Ctx->AecMinIris = IMX664_IRIS_MIN_VAL;
    pIMX664Ctx->AecMaxIris = IMX664_IRIS_MAX_VAL;

    pIMX664Ctx->original_vmax = 0;
    pIMX664Ctx->unlimit_fps = 0;
    pIMX664Ctx->unlimit_fps_vmax_changed = 0;
    pIMX664Ctx->flicker_fps_mode = ISI_AE_ANTIBANDING_MODE_OFF;

    pIMX664Ctx->MaxFps = pIMX664Ctx->SensorMode.fps;
    pIMX664Ctx->CurrFps = pIMX664Ctx->MaxFps;
    TRACE(IMX664_INFO, "%s - got caps - width %d height %d buswidth %d\n",
          __func__, pIMX664Ctx->SensorMode.size.width,
          pIMX664Ctx->SensorMode.size.height, pIMX664Ctx->SensorMode.bit_width);

    TRACE(IMX664_INFO, "%s - MinGain %f MaxGain %f\n", __func__, pIMX664Ctx->AecMinGain,
          pIMX664Ctx->AecMaxGain);

    memcpy(&pIMX664Ctx->CapsConfig, pCaps, sizeof(pIMX664Ctx->CapsConfig));

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful,
     * bits 6..0 are reserved, reset bit is not sticky */
    TRACE(IMX664_DEBUG, "%s: IMX664 System-Reset executed\n", __func__);
    osSleep(100);

    result = IMX664_AecSetModeParameters(pIMX664Ctx, pCaps);
    if (result != RET_SUCCESS) {
        TRACE(IMX664_ERROR, "%s: SetupOutputWindow failed.\n", __func__);
        return (result);
    }

    pIMX664Ctx->Configured = BOOL_TRUE;
    TRACE(IMX664_INFO, "%s: (exit)\n", __func__);
    return 0;
}

static RESULT IMX664_IsiCheckConnectionIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    return (result);
}

static RESULT IMX664_IsiGetRevisionIss(IsiSensorHandle_t handle,
                                       uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    int ret = 0;
    uint32_t reg_val;
    uint32_t sensor_id;

    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (pIMX664Ctx == NULL || pIMX664Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX664Ctx->IsiCtx.HalHandle;

    if (!pValue) return (RET_NULL_POINTER);

    if (pIMX664Ctx->KernelDriverFlag) {
        ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, &sensor_id);
        if (ret != 0) {
            TRACE(IMX664_ERROR, "%s: Read Sensor ID Error! \n", __func__);
            return (RET_FAILURE);
        }
    } else {
        reg_val = 0;
        result = IMX664_IsiReadRegIss(handle, 0x3a04, &reg_val);
        sensor_id = (reg_val & 0xff) << 8;

        reg_val = 0;
        result |= IMX664_IsiReadRegIss(handle, 0x3a05, &reg_val);
        sensor_id |= (reg_val & 0xff);
    }

    *pValue = sensor_id;
    TRACE(IMX664_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX664_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX664_INFO, "%s (enter)\n", __func__);

    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (pIMX664Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    pIMX664Ctx->Streaming = on;

    if (pIMX664Ctx->enableHdr)
        return result;

    TRACE(IMX664_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX664_IsiGetGainLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinGain, float* pMaxGain) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX664_INFO, "%s: (enter)\n", __func__);

    if (pIMX664Ctx == NULL) {
        TRACE(IMX664_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinGain == NULL) || (pMaxGain == NULL)) {
        TRACE(IMX664_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinGain = pIMX664Ctx->AecMinGain;
    *pMaxGain = pIMX664Ctx->AecMaxGain;

    TRACE(IMX664_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static size_t IMX664_GetNumExposures(IMX664_Context_t* pIMX664Ctx) {
    if (pIMX664Ctx == NULL) {
        return 0;
    }

    if (pIMX664Ctx->SensorMode.hdr_mode == SENSOR_MODE_LINEAR) {
        return 1; // SDR
    } else if (pIMX664Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        return IMX664_2DOL_NUM_EXP;
    } else if (pIMX664Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        return IMX664_3DOL_NUM_EXP;
    } else {
        TRACE(IMX664_ERROR, "%s: Unsupported HDR mode %d with stitching mode %d\n",
            __func__, pIMX664Ctx->SensorMode.hdr_mode, pIMX664Ctx->SensorMode.stitching_mode);
        return 0;
    }
}

static RESULT IMX664_UpdateCurrLEFIntegrationTimeFromFsc(IMX664_Context_t* pIMX664Ctx, uint32_t fsc, uint32_t shr0) {
    if (pIMX664Ctx == NULL) {
        return RET_NULL_POINTER;
    }

    float configuredIntegrationTime = (fsc - shr0) * pIMX664Ctx->one_line_exp_time;

    pIMX664Ctx->OldIntegrationTime = configuredIntegrationTime;
    pIMX664Ctx->AecCurIntegrationTimeLEF = configuredIntegrationTime;

    TRACE(IMX664_DEBUG, "%s: Updated LEF Integration Time = (fsc[%u] - shr0[%u]) * one_line_exp_time[%f] = %f\n",
          __func__, fsc, shr0, pIMX664Ctx->one_line_exp_time, configuredIntegrationTime);
    return RET_SUCCESS;
}

static RESULT IMX664_UpdateCurrLEFIntegrationTimeFromVmax(IMX664_Context_t* pIMX664Ctx, uint32_t vmax, uint32_t shr0) {
    if (pIMX664Ctx == NULL) {
        return RET_NULL_POINTER;
    }

    size_t dol = IMX664_GetNumExposures(pIMX664Ctx);
    uint32_t fsc = vmax * dol;
    TRACE(IMX664_DEBUG, "%s: fsc = vmax[%u] * dol[%zu] = %u\n", __func__, vmax, dol, fsc);
    return IMX664_UpdateCurrLEFIntegrationTimeFromFsc(pIMX664Ctx, fsc, shr0);
}

static inline int IMX664_getFlickerPeaksPerSec(IsiSensorAntibandingMode_t mode) {
    int num_modes = sizeof(flickerPeaksPerSecMap) / sizeof(FlickerModePeaksPerSec);
    for (int i = 0; i < num_modes; i++) {
        if (flickerPeaksPerSecMap[i].mode == mode) {
            return flickerPeaksPerSecMap[i].value;
        }
    }
    return 0; // Defaults to 0 if mode not found
}

static inline uint32_t IMX664_getNewVmaxAntiFlicker(IMX664_Context_t *pIMX664Ctx, uint32_t requestedVmax) {
    uint32_t closestVmax = requestedVmax;
    int peaks = 0;
    int difference = INT_MAX;
    int minDifference = INT_MAX;
    if (!pIMX664Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (-1);
    }
    peaks = IMX664_getFlickerPeaksPerSec(pIMX664Ctx->flicker_fps_mode);
    if (peaks == 0) {
        // No anti-flicker mode, return requested or original Vmax
        if (pIMX664Ctx->unlimit_fps && pIMX664Ctx->unlimit_fps_vmax_changed)
            return requestedVmax;
        else
            return pIMX664Ctx->original_vmax; 
    }

    for (int i = 1; i < peaks; ++i) {
        uint32_t candidateVmax = i / (peaks * pIMX664Ctx->one_line_exp_time);
        if (candidateVmax < pIMX664Ctx->SensorMode.size.height || candidateVmax < pIMX664Ctx->original_vmax)
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

static RESULT IMX664_IsiUnlimitFpsIss(IsiSensorHandle_t handle,
                                      float maxIntegrationTime) {
    RESULT result = RET_SUCCESS;
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;

    if (!pIMX664Ctx) {
        return RET_NULL_POINTER;
    }

    if (pIMX664Ctx->enableHdr) {
        return result;
    }
    pIMX664Ctx->unlimit_fps = 1;

    if (maxIntegrationTime < 0)
        pIMX664Ctx->MaxIntegrationLine = IMX664_VMAX_MAX - IMX664_MIN_SHR;
    else
        pIMX664Ctx->MaxIntegrationLine =
            MIN((uint32_t)(maxIntegrationTime / pIMX664Ctx->one_line_exp_time),
                IMX664_VMAX_MAX - IMX664_MIN_SHR);
    TRACE(IMX664_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
        pIMX664Ctx->MaxIntegrationLine);
    pIMX664Ctx->AecMaxIntegrationTime =
        pIMX664Ctx->one_line_exp_time * pIMX664Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX664_IsiLimitFpsIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    uint32_t current_vmax = 0;
    uint32_t new_vmax = 0;
    TRACE(IMX664_INFO, "%s (enter)\n", __func__);
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (!pIMX664Ctx) {
        return RET_NULL_POINTER;
    }

    if (pIMX664Ctx->enableHdr)
        return result;

    IMX664_ReadVmax(handle, &current_vmax);
    if (current_vmax == 0) {
        TRACE(IMX664_INFO, "%s - exit because current_vmax is 0\n", __func__);
        return result;
    }

    pIMX664Ctx->unlimit_fps = 0;
    pIMX664Ctx->unlimit_fps_vmax_changed = 0;
    if (pIMX664Ctx->original_vmax == 0) {
        pIMX664Ctx->original_vmax = current_vmax;
    }

    new_vmax = pIMX664Ctx->original_vmax;
    if (pIMX664Ctx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
        new_vmax = IMX664_getNewVmaxAntiFlicker(pIMX664Ctx, pIMX664Ctx->original_vmax);
        TRACE(IMX664_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX664Ctx->flicker_fps_mode, new_vmax);
    }
    if (current_vmax != new_vmax) {
        result |= IMX664_LockRegHold(handle);
        result |= IMX664_WriteVmax(handle, new_vmax);
        result |= IMX664_UnlockRegHold(handle);

        int shr = MAX((int)current_vmax - (int)(pIMX664Ctx->AecCurIntegrationTimeLEF / pIMX664Ctx->one_line_exp_time), IMX664_MIN_SHR);
        result |= IMX664_UpdateCurrLEFIntegrationTimeFromVmax(pIMX664Ctx, new_vmax, shr);
    }

    pIMX664Ctx->MaxIntegrationLine =
        MAX(new_vmax - IMX664_MIN_SHR, 1);
    TRACE(IMX664_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
        pIMX664Ctx->MaxIntegrationLine);
    pIMX664Ctx->AecMaxIntegrationTime =
        pIMX664Ctx->one_line_exp_time * pIMX664Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX664_IsiGetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinIris, float* pMaxIris) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX664_INFO, "%s: (enter)\n", __func__);

    if (pIMX664Ctx == NULL) {
        TRACE(IMX664_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIris == NULL) || (pMaxIris == NULL)) {
        TRACE(IMX664_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIris = pIMX664Ctx->AecMinIris;
    *pMaxIris = pIMX664Ctx->AecMaxIris;

    TRACE(IMX664_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX664_IsiSetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float minIris, float maxIris) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX664_INFO, "%s: (enter)\n", __func__);

    if (pIMX664Ctx == NULL) {
        TRACE(IMX664_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    pIMX664Ctx->AecMinIris = minIris;
    pIMX664Ctx->AecMaxIris = maxIris;

    TRACE(IMX664_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX664_IsiGetIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float* pMinIntegrationTime,
    float* pMaxIntegrationTime) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX664_INFO, "%s: (enter)\n", __func__);
    if (pIMX664Ctx == NULL) {
        TRACE(IMX664_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX664_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIntegrationTime = pIMX664Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMX664Ctx->AecMaxIntegrationTime;

    TRACE(IMX664_INFO, "%s: (exit) %f, %f\n", 
    __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

/* Gain get functions*/

RESULT IMX664_IsiGetGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;
	TRACE(IMX664_INFO, "%s: (enter)\n", __func__);

	if (pIMX664Ctx == NULL) {
		TRACE(IMX664_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	if (pIMX664Ctx->enableHdr)
		return IMX664_IsiGetSEF1GainIss(handle, pSetGain);

	return IMX664_IsiGetLEFGainIss(handle, pSetGain);
}

RESULT IMX664_IsiGetLEFGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	TRACE(IMX664_INFO, "%s: (enter)\n", __func__);

	if (pIMX664Ctx == NULL) {
		TRACE(IMX664_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}
	*pSetGain = pIMX664Ctx->AecCurGainLEF;
	TRACE(IMX664_DEBUG, "%s - returning %f\n", __func__, pIMX664Ctx->AecCurGainLEF);
	TRACE(IMX664_INFO, "%s: (exit)\n", __func__);
	return (result);
}

RESULT IMX664_IsiGetSEF1GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX664_INFO, "%s: (enter)\n", __func__);

	if (pIMX664Ctx == NULL) {
		TRACE(IMX664_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX664Ctx->AecCurGainSEF1;
	TRACE(IMX664_DEBUG, "%s - returning %f\n", __func__, pIMX664Ctx->AecCurGainSEF1);

	TRACE(IMX664_INFO, "%s: (exit)\n", __func__);

	return (result);
}

RESULT IMX664_IsiGetSEF2GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX664_INFO, "%s: (enter)\n", __func__);

	if (pIMX664Ctx == NULL) {
		TRACE(IMX664_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX664Ctx->AecCurGainSEF2;
	TRACE(IMX664_DEBUG, "%s - returning %f\n", __func__, pIMX664Ctx->AecCurGainSEF2);

	TRACE(IMX664_INFO, "%s: (exit)\n", __func__);

	return (result);
}

RESULT IMX664_IsiGetGainIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX664_INFO, "%s: (enter)\n", __func__);

    if (pIMX664Ctx == NULL) {
        TRACE(IMX664_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = pIMX664Ctx->AecGainIncrement;

    TRACE(IMX664_INFO, "%s: (exit)\n", __func__);

    return (result);
}

RESULT IMX664_IsiGetIrisIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX664_INFO, "%s: (enter)\n", __func__);
    if (pIMX664Ctx == NULL) {
        TRACE(IMX664_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = 1;

    TRACE(IMX664_INFO, "%s: (exit)\n", __func__);

    return (result);
}

/* Gain set functions*/

RESULT IMX664_IsiSetGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;
	if (pIMX664Ctx == NULL || pIMX664Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	if (pIMX664Ctx->enableHdr) {
		result = IMX664_IsiSetSEF1GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);

		result |= IMX664_IsiSetSEF2GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);
	}

	result |= IMX664_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
	return result;
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

RESULT IMX664_IsiSetLEFGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;
	if (pIMX664Ctx == NULL || pIMX664Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	TRACE(IMX664_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX664_DEBUG, "%s: writting 0x%x to GAIN\n", __func__, Gain);

    result |= IMX664_LockRegHold(handle);
	result |= IMX664_WriteGain(handle, Gain);
	result |= IMX664_UnlockRegHold(handle);

	if (result != 0) {
		return RET_FAILURE;
	}

	pIMX664Ctx->AecCurGainLEF = _sensorGain2linear(Gain);
	*pSetGain = pIMX664Ctx->AecCurGainLEF;
	TRACE(IMX664_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

RESULT IMX664_IsiSetSEF1GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX664Ctx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);

	TRACE(IMX664_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX664_DEBUG, "%s: writting 0x%x to GAIN1\n", __func__, Gain);

    result |= IMX664_LockRegHold(handle);
	result |= IMX664_WriteGain1(handle, Gain);
    result |= IMX664_UnlockRegHold(handle);

	pIMX664Ctx->AecCurGainSEF1 = _sensorGain2linear(Gain);
	*pSetGain = pIMX664Ctx->AecCurGainSEF1;

	TRACE(IMX664_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

RESULT IMX664_IsiSetSEF2GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX664Ctx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);
	
	TRACE(IMX664_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX664_DEBUG, "%s: writting 0x%x to GAIN2\n", __func__, Gain);

    result |= IMX664_LockRegHold(handle);
	result |= IMX664_WriteGain2(handle, Gain);
    result |= IMX664_UnlockRegHold(handle);

	pIMX664Ctx->AecCurGainSEF2 = _sensorGain2linear(Gain);
	*pSetGain = pIMX664Ctx->AecCurGainSEF2;

	TRACE(IMX664_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

/* Integration Time get functions*/

RESULT IMX664_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;

	if (!pIMX664Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	
	TRACE(IMX664_DEBUG, "%s - enter\n", __func__);
	
	if (pIMX664Ctx->enableHdr)
		return IMX664_IsiGetSEF1IntegrationTimeIss(handle, pSetIntegrationTime);
	
	return IMX664_IsiGetLEFIntegrationTimeIss(handle, pSetIntegrationTime);
}

RESULT IMX664_IsiGetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;
	RESULT result = RET_SUCCESS;


	if (!pIMX664Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX664Ctx->AecCurIntegrationTimeLEF;
	TRACE(IMX664_DEBUG, "%s - returning %f\n", __func__, pIMX664Ctx->AecCurIntegrationTimeLEF);
	return (result);
}

RESULT IMX664_IsiGetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX664Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX664Ctx->AecCurIntegrationTimeSEF1;
	TRACE(IMX664_DEBUG, "%s - returning %f\n", __func__, pIMX664Ctx->AecCurIntegrationTimeSEF1);
	return (result);
}

RESULT IMX664_IsiGetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX664Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);

	*pSetIntegrationTime = pIMX664Ctx->AecCurIntegrationTimeSEF2;
	TRACE(IMX664_DEBUG, "%s - returning %f\n", __func__, pIMX664Ctx->AecCurIntegrationTimeSEF2);
	return (result);
}

RESULT IMX664_IsiGetIntegrationTimeIncrementIss(IsiSensorHandle_t handle,
                                                float* pIncr) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (!pIMX664Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pIncr) return (RET_NULL_POINTER);

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders
    //in
    // the application)
    *pIncr = pIMX664Ctx->AecIntegrationTimeIncrement;
    return (result);
}

/* Integration Time set functions*/

RESULT IMX664_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;

	if (!pIMX664Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX664Ctx->enableHdr) {
		return IMX664_IsiSetSEF1IntegrationTimeIss(
			handle, NewIntegrationTime, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
	}

	return IMX664_IsiSetLEFIntegrationTimeIss(
		handle, NewIntegrationTime, pSetIntegrationTime,
		pNumberOfFramesToSkip, hdr_ratio);
}

RESULT IMX664_IsiSetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;

    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;

    int shr = 0;
    uint32_t exp = 0;
    uint32_t new_vmax = 0;
    uint32_t vmax_updated = 0;
    uint32_t current_vmax = 0;
    uint32_t rhs1;
    uint32_t rhs2;
    if (!pIMX664Ctx) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pSetIntegrationTime || !pNumberOfFramesToSkip) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMX664Ctx->Streaming) {
        TRACE(IMX664_ERROR, "%s: sensor not streaming\n", __func__);
        return RET_FAILURE;
    }
    exp = roundf(NewIntegrationTime / pIMX664Ctx->one_line_exp_time);

    TRACE(IMX664_DEBUG, "%s: set AEC_PK_EXPO=0x%05x\n", __func__, exp);

    if (fabs(NewIntegrationTime - pIMX664Ctx->AecCurIntegrationTimeLEF) > FLT_EPSILON) {

        if (pIMX664Ctx->enableHdr){
            if (pIMX664Ctx->cur_rhs1 == 0 || pIMX664Ctx->cur_rhs2 == 0) {
                TRACE(IMX664_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
                return (RET_WRONG_CONFIG);
            }
    

            rhs1 = pIMX664Ctx->cur_rhs1;
            rhs2 = pIMX664Ctx->cur_rhs2;
            if (pIMX664Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
                if(IMX664_ReadVmax(pIMX664Ctx, &new_vmax) != RET_SUCCESS){
                    TRACE(IMX664_ERROR, "%s: unable to read vmax\n", __func__);
                    new_vmax = IMX664_VMAX_2DOL_HDR;
                }else{
                    new_vmax *= IMX664_2DOL_NUM_EXP;
                }

                exp = new_vmax - exp;
                exp = exp > rhs1 + IMX664_2DOL_SHR0_RHS1_GAP ? exp : rhs1 + IMX664_2DOL_SHR0_RHS1_GAP;
                exp = exp < new_vmax - IMX664_2DOL_SHR0_FSC_GAP? exp : new_vmax - IMX664_2DOL_SHR0_FSC_GAP;
                shr = exp;
            } else {
                new_vmax = IMX664_VMAX_3DOL_HDR;
                exp = new_vmax - exp;
                exp = exp > rhs2 + IMX664_SHR0_RHS2_GAP ? exp : rhs2 + IMX664_SHR0_RHS2_GAP;
                exp = exp < new_vmax - IMX664_SHR0_FSC_GAP? exp : new_vmax - IMX664_SHR0_FSC_GAP;
                shr = exp;
            }
            
		} else {
            if (exp > pIMX664Ctx->MaxIntegrationLine || exp == 0) {
                TRACE(IMX664_ERROR, "%s: Integration time %f (exp %u) out of range (%u)\n", __func__,
                    NewIntegrationTime, exp, pIMX664Ctx->MaxIntegrationLine);
                return RET_FAILURE;
            }
            result = IMX664_ReadVmax(handle, &current_vmax);

            if (pIMX664Ctx->original_vmax == 0) {
                pIMX664Ctx->original_vmax = current_vmax;
            }

            if (pIMX664Ctx->original_vmax - IMX664_MIN_SHR >
                pIMX664Ctx->MaxIntegrationLine) {
                pIMX664Ctx->MaxIntegrationLine =
                    pIMX664Ctx->original_vmax - IMX664_MIN_SHR;
                TRACE(IMX664_DEBUG, "%s: set MaxIntegrationLine to %u\n", __func__,
                      pIMX664Ctx->MaxIntegrationLine);
                pIMX664Ctx->AecMaxIntegrationTime =
                    pIMX664Ctx->one_line_exp_time * pIMX664Ctx->MaxIntegrationLine;
                TRACE(IMX664_DEBUG, "%s: set AecMaxIntegrationTime to %f\n", __func__,
                      pIMX664Ctx->AecMaxIntegrationTime);
            }

            shr = current_vmax - exp;

            if (shr < IMX664_MIN_SHR || current_vmax > pIMX664Ctx->original_vmax) {
                if (shr < IMX664_MIN_SHR) {
                    // user asked for too big integration-line: more than (current_vmax - minimal_shutter)
                    // increase vmax and set new shutter to minimal value
                    new_vmax = MIN(exp + IMX664_MIN_SHR, pIMX664Ctx->MaxIntegrationLine + IMX664_MIN_SHR);
                } else {
                    // make sure that when we unlimit fps, stay below original fps.
                    // also, make sure we are not using high vmax + high shr more than needed (will slow fps for no reason)
                    new_vmax = MAX(current_vmax - shr + IMX664_MIN_SHR, pIMX664Ctx->original_vmax);
                }
                pIMX664Ctx->unlimit_fps_vmax_changed = new_vmax > pIMX664Ctx->original_vmax && pIMX664Ctx->unlimit_fps;
        
                if (pIMX664Ctx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
                    new_vmax = IMX664_getNewVmaxAntiFlicker(pIMX664Ctx, new_vmax);
                    TRACE(IMX664_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX664Ctx->flicker_fps_mode, new_vmax);
                }
                
                shr = MAX(IMX664_MIN_SHR, (int)new_vmax - (int)exp);
                vmax_updated = 1;

            } else {
                new_vmax = current_vmax;
            }
        }

        result |= IMX664_LockRegHold(handle);
        if (vmax_updated && pIMX664Ctx->unlimit_fps && !pIMX664Ctx->enableHdr) {
            result |= IMX664_WriteVmax(handle, new_vmax);
        }

        TRACE(IMX664_DEBUG, "%s - writing 0x%x to SHR0\n", __func__, shr);
        result |= IMX664_WriteShr0(handle, shr);
        result |= IMX664_UnlockRegHold(handle);

        // In this context, the "new_vmax" is actually the FSC (multiplied by DOL), not the VMAX.
        result |= IMX664_UpdateCurrLEFIntegrationTimeFromFsc(pIMX664Ctx, new_vmax, shr);

        *pNumberOfFramesToSkip = 1U;
    } else {
        *pNumberOfFramesToSkip = 0U;  // no frame skip
    }

    *pSetIntegrationTime = pIMX664Ctx->AecCurIntegrationTimeLEF;
    TRACE(IMX664_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTime);
    return (result);
}

RESULT IMX664_IsiSetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF1,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;

	if (!pIMX664Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX664Ctx->cur_rhs1 == 0 || pIMX664Ctx->cur_rhs2 == 0) {
		TRACE(IMX664_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX664Ctx->cur_rhs1;

	if (!pSetIntegrationTimeSEF1 || !pNumberOfFramesToSkip) {
		printf("%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX664_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = roundf(NewIntegrationTime / pIMX664Ctx->one_line_exp_time);

	if (fabs(NewIntegrationTime - pIMX664Ctx->AecCurIntegrationTimeSEF1) > FLT_EPSILON) {
		if (pIMX664Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
			exp = rhs1 - exp;
			exp = exp > IMX664_2DOL_SHR1_RHS1_GAP ? exp : IMX664_2DOL_SHR1_RHS1_GAP;
			exp = exp < rhs1 - IMX664_SHR1_RHS1_GAP ? exp : rhs1 - IMX664_SHR1_RHS1_GAP;
		} else {
			exp = rhs1 - exp;
			exp = exp > IMX664_3DOL_SHR1_RHS1_GAP ? exp : IMX664_3DOL_SHR1_RHS1_GAP;
			exp = exp < rhs1 - IMX664_SHR1_RHS1_GAP ? exp : rhs1 - IMX664_SHR1_RHS1_GAP;
		}

		TRACE(IMX664_DEBUG, "%s - writing 0x%x to SHR1\n", __func__, exp);

		result |= IMX664_LockRegHold(handle);
		result |= IMX664_WriteShr1(handle, exp);
		result |= IMX664_UnlockRegHold(handle);

		pIMX664Ctx->AecCurIntegrationTimeSEF1 = (rhs1 - exp) * pIMX664Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF1 = pIMX664Ctx->AecCurIntegrationTimeSEF1;

	TRACE(IMX664_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF1);
	return (result);
}

RESULT IMX664_IsiSetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF2,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX664_Context_t *pIMX664Ctx = (IMX664_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;
	uint32_t rhs2;

	if (pIMX664Ctx->cur_rhs1 == 0 || pIMX664Ctx->cur_rhs2 == 0) {
		printf("%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX664Ctx->cur_rhs1;
	rhs2 = pIMX664Ctx->cur_rhs2;

	if (!pIMX664Ctx) {
		printf("%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTimeSEF2 || !pNumberOfFramesToSkip) {
		printf("%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX664_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = roundf(NewIntegrationTime / pIMX664Ctx->one_line_exp_time);
	TRACE(IMX664_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX664Ctx->AecCurIntegrationTimeSEF2) > FLT_EPSILON) {
		exp = rhs2 - exp;
		exp = exp > rhs1 + IMX664_SHR2_RHS1_GAP ? exp : rhs1 + IMX664_SHR2_RHS1_GAP;
		exp = exp < rhs2 - IMX664_SHR2_RHS2_GAP ? exp : rhs2 - IMX664_SHR2_RHS2_GAP;
		TRACE(IMX664_DEBUG, "%s - writing 0x%x to SHR2\n", __func__, exp);

		result |= IMX664_LockRegHold(handle);
		result |= IMX664_WriteShr2(handle, exp);
		result |= IMX664_UnlockRegHold(handle);

		pIMX664Ctx->AecCurIntegrationTimeSEF2 = (rhs2 - exp) * pIMX664Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF2 = pIMX664Ctx->AecCurIntegrationTimeSEF2;

	TRACE(IMX664_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF2);
	return (result);
}

RESULT IMX664_Calculate3DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it, float *o_very_short_it,
                                    float *o_long_gain, float *o_short_gain, float *o_very_short_gain,
                                    float *hdr_ratio) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
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

    if (pIMX664Ctx == NULL || o_long_it == NULL || o_short_it == NULL ||
        o_very_short_it == NULL || o_long_gain == NULL || o_short_gain == NULL ||
        o_very_short_gain == NULL || hdr_ratio == NULL) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

	if (pIMX664Ctx->cur_rhs1 == 0 || pIMX664Ctx->cur_rhs2 == 0) {
		TRACE(IMX664_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX664Ctx->cur_rhs1;
	rhs2 = pIMX664Ctx->cur_rhs2;

    TRACE(IMX664_DEBUG, "%s: hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n", 
    __func__, hdr_ratio[0], hdr_ratio[1]);
    
    // Sometimes there is no actual input gain. In that case, we will read it from the sensor
    if (NewGain == 0) {
        TRACE(IMX664_DEBUG, "%s: Input NewGain is 0, reading gain from sensor\n", __func__);
        result = IMX664_IsiGetSEF1GainIss(handle, &NewGain);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // Same for integration time
    if (NewIntegrationTime == 0) {
        TRACE(IMX664_DEBUG, "%s: Input NewIntegrationTime is 0, reading integration time from sensor\n", __func__);
        result = IMX664_IsiGetSEF1IntegrationTimeIss(handle, &NewIntegrationTime);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    if(NewIntegrationTime < IMX664_SHR2_RHS2_GAP * pIMX664Ctx->one_line_exp_time * hdr_ratio[1]){
	    pIMX664Ctx->MinIntegrationLine =  IMX664_SHR2_RHS2_GAP * hdr_ratio[1];
	    pIMX664Ctx->AecMinIntegrationTime = pIMX664Ctx->MinIntegrationLine * pIMX664Ctx->one_line_exp_time;
    }

    // assume gain is 1 and see if ratio can be achieved with integration time
    long_it 		= NewIntegrationTime * hdr_ratio[0];
    short_it 		= NewIntegrationTime;
    very_short_it 	= NewIntegrationTime / hdr_ratio[1];
    
    TRACE(IMX664_DEBUG, "%s: requested IT long: %f, short: %f, very_short: %f\n", 
    __func__, long_it, short_it, very_short_it);
    long_exp_val 		= long_it / pIMX664Ctx->one_line_exp_time;
    short_exp_val 		= short_it / pIMX664Ctx->one_line_exp_time;
    very_short_exp_val 	= very_short_it / pIMX664Ctx->one_line_exp_time;

    TRACE(IMX664_DEBUG, "%s: requested IT in lines long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    long_exp_val 		= IMX664_VMAX_3DOL_HDR - long_exp_val;
    short_exp_val 		= rhs1 - short_exp_val;
    very_short_exp_val 	= rhs2 - very_short_exp_val;

    TRACE(IMX664_DEBUG, "%s: requested IT in shr long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    if(long_exp_val < rhs2 + IMX664_SHR0_RHS2_GAP) {
        long_exp_val = rhs2 + IMX664_SHR0_RHS2_GAP;
        long_it = (IMX664_VMAX_3DOL_HDR - long_exp_val) * pIMX664Ctx->one_line_exp_time;
        calculate_gain = true;
        optimize_long_gain = true;
        TRACE(IMX664_DEBUG, "%s: long_exp_val is too long, set to %u, new long_it = %f\n",
        __func__, rhs2 + IMX664_SHR0_RHS2_GAP, long_it);
    } else if(long_exp_val > IMX664_VMAX_3DOL_HDR - IMX664_SHR0_FSC_GAP) {
        long_exp_val = IMX664_VMAX_3DOL_HDR - IMX664_SHR0_FSC_GAP;
        long_it = (IMX664_VMAX_3DOL_HDR - long_exp_val) * pIMX664Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX664_DEBUG, "%s: long_exp_val is too short, set to %u, new long_it = %f\n",
        __func__, IMX664_VMAX_3DOL_HDR - IMX664_SHR0_FSC_GAP, long_it);
    }
    if(short_exp_val < IMX664_3DOL_SHR1_RHS1_GAP) {
        short_exp_val = IMX664_3DOL_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX664Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX664_DEBUG, "%s: short_exp_val is too long, set to %u, new short_it = %f\n",
        __func__, IMX664_3DOL_SHR1_RHS1_GAP, short_it);
    } else if(short_exp_val > rhs1 - IMX664_SHR1_RHS1_GAP) {
        short_exp_val = rhs1 - IMX664_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX664Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX664_DEBUG, "%s: short_exp_val is too short, set to %u, new short_it = %f\n",
        __func__, rhs1 - IMX664_SHR1_RHS1_GAP, short_it);
    }
    if(very_short_exp_val < rhs1 + IMX664_SHR2_RHS1_GAP) {
        very_short_exp_val = rhs1 + IMX664_SHR2_RHS1_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX664Ctx->one_line_exp_time;
        calculate_gain = true;
        optimize_short_gain = true;
        TRACE(IMX664_DEBUG, "%s: very_short_exp_val is too long, set to %u, new very_short_it = %f\n",
        __func__, rhs2 + IMX664_SHR2_RHS1_GAP, very_short_it);
    } else if(very_short_exp_val > rhs2 - IMX664_SHR2_RHS2_GAP) {
        very_short_exp_val = rhs2 - IMX664_SHR2_RHS2_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX664Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX664_DEBUG, "%s: very_short_exp_val is too short, set to %u, new very_short_it = %f\n",
        __func__, rhs2 - IMX664_SHR2_RHS2_GAP, very_short_it);
    }

    // need to use gain to achive ratio / requested gain update
    if(calculate_gain || NewGain != pIMX664Ctx->AecCurGainSEF1) {
        float real_short_gain = _sensorGain2linear(_linear2sensorGain(NewGain));
        long_gain = (short_it * NewGain * hdr_ratio[0]) / long_it;
        if(optimize_long_gain){
             long_gain = _sensorGain2linear(_linear2sensorGainCeil(long_gain));
             long_it = (short_it * real_short_gain * hdr_ratio[0]) / long_gain;
        }
        short_gain = NewGain;
        very_short_gain = (short_it * NewGain) / (very_short_it * hdr_ratio[1]);
        if(optimize_short_gain){
             very_short_gain = _sensorGain2linear(_linear2sensorGainCeil(very_short_gain));
             very_short_it = (short_it * real_short_gain) / (very_short_gain * hdr_ratio[1]);
        }
        TRACE(IMX664_DEBUG, "%s: calculated gain: long: %f, short: %f, very_short: %f\n",
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

RESULT IMX664_Calculate2DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it,
                                    float *o_long_gain, float *o_short_gain,
                                    float *hdr_ratio) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    float long_it = 0.0;
	float short_it = 0.0;
	float long_exp_val = 0.0;
	float short_exp_val = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	bool calculate_gain = false;
	uint32_t rhs1;
	uint32_t vmax = IMX664_VMAX_2DOL_HDR; 
	bool optimize_gain = false;

    if (pIMX664Ctx == NULL || o_long_it == NULL ||
        o_long_gain == NULL || o_short_gain == NULL ||
        hdr_ratio == NULL) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

	if (pIMX664Ctx->cur_rhs1 == 0) {
		TRACE(IMX664_ERROR, "%s: Invalid parameter (RHS1 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX664Ctx->cur_rhs1;

    TRACE(IMX664_DEBUG, "%s: hdr_ratio[0] = LS Ratio = %f\n", 
    __func__, hdr_ratio[0]);
    
    // Sometimes there is no actual input gain. In that case, we will read it from the sensor
    if (NewGain == 0) {
        TRACE(IMX664_DEBUG, "%s: Input NewGain is 0, reading gain from sensor\n", __func__);
        result = IMX664_IsiGetSEF1GainIss(handle, &NewGain);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // Same for integration time
    if (NewIntegrationTime == 0) {
        TRACE(IMX664_DEBUG, "%s: Input NewIntegrationTime is 0, reading integration time from sensor\n", __func__);
        result = IMX664_IsiGetSEF1IntegrationTimeIss(handle, &NewIntegrationTime);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }
	
    if(IMX664_ReadVmax(pIMX664Ctx, &vmax) != RET_SUCCESS){
	    TRACE(IMX664_ERROR, "%s: unable to read vmax\n", __func__);
    }

    vmax *= IMX664_2DOL_NUM_EXP;

    // assume gain is 1 and see if ratio can be achieved with integration time
    long_it 		= NewIntegrationTime * hdr_ratio[0];
    short_it 		= NewIntegrationTime;
    
    TRACE(IMX664_DEBUG, "%s: requested IT long: %f, short: %f\n", 
    __func__, long_it, short_it);
    long_exp_val 		= long_it / pIMX664Ctx->one_line_exp_time;
    short_exp_val 		= short_it / pIMX664Ctx->one_line_exp_time;

    TRACE(IMX664_DEBUG, "%s: requested IT in lines long: %f, short: %f\n", 
    __func__, long_exp_val, short_exp_val);
    long_exp_val 		= vmax - long_exp_val;
    short_exp_val 		= rhs1 - short_exp_val;

    TRACE(IMX664_DEBUG, "%s: requested IT in shr long: %f, short: %f\n",
    __func__, long_exp_val, short_exp_val);
    if(long_exp_val < rhs1 + IMX664_2DOL_SHR0_RHS1_GAP) {
        long_exp_val = rhs1 + IMX664_2DOL_SHR0_RHS1_GAP;
        long_it = (vmax - long_exp_val) * pIMX664Ctx->one_line_exp_time;
        optimize_gain = true;
        calculate_gain = true;
        TRACE(IMX664_DEBUG, "%s: long_exp_val is too long, set to %u, new long_it = %f\n",
        __func__, rhs1 + IMX664_2DOL_SHR0_RHS1_GAP, long_it);
    } else if(long_exp_val > vmax - IMX664_2DOL_SHR0_FSC_GAP) {
        long_exp_val = vmax - IMX664_2DOL_SHR0_FSC_GAP;
        long_it = (vmax - long_exp_val) * pIMX664Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX664_DEBUG, "%s: long_exp_val is too short, set to %u, new long_it = %f\n",
        __func__, vmax - IMX664_2DOL_SHR0_FSC_GAP, long_it);
    }
    if(short_exp_val < IMX664_2DOL_SHR1_MIN_GAP) {
        short_exp_val = IMX664_2DOL_SHR1_MIN_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX664Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX664_DEBUG, "%s: short_exp_val is too long, set to %u, new short_it = %f\n",
        __func__, IMX664_2DOL_SHR1_MIN_GAP, short_it);
    } else if(short_exp_val > rhs1 - IMX664_2DOL_SHR1_RHS1_GAP) {
        short_exp_val = rhs1 - IMX664_2DOL_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pIMX664Ctx->one_line_exp_time;
        calculate_gain = true;
        TRACE(IMX664_DEBUG, "%s: short_exp_val is too short, set to %u, new short_it = %f\n",
        __func__, rhs1 - IMX664_2DOL_SHR1_RHS1_GAP, short_it);
    }

    // need to use gain to achieve ratio / requested gain update
    if(calculate_gain || NewGain != pIMX664Ctx->AecCurGainSEF1) {
        float real_short_gain = _sensorGain2linear(_linear2sensorGain(NewGain));
        long_gain = (short_it * real_short_gain * hdr_ratio[0]) / long_it;
        if(optimize_gain){
             long_gain = _sensorGain2linear(_linear2sensorGainCeil(long_gain));
             long_it = (short_it * real_short_gain * hdr_ratio[0]) / long_gain;
        }

        short_gain = NewGain;
        TRACE(IMX664_DEBUG, "%s: calculated gain: long: %f, short: %f\n",
        __func__, long_gain, short_gain);
    }

    *o_long_it = long_it;
    *o_short_it = short_it;
    *o_long_gain = long_gain;
    *o_short_gain = short_gain;

    return RET_SUCCESS;
}

RESULT IMX664_IsiExposureControlIss(IsiSensorHandle_t handle, float NewGain,
                                    float NewIntegrationTime,
                                    uint8_t* pNumberOfFramesToSkip,
                                    float* pSetGain, float* pSetIntegrationTime,
                                    float* hdr_ratio) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;

    RESULT result = RET_SUCCESS;
    float long_it = 0.0;
	float short_it = 0.0;
	float very_short_it = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	float very_short_gain = 1;
    uint32_t hmax;

    TRACE(IMX664_INFO, "%s: enter with NewIntegrationTime: %f, NewGain: %f\n",
        __func__, NewIntegrationTime, NewGain);

    if (pIMX664Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pNumberOfFramesToSkip == NULL) || (pSetGain == NULL) ||
        (pSetIntegrationTime == NULL)) {
        printf("%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMX664Ctx->enableHdr) {
        result = IMX664_ReadRHS1(handle, &pIMX664Ctx->cur_rhs1);
        result |= IMX664_ReadRHS2(handle, &pIMX664Ctx->cur_rhs2);
        result |= IMX664_ReadHmax(handle, &hmax);
        if (result != RET_SUCCESS) {
            TRACE(IMX664_ERROR, "%s: Read RHS1, RHS2 or HMAX failed\n", __func__);
            return result;
        }

        pIMX664Ctx->SensorMode.ae_info.one_line_exp_time_ns = (uint32_t)(((float)hmax / IMX664_PIXEL_CLK_RATE) * MICRO_2_NANO);
        pIMX664Ctx->one_line_exp_time =
        (float)(pIMX664Ctx->SensorMode.ae_info.one_line_exp_time_ns) / 1000000000;

        if (pIMX664Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
            //2exposure
            result = IMX664_Calculate2DOLExposures(handle, NewIntegrationTime, NewGain,
                                    &long_it, &short_it,
                                    &long_gain, &short_gain,
                                    hdr_ratio);
        } else {
            //3exposure
            result = IMX664_Calculate3DOLExposures(handle, NewIntegrationTime, NewGain,
                                        &long_it, &short_it, &very_short_it,
                                        &long_gain, &short_gain, &very_short_gain,
                                        hdr_ratio);
        }
        
        if (result != RET_SUCCESS) {
            TRACE(IMX664_ERROR, "%s: CalculateHDRExposures failed\n", __func__);
            return result;
        }

		result = IMX664_IsiSetLEFIntegrationTimeIss(handle, long_it,
							pSetIntegrationTime,
							pNumberOfFramesToSkip,
							hdr_ratio);
		result |= IMX664_IsiSetLEFGainIss(handle, long_gain, pSetGain, hdr_ratio);
		result |= IMX664_IsiSetSEF1IntegrationTimeIss(
			handle, short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		result |= IMX664_IsiSetSEF1GainIss(handle, NewIntegrationTime,
						  short_gain, pSetGain, hdr_ratio);
        if (pIMX664Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            result |= IMX664_IsiSetSEF2IntegrationTimeIss(
			    handle, very_short_it, pSetIntegrationTime,
			    pNumberOfFramesToSkip, hdr_ratio);
            result |= IMX664_IsiSetSEF2GainIss(handle, NewIntegrationTime,
                            very_short_gain, pSetGain, hdr_ratio);
        }

        // Recalculate `io_hdr_ratio` according to the set values
        hdr_ratio[0] = (long_it * long_gain) / (short_it * short_gain);
        if (pIMX664Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            hdr_ratio[1] = (short_it * short_gain) / (very_short_it * very_short_gain);
        }

        // Set the output values to SEF1 values
        *pSetGain = short_gain;
        *pSetIntegrationTime = short_it;

        TRACE(IMX664_DEBUG, "%s: actual hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n",
            __func__, hdr_ratio[0], hdr_ratio[1]);
    } else {
        result |= IMX664_IsiSetLEFIntegrationTimeIss(handle, NewIntegrationTime,
                                                pSetIntegrationTime,
                                                pNumberOfFramesToSkip, hdr_ratio);
        result |= IMX664_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
    }
    return result;
}

RESULT IMX664_IsiExposureControlExpandedIss(
    IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime,
    float NewIris, uint8_t* pNumberOfFramesToSkip, float* pSetGain,
    float* pSetIntegrationTime, float* pSetIris, float* hdr_ratio) {

    if (pSetIris) {
        IMX664_IsiSetIrisIss(handle, NewIris);
        *pSetIris = NewIris;
    }

    return IMX664_IsiExposureControlIss(handle, NewGain, NewIntegrationTime,
                                        pNumberOfFramesToSkip, pSetGain,
                                        pSetIntegrationTime, hdr_ratio);
}

RESULT IMX664_IsiGetCurrentExposureIss(IsiSensorHandle_t handle,
                                       float* pSetGain,
                                       float* pSetIntegrationTime) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX664Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pSetGain == NULL) || (pSetIntegrationTime == NULL))
        return (RET_NULL_POINTER);

    if (pIMX664Ctx->enableHdr) {
		*pSetGain = pIMX664Ctx->AecCurGainSEF1;
		*pSetIntegrationTime = pIMX664Ctx->AecCurIntegrationTimeSEF1;
	} else {
		*pSetGain = pIMX664Ctx->AecCurGainLEF;
		*pSetIntegrationTime = pIMX664Ctx->AecCurIntegrationTimeLEF;
	}

    return (result);
}

RESULT IMX664_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t* pFps) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX664Ctx == NULL) {
        printf("%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *pFps = pIMX664Ctx->CurrFps;

    return (result);
}

RESULT IMX664_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t Fps) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX664_IsiSetFlickerFpsIss(IsiSensorHandle_t handle, uint32_t flickerMode) {
    RESULT result = RET_SUCCESS;
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    uint32_t current_vmax = 0;
    uint32_t requested_vmax = 0;
    uint32_t shr = 0;
    int exp = 0;

    TRACE(IMX664_DEBUG, "%s: set sensor flickerMode = %d\n", __func__, flickerMode);

    if (!pIMX664Ctx) {
        return RET_NULL_POINTER;
    }
    if (pIMX664Ctx->flicker_fps_mode == flickerMode) {
        return RET_SUCCESS;
    }
    if (pIMX664Ctx->enableHdr && (pIMX664Ctx->SensorMode.stitching_mode != SENSOR_STITCHING_L_AND_S)) {
        return RET_SUCCESS;
    }
    if (flickerMode > ISI_AE_ANTIBANDING_MODE_AUTO) {
        TRACE(IMX664_INFO, "%s: Invalid flickerMode (%d), setting ISI_AE_ANTIBANDING_MODE_AUTO instead.\n", __func__, flickerMode);
        flickerMode = ISI_AE_ANTIBANDING_MODE_AUTO;
    }
    pIMX664Ctx->flicker_fps_mode = flickerMode;

    result = IMX664_ReadVmax(handle, &current_vmax);
    if (result != RET_SUCCESS) {
        TRACE(IMX664_ERROR, "%s: Unable to read VMAX\n", __func__);
        return (result);
    }
    if (pIMX664Ctx->original_vmax == 0) {
        pIMX664Ctx->original_vmax = current_vmax;
    }

    exp = pIMX664Ctx->AecCurIntegrationTimeLEF / pIMX664Ctx->one_line_exp_time;
    shr = MAX((int)current_vmax - exp, IMX664_MIN_SHR);

    if (current_vmax > pIMX664Ctx->original_vmax) {
        current_vmax = MAX((int)current_vmax - (int)shr + IMX664_MIN_SHR, IMX664_MIN_SHR);
        shr = MAX((int)current_vmax - exp, IMX664_MIN_SHR);
        pIMX664Ctx->unlimit_fps_vmax_changed = current_vmax > pIMX664Ctx->original_vmax && pIMX664Ctx->unlimit_fps;
    }

    requested_vmax = IMX664_getNewVmaxAntiFlicker(pIMX664Ctx, current_vmax);
    requested_vmax = MAX( MIN(requested_vmax, IMX664_VMAX_MAX), 1);
    
    if (current_vmax != requested_vmax) {
        shr = MAX( (int)requested_vmax - (int)current_vmax + (int)shr , IMX664_MIN_SHR);
        TRACE(IMX664_DEBUG, "%s - writing 0x%x to VMAX, writing 0x%x to SHR0\n", __func__, requested_vmax, shr);
        
        result |= IMX664_LockRegHold(handle);
        result |= IMX664_WriteVmax(handle, requested_vmax);
        result |= IMX664_WriteShr0(handle, shr);
        result |= IMX664_UnlockRegHold(handle);
        result |= IMX664_UpdateCurrLEFIntegrationTimeFromVmax(pIMX664Ctx, requested_vmax, shr);
        if (result != RET_SUCCESS) {
            TRACE(IMX664_ERROR, "%s: Unable to write VMAX or Shr0\n", __func__);
            return (result);
        }
    }
    
    pIMX664Ctx->MaxIntegrationLine = MAX( MIN(requested_vmax - IMX664_MIN_SHR, IMX664_VMAX_MAX - IMX664_MIN_SHR), 1);
    pIMX664Ctx->AecMaxIntegrationTime = pIMX664Ctx->one_line_exp_time * pIMX664Ctx->MaxIntegrationLine;

    TRACE(IMX664_INFO, "%s: set sensor fps = %d\n", __func__,
          pIMX664Ctx->CurrFps);

    TRACE(IMX664_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX664_IsiGetAutoFpsInfoIss(IsiSensorHandle_t handle,
                                   IsiAutoFps_t* pAutoFpsInfo) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX664_IsiGetStartEvIss(IsiSensorHandle_t handle, uint64_t* pStartEv) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX664_IsiGetIspStatusIss(IsiSensorHandle_t handle,
                                 IsiIspStatus_t* pIspStatus) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (pIMX664Ctx == NULL || pIMX664Ctx->IsiCtx.HalHandle == NULL) {
        return RET_WRONG_HANDLE;
    }

    pIspStatus->useSensorAE = false;
    pIspStatus->useSensorBLC = false;
    pIspStatus->useSensorAWB = false;

    return RET_SUCCESS;
}

RESULT IMX664_IsiSetTpgIss(IsiSensorHandle_t handle, IsiTpg_t Tpg) {
    RESULT result = RET_SUCCESS;

    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (pIMX664Ctx == NULL || pIMX664Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX664Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (Tpg.enable == 0) {
        result = IMX664_IsiWriteRegIss(handle, 0x3253, 0x00);
    } else {
        result = IMX664_IsiWriteRegIss(handle, 0x3253, 0x80);
    }

    pIMX664Ctx->TestPattern = Tpg.enable;

    return (result);
}

RESULT IMX664_IsiGetTpgIss(IsiSensorHandle_t handle, IsiTpg_t* Tpg) {
    RESULT result = RET_SUCCESS;
    uint32_t value = 0;

    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (pIMX664Ctx == NULL || pIMX664Ctx->IsiCtx.HalHandle == NULL ||
        Tpg == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX664Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (!IMX664_IsiReadRegIss(handle, 0x5081, &value)) {
        Tpg->enable = ((value & 0x80) != 0) ? 1 : 0;
        if (Tpg->enable) {
            Tpg->pattern = (0xff & value);
        }
        pIMX664Ctx->TestPattern = Tpg->enable;
    }

    return (result);
}

RESULT IMX664_IsiFocusCreateIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX664_IsiFocusReleaseIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX664_IsiFocusGetCalibrateIss(IsiSensorHandle_t handle,
                                      IsiFocusCalibAttr_t* pFocusCalib) {
    return RET_SUCCESS;
}

RESULT IMX664_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

RESULT IMX664_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

// Taken from the IMX681 driver
RESULT IMX664_IsiSetAgainDgainIss(IsiSensorHandle_t handle,
                                  IsiUserGain_t Gain) {
    RESULT result = RET_SUCCESS;
    uint32_t Again = 0, Dgain = 0;
    TRACE(IMX664_INFO, "%s: (enter)\n", __func__);

    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (pIMX664Ctx == NULL) {
        TRACE(IMX664_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((Gain.again < 1) | (Gain.again > 16)) {
        TRACE(IMX664_ERROR, "%s: Invalid sensor again\n", __func__);
        return (RET_OUTOFRANGE);
    }
    // Again = (uint32_t)(1024 - (1024/Gain.again));
    Again = (uint32_t)(((2048 * Gain.again) - 2048) / Gain.again);

    result = IMX664_IsiWriteRegIss(handle, 0x309c, (Again & 0x0000FF));
    result = IMX664_IsiWriteRegIss(handle, 0x309d, (Again & 0x00FF00) >> 8);

    if ((Gain.dgain < 1) | (Gain.dgain > 16)) {
        TRACE(IMX664_ERROR, "%s: Invalid sensor dgain\n", __func__);
        return (RET_OUTOFRANGE);
    }
    Dgain = Gain.dgain * 256;

    result = IMX664_IsiWriteRegIss(handle, 0x308c, (Dgain & 0x0000FF));
    result = IMX664_IsiWriteRegIss(handle, 0x308d, (Dgain & 0x00FF00) >> 8);

    // pIMX664Ctx->CurAgain = Gain.again;
    // pIMX664Ctx->CurDgain = Gain.dgain;

    TRACE(IMX664_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX664_IsiGetIrisIss( IsiSensorHandle_t handle,
                                     float *pSetIris ) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX664_INFO, "%s: (enter)\n", __func__);

    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (pIMX664Ctx == NULL) {
        TRACE(IMX664_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    //TODO implement getting iris here!
    *pSetIris = 1.0;

    TRACE(IMX664_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX664_IsiSetIrisIss( IsiSensorHandle_t handle, 
                    const float NewIris) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX664_INFO, "%s: (enter)\n", __func__);

    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (pIMX664Ctx == NULL) {
        TRACE(IMX664_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (NewIris < IMX664_IRIS_MIN_VAL || NewIris > IMX664_IRIS_MAX_VAL) {
        TRACE(IMX664_ERROR, "%s: Invalid Iris %f\n", __func__, NewIris);
    }

    //TODO implement setting iris here!

    TRACE(IMX664_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX664_IsiGetHCGIss( IsiSensorHandle_t handle,
                                     bool *phcg ) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX664_INFO, "%s: (enter)\n", __func__);

    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    if (pIMX664Ctx == NULL) {
        TRACE(IMX664_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *phcg = pIMX664Ctx->hcg;

    TRACE(IMX664_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX664_IsiSetHCGIss(IsiSensorHandle_t handle, bool hcg) {
    
    RESULT result = RET_SUCCESS;

    TRACE(IMX664_INFO, "%s: (enter)\n", __func__);

    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;

    if (pIMX664Ctx == NULL) {
        TRACE(IMX664_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    result = IMX664_IsiWriteRegIss(handle, 0x3030 , hcg);
    if (result == RET_SUCCESS) {
        pIMX664Ctx->hcg = hcg;
    }

    TRACE(IMX664_INFO, "%s: (exit)\n", __func__);
    return result;
    
}

static RESULT IMX664_CalculateHdrBlankingLines(IsiSensorHandle_t handle,
        uint32_t *pBlankingLines, uint32_t rhs1, uint32_t rhs2) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;

    /* The following formulas are taken directly from Sony's IMX664 datasheet.
     * They are located at the "AppNote_DOL_E" document,
     * at "Operating Mode" -> "Number of Blanking Lines" section ("Virtual Channel Mode"). */

    /* The formula is different from AllPixel/Binning configurations,
     * so we will assume that FHD resolution will be binning */
    bool allPixelMode = (pIMX664Ctx->SensorMode.size.width == IMX664_RES_4MP_WIDTH &&
                         pIMX664Ctx->SensorMode.size.height == IMX664_RES_4MP_HEIGHT);

    if (!allPixelMode) {
        // Make sure the resolution matches a 2/2 binning mode
        if (pIMX664Ctx->SensorMode.size.width != (IMX664_RES_4MP_WIDTH / 2) ||
            pIMX664Ctx->SensorMode.size.height != (IMX664_RES_4MP_HEIGHT / 2)) {
            TRACE(IMX664_ERROR, "%s: Unsupported resolution %dx%d for HDR\n",
                  __func__, pIMX664Ctx->SensorMode.size.width,
                  pIMX664Ctx->SensorMode.size.height);
            return RET_NOTSUPP;
        }
    }

    // 2DOL
    if (pIMX664Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        if (allPixelMode) {
            pBlankingLines[0] = (rhs1 - 4) / 2 + 1;
        } else { // 2/2 Binning
            pBlankingLines[0] = (rhs1 - 6) / 4 + 1;
        }
        pBlankingLines[1] = 0; // Irrelevant in 2DOL
    // 3DOL
    } else if (pIMX664Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        if (allPixelMode) {
            pBlankingLines[0] = (rhs1 - 5) / 3 + 1;
            pBlankingLines[1] = (rhs2 - 7) / 3 + 1;
        } else { // 2/2 Binning
            pBlankingLines[0] = (rhs1 - 8) / 6 + 1;
            pBlankingLines[1] = (rhs2 - 10) / 6 + 1;
        }
    } else {
        TRACE(IMX664_ERROR, "%s: Unsupported stitching mode %d\n",
              __func__, pIMX664Ctx->SensorMode.stitching_mode);
        return RET_NOTSUPP;
    }

    return RET_SUCCESS;
}

static RESULT IMX664_IsiGetHdrBlankingLinesIss(IsiSensorHandle_t handle,
        uint32_t *pBlankingLines, size_t elementCount) {
    IMX664_Context_t* pIMX664Ctx = (IMX664_Context_t*)handle;
    HalContext_t* pHalCtx = (HalContext_t*)pIMX664Ctx->IsiCtx.HalHandle;
    RESULT result = RET_SUCCESS;
    uint32_t rhs1 = 0, rhs2 = 0;

    if (elementCount != 2) {
        TRACE(IMX664_ERROR, "%s: Invalid element count %zu, expected 2\n",
              __func__, elementCount);
        return RET_OUTOFRANGE;
    }

    rhs1 = IMX664_GetCtrl(pHalCtx->sensor_fd, IMX664_CID_RHS1);
    rhs2 = IMX664_GetCtrl(pHalCtx->sensor_fd, IMX664_CID_RHS2);

    if (rhs1 < 0 || rhs2 < 0) {
        TRACE(IMX664_ERROR, "%s: Unable to read RHS1 or RHS2, result: %d\n",
              __func__, result);
        return RET_FAILURE;
    }

    result = IMX664_CalculateHdrBlankingLines(handle, pBlankingLines, rhs1, rhs2);
    return result;
}

RESULT IMX664_IsiGetSensorIss(IsiSensor_t* pIsiSensor) {
    RESULT result = RET_SUCCESS;
    static const char SensorName[16] = "IMX664";
    TRACE(IMX664_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor != NULL) {
        pIsiSensor->pszName = SensorName;
        pIsiSensor->pIsiCreateIss = IMX664_IsiCreateIss;
        pIsiSensor->pIsiReleaseIss = IMX664_IsiReleaseIss;
        pIsiSensor->pIsiReadRegIss = IMX664_IsiReadRegIss;
        pIsiSensor->pIsiWriteRegIss = IMX664_IsiWriteRegIss;
        pIsiSensor->pIsiGetModeIss = IMX664_IsiGetModeIss;
        pIsiSensor->pIsiSetModeIss = IMX664_IsiSetModeIss;
        pIsiSensor->pIsiEnumModeIss = IMX664_IsiEnumModeIss;
        pIsiSensor->pIsiGetCapsIss = IMX664_IsiGetCapsIss;
        pIsiSensor->pIsiSetupIss = IMX664_IsiSetupIss;
        pIsiSensor->pIsiCheckConnectionIss = IMX664_IsiCheckConnectionIss;
        pIsiSensor->pIsiGetRevisionIss = IMX664_IsiGetRevisionIss;
        pIsiSensor->pIsiSetStreamingIss = IMX664_IsiSetStreamingIss;

        /* AEC functions */
        pIsiSensor->pIsiGetGainLimitsIss = 					IMX664_IsiGetGainLimitsIss;
		pIsiSensor->pIsiGetIrisLimitsIss = 					IMX664_IsiGetIrisLimitsIss;
		pIsiSensor->pIsiSetIrisLimitsIss = 					IMX664_IsiSetIrisLimitsIss;
		pIsiSensor->pIsiGetIntegrationTimeLimitsIss =		IMX664_IsiGetIntegrationTimeLimitsIss;

		pIsiSensor->pIsiExposureControlIss =				IMX664_IsiExposureControlIss;
		pIsiSensor->pIsiExposureControlExpandedIss =		IMX664_IsiExposureControlExpandedIss;
		pIsiSensor->pIsiSetIntegrationTimeIss =				IMX664_IsiSetIntegrationTimeIss;

		pIsiSensor->pIsiGetLongIntegrationTimeIss =			IMX664_IsiGetLEFIntegrationTimeIss;
		pIsiSensor->pIsiGetIntegrationTimeIss =				IMX664_IsiGetIntegrationTimeIss;
		pIsiSensor->pIsiGetShortIntegrationTimeIss =	    IMX664_IsiGetSEF1IntegrationTimeIss;
		pIsiSensor->pIsiGetVSIntegrationTimeIss =			IMX664_IsiGetSEF2IntegrationTimeIss;

		pIsiSensor->pIsiGetLongGainIss = 					IMX664_IsiGetLEFGainIss;
		pIsiSensor->pIsiGetGainIss = 						IMX664_IsiGetGainIss;
		pIsiSensor->pIsiGetShortGainIss = 					IMX664_IsiGetSEF1GainIss;
		pIsiSensor->pIsiGetVSGainIss = 						IMX664_IsiGetSEF2GainIss;

		pIsiSensor->pIsiGetGainIncrementIss =				IMX664_IsiGetGainIncrementIss;
		pIsiSensor->pIsiGetIrisIncrementIss =				IMX664_IsiGetIrisIncrementIss;
		pIsiSensor->pIsiGetIntegrationTimeIncrementIss =	IMX664_IsiGetIntegrationTimeIncrementIss;
		pIsiSensor->pIsiSetGainIss = 						IMX664_IsiSetGainIss;
		pIsiSensor->pIsiGetFpsIss = 						IMX664_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss = 						IMX664_IsiSetFpsIss;
        pIsiSensor->pIsiSetFlickerFpsIss = 					IMX664_IsiSetFlickerFpsIss;
        pIsiSensor->pIsiUnlimitFpsIss =						IMX664_IsiUnlimitFpsIss;
        pIsiSensor->pIsiLimitFpsIss =						IMX664_IsiLimitFpsIss;
		pIsiSensor->pIsiGetAutoFpsInfoIss = 				IMX664_IsiGetAutoFpsInfoIss;
		pIsiSensor->pIsiGetStartEvIss = 					IMX664_IsiGetStartEvIss;
        pIsiSensor->pIsiGetIrisIss =						IMX664_IsiGetIrisIss;
        pIsiSensor->pIsiSetIrisIss =						IMX664_IsiSetIrisIss;
        pIsiSensor->pIsiGetHCGIss =                         IMX664_IsiGetHCGIss;
        pIsiSensor->pIsiSetHCGIss =                         IMX664_IsiSetHCGIss;

        /* SENSOR ISP */
        pIsiSensor->pIsiGetIspStatusIss = IMX664_IsiGetIspStatusIss;
        // pIsiSensor->pIsiSetBlcIss                       =
        // IMX664_IsiSetBlcIss; pIsiSensor->pIsiSetWBIss = IMX664_IsiSetWBIss;

        /* SENSOE OTHER FUNC*/
        pIsiSensor->pIsiSetPowerIss = IMX664_IsiSetPowerIss;
        pIsiSensor->pIsiSetTpgIss = IMX664_IsiSetTpgIss;
        pIsiSensor->pIsiGetTpgIss = IMX664_IsiGetTpgIss;
        // pIsiSensor->pIsiGetExpandCurveIss               =
        // IMX664_IsiGetExpandCurveIss; pIsiSensor->pIsiGetCompressCurveIss =
        // IMX664_IsiGetCompressCurveIss; pIsiSensor->pIsiExtendFuncIss =
        // IMX664_IsiExtendFuncIss; pIsiSensor->pIsiGetOtpDataIss =
        // IMX664_IsiGetOtpDataIss;
        pIsiSensor->pIsiGetHdrBlankingLinesIss = IMX664_IsiGetHdrBlankingLinesIss;

        /* AF */
        pIsiSensor->pIsiFocusCreateIss = IMX664_IsiFocusCreateIss;
        pIsiSensor->pIsiFocusReleaseIss = IMX664_IsiFocusReleaseIss;
        pIsiSensor->pIsiFocusGetCalibrateIss = IMX664_IsiFocusGetCalibrateIss;
        pIsiSensor->pIsiFocusSetIss = IMX664_IsiFocusSetIss;
        pIsiSensor->pIsiFocusGetIss = IMX664_IsiFocusGetIss;
        pIsiSensor->pIsiSetAgainDgainIss = IMX664_IsiSetAgainDgainIss;

    } else {
        result = RET_NULL_POINTER;
    }

    TRACE(IMX664_INFO, "%s (exit)\n", __func__);
    return (result);
}

/*****************************************************************************
 * each sensor driver need declare this struct for isi load
 *****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x9012,
    .pIsiHalEnumModeIss = IMX664_IsiHalEnumModeIss,
    .pIsiGetSensorIss = IMX664_IsiGetSensorIss,
};
