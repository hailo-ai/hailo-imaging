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

#include "IMX678_priv.h"
#include "vvsensor.h"
#include <linux/i2c.h>

CREATE_TRACER(IMX678_INFO, "IMX678: ", INFO, 1)
CREATE_TRACER(IMX678_WARN, "IMX678: ", WARNING, 1)
CREATE_TRACER(IMX678_ERROR, "IMX678: ", ERROR, 1)
CREATE_TRACER(IMX678_DEBUG, "IMX678: ", INFO, 1)
CREATE_TRACER(IMX678_REG_INFO, "IMX678: ", INFO, 1)
CREATE_TRACER(IMX678_REG_DEBUG, "IMX678: ", INFO, 1)

#define LOG_ERROR(...) TRACE(IMX678_ERROR, __VA_ARGS__)

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
#define IMX678_VMAX_2DOL_HDR 4500
#define IMX678_VMAX_MAX 135000
#define IMX678_MIN_SHR 3
#define IMX678_MIN_LINES 1
#define IMX678_MAX_GAIN_AEC                                                    \
    (32.0f) /**< max. gain used by the AEC (arbitrarily chosen, hardware limit \
               = 62.0, driver limit = 32.0 ) */
#define IMX678_VS_MAX_INTEGRATION_TIME (0.0018)
#define IMX678_TRANSFER_BUFFER_LENGTH 3
#define IMX678_MAX_GAIN 3981
#define IMX678_3DOL_SHR0_RHS2_GAP 7
#define IMX678_2DOL_SHR0_RHS1_GAP 5
#define IMX678_3DOL_SHR0_FSC_GAP 3
#define IMX678_2DOL_SHR0_FSC_GAP 2
/* When required long EV is within this many lines of max_long_it, use unity gain to avoid quantization noise. */
#define IMX678_2DOL_MAX_IT_NEAR_LINES 2
#define IMX678_3DOL_SHR1_RHS1_GAP 3
#define IMX678_2DOL_SHR1_MIN_GAP 5
#define IMX678_3DOL_SHR1_MIN_GAP 7
#define IMX678_2DOL_SHR1_RHS1_GAP 2
#define IMX678_3DOL_SHR1_RHS1_GAP 3
#define IMX678_SHR2_RHS1_GAP 7
#define IMX678_SHR2_RHS2_GAP 3
#define IMX678_PIXEL_CLK_RATE 74.25

#define IMX678_RES_4K_HEIGHT 2160
#define IMX678_RES_4K_WIDTH 3840
#define IMX678_RES_FHD_HEIGHT 1080
#define IMX678_RES_FHD_WIDTH 1920

#define MICRO_2_NANO 1000
#define IMX678_2DOL_NUM_EXP 2
#define IMX678_3DOL_NUM_EXP 3

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
			.bounds_width  = IMX678_RES_4K_WIDTH,
			.bounds_height = IMX678_RES_4K_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX678_RES_4K_WIDTH,
			.height        = IMX678_RES_4K_HEIGHT,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_RGGB,
	.ae_info = {
		.one_line_exp_time_ns = 7407,
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
			.bounds_width  = IMX678_RES_FHD_WIDTH,
			.bounds_height = IMX678_RES_FHD_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX678_RES_FHD_WIDTH,
			.height        = IMX678_RES_FHD_HEIGHT,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_RGGB,
	.ae_info = {
		.one_line_exp_time_ns = 7407,
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
			.bounds_width  = IMX678_RES_FHD_WIDTH,
			.bounds_height = IMX678_RES_FHD_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX678_RES_FHD_WIDTH,
			.height        = IMX678_RES_FHD_HEIGHT,
		},
		.fps       = 20 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_3DOL,
		.bit_width = 12,
		.bayer_pattern = BAYER_RGGB,
        .ae_info = {
		.one_line_exp_time_ns = 7407,
		.max_integration_time = 1,
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
			.bounds_width  = IMX678_RES_4K_WIDTH,
			.bounds_height = IMX678_RES_4K_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX678_RES_4K_WIDTH,
			.height        = IMX678_RES_4K_HEIGHT,
		},
		.fps       = 20 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_3DOL,
		.bit_width = 12,
		.bayer_pattern = BAYER_RGGB,
        .ae_info = {
		.one_line_exp_time_ns = 7407,
		.max_integration_time = 1,
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
			.bounds_width  = IMX678_RES_4K_WIDTH,
			.bounds_height = IMX678_RES_4K_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX678_RES_4K_WIDTH,
			.height        = IMX678_RES_4K_HEIGHT,
		},
		.fps       = 30 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_L_AND_S,
		.bit_width = 12,
		.bayer_pattern = BAYER_RGGB,
        .ae_info = {
		.one_line_exp_time_ns = 7407,
		.max_integration_time = 1,
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

static RESULT IMX678_GetCtrl(int sensor_fd, unsigned int ctrl_cid, int32_t *value)
{
	struct v4l2_control control;
	memset(&control, 0, sizeof(control));
	control.id = ctrl_cid;

	int ret = ioctl(sensor_fd, VIDIOC_G_CTRL, &control);
	if (ret < 0) {
		TRACE(IMX678_ERROR, "%s: VIDIOC_G_CTRL ioctl failed with error %d (%s). ret = %d\n",
			__func__, errno, strerror(errno), ret);
		return RET_FAILURE;
	}

	*value = control.value;
	return RET_SUCCESS;
}

static uint32_t IMX678_CalculateModeMaxIntegrationTime(
        uint32_t hdr_mode, uint32_t stitching_mode, uint32_t rhs1) {
	if (hdr_mode == SENSOR_MODE_LINEAR) {
		return IMX678_VMAX_30FPS - IMX678_MIN_SHR; // SDR
	} else {
        uint32_t rhs1_gap = (stitching_mode == SENSOR_STITCHING_3DOL) ?
            IMX678_3DOL_SHR1_RHS1_GAP : IMX678_2DOL_SHR1_RHS1_GAP;
        return rhs1 - rhs1_gap; // HDR
	}
}

static RESULT IMX678_SetSensorModeData(IMX678_Context_t* pIMX678Ctx, size_t index) {
    int sensor_fd;
    int32_t rhs1 = 0;
    RESULT result = RET_SUCCESS;

    if (pIMX678Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    sensor_fd = ((HalContext_t*)pIMX678Ctx->IsiCtx.HalHandle)->sensor_fd;

    if (index >= ARRAY_SIZE(pimx678_mode_info)) {
        return RET_OUTOFRANGE;
    }

    pIMX678Ctx->SensorMode.index = index;
    memcpy(&pIMX678Ctx->SensorMode, &pimx678_mode_info[index], sizeof(pIMX678Ctx->SensorMode));
    if (pIMX678Ctx->SensorMode.index != index) {
        TRACE(IMX678_ERROR, "%s: Sensor mode index mismatch: expected %zu, got %u\n",
              __func__, index, pIMX678Ctx->SensorMode.index);
        return RET_FAILURE;
    }

    // If the mode is HDR, we need to get RHS1 value from the sensor.
    if (pIMX678Ctx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        /* Get the RHS1 value via a control rather then reading the register,
         * because the register is not available until a stream is running. */
        result = IMX678_GetCtrl(sensor_fd, IMX678_CID_RHS1, &rhs1);
        if ((result != RET_SUCCESS) || rhs1 == 0) {
            TRACE(IMX678_ERROR, "%s: Failed to get RHS1 value from sensor (result=%d)\n",
                __func__, result);
            return result;
        }
    }

    pIMX678Ctx->SensorMode.ae_info.max_integration_time =
        IMX678_CalculateModeMaxIntegrationTime(
            pIMX678Ctx->SensorMode.hdr_mode, pIMX678Ctx->SensorMode.stitching_mode, rhs1);

    if (pIMX678Ctx->SensorMode.ae_info.max_integration_time == 0) {
        TRACE(IMX678_ERROR, "%s: Invalid max integration time calculated (%u) for HDR mode %d, index %zu\n",
              __func__, pIMX678Ctx->SensorMode.ae_info.max_integration_time,
              pIMX678Ctx->SensorMode.hdr_mode, index);
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

static RESULT IMX678_IsiCreateIss(IsiSensorInstanceConfig_t* pConfig) {
    RESULT result = RET_SUCCESS;
    IMX678_Context_t* pIMX678Ctx;
    char i2c_file_path[PATH_MAX];

    memset(i2c_file_path, 0, PATH_MAX);

    if (!pConfig || !pConfig->pSensor) return (RET_NULL_POINTER);

    pIMX678Ctx = (IMX678_Context_t*)malloc(sizeof(IMX678_Context_t));
    if (!pIMX678Ctx) {
        TRACE(IMX678_ERROR, "%s: Failed to allocate memory for context\n", __func__);
        result = RET_OUTOFMEM;
        goto error;
    }

    MEMSET(pIMX678Ctx, 0, sizeof(IMX678_Context_t));
    pIMX678Ctx->i2c_fd = -1;
    pIMX678Ctx->i2c_af_fd = -1;

    result = HalAddRef(pConfig->HalHandle);
    if (result != RET_SUCCESS) {
        TRACE(IMX678_ERROR, "%s: HalAddRef failed (result=%d)\n", __func__, result);
        goto error_free_ctx;
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

    // By default, until specified otherwise, all ratios are 1 (SDR)
    pIMX678Ctx->hdr_ratio[0] = 1.0f;
    pIMX678Ctx->hdr_ratio[1] = 1.0f;
    
    result = IMX678_SetSensorModeData(pIMX678Ctx, pConfig->SensorModeIndex);
    if (result != RET_SUCCESS) {
        TRACE(IMX678_ERROR, "%s: Set sensor mode data failed! (result=%d)\n", __func__, result);
        goto error_hal_del_ref;
    }

    pConfig->hSensor = (IsiSensorHandle_t)pIMX678Ctx;
    pIMX678Ctx->pattern = ISI_BPAT_RGRGGBGB;
    pIMX678Ctx->subdev = HalGetFdHandle(pConfig->HalHandle,
                                        HAL_MODULE_SENSOR);  // two sensors??
    pIMX678Ctx->KernelDriverFlag = 1;
    TRACE(IMX678_INFO, "%s - sensor i2c bus: %d, af i2c bus: %d, sensor i2c addr: 0x%x, af i2c addr: 0x%x\n", __func__,
           pConfig->I2cBusNum, pConfig->I2cAfBusNum, pConfig->SlaveAddr, pConfig->SlaveAfAddr);
    pIMX678Ctx->i2c_addr = pConfig->SlaveAddr;
    pIMX678Ctx->i2c_af_addr = pConfig->SlaveAfAddr;

    sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cBusNum);
    pIMX678Ctx->i2c_fd = open(i2c_file_path, O_RDWR);
    if (pIMX678Ctx->i2c_fd < 0) {
        TRACE(IMX678_ERROR, "%s: unable to open /dev/i2c-%d: %d (%s)\n", __func__, pConfig->I2cBusNum, errno, strerror(errno));
        result = RET_FAILURE;
        goto error_hal_del_ref;
    }

    int ret = ioctl(pIMX678Ctx->i2c_fd, I2C_SLAVE_FORCE, pIMX678Ctx->i2c_addr);
    if (ret < 0) {
        TRACE(IMX678_ERROR, "%s: unable to set I2C_SLAVE_FORCE on /dev/i2c-%d. I2C_SLAVE_FORCE ioctl failed with error %d (%s). ret = %d\n",
              __func__, pConfig->I2cBusNum, errno, strerror(errno), ret);
        result = RET_FAILURE;
        goto error_close_i2c_fd;
    }

    if (pConfig->I2cAfBusNum < 0) {
        TRACE(IMX678_INFO, "no af i2c bus\n");
        pIMX678Ctx->SensorMode.af_mode = ISI_SENSOR_AF_MODE_NOTSUPP;
    } else {
        sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cAfBusNum);
        pIMX678Ctx->i2c_af_fd = open(i2c_file_path, O_RDWR);
        if (pIMX678Ctx->i2c_af_fd < 0) {
            TRACE(IMX678_ERROR, "%s: unable to open /dev/i2c-%d: %d (%s)\n",
                  __func__, pConfig->I2cAfBusNum, errno, strerror(errno));
            result = RET_FAILURE;
            goto error_close_i2c_fd;
        }
        pIMX678Ctx->SensorMode.af_mode = ISI_SENSOR_AF_MODE_CDAF;
    }

    return RET_SUCCESS;

error_close_i2c_fd:
    close(pIMX678Ctx->i2c_fd);
error_hal_del_ref:
    HalDelRef(pIMX678Ctx->IsiCtx.HalHandle);
error_free_ctx:
    free(pIMX678Ctx);
error:
    return (result);
}

static RESULT IMX678_IsiReleaseIss(IsiSensorHandle_t handle) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    RESULT cur_result = RET_SUCCESS;

    if (pIMX678Ctx == NULL) return (RET_WRONG_HANDLE);

    cur_result = IMX678_IsiSetStreamingIss(pIMX678Ctx, BOOL_FALSE);
    UPDATE_RESULT_LOG(result, cur_result, "SetStreaming off");
    cur_result = IMX678_IsiSetPowerIss(pIMX678Ctx, BOOL_FALSE);
    UPDATE_RESULT_LOG(result, cur_result, "SetPower off");
    (void)HalDelRef(pIMX678Ctx->IsiCtx.HalHandle);
    if (pIMX678Ctx->i2c_af_fd >= 0) {
        close(pIMX678Ctx->i2c_af_fd);
    }
    close(pIMX678Ctx->i2c_fd);
    MEMSET(pIMX678Ctx, 0, sizeof(IMX678_Context_t));
    free(pIMX678Ctx);
    pIMX678Ctx = NULL;
    return (result);
}

static RESULT IMX678_IsiReadRegIss(IsiSensorHandle_t handle,
                                   const uint32_t Addr, uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    struct i2c_rdwr_ioctl_data ioctl_data;
    unsigned char out[IMX678_TRANSFER_BUFFER_LENGTH];
    struct i2c_msg msgs[2];
    uint8_t addr_buf[2] = { (Addr >> 8) & 0xff, Addr & 0xff };
    int ret = 0;

    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR, "%s: Invalid sensor handle\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    msgs[0].addr = pIMX678Ctx->i2c_addr;
    msgs[0].flags = 0; // Write
    msgs[0].len = sizeof(addr_buf);
    msgs[0].buf = addr_buf;

    msgs[1].addr = pIMX678Ctx->i2c_addr;
    msgs[1].flags = I2C_M_RD; // Read
    msgs[1].len = 1;
    msgs[1].buf = out;

    ioctl_data.msgs = msgs;
    ioctl_data.nmsgs = 2;

    ret = ioctl(pIMX678Ctx->i2c_fd, I2C_RDWR, &ioctl_data);
    if (ret < 0) {
        TRACE(IMX678_ERROR, "%s: I2C_RDWR ioctl failed with error %d (%s). ret = %d\n",
            __func__, errno, strerror(errno), ret);
        return RET_FAILURE;
    }

    *pValue = out[0];

    return (result);
}

static RESULT IMX678_IsiWriteRegIss(IsiSensorHandle_t handle,
                                    const uint32_t Addr, const uint32_t Value) {
    char out[IMX678_TRANSFER_BUFFER_LENGTH];
    ssize_t write_ret = 0;

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR, "%s: Invalid sensor handle\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX678_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    out[2] = Value;

    write_ret = write(pIMX678Ctx->i2c_fd, out, sizeof(out));
    if (write_ret != sizeof(out)) {
        if (write_ret < 0) {
            TRACE(IMX678_ERROR, "%s: I2C write failed with error %d (%s)\n",
                  __func__, errno, strerror(errno));
        } else {
            TRACE(IMX678_ERROR, "%s: I2C write incomplete. Wrote %zd of %zu bytes\n",
                  __func__, write_ret, sizeof(out));
        }
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

static RESULT IMX678_UpdateFps(IMX678_Context_t *pIMX678Ctx, uint32_t vmax) {
    float frame_time = 0;
    frame_time = (vmax * pIMX678Ctx->one_line_exp_time);
    if (frame_time == 0) {
        TRACE(IMX678_ERROR, "%s: Invalid frame_time (vmax=%u, one_line_exp_time=%f)\n",
              __func__, vmax, pIMX678Ctx->one_line_exp_time);
        return RET_FAILURE;
    }

    pIMX678Ctx->CurrFps = (uint32_t)(ceil(1 / frame_time)) * ISI_FPS_ACCURACY;
    return RET_SUCCESS;
}

static RESULT IMX678_ReadVmax(IsiSensorHandle_t handle, uint32_t* vmax) {
    uint32_t vmax_low = 0, vmax_mid = 0, vmax_high = 0;
    RESULT result;

    result = IMX678_IsiReadRegIss(handle, 0x3028, &vmax_low);
    CHECK_RESULT_RET(result, "ReadVmax low");
    result = IMX678_IsiReadRegIss(handle, 0x3029, &vmax_mid);
    CHECK_RESULT_RET(result, "ReadVmax mid");
    result = IMX678_IsiReadRegIss(handle, 0x302a, &vmax_high);
    CHECK_RESULT_RET(result, "ReadVmax high");

    *vmax = (vmax_high << 16) | (vmax_mid << 8) | vmax_low;
    return RET_SUCCESS;
}

static RESULT IMX678_ReadHmax(IsiSensorHandle_t handle, uint32_t* hmax) {
    uint32_t hmax_low = 0, hmax_high = 0;
    RESULT result;

    result = IMX678_IsiReadRegIss(handle, 0x302c, &hmax_low);
    CHECK_RESULT_RET(result, "ReadHmax low");
    result = IMX678_IsiReadRegIss(handle, 0x302d, &hmax_high);
    CHECK_RESULT_RET(result, "ReadHmax high");

    *hmax = (hmax_high << 8) | hmax_low;
    return RET_SUCCESS;
}

static RESULT IMX678_WriteVmax(IsiSensorHandle_t handle, uint32_t vmax) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3028, vmax & 0xff);
    CHECK_RESULT_RET(result, "WriteVmax low");
    result = IMX678_IsiWriteRegIss(handle, 0x3029, (vmax >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteVmax mid");
    result = IMX678_IsiWriteRegIss(handle, 0x302a, (vmax >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteVmax high");

    result = IMX678_UpdateFps((IMX678_Context_t *)handle, vmax);
    CHECK_RESULT_RET(result, "UpdateFps");
    return RET_SUCCESS;
}

static RESULT IMX678_ReadRHS1(IsiSensorHandle_t handle, uint32_t* rhs1) {
    uint32_t rhs1_low = 0, rhs1_mid = 0, rhs1_high = 0;
    RESULT result;

    result = IMX678_IsiReadRegIss(handle, 0x3060, &rhs1_low);
    CHECK_RESULT_RET(result, "ReadRHS1 low");
    result = IMX678_IsiReadRegIss(handle, 0x3061, &rhs1_mid);
    CHECK_RESULT_RET(result, "ReadRHS1 mid");
    result = IMX678_IsiReadRegIss(handle, 0x3062, &rhs1_high);
    CHECK_RESULT_RET(result, "ReadRHS1 high");

    *rhs1 = (rhs1_high << 16) | (rhs1_mid << 8) | rhs1_low;
    return RET_SUCCESS;
}

static RESULT IMX678_ReadRHS2(IsiSensorHandle_t handle, uint32_t* rhs2) {
    uint32_t rhs2_low = 0, rhs2_mid = 0, rhs2_high = 0;
    RESULT result;

    result = IMX678_IsiReadRegIss(handle, 0x3064, &rhs2_low);
    CHECK_RESULT_RET(result, "ReadRHS2 low");
    result = IMX678_IsiReadRegIss(handle, 0x3065, &rhs2_mid);
    CHECK_RESULT_RET(result, "ReadRHS2 mid");
    result = IMX678_IsiReadRegIss(handle, 0x3066, &rhs2_high);
    CHECK_RESULT_RET(result, "ReadRHS2 high");

    *rhs2 = (rhs2_high << 16) | (rhs2_mid << 8) | rhs2_low;
    return RET_SUCCESS;
}


static RESULT IMX678_WriteShr0(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3050, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr0 low");
    result = IMX678_IsiWriteRegIss(handle, 0x3051, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr0 mid");
    result = IMX678_IsiWriteRegIss(handle, 0x3052, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr0 high");

    return RET_SUCCESS;
}

static RESULT IMX678_WriteShr1(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3054, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr1 low");
    result = IMX678_IsiWriteRegIss(handle, 0x3055, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr1 mid");
    result = IMX678_IsiWriteRegIss(handle, 0x3056, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr1 high");

    return RET_SUCCESS;
}

static RESULT IMX678_WriteShr2(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3058, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr2 low");
    result = IMX678_IsiWriteRegIss(handle, 0x3059, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr2 mid");
    result = IMX678_IsiWriteRegIss(handle, 0x305a, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr2 high");

    return RET_SUCCESS;
}

static RESULT IMX678_WriteGain(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3070, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain low");
    result = IMX678_IsiWriteRegIss(handle, 0x3071, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain high");

    return RET_SUCCESS;
}

static RESULT IMX678_WriteGain1(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3072, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain1 low");
    result = IMX678_IsiWriteRegIss(handle, 0x3073, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain1 high");

    return RET_SUCCESS;
}

static RESULT IMX678_WriteGain2(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3074, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain2 low");
    result = IMX678_IsiWriteRegIss(handle, 0x3075, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain2 high");

    return RET_SUCCESS;
}

static RESULT IMX678_LockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3001, 0x1);
    CHECK_RESULT_RET(result, "LockRegHold");

    return result;
}

static RESULT IMX678_UnlockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX678_IsiWriteRegIss(handle, 0x3001, 0x0);
    CHECK_RESULT_RET(result, "UnlockRegHold");

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
        TRACE(IMX678_ERROR, "%s: VVSENSORIOC_S_SENSOR_MODE ioctl failed with error %d (%s). ret = %d\n",
            __func__, errno, strerror(errno), ret);
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
            TRACE(IMX678_INFO, "%s (exit)\n", __func__);
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
    CHECK_RESULT_RET(result, "sensor enum mode");

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

    if (!pIMX678Ctx->Configured) {
        result = IMX678_IsiSetupIss(handle, pCaps);
        CHECK_RESULT_RET(result, "Setup");
    }

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

    /* SW reset comment preserved - no actual reset code was present,
     * and the 100ms sleep was unnecessary (no hardware access in this function). */

    result = IMX678_AecSetModeParameters(pIMX678Ctx, pCaps);
    CHECK_RESULT_RET(result, "AecSetModeParameters");

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
            TRACE(IMX678_ERROR, "%s: Read Sensor ID Error! VVSENSORIOC_G_CHIP_ID ioctl failed with error %d (%s). ret = %d\n",
                  __func__, errno, strerror(errno), ret);
            return (RET_FAILURE);
        }
    } else {
        reg_val = 0;
        result = IMX678_IsiReadRegIss(handle, 0x3a04, &reg_val);
        CHECK_RESULT_RET(result, "read sensor ID high");
        sensor_id = (reg_val & 0xff) << 8;

        reg_val = 0;
        result = IMX678_IsiReadRegIss(handle, 0x3a05, &reg_val);
        CHECK_RESULT_RET(result, "read sensor ID low");
        sensor_id |= (reg_val & 0xff);
    }

    *pValue = sensor_id;
    TRACE(IMX678_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX678_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t is_on) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX678_INFO, "%s (enter)\n", __func__);

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    pIMX678Ctx->Streaming = is_on;

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
    return 0; // Defaults to 0 if mode not found
}

static RESULT IMX678_GetNumExposures(IMX678_Context_t* pIMX678Ctx, size_t* num_exposures) {
    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    if (num_exposures == NULL) {
        TRACE(IMX678_ERROR, "%s: Invalid output parameter\n", __func__);
        return RET_NULL_POINTER;
    }

    if (pIMX678Ctx->SensorMode.hdr_mode == SENSOR_MODE_LINEAR) {
        *num_exposures = 1; // SDR
    } else if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        *num_exposures = IMX678_2DOL_NUM_EXP;
    } else if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        *num_exposures = IMX678_3DOL_NUM_EXP;
    } else {
        TRACE(IMX678_ERROR, "%s: Unsupported HDR mode %d with stitching mode %d\n",
            __func__, pIMX678Ctx->SensorMode.hdr_mode, pIMX678Ctx->SensorMode.stitching_mode);
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

static RESULT IMX678_UpdateCurrLEFIntegrationTimeFromFsc(IMX678_Context_t* pIMX678Ctx, uint32_t fsc, uint32_t shr0) {
    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    float configuredIntegrationTime = (fsc - shr0) * pIMX678Ctx->one_line_exp_time;

    pIMX678Ctx->OldIntegrationTime = configuredIntegrationTime;
    pIMX678Ctx->AecCurIntegrationTimeLEF = configuredIntegrationTime;

    TRACE(IMX678_DEBUG, "%s: Updated LEF Integration Time = (fsc[%u] - shr0[%u]) * one_line_exp_time[%f] = %f\n",
          __func__, fsc, shr0, pIMX678Ctx->one_line_exp_time, configuredIntegrationTime);
    return RET_SUCCESS;
}

static RESULT IMX678_UpdateCurrLEFIntegrationTimeFromVmax(IMX678_Context_t* pIMX678Ctx, uint32_t vmax, uint32_t shr0) {
    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    size_t dol = 0;
    RESULT result = IMX678_GetNumExposures(pIMX678Ctx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");
    uint32_t fsc = vmax * dol;
    TRACE(IMX678_DEBUG, "%s: fsc = vmax[%u] * dol[%zu] = %u\n", __func__, vmax, dol, fsc);
    result = IMX678_UpdateCurrLEFIntegrationTimeFromFsc(pIMX678Ctx, fsc, shr0);
    CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromFsc");
    return RET_SUCCESS;
}

static inline RESULT IMX678_getNewVmaxAntiFlicker(IMX678_Context_t *pIMX678Ctx, uint32_t requestedVmax, uint32_t *outClosestVmax) {
    uint32_t closestVmax = requestedVmax;
    int peaks = 0;
    int difference = INT_MAX;
    int minDifference = INT_MAX;
    if (!pIMX678Ctx) {
        TRACE(IMX678_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return RET_FAILURE;
    }
    peaks = IMX678_getFlickerPeaksPerSec(pIMX678Ctx->flicker_fps_mode);
    if (peaks == 0) {
        // No anti-flicker mode, return requested or original Vmax
        if (pIMX678Ctx->unlimit_fps && pIMX678Ctx->unlimit_fps_vmax_changed)
            *outClosestVmax = requestedVmax;
        else
            *outClosestVmax = pIMX678Ctx->original_vmax;
        return RET_SUCCESS;
    }

    for (int i = 1; i < peaks; ++i) {
        // VMAX = i / (peaks * one_line_exp_time) can be obtained by algebra: comparing fps = 1 / (vmax * one_line_exp_time) = peaks / i
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
    *outClosestVmax = closestVmax;
    return RET_SUCCESS;
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
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return RET_NULL_POINTER;
    }

    if (pIMX678Ctx->enableHdr)
        return result;

    result = IMX678_ReadVmax(handle, &current_vmax);
    CHECK_RESULT_RET(result, "ReadVmax");
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
        result = IMX678_getNewVmaxAntiFlicker(pIMX678Ctx, pIMX678Ctx->original_vmax, &new_vmax);
        CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
        TRACE(IMX678_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX678Ctx->flicker_fps_mode, new_vmax);
    }
    if (current_vmax != new_vmax) {
        result = IMX678_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        result = IMX678_WriteVmax(handle, new_vmax);
        CHECK_RESULT_RET(result, "WriteVmax");
        result = IMX678_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");

        int shr = MAX((int)current_vmax - (int)(pIMX678Ctx->AecCurIntegrationTimeLEF / pIMX678Ctx->one_line_exp_time), IMX678_MIN_SHR);
        result = IMX678_UpdateCurrLEFIntegrationTimeFromVmax(pIMX678Ctx, new_vmax, shr);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromVmax");
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
    float max_short_it, min_short_it;
    RESULT result = RET_SUCCESS;
    int32_t rhs1 = -1;
    HalContext_t* pHalCtx = NULL;

    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    TRACE(IMX678_INFO, "%s: (enter). prev values: min: %f, max: %f\n", __func__, pIMX678Ctx->AecMinIntegrationTime, pIMX678Ctx->AecMaxIntegrationTime);

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX678_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMX678Ctx->enableHdr) {
        *pMinIntegrationTime = pIMX678Ctx->AecMinIntegrationTime;
        *pMaxIntegrationTime = pIMX678Ctx->AecMaxIntegrationTime;

            TRACE(IMX678_INFO, "%s: (exit - SDR) %f, %f\n", 
        __func__, *pMinIntegrationTime, *pMaxIntegrationTime);

            return RET_SUCCESS;
    }

    pHalCtx = (HalContext_t*)pIMX678Ctx->IsiCtx.HalHandle;
    if (!pHalCtx) {
        TRACE(IMX678_ERROR,
              "%s: Invalid HAL handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIMX678Ctx->cur_rhs1 == 0) {
        result = IMX678_GetCtrl(pHalCtx->sensor_fd, IMX678_CID_RHS1, &rhs1);
        CHECK_RESULT_RET(result, "GetCtrl RHS1");
        pIMX678Ctx->cur_rhs1 = (uint32_t)rhs1;
    }

    if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        max_short_it = (pIMX678Ctx->cur_rhs1 - IMX678_2DOL_SHR1_MIN_GAP) * pIMX678Ctx->one_line_exp_time;
        min_short_it = IMX678_2DOL_SHR1_RHS1_GAP * pIMX678Ctx->one_line_exp_time;
    } else {
        max_short_it = (pIMX678Ctx->cur_rhs1 - IMX678_3DOL_SHR1_MIN_GAP) * pIMX678Ctx->one_line_exp_time;
        min_short_it = IMX678_3DOL_SHR1_RHS1_GAP * pIMX678Ctx->one_line_exp_time;
    }

    /*
     * Report the actual SEF1 hardware limits.  The ratio is enforced inside
     * Calculate2DOLExposures / Calculate3DOLExposures which will maximise the
     * LEF integration time and apply LEF gain when needed to meet the ratio.
     */
    pIMX678Ctx->AecMinIntegrationTime = min_short_it;
    pIMX678Ctx->AecMaxIntegrationTime = max_short_it;

    *pMinIntegrationTime = pIMX678Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMX678Ctx->AecMaxIntegrationTime;

    TRACE(IMX678_INFO, "%s: (exit - HDR). min: %f, max: %f\n",  __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

static RESULT IMX678_IsiGetAbsoluteIntegrationTimeLimitsIss(
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

    *pMinIntegrationTime = IMX678_MIN_LINES * pIMX678Ctx->one_line_exp_time;
    *pMaxIntegrationTime =  (IMX678_VMAX_MAX - IMX678_MIN_SHR) * pIMX678Ctx->one_line_exp_time;

    TRACE(IMX678_INFO, "%s: (exit) %f, %f\n", 
    __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

/* Gain get functions*/

RESULT IMX678_IsiGetGainIss(IsiSensorHandle_t handle, float *pSetGain)
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

	if (pIMX678Ctx->enableHdr) {
		result = IMX678_IsiGetSEF1GainIss(handle, pSetGain);
		CHECK_RESULT_RET(result, "GetSEF1Gain");
	} else {
		result = IMX678_IsiGetLEFGainIss(handle, pSetGain);
		CHECK_RESULT_RET(result, "GetLEFGain");
	}

	return result;
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
		CHECK_RESULT_RET(result, "SetSEF1Gain");

		result = IMX678_IsiSetSEF2GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF2Gain");
	}

	result = IMX678_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
	CHECK_RESULT_RET(result, "SetLEFGain");
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

	result = IMX678_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX678_WriteGain(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain");
	result = IMX678_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

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

	result = IMX678_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX678_WriteGain1(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain1");
	result = IMX678_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

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

	result = IMX678_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX678_WriteGain2(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain2");
	result = IMX678_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

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
	RESULT result = RET_SUCCESS;

	if (!pIMX678Ctx) {
		TRACE(IMX678_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime) {
		TRACE(IMX678_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
		      __func__);
		return (RET_NULL_POINTER);
	}
	
	TRACE(IMX678_DEBUG, "%s - enter\n", __func__);
	
	if (pIMX678Ctx->enableHdr) {
		result = IMX678_IsiGetSEF1IntegrationTimeIss(handle, pSetIntegrationTime);
		CHECK_RESULT_RET(result, "GetSEF1IntegrationTime");
	} else {
		result = IMX678_IsiGetLEFIntegrationTimeIss(handle, pSetIntegrationTime);
		CHECK_RESULT_RET(result, "GetLEFIntegrationTime");
	}
	
	return result;
}

RESULT IMX678_IsiGetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX678_Context_t *pIMX678Ctx = (IMX678_Context_t *)handle;
	RESULT result = RET_SUCCESS;


	if (!pIMX678Ctx) {
		TRACE(IMX678_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
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
		TRACE(IMX678_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime) {
		TRACE(IMX678_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
		      __func__);
		return (RET_NULL_POINTER);
	}
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
		TRACE(IMX678_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime) {
		TRACE(IMX678_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
				__func__);
		return (RET_NULL_POINTER);
	}

	*pSetIntegrationTime = pIMX678Ctx->AecCurIntegrationTimeSEF2;
	TRACE(IMX678_DEBUG, "%s - returning %f\n", __func__, pIMX678Ctx->AecCurIntegrationTimeSEF2);
	return (result);
}

RESULT IMX678_IsiGetIntegrationTimeIncrementIss(IsiSensorHandle_t handle,
                                                float* pIncr) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (!pIMX678Ctx) {
        TRACE(IMX678_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pIncr) {
        TRACE(IMX678_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }
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
	RESULT result = RET_SUCCESS;

	if (!pIMX678Ctx) {
		TRACE(IMX678_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX678Ctx->enableHdr) {
		result = IMX678_IsiSetSEF1IntegrationTimeIss(
			handle, NewIntegrationTime, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1IntegrationTime");
	} else {
		result = IMX678_IsiSetLEFIntegrationTimeIss(
			handle, NewIntegrationTime, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
	}

	return result;
}

RESULT IMX678_IsiSetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;

    int shr0 = 0;
    uint32_t exp = 0;
    uint32_t new_vmax = 0;
    uint32_t vmax_updated = 0;
    uint32_t current_vmax = 0;
    uint32_t rhs1;
    uint32_t rhs2;
    if (!pIMX678Ctx) {
        TRACE(IMX678_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pSetIntegrationTime || !pNumberOfFramesToSkip) {
        TRACE(IMX678_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMX678Ctx->Streaming) {
        TRACE(IMX678_ERROR, "%s: sensor not streaming\n", __func__);
        return RET_FAILURE;
    }
    exp = roundf(NewIntegrationTime / pIMX678Ctx->one_line_exp_time);

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
                result = IMX678_ReadVmax(pIMX678Ctx, &new_vmax);
                if(result != RET_SUCCESS){
                    TRACE(IMX678_ERROR, "%s: unable to read vmax (result=%d)\n", __func__, result);
                    new_vmax = IMX678_VMAX_2DOL_HDR;
                }else{
                    new_vmax *= IMX678_2DOL_NUM_EXP;
                }

                shr0 = new_vmax - exp;
                shr0 = shr0 > rhs1 + IMX678_2DOL_SHR0_RHS1_GAP ? shr0 : rhs1 + IMX678_2DOL_SHR0_RHS1_GAP;
                shr0 = shr0 < new_vmax - IMX678_2DOL_SHR0_FSC_GAP? shr0 : new_vmax - IMX678_2DOL_SHR0_FSC_GAP;
            } else {
                new_vmax = IMX678_VMAX_3DOL_HDR;
                shr0 = new_vmax - exp;
                shr0 = shr0 > rhs2 + IMX678_3DOL_SHR0_RHS2_GAP ? shr0 : rhs2 + IMX678_3DOL_SHR0_RHS2_GAP;
                shr0 = shr0 < new_vmax - IMX678_3DOL_SHR0_FSC_GAP? shr0 : new_vmax - IMX678_3DOL_SHR0_FSC_GAP;
            }
            
		} else {
            if (exp > pIMX678Ctx->MaxIntegrationLine || exp == 0) {
                TRACE(IMX678_ERROR, "%s: Integration time %f (exp %u) out of range (%u)\n", __func__,
                    NewIntegrationTime, exp, pIMX678Ctx->MaxIntegrationLine);
                return RET_FAILURE;
            }
            result = IMX678_ReadVmax(handle, &current_vmax);
            CHECK_RESULT_RET(result, "ReadVmax");

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

            shr0 = current_vmax - exp;

            if (shr0 < IMX678_MIN_SHR || current_vmax > pIMX678Ctx->original_vmax) {
                if (shr0 < IMX678_MIN_SHR) {
                    // user asked for too big integration-line: more than (current_vmax - minimal_shutter)
                    // increase vmax and set new shutter to minimal value
                    new_vmax = MIN(exp + IMX678_MIN_SHR, pIMX678Ctx->MaxIntegrationLine + IMX678_MIN_SHR);
                } else {
                    // make sure that when we unlimit fps, stay below original fps.
                    // also, make sure we are not using high vmax + high shr more than needed (will slow fps for no reason)
                    new_vmax = MAX(current_vmax - shr0 + IMX678_MIN_SHR, pIMX678Ctx->original_vmax);
                }
                pIMX678Ctx->unlimit_fps_vmax_changed = new_vmax > pIMX678Ctx->original_vmax && pIMX678Ctx->unlimit_fps;
        
                if (pIMX678Ctx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
                    result = IMX678_getNewVmaxAntiFlicker(pIMX678Ctx, new_vmax, &new_vmax);
                    CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
                    TRACE(IMX678_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX678Ctx->flicker_fps_mode, new_vmax);
                }

                shr0 = MAX(IMX678_MIN_SHR, (int)new_vmax - (int)exp);
                vmax_updated = 1;

            } else {
                new_vmax = current_vmax;
            }
        }

        result = IMX678_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        if (vmax_updated && pIMX678Ctx->unlimit_fps && !pIMX678Ctx->enableHdr) {
            result = IMX678_WriteVmax(handle, new_vmax);
            CHECK_RESULT_RET(result, "WriteVmax");
        }

        TRACE(IMX678_DEBUG, "%s - writing 0x%x to SHR0\n", __func__, shr0);
        result = IMX678_WriteShr0(handle, shr0);
        CHECK_RESULT_RET(result, "WriteShr0");
        result = IMX678_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");

        // In this context, the "new_vmax" is actually the FSC (multiplied by DOL), not the VMAX.
        result = IMX678_UpdateCurrLEFIntegrationTimeFromFsc(pIMX678Ctx, new_vmax, shr0);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromFsc");

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
		TRACE(IMX678_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX678Ctx->cur_rhs1 == 0 || pIMX678Ctx->cur_rhs2 == 0) {
		TRACE(IMX678_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX678Ctx->cur_rhs1;

	if (!pSetIntegrationTimeSEF1 || !pNumberOfFramesToSkip) {
		TRACE(IMX678_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX678_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = roundf(NewIntegrationTime / pIMX678Ctx->one_line_exp_time);

	if (fabs(NewIntegrationTime - pIMX678Ctx->AecCurIntegrationTimeSEF1) > FLT_EPSILON) {
		if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
			exp = rhs1 - exp;
			exp = exp > IMX678_2DOL_SHR1_RHS1_GAP ? exp : IMX678_2DOL_SHR1_RHS1_GAP;
			exp = exp < rhs1 - IMX678_2DOL_SHR1_RHS1_GAP ? exp : rhs1 - IMX678_2DOL_SHR1_RHS1_GAP;
		} else {
			exp = rhs1 - exp;
			exp = exp > IMX678_3DOL_SHR1_RHS1_GAP ? exp : IMX678_3DOL_SHR1_RHS1_GAP;
			exp = exp < rhs1 - IMX678_3DOL_SHR1_RHS1_GAP ? exp : rhs1 - IMX678_3DOL_SHR1_RHS1_GAP;
		}

		TRACE(IMX678_DEBUG, "%s - writing 0x%x to SHR1\n", __func__, exp);

		result = IMX678_LockRegHold(handle);
		CHECK_RESULT_RET(result, "LockRegHold");
		result = IMX678_WriteShr1(handle, exp);
		CHECK_RESULT_RET(result, "WriteShr1");
		result = IMX678_UnlockRegHold(handle);
		CHECK_RESULT_RET(result, "UnlockRegHold");

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

	if (!pIMX678Ctx) {
		TRACE(IMX678_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX678Ctx->cur_rhs1 == 0 || pIMX678Ctx->cur_rhs2 == 0) {
		TRACE(IMX678_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX678Ctx->cur_rhs1;
	rhs2 = pIMX678Ctx->cur_rhs2;

	if (!pSetIntegrationTimeSEF2 || !pNumberOfFramesToSkip) {
		TRACE(IMX678_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX678_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = roundf(NewIntegrationTime / pIMX678Ctx->one_line_exp_time);
	TRACE(IMX678_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX678Ctx->AecCurIntegrationTimeSEF2) > FLT_EPSILON) {
		exp = rhs2 - exp;
		exp = exp > rhs1 + IMX678_SHR2_RHS1_GAP ? exp : rhs1 + IMX678_SHR2_RHS1_GAP;
		exp = exp < rhs2 - IMX678_SHR2_RHS2_GAP ? exp : rhs2 - IMX678_SHR2_RHS2_GAP;
		TRACE(IMX678_DEBUG, "%s - writing 0x%x to SHR2\n", __func__, exp);

		result = IMX678_LockRegHold(handle);
		CHECK_RESULT_RET(result, "LockRegHold");
		result = IMX678_WriteShr2(handle, exp);
		CHECK_RESULT_RET(result, "WriteShr2");
		result = IMX678_UnlockRegHold(handle);
		CHECK_RESULT_RET(result, "UnlockRegHold");

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
	float very_short_it, very_short_exp_val;
	bool recalc_vs_gain = false;
	float very_short_gain = 1;
	uint32_t rhs1;
	uint32_t rhs2;

    if (pIMX678Ctx == NULL || o_long_it == NULL || o_short_it == NULL ||
        o_very_short_it == NULL || o_long_gain == NULL || o_short_gain == NULL ||
        o_very_short_gain == NULL || hdr_ratio == NULL) {
        TRACE(IMX678_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (NewIntegrationTime == 0 || NewGain == 0) {
        TRACE(IMX678_ERROR, "%s: Invalid parameter (NewIntegrationTime or NewGain is 0)\n", __func__);
        return (RET_WRONG_CONFIG);
    }

	if (pIMX678Ctx->cur_rhs1 == 0 || pIMX678Ctx->cur_rhs2 == 0) {
		TRACE(IMX678_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX678Ctx->cur_rhs1;
	rhs2 = pIMX678Ctx->cur_rhs2;

    if(NewIntegrationTime < IMX678_SHR2_RHS2_GAP * pIMX678Ctx->one_line_exp_time * hdr_ratio[1]){
	    pIMX678Ctx->MinIntegrationLine =  IMX678_SHR2_RHS2_GAP * hdr_ratio[1];
	    pIMX678Ctx->AecMinIntegrationTime = pIMX678Ctx->MinIntegrationLine * pIMX678Ctx->one_line_exp_time;
    }

    very_short_it = NewIntegrationTime / hdr_ratio[1];
    very_short_exp_val = very_short_it / pIMX678Ctx->one_line_exp_time;
    very_short_gain = _sensorGain2linear(_linear2sensorGain(NewGain));

    if (very_short_exp_val < rhs1 + IMX678_SHR2_RHS1_GAP) {
        very_short_exp_val = rhs1 + IMX678_SHR2_RHS1_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX678Ctx->one_line_exp_time;
        recalc_vs_gain = true;
        TRACE(IMX678_DEBUG, "%s: very_short_exp_val is too long, set to %u, new very_short_it = %f\n",
        __func__, rhs2 + IMX678_SHR2_RHS1_GAP, very_short_it);
    } else if(very_short_exp_val > rhs2 - IMX678_SHR2_RHS2_GAP) {
        very_short_exp_val = rhs2 - IMX678_SHR2_RHS2_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX678Ctx->one_line_exp_time;
        recalc_vs_gain = true;
        TRACE(IMX678_DEBUG, "%s: very_short_exp_val is too short, set to %u, new very_short_it = %f\n",
        __func__, rhs2 - IMX678_SHR2_RHS2_GAP, very_short_it);
    }
    
    *o_long_it = NewIntegrationTime * hdr_ratio[0];
    *o_long_gain = _sensorGain2linear(_linear2sensorGain(NewGain));
    *o_short_it = NewIntegrationTime;
    *o_short_gain = _sensorGain2linear(_linear2sensorGain(NewGain));

    if (recalc_vs_gain) {
        very_short_gain = (NewIntegrationTime * (*o_short_gain)) / (very_short_it * hdr_ratio[1]);
        very_short_gain = _sensorGain2linear(_linear2sensorGainCeil(very_short_gain));
        very_short_it = (NewIntegrationTime * (*o_short_gain)) / (very_short_gain * hdr_ratio[1]);
    }

    *o_very_short_it = very_short_it;
    *o_very_short_gain = very_short_gain;

    return RET_SUCCESS;
}

/*
 * Calculate 2DOL long/short exposure values.
 *
 * NewIntegrationTime and NewGain are the AE-requested short exposure values.
 * The long exposure is derived to satisfy hdr_ratio[0]:
 *     (long_it * long_gain) / (short_it * short_gain) == hdr_ratio[0]
 *
 * Policy: maximize the long integration time before resorting to gain.
 * This keeps SNR high on the long frame and avoids amplifying sensor noise.
 * Gain is only applied when the required exposure exceeds the hardware
 * maximum integration time (FSC - RHS1 - SHR0_RHS1_GAP lines).
 *
 * All outputs are quantized to sensor-realizable values (line-quantized IT,
 * 0.3 dB-step quantized gain).
 */
RESULT IMX678_Calculate2DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it,
                                    float *o_long_gain, float *o_short_gain,
                                    float *hdr_ratio) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
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

    TRACE(IMX678_DEBUG, "%s: enter with NewIntegrationTime=%.6f NewGain=%.4f ratio=%.1f\n",
        __func__, NewIntegrationTime, NewGain, hdr_ratio[0]);

    result = IMX678_ReadVmax(handle, &vmax);
    CHECK_RESULT_RET(result, "ReadVmax");

    result = IMX678_GetNumExposures(pIMX678Ctx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");

    fsc = vmax * (uint32_t)dol;
    rhs1 = pIMX678Ctx->cur_rhs1;
    one_line = pIMX678Ctx->one_line_exp_time;

    if (one_line <= 0.0f) {
        TRACE(IMX678_ERROR, "%s: Invalid one_line_exp_time (%.9f)\n", __func__, one_line);
        return (RET_WRONG_CONFIG);
    }

    if (fsc <= rhs1 + IMX678_2DOL_SHR0_RHS1_GAP) {
        TRACE(IMX678_ERROR, "%s: fsc(%u) <= rhs1(%u) + gap, invalid config\n",
              __func__, fsc, rhs1);
        return (RET_WRONG_CONFIG);
    }

    /* Hardware limits: SHR0 must satisfy  rhs1 + gap <= SHR0 <= fsc - gap. */
    max_long_it_lines = fsc - rhs1 - IMX678_2DOL_SHR0_RHS1_GAP;
    min_long_it_lines = IMX678_2DOL_SHR0_FSC_GAP;

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
     * Step 1b: When the required EV is within IMX678_2DOL_MAX_IT_NEAR_LINES of
     * max_long_it, the shortfall is a float-rounding artifact — accept unity
     * gain and the negligible ratio deviation instead of bumping a gain step.
     */
    if (long_gain_needed > 1.0f && long_it_lines == max_long_it_lines) {
        required_lines = required_long_ev / one_line;
        if (required_lines - (float)max_long_it_lines < (float)IMX678_2DOL_MAX_IT_NEAR_LINES) {
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

        max_gain_db = _linear2sensorGain(IMX678_MAX_GAIN_AEC);
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

    TRACE(IMX678_DEBUG, "%s: ratio=%.1f vmax=%u fsc=%u rhs1=%u max_long_it_lines=%u\n",
        __func__, hdr_ratio[0], vmax, fsc, rhs1, max_long_it_lines);
    TRACE(IMX678_DEBUG, "%s: required_long_ev=%.6f long_it=%.6f(%u lines) long_gain=%.4f\n",
        __func__, required_long_ev, long_it, long_it_lines, long_gain);
    TRACE(IMX678_DEBUG, "%s: short_it=%.6f short_gain=%.4f\n",
        __func__, *o_short_it, *o_short_gain);

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
        TRACE(IMX678_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pNumberOfFramesToSkip == NULL) || (pSetGain == NULL) ||
        (pSetIntegrationTime == NULL)) {
        TRACE(IMX678_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    // Regardless of 2dol/3dol - we should get the up-to-date hdr_ratio (if 2dol, hdr_ratio[1] = 1)
    pIMX678Ctx->hdr_ratio[0] = hdr_ratio[0];
    pIMX678Ctx->hdr_ratio[1] = hdr_ratio[1];

    if (pIMX678Ctx->enableHdr) {
        result = IMX678_ReadRHS1(handle, &pIMX678Ctx->cur_rhs1);
        CHECK_RESULT_RET(result, "ReadRHS1");
        result = IMX678_ReadRHS2(handle, &pIMX678Ctx->cur_rhs2);
        CHECK_RESULT_RET(result, "ReadRHS2");
        result = IMX678_ReadHmax(handle, &hmax);
        CHECK_RESULT_RET(result, "ReadHmax");

        pIMX678Ctx->SensorMode.ae_info.one_line_exp_time_ns = (uint32_t)(((float)hmax / IMX678_PIXEL_CLK_RATE) * MICRO_2_NANO);
        pIMX678Ctx->one_line_exp_time =
        (float)(pIMX678Ctx->SensorMode.ae_info.one_line_exp_time_ns) / 1000000000;

        if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
            //2exposure
            result = IMX678_Calculate2DOLExposures(handle, NewIntegrationTime, NewGain,
                                    &long_it, &short_it,
                                    &long_gain, &short_gain,
                                    hdr_ratio);
            CHECK_RESULT_RET(result, "Calculate2DOLExposures");
        } else {
            //3exposure
            result = IMX678_Calculate3DOLExposures(handle, NewIntegrationTime, NewGain,
                                        &long_it, &short_it, &very_short_it,
                                        &long_gain, &short_gain, &very_short_gain,
                                        hdr_ratio);
            CHECK_RESULT_RET(result, "Calculate3DOLExposures");
        }

		result = IMX678_IsiSetLEFIntegrationTimeIss(handle, long_it,
							pSetIntegrationTime,
							pNumberOfFramesToSkip,
							hdr_ratio);
		CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
		result = IMX678_IsiSetLEFGainIss(handle, long_gain, pSetGain, hdr_ratio);
		CHECK_RESULT_RET(result, "SetLEFGain");
		result = IMX678_IsiSetSEF1IntegrationTimeIss(
			handle, short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1IntegrationTime");
		result = IMX678_IsiSetSEF1GainIss(handle, NewIntegrationTime,
						  short_gain, pSetGain, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1Gain");
        if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            result = IMX678_IsiSetSEF2IntegrationTimeIss(
			    handle, very_short_it, pSetIntegrationTime,
			    pNumberOfFramesToSkip, hdr_ratio);
			CHECK_RESULT_RET(result, "SetSEF2IntegrationTime");
            result = IMX678_IsiSetSEF2GainIss(handle, NewIntegrationTime,
                            very_short_gain, pSetGain, hdr_ratio);
			CHECK_RESULT_RET(result, "SetSEF2Gain");
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
        result = IMX678_IsiSetLEFIntegrationTimeIss(handle, NewIntegrationTime,
                                                pSetIntegrationTime,
                                                pNumberOfFramesToSkip, hdr_ratio);
        CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
        result = IMX678_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
        CHECK_RESULT_RET(result, "SetLEFGain");
    }
    return result;
}

RESULT IMX678_IsiExposureControlExpandedIss(
    IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime,
    float NewIris, uint8_t* pNumberOfFramesToSkip, float* pSetGain,
    float* pSetIntegrationTime, float* pSetIris, float* hdr_ratio) {
    RESULT result = RET_SUCCESS;

    if (pSetIris) {
        result = IMX678_IsiSetIrisIss(handle, NewIris);
        CHECK_RESULT_RET(result, "SetIris");
        *pSetIris = NewIris;
    }

    result = IMX678_IsiExposureControlIss(handle, NewGain, NewIntegrationTime,
                                        pNumberOfFramesToSkip, pSetGain,
                                        pSetIntegrationTime, hdr_ratio);
    CHECK_RESULT_RET(result, "ExposureControl");
    return (result);
}

RESULT IMX678_IsiGetCurrentExposureIss(IsiSensorHandle_t handle,
                                       float* pSetGain,
                                       float* pSetIntegrationTime) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pSetGain == NULL) || (pSetIntegrationTime == NULL)) {
        TRACE(IMX678_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

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
        TRACE(IMX678_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
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
    uint32_t requested_fsc = 0;
    uint32_t shr = 0;
    size_t dol = 0;
    uint32_t fsc = 0;
    uint32_t min_shr0 = (dol == 1) ? IMX678_MIN_SHR : IMX678_2DOL_SHR0_RHS1_GAP + pIMX678Ctx->cur_rhs1;
    int exp = 0;

    TRACE(IMX678_DEBUG, "%s: set sensor flickerMode = %d\n", __func__, flickerMode);

    if (!pIMX678Ctx) {
        return RET_NULL_POINTER;
    }
    if (pIMX678Ctx->flicker_fps_mode == flickerMode) {
        return RET_SUCCESS;
    }
    if (pIMX678Ctx->enableHdr && (pIMX678Ctx->SensorMode.stitching_mode != SENSOR_STITCHING_L_AND_S)) {
        return RET_SUCCESS;
    }

    result = IMX678_GetNumExposures(pIMX678Ctx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");

    if (flickerMode > ISI_AE_ANTIBANDING_MODE_AUTO) {
        TRACE(IMX678_INFO, "%s: Invalid flickerMode (%d), setting ISI_AE_ANTIBANDING_MODE_AUTO instead.\n", __func__, flickerMode);
        flickerMode = ISI_AE_ANTIBANDING_MODE_AUTO;
    }
    pIMX678Ctx->flicker_fps_mode = flickerMode;

    result = IMX678_ReadVmax(handle, &current_vmax);
    CHECK_RESULT_RET(result, "ReadVmax");
    if (pIMX678Ctx->original_vmax == 0) {
        pIMX678Ctx->original_vmax = current_vmax;
    }

    fsc = current_vmax * dol;

    exp = pIMX678Ctx->AecCurIntegrationTimeLEF / pIMX678Ctx->one_line_exp_time;
    shr = MAX((int)(fsc) - exp, min_shr0);

    if (current_vmax > pIMX678Ctx->original_vmax) {
        current_vmax = MAX((int)fsc - (int)shr + min_shr0, min_shr0);
        fsc = current_vmax * dol;
        shr = MAX((int)fsc - exp, min_shr0);
        pIMX678Ctx->unlimit_fps_vmax_changed = current_vmax > pIMX678Ctx->original_vmax && pIMX678Ctx->unlimit_fps;
    }

    result = IMX678_getNewVmaxAntiFlicker(pIMX678Ctx, current_vmax, &requested_vmax);
    CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
    requested_vmax = MAX( MIN(requested_vmax, IMX678_VMAX_MAX), 1);
    requested_fsc = requested_vmax * dol;
    
    if (current_vmax != requested_vmax) {
        shr = MAX( (int)requested_fsc - (int)fsc + (int)shr, min_shr0);
        TRACE(IMX678_DEBUG, "%s - writing 0x%x to VMAX, writing 0x%x to SHR0\n", __func__, requested_vmax, shr);
        
        result = IMX678_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        result = IMX678_WriteVmax(handle, requested_vmax);
        CHECK_RESULT_RET(result, "WriteVmax");
        result = IMX678_WriteShr0(handle, shr);
        CHECK_RESULT_RET(result, "WriteShr0");
        result = IMX678_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");
        result = IMX678_UpdateCurrLEFIntegrationTimeFromVmax(handle, requested_vmax, shr);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromVmax");
    }
    
    // these 2 are being used only in SDR
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
        CHECK_RESULT_RET(result, "disable test pattern");
    } else {
        result = IMX678_IsiWriteRegIss(handle, 0x3253, 0x80);
        CHECK_RESULT_RET(result, "enable test pattern");
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

    result = IMX678_IsiReadRegIss(handle, 0x5081, &value);
    CHECK_RESULT_RET(result, "read TPG register");

    Tpg->enable = ((value & 0x80) != 0) ? 1 : 0;
    if (Tpg->enable) {
        Tpg->pattern = (0xff & value);
    }
    pIMX678Ctx->TestPattern = Tpg->enable;

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
        TRACE(IMX678_ERROR, "%s: Invalid sensor again (%f)\n", __func__, Gain.again);
        return (RET_OUTOFRANGE);
    }
    // Again = (uint32_t)(1024 - (1024/Gain.again));
    Again = (uint32_t)(((2048 * Gain.again) - 2048) / Gain.again);

    result = IMX678_IsiWriteRegIss(handle, 0x309c, (Again & 0x0000FF));
    CHECK_RESULT_RET(result, "write again low");
    result = IMX678_IsiWriteRegIss(handle, 0x309d, (Again & 0x00FF00) >> 8);
    CHECK_RESULT_RET(result, "write again high");

    if ((Gain.dgain < 1) | (Gain.dgain > 16)) {
        TRACE(IMX678_ERROR, "%s: Invalid sensor dgain (%f)\n", __func__, Gain.dgain);
        return (RET_OUTOFRANGE);
    }
    Dgain = Gain.dgain * 256;

    result = IMX678_IsiWriteRegIss(handle, 0x308c, (Dgain & 0x0000FF));
    CHECK_RESULT_RET(result, "write dgain low");
    result = IMX678_IsiWriteRegIss(handle, 0x308d, (Dgain & 0x00FF00) >> 8);
    CHECK_RESULT_RET(result, "write dgain high");

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
        return RET_OUTOFRANGE;
    }

    //TODO implement setting iris here!

    TRACE(IMX678_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX678_IsiGetHCGIss( IsiSensorHandle_t handle,
                                     bool *phcg ) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX678_INFO, "%s: (enter)\n", __func__);

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *phcg = pIMX678Ctx->hcg;

    TRACE(IMX678_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX678_IsiSetHCGIss(IsiSensorHandle_t handle, bool hcg) {
    
    RESULT result = RET_SUCCESS;

    TRACE(IMX678_INFO, "%s: (enter)\n", __func__);

    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;

    if (pIMX678Ctx == NULL) {
        TRACE(IMX678_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    result = IMX678_IsiWriteRegIss(handle, 0x3030 , hcg);
    CHECK_RESULT_RET(result, "write HCG");
    pIMX678Ctx->hcg = hcg;

    if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S ||
        pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        result = IMX678_IsiWriteRegIss(handle, 0x3031 , hcg);
        CHECK_RESULT_RET(result, "write HCG SEF1");
    }
    if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        result = IMX678_IsiWriteRegIss(handle, 0x3032 , hcg);
        CHECK_RESULT_RET(result, "write HCG SEF2");
    }

    TRACE(IMX678_INFO, "%s: (exit)\n", __func__);
    return result;
}

static RESULT IMX678_CalculateHdrBlankingLines(IsiSensorHandle_t handle,
        uint32_t *pBlankingLines, uint32_t rhs1, uint32_t rhs2) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;

    /* The following formulas are taken directly from Sony's IMX678 datasheet.
     * They are located at the "AppNote_DOL_E" document,
     * at "Operating Mode" -> "Number of Blanking Lines" section ("Virtual Channel Mode"). */

    /* The formula is different from AllPixel/Binning configurations,
     * so we will assume that FHD resolution will be binning */
    bool allPixelMode = (pIMX678Ctx->SensorMode.size.width == IMX678_RES_4K_WIDTH &&
                         pIMX678Ctx->SensorMode.size.height == IMX678_RES_4K_HEIGHT);

    if (!allPixelMode) {
        // Make sure the resolution matches a 2/2 binning mode
        if (pIMX678Ctx->SensorMode.size.width != IMX678_RES_FHD_WIDTH ||
            pIMX678Ctx->SensorMode.size.height != IMX678_RES_FHD_HEIGHT) {
            TRACE(IMX678_ERROR, "%s: Unsupported resolution %dx%d for HDR\n",
                  __func__, pIMX678Ctx->SensorMode.size.width,
                  pIMX678Ctx->SensorMode.size.height);
            return RET_NOTSUPP;
        }
    }

    // 2DOL
    if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        if (allPixelMode) {
            pBlankingLines[0] = (rhs1 - 1) / 2;
        } else { // 2/2 Binning
            pBlankingLines[0] = (rhs1 - 1) / 4;
        }
        pBlankingLines[1] = 0; // Irrelevant in 2DOL
    // 3DOL
    } else if (pIMX678Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        if (allPixelMode) {
            pBlankingLines[0] = (rhs1 - 1) / 3;
            pBlankingLines[1] = (rhs2 - 2) / 3;
        } else { // 2/2 Binning
            pBlankingLines[0] = (rhs1 - 1) / 6;
            pBlankingLines[1] = (rhs2 - 2) / 6;
        }
    } else {
        TRACE(IMX678_ERROR, "%s: Unsupported stitching mode %d\n",
              __func__, pIMX678Ctx->SensorMode.stitching_mode);
        return RET_NOTSUPP;
    }

    return RET_SUCCESS;
}

static RESULT IMX678_IsiGetHdrBlankingLinesIss(IsiSensorHandle_t handle,
        uint32_t *pBlankingLines, size_t elementCount) {
    IMX678_Context_t* pIMX678Ctx = (IMX678_Context_t*)handle;
    HalContext_t* pHalCtx = (HalContext_t*)pIMX678Ctx->IsiCtx.HalHandle;
    RESULT result = RET_SUCCESS;
    int32_t rhs1 = 0, rhs2 = 0;

    if (elementCount != 2) {
        TRACE(IMX678_ERROR, "%s: Invalid element count %zu, expected 2\n",
              __func__, elementCount);
        return RET_OUTOFRANGE;
    }

    result = IMX678_GetCtrl(pHalCtx->sensor_fd, IMX678_CID_RHS1, &rhs1);
    CHECK_RESULT_RET(result, "GetCtrl RHS1");
    result = IMX678_GetCtrl(pHalCtx->sensor_fd, IMX678_CID_RHS2, &rhs2);
    CHECK_RESULT_RET(result, "GetCtrl RHS2");

    result = IMX678_CalculateHdrBlankingLines(handle, pBlankingLines, rhs1, rhs2);
    CHECK_RESULT_RET(result, "CalculateHdrBlankingLines");
    return result;
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
		pIsiSensor->pIsiGetAbsoluteIntegrationTimeLimitsIss =		IMX678_IsiGetAbsoluteIntegrationTimeLimitsIss;

		pIsiSensor->pIsiExposureControlIss =				IMX678_IsiExposureControlIss;
		pIsiSensor->pIsiExposureControlExpandedIss =		IMX678_IsiExposureControlExpandedIss;
		pIsiSensor->pIsiSetIntegrationTimeIss =				IMX678_IsiSetIntegrationTimeIss;

		pIsiSensor->pIsiGetLongIntegrationTimeIss =			IMX678_IsiGetLEFIntegrationTimeIss;
		pIsiSensor->pIsiGetIntegrationTimeIss =				IMX678_IsiGetIntegrationTimeIss;
		pIsiSensor->pIsiGetShortIntegrationTimeIss =	    IMX678_IsiGetSEF1IntegrationTimeIss;
		pIsiSensor->pIsiGetVSIntegrationTimeIss =			IMX678_IsiGetSEF2IntegrationTimeIss;

		pIsiSensor->pIsiGetLongGainIss = 					IMX678_IsiGetLEFGainIss;
		pIsiSensor->pIsiGetGainIss = 						IMX678_IsiGetGainIss;
		pIsiSensor->pIsiGetShortGainIss = 					IMX678_IsiGetSEF1GainIss;
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
        pIsiSensor->pIsiGetHCGIss =                         IMX678_IsiGetHCGIss;
        pIsiSensor->pIsiSetHCGIss =                         IMX678_IsiSetHCGIss;

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
        pIsiSensor->pIsiGetHdrBlankingLinesIss = IMX678_IsiGetHdrBlankingLinesIss;

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
