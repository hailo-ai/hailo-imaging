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
#include <errno.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "IMX662_priv.h"
#include "vvsensor.h"
#include <linux/i2c.h>

CREATE_TRACER(IMX662_INFO, "IMX662: ", INFO, 1)
CREATE_TRACER(IMX662_WARN, "IMX662: ", WARNING, 1)
CREATE_TRACER(IMX662_ERROR, "IMX662: ", ERROR, 1)
CREATE_TRACER(IMX662_DEBUG, "IMX662: ", INFO, 1)
CREATE_TRACER(IMX662_REG_INFO, "IMX662: ", INFO, 1)
CREATE_TRACER(IMX662_REG_DEBUG, "IMX662: ", INFO, 1)

#include <fcntl.h>
#define LOG_ERROR(...) TRACE(IMX662_ERROR, __VA_ARGS__)

#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

/* I2C */
#define IMX662_I2C_ADDR 0x10
#define IMX662_TRANSFER_BUFFER_LENGTH 3
/* Sensor returns EREMOTEIO while in runtime_suspend; retry briefly
 * until power-on completes (~20ms). */
#define IMX662_I2C_READ_MAX_ATTEMPTS 3
#define IMX662_I2C_READ_RETRY_INTERVAL_US 10000

/* AE */
#define IMX662_MIN_SHR 4 /* Minimum SHR0 length */
#define IMX662_IRIS_MIN_VAL 1
#define IMX662_IRIS_MAX_VAL 1
#define IMX662_EXP_MIN_LINES 1
#define IMX662_MAX_GAIN 3981
#define IMX662_MAX_GAIN_AEC                                                    \
    (32.0f) /**< max. gain used by the AEC (arbitrarily chosen, hardware limit \
               = 62.0, driver limit = 32.0 ) */
#define IMX662_MIN_GAIN_STEP                                          \
    (0.035) /**< min gain step size used by GUI (hardware min = 1/16; \
               1/16..32/16 depending on actual gain ) */

#define IMX662_RES_FHD_HEIGHT 1080
#define IMX662_RES_FHD_WIDTH 1920

/* HDR constants */
#define IMX662_2DOL_NUM_EXP 2
#define IMX662_3DOL_NUM_EXP 3
/* When required long EV is within this many lines of max_long_it, use unity gain to avoid quantization noise. */
#define IMX662_2DOL_MAX_IT_NEAR_LINES 2
#define IMX662_VMAX_MAX ((1 << 20) - 2) // max even value of unsigned 20 bits
#define IMX662_HDR_VMAX_30FPS 0x804 // 2052
#define IMX662_2DOL_HMAX 0x3DE // 990
#define IMX662_SDR_FSC  IMX662_SDR_VMAX_30FPS // Frame Set Count
#define IMX662_2DOL_FSC (IMX662_HDR_VMAX_30FPS * IMX662_2DOL_NUM_EXP)
#define IMX662_3DOL_FSC (IMX662_HDR_VMAX_30FPS * IMX662_3DOL_NUM_EXP)

#define IMX662_2DOL_SMALL_GAP 2
#define IMX662_2DOL_LARGE_GAP 5
#define IMX662_3DOL_SMALL_GAP 3
#define IMX662_3DOL_LARGE_GAP 7

#define IMX662_2DOL_SHR0_RHS1_GAP   IMX662_2DOL_LARGE_GAP
#define IMX662_2DOL_SHR0_FSC_GAP    IMX662_2DOL_SMALL_GAP
#define IMX662_2DOL_SHR1_MIN_GAP    IMX662_2DOL_LARGE_GAP
#define IMX662_2DOL_SHR1_RHS1_GAP   IMX662_2DOL_SMALL_GAP

#define IMX662_3DOL_SHR0_RHS2_GAP   IMX662_3DOL_LARGE_GAP
#define IMX662_3DOL_SHR0_FSC_GAP    IMX662_3DOL_SMALL_GAP
#define IMX662_3DOL_SHR1_MIN_GAP    IMX662_3DOL_LARGE_GAP
#define IMX662_3DOL_SHR1_RHS1_GAP   IMX662_3DOL_SMALL_GAP
#define IMX662_3DOL_SHR2_RHS1_GAP   IMX662_3DOL_LARGE_GAP
#define IMX662_3DOL_SHR2_RHS2_GAP   IMX662_3DOL_SMALL_GAP

#define IMX662_SDR_VMAX_30FPS 0x4E2 // 1250 (default VMAX for IMX662)
#define IMX662_SDR_HMAX 0x7BC // 1980
#define IMX662_PLL_PCLK 74250000 // 74.25 MHz

#define HMAX_TO_ONE_LINE_EXP_NS(hmax)       (((unsigned long long)(hmax) * 1000000000ULL) / IMX662_PLL_PCLK)
#define IMX662_SDR_ONE_LINE_EXP_TIME_NS     HMAX_TO_ONE_LINE_EXP_NS(IMX662_SDR_HMAX)
#define IMX662_2DOL_ONE_LINE_EXP_TIME_NS    HMAX_TO_ONE_LINE_EXP_NS(IMX662_2DOL_HMAX)

//TODO: Check flickr
FlickerModePeaksPerSec flickerPeaksPerSecMap[] = {
    { ISI_AE_ANTIBANDING_MODE_OFF, 0 },
    { ISI_AE_ANTIBANDING_MODE_50HZ, 100 },
    { ISI_AE_ANTIBANDING_MODE_60HZ, 120 },
    { ISI_AE_ANTIBANDING_MODE_AUTO, 100 }
};

/*****************************************************************************
 *Forward Declarations
*****************************************************************************/
RESULT IMX662_IsiSetIrisIss( IsiSensorHandle_t handle,
                    const float NewIris);

/*****************************************************************************
 *Sensor Info
*****************************************************************************/

static struct vvsensor_mode_s pimx662_mode_info[] = {
    /* 1920x1080 @ 30fps, 12-bit, SDR */
    {
        .index     = 0,
        .size      ={
			.bounds_width  = IMX662_RES_FHD_WIDTH,
			.bounds_height = IMX662_RES_FHD_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX662_RES_FHD_WIDTH,
			.height        = IMX662_RES_FHD_HEIGHT,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_RGGB,
	.ae_info = {
		.one_line_exp_time_ns = IMX662_SDR_ONE_LINE_EXP_TIME_NS,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX662_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30,
		.max_integration_time = IMX662_SDR_VMAX_30FPS - IMX662_MIN_SHR,
	}
    },
    /* 1920x1080 @ 30fps, 12-bit, DOL 2-frame HDR */
    {
        .index     = 1,
        .size      ={
			.bounds_width  = IMX662_RES_FHD_WIDTH,
			.bounds_height = IMX662_RES_FHD_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX662_RES_FHD_WIDTH,
			.height        = IMX662_RES_FHD_HEIGHT,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_HDR_STITCH,
	.stitching_mode = SENSOR_STITCHING_L_AND_S,
	.bit_width = 12,
	.bayer_pattern = BAYER_RGGB,
	.ae_info = {
		.one_line_exp_time_ns = IMX662_2DOL_ONE_LINE_EXP_TIME_NS,
		.max_integration_time = 1, /* Recalculated from RHS1 at init */
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX662_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30,
	}
    },
};

static RESULT IMX662_IsiSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    RESULT result = RET_SUCCESS;
    return (result);
}

static RESULT IMX662_GetCtrl(int sensor_fd, unsigned int ctrl_cid, int32_t *value)
{
	struct v4l2_control control;
	memset(&control, 0, sizeof(control));
	control.id = ctrl_cid;

	int ret = ioctl(sensor_fd, VIDIOC_G_CTRL, &control);
	if (ret < 0) {
		TRACE(IMX662_ERROR, "%s: VIDIOC_G_CTRL ioctl failed with error %d (%s). ret = %d\n",
			__func__, errno, strerror(errno), ret);
		return RET_FAILURE;
	}

	*value = control.value;
	return RET_SUCCESS;
}

static uint32_t IMX662_CalculateModeMaxIntegrationTime(
        uint32_t hdr_mode, uint32_t stitching_mode, uint32_t rhs1) {
	if (hdr_mode == SENSOR_MODE_LINEAR) {
		return IMX662_SDR_FSC - IMX662_MIN_SHR; // SDR
	} else {
        uint32_t rhs1_gap = (stitching_mode == SENSOR_STITCHING_3DOL) ?
        IMX662_3DOL_SHR1_RHS1_GAP : IMX662_2DOL_SHR1_RHS1_GAP;
        return rhs1 - rhs1_gap; // HDR
    }
}

static RESULT IMX662_SetSensorModeData(IMX662_Context_t* pIMX662Ctx, size_t index) {
    int sensor_fd = 0;
    int32_t rhs1 = 0;
    RESULT result = RET_SUCCESS;

    if (pIMX662Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    sensor_fd = ((HalContext_t*)pIMX662Ctx->IsiCtx.HalHandle)->sensor_fd;

    if (index >= ARRAY_SIZE(pimx662_mode_info)) {
        return RET_OUTOFRANGE;
    }

    pIMX662Ctx->SensorMode.index = index;
    memcpy(&pIMX662Ctx->SensorMode, &pimx662_mode_info[index], sizeof(pIMX662Ctx->SensorMode));
    if (pIMX662Ctx->SensorMode.index != index) {
        TRACE(IMX662_ERROR, "%s: Sensor mode index mismatch: expected %zu, got %u\n",
              __func__, index, pIMX662Ctx->SensorMode.index);
        return RET_FAILURE;
    }

    // If the mode is HDR, we need to get RHS1 value from the sensor.
    if (pIMX662Ctx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        /* Get the RHS1 value via a control rather then reading the register,
         * because the register is not available until a stream is running. */
        result = IMX662_GetCtrl(sensor_fd, IMX662_CID_RHS1, &rhs1);
        if ((result != RET_SUCCESS) || rhs1 == 0) {
            TRACE(IMX662_ERROR, "%s: Failed to get RHS1 value from sensor (result=%d)\n",
                __func__, result);
            return result;
        }
    }

    pIMX662Ctx->SensorMode.ae_info.max_integration_time =
        IMX662_CalculateModeMaxIntegrationTime(
            pIMX662Ctx->SensorMode.hdr_mode, pIMX662Ctx->SensorMode.stitching_mode, rhs1);

    if (pIMX662Ctx->SensorMode.ae_info.max_integration_time == 0) {
        TRACE(IMX662_ERROR, "%s: Invalid max integration time calculated (%u) for HDR mode %d, index %zu\n",
              __func__, pIMX662Ctx->SensorMode.ae_info.max_integration_time,
              pIMX662Ctx->SensorMode.hdr_mode, index);
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

static RESULT IMX662_IsiCreateIss(IsiSensorInstanceConfig_t* pConfig) {
    RESULT result = RET_SUCCESS;
    IMX662_Context_t* pIMX662Ctx;
    char i2c_file_path[PATH_MAX];

    memset(i2c_file_path, 0, PATH_MAX);

    if (!pConfig || !pConfig->pSensor) return (RET_NULL_POINTER);

    pIMX662Ctx = (IMX662_Context_t*)malloc(sizeof(IMX662_Context_t));
    if (!pIMX662Ctx) {
        return (RET_OUTOFMEM);
    }

    MEMSET(pIMX662Ctx, 0, sizeof(IMX662_Context_t));
    pIMX662Ctx->i2c_fd = -1;
    pIMX662Ctx->i2c_af_fd = -1;

    result = HalAddRef(pConfig->HalHandle);
    if (result != RET_SUCCESS) {
        goto error_free_ctx;
    }

    pIMX662Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pIMX662Ctx->IsiCtx.pSensor = pConfig->pSensor;
    pIMX662Ctx->GroupHold = BOOL_FALSE;
    pIMX662Ctx->OldGain = 1.0;
    pIMX662Ctx->OldIntegrationTime = 0.01;
    pIMX662Ctx->Configured = BOOL_FALSE;
    pIMX662Ctx->Streaming = BOOL_FALSE;
    pIMX662Ctx->TestPattern = BOOL_FALSE;
    pIMX662Ctx->isAfpsRun = BOOL_FALSE;
    
    // By default, until specified otherwise, all ratios are 1 (SDR)
    pIMX662Ctx->hdr_ratio[0] = 1.0f;
    pIMX662Ctx->hdr_ratio[1] = 1.0f;
    pIMX662Ctx->hcg_factor = 5.8f; /* Rcg typical, IMX662-AAQR1-C Datasheet p.23, Note 8 */

    result = IMX662_SetSensorModeData(pIMX662Ctx, pConfig->SensorModeIndex);
    if (result != RET_SUCCESS) {
        TRACE(IMX662_ERROR, "%s: Set sensor mode data failed! (%d)\n", __func__, result);
        goto error_hal_del_ref;
    }
    
    pConfig->hSensor = (IsiSensorHandle_t)pIMX662Ctx;
    pIMX662Ctx->pattern = ISI_BPAT_RGRGGBGB;
    pIMX662Ctx->subdev = HalGetFdHandle(pConfig->HalHandle,
                                        HAL_MODULE_SENSOR);  // two sensors??
    pIMX662Ctx->KernelDriverFlag = 1;
    TRACE(IMX662_INFO, "%s - sensor i2c bus: %d, af i2c bus: %d, sensor i2c addr: 0x%x, af i2c addr: 0x%x\n", __func__,
           pConfig->I2cBusNum, pConfig->I2cAfBusNum, pConfig->SlaveAddr, pConfig->SlaveAfAddr);
    pIMX662Ctx->i2c_addr = pConfig->SlaveAddr;
    pIMX662Ctx->i2c_af_addr = pConfig->SlaveAfAddr;

    sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cBusNum);
    pIMX662Ctx->i2c_fd = open(i2c_file_path, O_RDWR);
    if (pIMX662Ctx->i2c_fd < 0) {
        TRACE(IMX662_ERROR, "%s: unable to open /dev/i2c-%d, error %d (%s)\n",
              __func__, pConfig->I2cBusNum, errno, strerror(errno));
        result = RET_FAILURE;
        goto error_hal_del_ref;
    }

    if (ioctl(pIMX662Ctx->i2c_fd, I2C_SLAVE_FORCE, pIMX662Ctx->i2c_addr) < 0) {
        TRACE(IMX662_ERROR, "%s: unable to set I2C_SLAVE_FORCE on /dev/i2c-%d, error %d (%s)\n",
              __func__, pConfig->I2cBusNum, errno, strerror(errno));
        result = RET_FAILURE;
        goto error_close_i2c_fd;
    }

    if (pConfig->I2cAfBusNum < 0) {
        TRACE(IMX662_INFO, "no af i2c bus\n");
        pIMX662Ctx->SensorMode.af_mode = ISI_SENSOR_AF_MODE_NOTSUPP;
    } else {
        sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cAfBusNum);
        pIMX662Ctx->i2c_af_fd = open(i2c_file_path, O_RDWR);
        if (pIMX662Ctx->i2c_af_fd < 0) {
            TRACE(IMX662_ERROR, "%s: unable to open /dev/i2c-%d for af, error %d (%s)\n",
                  __func__, pConfig->I2cAfBusNum, errno, strerror(errno));
            result = RET_FAILURE;
            goto error_close_i2c_fd;
        }
        pIMX662Ctx->SensorMode.af_mode = ISI_SENSOR_AF_MODE_CDAF;
    }

    return (result);

error_close_i2c_fd:
    close(pIMX662Ctx->i2c_fd);
error_hal_del_ref:
    (void)HalDelRef(pConfig->HalHandle);
error_free_ctx:
    free(pIMX662Ctx);
    pIMX662Ctx = NULL;
    return (result);
}

static RESULT IMX662_IsiReleaseIss(IsiSensorHandle_t handle) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    RESULT cur_result = RET_SUCCESS;

    if (pIMX662Ctx == NULL) return (RET_WRONG_HANDLE);

    cur_result = IMX662_IsiSetStreamingIss(pIMX662Ctx, BOOL_FALSE);
    UPDATE_RESULT_LOG(result, cur_result, "SetStreaming off");
    cur_result = IMX662_IsiSetPowerIss(pIMX662Ctx, BOOL_FALSE);
    UPDATE_RESULT_LOG(result, cur_result, "SetPower off");
    (void)HalDelRef(pIMX662Ctx->IsiCtx.HalHandle);
    if (pIMX662Ctx->i2c_af_fd >= 0) {
        close(pIMX662Ctx->i2c_af_fd);
    }
    close(pIMX662Ctx->i2c_fd);
    MEMSET(pIMX662Ctx, 0, sizeof(IMX662_Context_t));
    free(pIMX662Ctx);
    pIMX662Ctx = NULL;
    return (result);
}

static RESULT IMX662_IsiReadRegIss(IsiSensorHandle_t handle,
                                   const uint32_t Addr, uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    struct i2c_rdwr_ioctl_data ioctl_data;
    unsigned char out[IMX662_TRANSFER_BUFFER_LENGTH];
    struct i2c_msg msgs[2];
    uint8_t addr_buf[2] = { (Addr >> 8) & 0xff, Addr & 0xff };

    if (pIMX662Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    msgs[0].addr = pIMX662Ctx->i2c_addr;
    msgs[0].flags = 0; // Write
    msgs[0].len = sizeof(addr_buf);
    msgs[0].buf = addr_buf;

    msgs[1].addr = pIMX662Ctx->i2c_addr;
    msgs[1].flags = I2C_M_RD; // Read
    msgs[1].len = 1;
    msgs[1].buf = out;

    ioctl_data.msgs = msgs;
    ioctl_data.nmsgs = 2;

    int ret = 0;
    int attempt;
    for (attempt = 0; attempt < IMX662_I2C_READ_MAX_ATTEMPTS; attempt++) {
        ret = ioctl(pIMX662Ctx->i2c_fd, I2C_RDWR, &ioctl_data);
        if (ret >= 0)
            break;
        if (errno != EREMOTEIO)
            break;
        usleep(IMX662_I2C_READ_RETRY_INTERVAL_US);
    }
    if (ret < 0) {
        TRACE(IMX662_ERROR, "%s: I2C_RDWR ioctl failed for addr 0x%04x, error %d (%s) after %d attempt(s)\n",
              __func__, Addr, errno, strerror(errno), attempt + 1);
        return RET_FAILURE;
    }
    if (attempt > 0) {
        TRACE(IMX662_INFO, "%s: I2C_RDWR ioctl recovered after %d EREMOTEIO retries (addr 0x%04x)\n",
              __func__, attempt, Addr);
    }

    *pValue = out[0];

    return (result);
}

static RESULT IMX662_IsiWriteRegIss(IsiSensorHandle_t handle,
                                    const uint32_t Addr, const uint32_t Value) {
    char out[IMX662_TRANSFER_BUFFER_LENGTH];
    ssize_t write_ret;

    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (pIMX662Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX662_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    out[2] = Value;
    write_ret = write(pIMX662Ctx->i2c_fd, out, sizeof(out));
    if (write_ret != sizeof(out)) {
        if (write_ret < 0) {
            TRACE(IMX662_ERROR, "%s: I2C write failed with error %d (%s)\n",
                  __func__, errno, strerror(errno));
        } else {
            TRACE(IMX662_ERROR, "%s: I2C write incomplete. Wrote %zd of %zu bytes\n",
                  __func__, write_ret, sizeof(out));
        }
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

static RESULT IMX662_UpdateFps(IMX662_Context_t *pIMX662Ctx, uint32_t vmax) {
    float frame_time = 0;
    frame_time = (vmax * pIMX662Ctx->one_line_exp_time);

    if (frame_time == 0) return RET_FAILURE;

    pIMX662Ctx->CurrFps = (uint32_t)(ceil(1 / frame_time)) * ISI_FPS_ACCURACY;
    return RET_SUCCESS;
}

static RESULT IMX662_ReadVmax(IsiSensorHandle_t handle, uint32_t* vmax) {
    uint32_t vmax_low = 0, vmax_mid = 0, vmax_high = 0;
    RESULT result;

    result = IMX662_IsiReadRegIss(handle, 0x3028, &vmax_low);
    CHECK_RESULT_RET(result, "ReadVmax low");
    result = IMX662_IsiReadRegIss(handle, 0x3029, &vmax_mid);
    CHECK_RESULT_RET(result, "ReadVmax mid");
    result = IMX662_IsiReadRegIss(handle, 0x302a, &vmax_high);
    CHECK_RESULT_RET(result, "ReadVmax high");

    *vmax = (vmax_high << 16) | (vmax_mid << 8) | vmax_low;
    return RET_SUCCESS;
}

static RESULT IMX662_ReadHmax(IsiSensorHandle_t handle, uint32_t* hmax) {
    uint32_t hmax_low = 0, hmax_high = 0;
    RESULT result;

    result = IMX662_IsiReadRegIss(handle, 0x302c, &hmax_low);
    CHECK_RESULT_RET(result, "ReadHmax low");
    result = IMX662_IsiReadRegIss(handle, 0x302d, &hmax_high);
    CHECK_RESULT_RET(result, "ReadHmax high");

    *hmax = (hmax_high << 8) | hmax_low;
    return RET_SUCCESS;
}

static RESULT IMX662_WriteVmax(IsiSensorHandle_t handle, uint32_t vmax) {
    RESULT result;

    result = IMX662_IsiWriteRegIss(handle, 0x3028, vmax & 0xff);
    CHECK_RESULT_RET(result, "WriteVmax low");
    result = IMX662_IsiWriteRegIss(handle, 0x3029, (vmax >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteVmax mid");
    result = IMX662_IsiWriteRegIss(handle, 0x302a, (vmax >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteVmax high");

    result = IMX662_UpdateFps((IMX662_Context_t *)handle, vmax);
    CHECK_RESULT_RET(result, "UpdateFps");
    return RET_SUCCESS;
}

static RESULT IMX662_ReadRHS1(IsiSensorHandle_t handle, uint32_t* rhs1) {
    uint32_t rhs1_low = 0, rhs1_mid = 0, rhs1_high = 0;
    RESULT result;

    result = IMX662_IsiReadRegIss(handle, 0x3060, &rhs1_low);
    CHECK_RESULT_RET(result, "ReadRHS1 low");
    result = IMX662_IsiReadRegIss(handle, 0x3061, &rhs1_mid);
    CHECK_RESULT_RET(result, "ReadRHS1 mid");
    result = IMX662_IsiReadRegIss(handle, 0x3062, &rhs1_high);
    CHECK_RESULT_RET(result, "ReadRHS1 high");

    *rhs1 = (rhs1_high << 16) | (rhs1_mid << 8) | rhs1_low;
    return RET_SUCCESS;
}

static RESULT IMX662_ReadRHS2(IsiSensorHandle_t handle, uint32_t* rhs2) {
    uint32_t rhs2_low = 0, rhs2_mid = 0, rhs2_high = 0;
    RESULT result;

    result = IMX662_IsiReadRegIss(handle, 0x3064, &rhs2_low);
    CHECK_RESULT_RET(result, "ReadRHS2 low");
    result = IMX662_IsiReadRegIss(handle, 0x3065, &rhs2_mid);
    CHECK_RESULT_RET(result, "ReadRHS2 mid");
    result = IMX662_IsiReadRegIss(handle, 0x3066, &rhs2_high);
    CHECK_RESULT_RET(result, "ReadRHS2 high");

    *rhs2 = (rhs2_high << 16) | (rhs2_mid << 8) | rhs2_low;
    return RET_SUCCESS;
}

static RESULT IMX662_WriteShr0(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX662_IsiWriteRegIss(handle, 0x3050, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr0 low");
    result = IMX662_IsiWriteRegIss(handle, 0x3051, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr0 mid");
    result = IMX662_IsiWriteRegIss(handle, 0x3052, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr0 high");

    return RET_SUCCESS;
}

static RESULT IMX662_WriteShr1(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX662_IsiWriteRegIss(handle, 0x3054, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr1 low");
    result = IMX662_IsiWriteRegIss(handle, 0x3055, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr1 mid");
    result = IMX662_IsiWriteRegIss(handle, 0x3056, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr1 high");

    return RET_SUCCESS;
}

static RESULT IMX662_WriteShr2(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX662_IsiWriteRegIss(handle, 0x3058, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr2 low");
    result = IMX662_IsiWriteRegIss(handle, 0x3059, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr2 mid");
    result = IMX662_IsiWriteRegIss(handle, 0x305a, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr2 high");

    return RET_SUCCESS;
}

static RESULT IMX662_WriteGain(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX662_IsiWriteRegIss(handle, 0x3070, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain low");
    result = IMX662_IsiWriteRegIss(handle, 0x3071, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain high");

    return RET_SUCCESS;
}

static RESULT IMX662_WriteGain1(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX662_IsiWriteRegIss(handle, 0x3072, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain1 low");
    result = IMX662_IsiWriteRegIss(handle, 0x3073, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain1 high");

    return RET_SUCCESS;
}

static RESULT IMX662_WriteGain2(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX662_IsiWriteRegIss(handle, 0x3074, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain2 low");
    result = IMX662_IsiWriteRegIss(handle, 0x3075, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain2 high");

    return RET_SUCCESS;
}

static RESULT IMX662_LockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX662_IsiWriteRegIss(handle, 0x3001, 0x1);
    CHECK_RESULT_RET(result, "LockRegHold");

    return result;
}

static RESULT IMX662_UnlockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX662_IsiWriteRegIss(handle, 0x3001, 0x0);
    CHECK_RESULT_RET(result, "UnlockRegHold");

    return result;
}

static RESULT IMX662_IsiGetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    TRACE(IMX662_INFO, "%s (enter)\n", __func__);
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (pIMX662Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    memcpy(pMode, &(pIMX662Ctx->SensorMode), sizeof(pIMX662Ctx->SensorMode));

    TRACE(IMX662_INFO, "%s (exit)\n", __func__);
    return (RET_SUCCESS);
}

static RESULT IMX662_IsiSetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    int ret = 0;
    TRACE(IMX662_INFO, "%s (enter)\n", __func__);

    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR, "%s: Invalid sensor handle\n", __func__);
        return (RET_WRONG_HANDLE);
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX662Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, pMode);
    if (ret != 0) {
        TRACE(IMX662_ERROR, "%s: VVSENSORIOC_S_SENSOR_MODE ioctl failed with error %d (%s). ret = %d\n",
            __func__, errno, strerror(errno), ret);
        return (RET_FAILURE);
    }

    return (RET_SUCCESS);
}

static RESULT IMX662_IsiHalEnumModeIss(HalHandle_t HalHandle,
                                       IsiEnumMode_t* pEnumMode) {
    HalContext_t* pHalCtx = HalHandle;
    if (pHalCtx == NULL) {
        return RET_NULL_POINTER;
    }

    if (pEnumMode->index >=
        (sizeof(pimx662_mode_info) / sizeof(pimx662_mode_info[0])))
        return RET_OUTOFRANGE;

    for (uint32_t i = 0;
         i < (sizeof(pimx662_mode_info) / sizeof(pimx662_mode_info[0])); i++) {
        if (pimx662_mode_info[i].index == pEnumMode->index) {
            memcpy(&pEnumMode->mode, &pimx662_mode_info[i], sizeof(IsiMode_t));
            TRACE(IMX662_INFO, "%s (exit)\n", __func__);
            return RET_SUCCESS;
        }
    }

    return RET_NOTSUPP;
}

static RESULT IMX662_IsiEnumModeIss(IsiSensorHandle_t handle,
                                    IsiEnumMode_t* pEnumMode) {
    RESULT result = RET_SUCCESS;
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (pIMX662Ctx == NULL || pIMX662Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    HalContext_t* pHalCtx = (HalContext_t*)pIMX662Ctx->IsiCtx.HalHandle;
    result = IMX662_IsiHalEnumModeIss(pHalCtx, pEnumMode);
    CHECK_RESULT_RET(result, "sensor enum mode");

    return result;
}

static RESULT IMX662_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t* pCaps) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;

    RESULT result = RET_SUCCESS;

    TRACE(IMX662_INFO, "%s (enter)\n", __func__);

    if (pIMX662Ctx == NULL) return (RET_WRONG_HANDLE);

    if (pCaps == NULL) {
        return (RET_NULL_POINTER);
    }

    if (!pIMX662Ctx->Configured) {
        result = IMX662_IsiSetupIss(handle, pCaps);
        CHECK_RESULT_RET(result, "Setup");
    }

    pCaps->BusWidth = pIMX662Ctx->SensorMode.bit_width;
    pCaps->Mode = ISI_MODE_BAYER;
    pCaps->FieldSelection = ISI_FIELDSEL_BOTH;
    pCaps->YCSequence = ISI_YCSEQ_YCBYCR;
    pCaps->Conv422 = ISI_CONV422_COSITED;
    pCaps->BPat = pIMX662Ctx->SensorMode.bayer_pattern;
    pCaps->HPol = ISI_HPOL_REFPOS;
    pCaps->VPol = ISI_VPOL_POS;
    pCaps->Edge = ISI_EDGE_RISING;
    pCaps->Resolution.width = pIMX662Ctx->SensorMode.size.width;
    pCaps->Resolution.height = pIMX662Ctx->SensorMode.size.height;
    pCaps->SmiaMode = ISI_SMIA_OFF;
    pCaps->MipiLanes = ISI_MIPI_4LANES;

    if (pCaps->BusWidth == 10) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_10;
    } else if (pCaps->BusWidth == 12) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_12;
    } else {
        pCaps->MipiMode = ISI_MIPI_OFF;
    }
    TRACE(IMX662_INFO, "got caps - width %d height %d buswidth %d\n",
          pCaps->Resolution.width, pCaps->Resolution.height, pCaps->BusWidth);
    TRACE(IMX662_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX662_AecSetModeParameters(IMX662_Context_t* pIMX662Ctx,
                                          const IsiCaps_t* pConfig) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX662_INFO, "%s%s: (enter)\n", __func__,
          pIMX662Ctx->isAfpsRun ? "(AFPS)" : "");

    pIMX662Ctx->AecIntegrationTimeIncrement = pIMX662Ctx->one_line_exp_time;
    pIMX662Ctx->AecMinIntegrationTime =
        pIMX662Ctx->one_line_exp_time * pIMX662Ctx->MinIntegrationLine;
    pIMX662Ctx->AecMaxIntegrationTime =
        pIMX662Ctx->one_line_exp_time * pIMX662Ctx->MaxIntegrationLine;

    TRACE(IMX662_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __func__,
          pIMX662Ctx->isAfpsRun ? "(AFPS)" : "",
          pIMX662Ctx->AecMaxIntegrationTime);

    pIMX662Ctx->AecGainIncrement = IMX662_MIN_GAIN_STEP;

    // reflects the state of the sensor registers, must equal default settings
    pIMX662Ctx->AecCurGainLEF = pIMX662Ctx->AecMinGain;
	pIMX662Ctx->AecCurGainSEF1 = pIMX662Ctx->AecMinGain;
	pIMX662Ctx->AecCurGainSEF2 = pIMX662Ctx->AecMinGain;
	pIMX662Ctx->AecCurIntegrationTimeLEF = pIMX662Ctx->AecMaxIntegrationTime;
	pIMX662Ctx->AecCurIntegrationTimeSEF1 = pIMX662Ctx->AecMaxIntegrationTime;
	pIMX662Ctx->AecCurIntegrationTimeSEF2 = pIMX662Ctx->AecMaxIntegrationTime;
    pIMX662Ctx->OldGain = 1;
    pIMX662Ctx->OldIntegrationTime = 0.0f;

    TRACE(IMX662_INFO, "%s%s: (exit)\n", __func__,
          pIMX662Ctx->isAfpsRun ? "(AFPS)" : "");

    return (result);
}

static RESULT IMX662_IsiSetupIss(IsiSensorHandle_t handle,
                                 const IsiCaps_t* pCaps) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX662_INFO, "%s (enter)\n", __func__);

    if (!pIMX662Ctx) {
        TRACE(IMX662_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pCaps == NULL) {
        TRACE(IMX662_ERROR,
              "%s: Invalid configuration (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMX662Ctx->Streaming != BOOL_FALSE) {
        return RET_WRONG_STATE;
    }

    if (pIMX662Ctx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        pIMX662Ctx->enableHdr = true;
    } else {
        pIMX662Ctx->enableHdr = false;
    }

    pIMX662Ctx->one_line_exp_time =
        (float)(pIMX662Ctx->SensorMode.ae_info.one_line_exp_time_ns) /
        1000000000;
    pIMX662Ctx->MaxIntegrationLine =
        pIMX662Ctx->SensorMode.ae_info.max_integration_time;
    TRACE(IMX662_INFO, "%s: MaxIntegrationLine %u\n", __func__,
          pIMX662Ctx->MaxIntegrationLine);
    pIMX662Ctx->MinIntegrationLine =
        pIMX662Ctx->SensorMode.ae_info.min_integration_time;
    pIMX662Ctx->gain_accuracy = pIMX662Ctx->SensorMode.ae_info.gain_accuracy;
    pIMX662Ctx->AecMaxGain = (float)(pIMX662Ctx->SensorMode.ae_info.max_gain) /
                             pIMX662Ctx->gain_accuracy;
    pIMX662Ctx->AecMinGain = (float)(pIMX662Ctx->SensorMode.ae_info.min_gain) /
                             pIMX662Ctx->gain_accuracy;

    pIMX662Ctx->AecMinIris = IMX662_IRIS_MIN_VAL;
    pIMX662Ctx->AecMaxIris = IMX662_IRIS_MAX_VAL;

    pIMX662Ctx->original_vmax = 0;
    pIMX662Ctx->unlimit_fps = 0;
    pIMX662Ctx->unlimit_fps_vmax_changed = 0;
    pIMX662Ctx->flicker_fps_mode = ISI_AE_ANTIBANDING_MODE_OFF;

    pIMX662Ctx->MaxFps = pIMX662Ctx->SensorMode.fps;
    pIMX662Ctx->CurrFps = pIMX662Ctx->MaxFps;
    TRACE(IMX662_INFO, "%s - got caps - width %d height %d buswidth %d\n",
          __func__, pIMX662Ctx->SensorMode.size.width,
          pIMX662Ctx->SensorMode.size.height, pIMX662Ctx->SensorMode.bit_width);

    TRACE(IMX662_INFO, "%s - MinGain %f MaxGain %f\n", __func__, pIMX662Ctx->AecMinGain,
          pIMX662Ctx->AecMaxGain);

    memcpy(&pIMX662Ctx->CapsConfig, pCaps, sizeof(pIMX662Ctx->CapsConfig));

    /* SW reset comment preserved - no actual reset code was present,
     * and the 100ms sleep was unnecessary (no hardware access in this function). */

    result = IMX662_AecSetModeParameters(pIMX662Ctx, pCaps);
    CHECK_RESULT_RET(result, "AecSetModeParameters");

    pIMX662Ctx->Configured = BOOL_TRUE;
    TRACE(IMX662_INFO, "%s: (exit)\n", __func__);
    return 0;
}

static RESULT IMX662_IsiCheckConnectionIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    return (result);
}

static RESULT IMX662_IsiGetRevisionIss(IsiSensorHandle_t handle,
                                       uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    int ret = 0;
    uint32_t reg_val;
    uint32_t sensor_id;

    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (pIMX662Ctx == NULL || pIMX662Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX662Ctx->IsiCtx.HalHandle;

    if (!pValue) return (RET_NULL_POINTER);

    if (pIMX662Ctx->KernelDriverFlag) {
        ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, &sensor_id);
        if (ret != 0) {
            TRACE(IMX662_ERROR, "%s: Read Sensor ID Error! \n", __func__);
            return (RET_FAILURE);
        }
    } else {
        reg_val = 0;
        result = IMX662_IsiReadRegIss(handle, 0x3cb6, &reg_val);
        CHECK_RESULT_RET(result, "read sensor ID high");
        sensor_id = (reg_val & 0xff) << 8;

        reg_val = 0;
        result = IMX662_IsiReadRegIss(handle, 0x3cc4, &reg_val);
        CHECK_RESULT_RET(result, "read sensor ID low");
        sensor_id |= (reg_val & 0xff);
    }

    *pValue = sensor_id;
    TRACE(IMX662_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX662_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t is_on) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX662_INFO, "%s (enter)\n", __func__);

    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (pIMX662Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    pIMX662Ctx->Streaming = is_on;

    if (pIMX662Ctx->enableHdr)
        return result;

    TRACE(IMX662_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX662_IsiGetGainLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinGain, float* pMaxGain) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX662_INFO, "%s: (enter)\n", __func__);

    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinGain == NULL) || (pMaxGain == NULL)) {
        TRACE(IMX662_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinGain = pIMX662Ctx->AecMinGain;
    *pMaxGain = pIMX662Ctx->AecMaxGain;

    TRACE(IMX662_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static inline int IMX662_getFlickerPeaksPerSec(IsiSensorAntibandingMode_t mode) {
    int num_modes = sizeof(flickerPeaksPerSecMap) / sizeof(FlickerModePeaksPerSec);
    for (int i = 0; i < num_modes; i++) {
        if (flickerPeaksPerSecMap[i].mode == mode) {
            return flickerPeaksPerSecMap[i].value;
        }
    }
    return 0; // Defaults to 0 if mode not found
}

static RESULT IMX662_GetNumExposures(IMX662_Context_t* pIMX662Ctx, size_t* num_exposures) {
    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    if (num_exposures == NULL) {
        TRACE(IMX662_ERROR, "%s: Invalid output parameter\n", __func__);
        return RET_NULL_POINTER;
    }

    if (pIMX662Ctx->SensorMode.hdr_mode == SENSOR_MODE_LINEAR) {
        *num_exposures = 1; // SDR
    } else if (pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        *num_exposures = IMX662_2DOL_NUM_EXP;
    } else if (pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        *num_exposures = IMX662_3DOL_NUM_EXP;
    } else {
        TRACE(IMX662_ERROR, "%s: Unsupported HDR mode %d with stitching mode %d\n",
            __func__, pIMX662Ctx->SensorMode.hdr_mode, pIMX662Ctx->SensorMode.stitching_mode);
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

static RESULT IMX662_UpdateCurrLEFIntegrationTimeFromFsc(IMX662_Context_t* pIMX662Ctx, uint32_t fsc, uint32_t shr0) {
    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    float configuredIntegrationTime = (fsc - shr0) * pIMX662Ctx->one_line_exp_time;

    pIMX662Ctx->OldIntegrationTime = configuredIntegrationTime;
    pIMX662Ctx->AecCurIntegrationTimeLEF = configuredIntegrationTime;

    TRACE(IMX662_DEBUG, "%s: Updated LEF Integration Time = (fsc[%u] - shr0[%u]) * one_line_exp_time[%f] = %f\n",
          __func__, fsc, shr0, pIMX662Ctx->one_line_exp_time, configuredIntegrationTime);
    return RET_SUCCESS;
}

static RESULT IMX662_UpdateCurrLEFIntegrationTimeFromVmax(IMX662_Context_t* pIMX662Ctx, uint32_t vmax, uint32_t shr0) {
    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    size_t dol = 0;
    RESULT result = IMX662_GetNumExposures(pIMX662Ctx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");
    uint32_t fsc = vmax * dol;
    TRACE(IMX662_DEBUG, "%s: fsc = vmax[%u] * dol[%zu] = %u\n", __func__, vmax, dol, fsc);
    result = IMX662_UpdateCurrLEFIntegrationTimeFromFsc(pIMX662Ctx, fsc, shr0);
    CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromFsc");
    return result;
}

static inline RESULT IMX662_getNewVmaxAntiFlicker(IMX662_Context_t *pIMX662Ctx, uint32_t requestedVmax, uint32_t *outClosestVmax) {
    uint32_t closestVmax = requestedVmax;
    int peaks = 0;
    int difference = INT_MAX;
    int minDifference = INT_MAX;
    if (!pIMX662Ctx) {
        TRACE(IMX662_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return RET_FAILURE;
    }
    peaks = IMX662_getFlickerPeaksPerSec(pIMX662Ctx->flicker_fps_mode);
    if (peaks == 0) {
        // No anti-flicker mode, return requested or original Vmax
        if (pIMX662Ctx->unlimit_fps && pIMX662Ctx->unlimit_fps_vmax_changed)
            *outClosestVmax = requestedVmax;
        else
            *outClosestVmax = pIMX662Ctx->original_vmax;
        return RET_SUCCESS;
    }

    for (int i = 1; i < peaks; ++i) {
        uint32_t candidateVmax = i / (peaks * pIMX662Ctx->one_line_exp_time);
        if (candidateVmax < pIMX662Ctx->SensorMode.size.height || candidateVmax < pIMX662Ctx->original_vmax)
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

static RESULT IMX662_IsiUnlimitFpsIss(IsiSensorHandle_t handle,
                                      float maxIntegrationTime) {
    RESULT result = RET_SUCCESS;
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;

    if (!pIMX662Ctx) {
        return RET_NULL_POINTER;
    }

    if (pIMX662Ctx->enableHdr) {
        return result;
    }
    pIMX662Ctx->unlimit_fps = 1;

    if (maxIntegrationTime < 0)
        pIMX662Ctx->MaxIntegrationLine = IMX662_VMAX_MAX - IMX662_MIN_SHR;
    else
        pIMX662Ctx->MaxIntegrationLine =
            MIN((uint32_t)(maxIntegrationTime / pIMX662Ctx->one_line_exp_time),
                IMX662_VMAX_MAX - IMX662_MIN_SHR);
    TRACE(IMX662_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
        pIMX662Ctx->MaxIntegrationLine);
    pIMX662Ctx->AecMaxIntegrationTime =
        pIMX662Ctx->one_line_exp_time * pIMX662Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX662_IsiLimitFpsIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    uint32_t current_vmax = 0;
    uint32_t new_vmax = 0;
    TRACE(IMX662_INFO, "%s (enter)\n", __func__);
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (!pIMX662Ctx) {
        return RET_NULL_POINTER;
    }

    if (pIMX662Ctx->enableHdr)
        return result;

    result = IMX662_ReadVmax(handle, &current_vmax);
    CHECK_RESULT_RET(result, "ReadVmax");
    if (current_vmax == 0) {
        TRACE(IMX662_INFO, "%s - exit because current_vmax is 0\n", __func__);
        return result;
    }

    pIMX662Ctx->unlimit_fps = 0;
    pIMX662Ctx->unlimit_fps_vmax_changed = 0;
    if (pIMX662Ctx->original_vmax == 0) {
        pIMX662Ctx->original_vmax = current_vmax;
    }

    new_vmax = pIMX662Ctx->original_vmax;
    if (pIMX662Ctx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
        result = IMX662_getNewVmaxAntiFlicker(pIMX662Ctx, pIMX662Ctx->original_vmax, &new_vmax);
        CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
        TRACE(IMX662_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX662Ctx->flicker_fps_mode, new_vmax);
    }
    if (current_vmax != new_vmax) {
        result = IMX662_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        result = IMX662_WriteVmax(handle, new_vmax);
        CHECK_RESULT_RET(result, "WriteVmax");
        result = IMX662_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");

        int shr = MAX((int)current_vmax - (int)(pIMX662Ctx->AecCurIntegrationTimeLEF / pIMX662Ctx->one_line_exp_time), IMX662_MIN_SHR);
        result = IMX662_UpdateCurrLEFIntegrationTimeFromVmax(pIMX662Ctx, new_vmax, shr);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromVmax");
    }

    pIMX662Ctx->MaxIntegrationLine =
        MAX(new_vmax - IMX662_MIN_SHR, 1);
    TRACE(IMX662_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
          pIMX662Ctx->MaxIntegrationLine);
    pIMX662Ctx->AecMaxIntegrationTime =
        pIMX662Ctx->one_line_exp_time * pIMX662Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX662_IsiGetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinIris, float* pMaxIris) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX662_INFO, "%s: (enter)\n", __func__);

    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIris == NULL) || (pMaxIris == NULL)) {
        TRACE(IMX662_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIris = pIMX662Ctx->AecMinIris;
    *pMaxIris = pIMX662Ctx->AecMaxIris;

    TRACE(IMX662_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX662_IsiSetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float minIris, float maxIris) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX662_INFO, "%s: (enter)\n", __func__);

    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    pIMX662Ctx->AecMinIris = minIris;
    pIMX662Ctx->AecMaxIris = maxIris;

    TRACE(IMX662_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX662_IsiGetIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float* pMinIntegrationTime,
    float* pMaxIntegrationTime) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    float max_short_it, min_short_it;
    RESULT result = RET_SUCCESS;
    int rhs1 = -1;
    HalContext_t* pHalCtx = NULL;

    TRACE(IMX662_INFO, "%s: (enter). prev values: min: %f, max: %f\n", __func__, pIMX662Ctx->AecMinIntegrationTime, pIMX662Ctx->AecMaxIntegrationTime);

    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX662_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMX662Ctx->enableHdr) {
    *pMinIntegrationTime = pIMX662Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMX662Ctx->AecMaxIntegrationTime;

        TRACE(IMX662_INFO, "%s: (exit - SDR) %f, %f\n", 
    __func__, *pMinIntegrationTime, *pMaxIntegrationTime);

        return RET_SUCCESS;
    }

    pHalCtx = (HalContext_t*)pIMX662Ctx->IsiCtx.HalHandle;
    if (!pHalCtx) {
        TRACE(IMX662_ERROR,
              "%s: Invalid HAL handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIMX662Ctx->cur_rhs1 == 0) {
        result = IMX662_GetCtrl(pHalCtx->sensor_fd, IMX662_CID_RHS1, &rhs1);
        CHECK_RESULT_RET(result, "GetCtrl RHS1");
        pIMX662Ctx->cur_rhs1 = (uint32_t)rhs1;
    }

    if (pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        max_short_it = (pIMX662Ctx->cur_rhs1 - IMX662_2DOL_SHR1_MIN_GAP) * pIMX662Ctx->one_line_exp_time;
        min_short_it = IMX662_2DOL_SHR1_RHS1_GAP * pIMX662Ctx->one_line_exp_time;
    } else {
        /* TODO: implement 3dol exposure */
        max_short_it = (pIMX662Ctx->cur_rhs1 - IMX662_2DOL_SHR1_MIN_GAP) * pIMX662Ctx->one_line_exp_time;
        min_short_it = IMX662_2DOL_SHR1_RHS1_GAP * pIMX662Ctx->one_line_exp_time;
    }

    /*
     * Report the actual SEF1 hardware limits.  The ratio is enforced inside
     * Calculate2DOLExposures / Calculate3DOLExposures which will maximise the
     * LEF integration time and apply LEF gain when needed to meet the ratio.
     */
    pIMX662Ctx->AecMinIntegrationTime = min_short_it;
    pIMX662Ctx->AecMaxIntegrationTime = max_short_it;

    *pMinIntegrationTime = pIMX662Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMX662Ctx->AecMaxIntegrationTime;

    TRACE(IMX662_INFO, "%s: (exit - HDR). min: %f, max: %f\n",  __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

static RESULT IMX662_IsiGetAbsoluteIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float* pMinIntegrationTime,
    float* pMaxIntegrationTime) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX662_INFO, "%s: (enter)\n", __func__);
    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX662_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIntegrationTime = IMX662_EXP_MIN_LINES * pIMX662Ctx->one_line_exp_time;
    *pMaxIntegrationTime = (IMX662_VMAX_MAX - IMX662_MIN_SHR) * pIMX662Ctx->one_line_exp_time;

    TRACE(IMX662_INFO, "%s: (exit) %f, %f\n", 
    __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

/* Gain get functions*/

RESULT IMX662_IsiGetGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;
	TRACE(IMX662_INFO, "%s: (enter)\n", __func__);

	if (pIMX662Ctx == NULL) {
		TRACE(IMX662_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	if (pIMX662Ctx->enableHdr) {
		RESULT result = IMX662_IsiGetSEF1GainIss(handle, pSetGain);
		CHECK_RESULT_RET(result, "GetSEF1Gain");
		return result;
	}

	RESULT result = IMX662_IsiGetLEFGainIss(handle, pSetGain);
	CHECK_RESULT_RET(result, "GetLEFGain");
	return result;
}

RESULT IMX662_IsiGetLEFGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	TRACE(IMX662_INFO, "%s: (enter)\n", __func__);

	if (pIMX662Ctx == NULL) {
		TRACE(IMX662_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}
	*pSetGain = pIMX662Ctx->AecCurGainLEF;
	TRACE(IMX662_DEBUG, "%s - returning %f\n", __func__, pIMX662Ctx->AecCurGainLEF);
	TRACE(IMX662_INFO, "%s: (exit)\n", __func__);
	return (result);
}

RESULT IMX662_IsiGetSEF1GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX662_INFO, "%s: (enter)\n", __func__);

	if (pIMX662Ctx == NULL) {
		TRACE(IMX662_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX662Ctx->AecCurGainSEF1;
	TRACE(IMX662_DEBUG, "%s - returning %f\n", __func__, pIMX662Ctx->AecCurGainSEF1);

	TRACE(IMX662_INFO, "%s: (exit)\n", __func__);

	return (result);
}

RESULT IMX662_IsiGetSEF2GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX662_INFO, "%s: (enter)\n", __func__);

	if (pIMX662Ctx == NULL) {
		TRACE(IMX662_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX662Ctx->AecCurGainSEF2;
	TRACE(IMX662_DEBUG, "%s - returning %f\n", __func__, pIMX662Ctx->AecCurGainSEF2);

	TRACE(IMX662_INFO, "%s: (exit)\n", __func__);

	return (result);
}


RESULT IMX662_IsiGetGainIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX662_INFO, "%s: (enter)\n", __func__);

    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = pIMX662Ctx->AecGainIncrement;

    TRACE(IMX662_INFO, "%s: (exit)\n", __func__);

    return (result);
}

RESULT IMX662_IsiGetIrisIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX662_INFO, "%s: (enter)\n", __func__);
    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = 1;

    TRACE(IMX662_INFO, "%s: (exit)\n", __func__);

    return (result);
}

/* Gain set functions*/

RESULT IMX662_IsiSetGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;
	if (pIMX662Ctx == NULL || pIMX662Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	if (pIMX662Ctx->enableHdr) {
		result = IMX662_IsiSetSEF1GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);
        CHECK_RESULT_RET(result, "SetSEF1Gain");

        if (pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            result = IMX662_IsiSetSEF2GainIss(
                handle, 0, NewGain, pSetGain, hdr_ratio);
            CHECK_RESULT_RET(result, "SetSEF2Gain");
        }
	}

	result = IMX662_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
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

RESULT IMX662_IsiSetLEFGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;
	if (pIMX662Ctx == NULL || pIMX662Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	TRACE(IMX662_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX662_DEBUG, "%s: writting 0x%x to GAIN\n", __func__, Gain);

	result = IMX662_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX662_WriteGain(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain");
	result = IMX662_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pIMX662Ctx->AecCurGainLEF = _sensorGain2linear(Gain);
	*pSetGain = pIMX662Ctx->AecCurGainLEF;
	TRACE(IMX662_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

RESULT IMX662_IsiSetSEF1GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX662Ctx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);

	TRACE(IMX662_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX662_DEBUG, "%s: writting 0x%x to GAIN1\n", __func__, Gain);

	result = IMX662_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX662_WriteGain1(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain1");
	result = IMX662_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pIMX662Ctx->AecCurGainSEF1 = _sensorGain2linear(Gain);
	*pSetGain = pIMX662Ctx->AecCurGainSEF1;

	TRACE(IMX662_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

RESULT IMX662_IsiSetSEF2GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX662Ctx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);
	
	TRACE(IMX662_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX662_DEBUG, "%s: writting 0x%x to GAIN2\n", __func__, Gain);

	result = IMX662_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX662_WriteGain2(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain2");
	result = IMX662_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pIMX662Ctx->AecCurGainSEF2 = _sensorGain2linear(Gain);
	*pSetGain = pIMX662Ctx->AecCurGainSEF2;

	TRACE(IMX662_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

/* Integration Time get functions*/

RESULT IMX662_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;

	if (!pIMX662Ctx) {
		TRACE(IMX662_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	
	TRACE(IMX662_DEBUG, "%s - enter\n", __func__);
	
	if (pIMX662Ctx->enableHdr) {
		RESULT result = IMX662_IsiGetSEF1IntegrationTimeIss(handle, pSetIntegrationTime);
		CHECK_RESULT_RET(result, "GetSEF1IntegrationTime");
		return result;
	}
	
	RESULT result = IMX662_IsiGetLEFIntegrationTimeIss(handle, pSetIntegrationTime);
	CHECK_RESULT_RET(result, "GetLEFIntegrationTime");
	return result;
}

RESULT IMX662_IsiGetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;
	RESULT result = RET_SUCCESS;


	if (!pIMX662Ctx) {
		TRACE(IMX662_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX662Ctx->AecCurIntegrationTimeLEF;
	TRACE(IMX662_DEBUG, "%s - returning %f\n", __func__, pIMX662Ctx->AecCurIntegrationTimeLEF);
	return (result);
}

RESULT IMX662_IsiGetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX662Ctx) {
		TRACE(IMX662_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX662Ctx->AecCurIntegrationTimeSEF1;
	TRACE(IMX662_DEBUG, "%s - returning %f\n", __func__, pIMX662Ctx->AecCurIntegrationTimeSEF1);
	return (result);
}

RESULT IMX662_IsiGetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX662Ctx) {
		TRACE(IMX662_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);

	*pSetIntegrationTime = pIMX662Ctx->AecCurIntegrationTimeSEF2;
	TRACE(IMX662_DEBUG, "%s - returning %f\n", __func__, pIMX662Ctx->AecCurIntegrationTimeSEF2);
	return (result);
}

RESULT IMX662_IsiGetIntegrationTimeIncrementIss(IsiSensorHandle_t handle,
                                                float* pIncr) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (!pIMX662Ctx) {
        TRACE(IMX662_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pIncr) return (RET_NULL_POINTER);

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders
    //in
    // the application)
    *pIncr = pIMX662Ctx->AecIntegrationTimeIncrement;
    return (result);
}

/* Integration Time set functions*/

RESULT IMX662_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;

	if (!pIMX662Ctx) {
		TRACE(IMX662_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX662Ctx->enableHdr) {
		RESULT result = IMX662_IsiSetSEF1IntegrationTimeIss(
			handle, NewIntegrationTime, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1IntegrationTime");
		return result;
	}

	RESULT result = IMX662_IsiSetLEFIntegrationTimeIss(
		handle, NewIntegrationTime, pSetIntegrationTime,
		pNumberOfFramesToSkip, hdr_ratio);
	CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
	return result;
}

RESULT IMX662_IsiSetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;

    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;

    int shr = 0;
    uint32_t exp = 0;
    uint32_t new_vmax = 0;
    uint32_t vmax_updated = 0;
    uint32_t current_vmax = 0;
    uint32_t rhs1;
    uint32_t rhs2;

    if (!pIMX662Ctx) {
        TRACE(IMX662_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pSetIntegrationTime || !pNumberOfFramesToSkip) {
        TRACE(IMX662_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMX662Ctx->Streaming) {
        TRACE(IMX662_ERROR, "%s: sensor not streaming\n", __func__);
        return RET_FAILURE;
    }
    exp = roundf(NewIntegrationTime / pIMX662Ctx->one_line_exp_time);

    TRACE(IMX662_DEBUG, "%s: set AEC_PK_EXPO=0x%05x\n", __func__, exp);

    if (fabs(NewIntegrationTime - pIMX662Ctx->AecCurIntegrationTimeLEF) > FLT_EPSILON) {

        if (pIMX662Ctx->enableHdr){
            if (pIMX662Ctx->cur_rhs1 == 0 || pIMX662Ctx->cur_rhs2 == 0) {
                TRACE(IMX662_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
                return (RET_WRONG_CONFIG);
            }

            rhs1 = pIMX662Ctx->cur_rhs1;
            rhs2 = pIMX662Ctx->cur_rhs2;
            if (pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
                result = IMX662_ReadVmax(pIMX662Ctx, &new_vmax);
                CHECK_RESULT_RET(result, "ReadVmax");
                new_vmax *= IMX662_2DOL_NUM_EXP;

                exp = new_vmax - exp;
                exp = exp > rhs1 + IMX662_2DOL_SHR0_RHS1_GAP ? exp : rhs1 + IMX662_2DOL_SHR0_RHS1_GAP;
                exp = exp < new_vmax - IMX662_2DOL_SHR0_FSC_GAP? exp : new_vmax - IMX662_2DOL_SHR0_FSC_GAP;
                shr = exp;
            } else {
                new_vmax = IMX662_3DOL_FSC;
                exp = new_vmax - exp;
                exp = exp > rhs2 + IMX662_3DOL_SHR0_RHS2_GAP ? exp : rhs2 + IMX662_3DOL_SHR0_RHS2_GAP;
                exp = exp < new_vmax - IMX662_3DOL_SHR0_FSC_GAP? exp : new_vmax - IMX662_3DOL_SHR0_FSC_GAP;
                shr = exp;
            }
		} else {
            if (exp > pIMX662Ctx->MaxIntegrationLine || exp == 0) {
                TRACE(IMX662_ERROR, "%s: Integration time %f (exp %u) out of range (%u)\n", __func__,
                    NewIntegrationTime, exp, pIMX662Ctx->MaxIntegrationLine);
                return RET_FAILURE;
            }
            result = IMX662_ReadVmax(handle, &current_vmax);
            CHECK_RESULT_RET(result, "ReadVmax");

            if (pIMX662Ctx->original_vmax == 0) {
                pIMX662Ctx->original_vmax = current_vmax;
            }

            if (pIMX662Ctx->original_vmax - IMX662_MIN_SHR >
                pIMX662Ctx->MaxIntegrationLine) {
                pIMX662Ctx->MaxIntegrationLine =
                    pIMX662Ctx->original_vmax - IMX662_MIN_SHR;
                TRACE(IMX662_DEBUG, "%s: set MaxIntegrationLine to %u\n", __func__,
                      pIMX662Ctx->MaxIntegrationLine);
                pIMX662Ctx->AecMaxIntegrationTime =
                    pIMX662Ctx->one_line_exp_time * pIMX662Ctx->MaxIntegrationLine;
                TRACE(IMX662_DEBUG, "%s: set AecMaxIntegrationTime to %f\n", __func__,
                      pIMX662Ctx->AecMaxIntegrationTime);
            }

            shr = current_vmax - exp;

            if (shr < IMX662_MIN_SHR || current_vmax > pIMX662Ctx->original_vmax) {
                if (shr < IMX662_MIN_SHR) {
                    // user asked for too big integration-line: more than (current_vmax - minimal_shutter)
                    // increase vmax and set new shutter to minimal value
                    new_vmax = MIN(exp + IMX662_MIN_SHR, pIMX662Ctx->MaxIntegrationLine + IMX662_MIN_SHR);
                } else {
                    // make sure that when we unlimit fps, stay below original fps.
                    // also, make sure we are not using high vmax + high shr more than needed (will slow fps for no reason)
                    new_vmax = MAX(current_vmax - shr + IMX662_MIN_SHR, pIMX662Ctx->original_vmax);
                }
                pIMX662Ctx->unlimit_fps_vmax_changed = new_vmax > pIMX662Ctx->original_vmax && pIMX662Ctx->unlimit_fps;
        
                if (pIMX662Ctx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
                    result = IMX662_getNewVmaxAntiFlicker(pIMX662Ctx, new_vmax, &new_vmax);
                    CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
                    TRACE(IMX662_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX662Ctx->flicker_fps_mode, new_vmax);
                }
                
                shr = MAX(IMX662_MIN_SHR, (int)new_vmax - (int)exp);
                vmax_updated = 1;

            } else {
                new_vmax = current_vmax;
            }
        }

        result = IMX662_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        if (vmax_updated && pIMX662Ctx->unlimit_fps && !pIMX662Ctx->enableHdr) {
            result = IMX662_WriteVmax(handle, new_vmax);
            CHECK_RESULT_RET(result, "WriteVmax");
        }

        TRACE(IMX662_DEBUG, "%s - writing 0x%x to SHR0\n", __func__, shr);
        result = IMX662_WriteShr0(handle, shr);
        CHECK_RESULT_RET(result, "WriteShr0");
        result = IMX662_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");

        // In this context, the "new_vmax" is actually the FSC (multiplied by DOL), not the VMAX.
        result = IMX662_UpdateCurrLEFIntegrationTimeFromFsc(pIMX662Ctx, new_vmax, shr);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromFsc");

        *pNumberOfFramesToSkip = 1U;
    } else {
        *pNumberOfFramesToSkip = 0U;  // no frame skip
    }

    *pSetIntegrationTime = pIMX662Ctx->AecCurIntegrationTimeLEF;
    TRACE(IMX662_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTime);
    return (result);
}

RESULT IMX662_IsiSetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF1,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;

	if (!pIMX662Ctx) {
		TRACE(IMX662_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX662Ctx->cur_rhs1 == 0 || pIMX662Ctx->cur_rhs2 == 0) {
		TRACE(IMX662_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX662Ctx->cur_rhs1;

	if (!pSetIntegrationTimeSEF1 || !pNumberOfFramesToSkip) {
		TRACE(IMX662_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX662_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = roundf(NewIntegrationTime / pIMX662Ctx->one_line_exp_time);
	TRACE(IMX662_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX662Ctx->AecCurIntegrationTimeSEF1) > FLT_EPSILON) {
        if (pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
			exp = rhs1 - exp;
			exp = exp > IMX662_2DOL_SHR1_RHS1_GAP ? exp : IMX662_2DOL_SHR1_RHS1_GAP;
			exp = exp < rhs1 - IMX662_2DOL_SHR1_RHS1_GAP ? exp : rhs1 - IMX662_2DOL_SHR1_RHS1_GAP;
		} else {
			exp = rhs1 - exp;
			exp = exp > IMX662_3DOL_SHR1_RHS1_GAP ? exp : IMX662_3DOL_SHR1_RHS1_GAP;
			exp = exp < rhs1 - IMX662_3DOL_SHR1_RHS1_GAP ? exp : rhs1 - IMX662_3DOL_SHR1_RHS1_GAP;
		}
		TRACE(IMX662_DEBUG, "%s - writing 0x%x to SHR1\n", __func__, exp);

		result = IMX662_LockRegHold(handle);
		CHECK_RESULT_RET(result, "LockRegHold");
		result = IMX662_WriteShr1(handle, exp);
		CHECK_RESULT_RET(result, "WriteShr1");
		result = IMX662_UnlockRegHold(handle);
		CHECK_RESULT_RET(result, "UnlockRegHold");

		pIMX662Ctx->AecCurIntegrationTimeSEF1 = (rhs1 - exp) * pIMX662Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF1 = pIMX662Ctx->AecCurIntegrationTimeSEF1;

	TRACE(IMX662_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF1);
	return (result);
}

RESULT IMX662_IsiSetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF2,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX662_Context_t *pIMX662Ctx = (IMX662_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;
	uint32_t rhs2;

	if (pIMX662Ctx->cur_rhs1 == 0 || pIMX662Ctx->cur_rhs2 == 0) {
		TRACE(IMX662_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX662Ctx->cur_rhs1;
	rhs2 = pIMX662Ctx->cur_rhs2;

	if (!pIMX662Ctx) {
		TRACE(IMX662_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTimeSEF2 || !pNumberOfFramesToSkip) {
		TRACE(IMX662_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX662_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = roundf(NewIntegrationTime / pIMX662Ctx->one_line_exp_time);
	TRACE(IMX662_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX662Ctx->AecCurIntegrationTimeSEF2) > FLT_EPSILON) {
		exp = rhs2 - exp;
		exp = exp > rhs1 + IMX662_3DOL_SHR2_RHS1_GAP ? exp : rhs1 + IMX662_3DOL_SHR2_RHS1_GAP;
		exp = exp < rhs2 - IMX662_3DOL_SHR2_RHS2_GAP ? exp : rhs2 - IMX662_3DOL_SHR2_RHS2_GAP;
		TRACE(IMX662_DEBUG, "%s - writing 0x%x to SHR2\n", __func__, exp);

		result = IMX662_LockRegHold(handle);
		CHECK_RESULT_RET(result, "LockRegHold");
		result = IMX662_WriteShr2(handle, exp);
		CHECK_RESULT_RET(result, "WriteShr2");
		result = IMX662_UnlockRegHold(handle);
		CHECK_RESULT_RET(result, "UnlockRegHold");

		pIMX662Ctx->AecCurIntegrationTimeSEF2 = (rhs2 - exp) * pIMX662Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF2 = pIMX662Ctx->AecCurIntegrationTimeSEF2;

	TRACE(IMX662_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF2);
	return (result);
}

RESULT IMX662_Calculate3DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it, float *o_very_short_it,
                                    float *o_long_gain, float *o_short_gain, float *o_very_short_gain,
                                    float *hdr_ratio) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
	float very_short_it, very_short_exp_val;
	bool recalc_vs_gain = false;
	float very_short_gain = 1;
	uint32_t rhs1;
	uint32_t rhs2;

	if (pIMX662Ctx == NULL || o_long_it == NULL || o_short_it == NULL ||
        o_very_short_it == NULL || o_long_gain == NULL || o_short_gain == NULL ||
        o_very_short_gain == NULL || hdr_ratio == NULL) {
        TRACE(IMX662_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

	if (pIMX662Ctx->cur_rhs1 == 0 || pIMX662Ctx->cur_rhs2 == 0) {
		TRACE(IMX662_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX662Ctx->cur_rhs1;
	rhs2 = pIMX662Ctx->cur_rhs2;

	if(NewIntegrationTime < IMX662_3DOL_SHR2_RHS2_GAP * pIMX662Ctx->one_line_exp_time * hdr_ratio[1]){
		pIMX662Ctx->MinIntegrationLine =  IMX662_3DOL_SHR2_RHS2_GAP * hdr_ratio[1];
		pIMX662Ctx->AecMinIntegrationTime = pIMX662Ctx->MinIntegrationLine * pIMX662Ctx->one_line_exp_time;
    }

    {
        float lef_sens = pIMX662Ctx->hcg_lef ? pIMX662Ctx->hcg_factor : 1.0f;
        float sef_sens = pIMX662Ctx->hcg_sef1 ? pIMX662Ctx->hcg_factor : 1.0f;
        float vs_sens  = pIMX662Ctx->hcg_sef2 ? pIMX662Ctx->hcg_factor : 1.0f;
        float ls_adjusted = hdr_ratio[0] * sef_sens / lef_sens;
        float sv_adjusted = hdr_ratio[1] * vs_sens / sef_sens;

        very_short_it = NewIntegrationTime / sv_adjusted;
        very_short_exp_val = very_short_it / pIMX662Ctx->one_line_exp_time;
        very_short_gain = _sensorGain2linear(_linear2sensorGain(NewGain));

        if (very_short_exp_val < rhs1 + IMX662_3DOL_SHR2_RHS1_GAP) {
            very_short_exp_val = rhs1 + IMX662_3DOL_SHR2_RHS1_GAP;
            very_short_it = (rhs2 - very_short_exp_val) * pIMX662Ctx->one_line_exp_time;
            recalc_vs_gain = true;
            TRACE(IMX662_DEBUG, "%s: very_short_exp_val is too long, set to %u, new very_short_it = %f\n",
            __func__, rhs2 + IMX662_3DOL_SHR2_RHS1_GAP, very_short_it);
        } else if(very_short_exp_val > rhs2 - IMX662_3DOL_SHR2_RHS2_GAP) {
            very_short_exp_val = rhs2 - IMX662_3DOL_SHR2_RHS2_GAP;
            very_short_it = (rhs2 - very_short_exp_val) * pIMX662Ctx->one_line_exp_time;
            recalc_vs_gain = true;
            TRACE(IMX662_DEBUG, "%s: very_short_exp_val is too short, set to %u, new very_short_it = %f\n",
            __func__, rhs2 - IMX662_3DOL_SHR2_RHS2_GAP, very_short_it);
        }

        *o_long_it = NewIntegrationTime * ls_adjusted;
        *o_long_gain = _sensorGain2linear(_linear2sensorGain(NewGain));
        *o_short_it = NewIntegrationTime;
        *o_short_gain = _sensorGain2linear(_linear2sensorGain(NewGain));

        if (recalc_vs_gain) {
            very_short_gain = (NewIntegrationTime * (*o_short_gain)) / (very_short_it * sv_adjusted);
                 very_short_gain = _sensorGain2linear(_linear2sensorGainCeil(very_short_gain));
            very_short_it = (NewIntegrationTime * (*o_short_gain)) / (very_short_gain * sv_adjusted);
        }
    }

    *o_very_short_it = very_short_it;
    *o_very_short_gain = very_short_gain;

    return RET_SUCCESS;
}


RESULT IMX662_Calculate2DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it,
                                    float *o_long_gain, float *o_short_gain,
                                    float *hdr_ratio) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
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

    TRACE(IMX662_DEBUG, "%s: enter with NewIntegrationTime=%.6f NewGain=%.4f ratio=%.1f\n",
        __func__, NewIntegrationTime, NewGain, hdr_ratio[0]);

    result = IMX662_ReadVmax(handle, &vmax);
    CHECK_RESULT_RET(result, "ReadVmax");

    result = IMX662_GetNumExposures(pIMX662Ctx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");

    fsc = vmax * (uint32_t)dol;
    rhs1 = pIMX662Ctx->cur_rhs1;
    one_line = pIMX662Ctx->one_line_exp_time;

    if (one_line <= 0.0f) {
        TRACE(IMX662_ERROR, "%s: Invalid one_line_exp_time (%.9f)\n", __func__, one_line);
        return (RET_WRONG_CONFIG);
    }

    if (fsc <= rhs1 + IMX662_2DOL_SHR0_RHS1_GAP) {
        TRACE(IMX662_ERROR, "%s: fsc(%u) <= rhs1(%u) + gap, invalid config\n",
              __func__, fsc, rhs1);
        return (RET_WRONG_CONFIG);
    }

    /* Hardware limits: SHR0 must satisfy  rhs1 + gap <= SHR0 <= fsc - gap. */
    max_long_it_lines = fsc - rhs1 - IMX662_2DOL_SHR0_RHS1_GAP;
    min_long_it_lines = IMX662_2DOL_SHR0_FSC_GAP;

    /* Quantize short gain to sensor dB steps and compute the total long EV target. */
    short_gain = _sensorGain2linear(_linear2sensorGain(NewGain));

    {
        float lef_sens = pIMX662Ctx->hcg_lef ? pIMX662Ctx->hcg_factor : 1.0f;
        float sef_sens = pIMX662Ctx->hcg_sef1 ? pIMX662Ctx->hcg_factor : 1.0f;
        float adjusted_ratio = hdr_ratio[0] * sef_sens / lef_sens;
        required_long_ev = NewIntegrationTime * short_gain * adjusted_ratio;
    }

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
     * Step 1b: When the required EV is within IMX662_2DOL_MAX_IT_NEAR_LINES of
     * max_long_it, the shortfall is a float-rounding artifact -- accept unity
     * gain and the negligible ratio deviation instead of bumping a gain step.
     */
    if (long_gain_needed > 1.0f && long_it_lines == max_long_it_lines) {
        required_lines = required_long_ev / one_line;
        if (required_lines - (float)max_long_it_lines < (float)IMX662_2DOL_MAX_IT_NEAR_LINES) {
            long_gain_needed = 1.0f;
        }
    }

    if (long_gain_needed <= 1.0f) {
        long_gain = 1.0f;
    } else {
        /*
         * Step 2: IT at maximum is not enough -- apply the minimum gain that
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

        max_gain_db = _linear2sensorGain(IMX662_MAX_GAIN_AEC);
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

    TRACE(IMX662_DEBUG, "%s: ratio=%.1f vmax=%u fsc=%u rhs1=%u max_long_it_lines=%u\n",
        __func__, hdr_ratio[0], vmax, fsc, rhs1, max_long_it_lines);
    TRACE(IMX662_DEBUG, "%s: required_long_ev=%.6f long_it=%.6f(%u lines) long_gain=%.4f\n",
        __func__, required_long_ev, long_it, long_it_lines, long_gain);
    TRACE(IMX662_DEBUG, "%s: short_it=%.6f short_gain=%.4f\n",
        __func__, *o_short_it, *o_short_gain);

    return RET_SUCCESS;
}

RESULT IMX662_IsiExposureControlIss(IsiSensorHandle_t handle, float NewGain,
                                    float NewIntegrationTime,
                                    uint8_t* pNumberOfFramesToSkip,
                                    float* pSetGain, float* pSetIntegrationTime,
                                    float* hdr_ratio) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;

    RESULT result = RET_SUCCESS;
    float long_it = 0.0;
	float short_it = 0.0;
	float very_short_it = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	float very_short_gain = 1;
    uint32_t hmax;

    TRACE(IMX662_INFO, "%s: enter with NewIntegrationTime: %f, NewGain: %f\n",
        __func__, NewIntegrationTime, NewGain);

    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pNumberOfFramesToSkip == NULL) || (pSetGain == NULL) ||
        (pSetIntegrationTime == NULL)) {
        TRACE(IMX662_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    /* Skip silently when PRE_STREAMOFF has paused 3A; trailing calls are expected. */
    if (!pIMX662Ctx->Streaming) {
        return RET_SUCCESS;
    }

    if (NewIntegrationTime == 0 || NewGain == 0) {
        TRACE(IMX662_ERROR, "%s: Invalid parameter (NewIntegrationTime or NewGain is 0)\n", __func__);
        return (RET_WRONG_CONFIG);
    }

    // Regardless of 2dol/3dol (or SDR) - we should get the up-to-date hdr_ratio (if 2dol/sdr, hdr_ratio[1] = 1)
    pIMX662Ctx->hdr_ratio[0] = hdr_ratio[0];
    pIMX662Ctx->hdr_ratio[1] = hdr_ratio[1];

    if (pIMX662Ctx->enableHdr) {
        result = IMX662_ReadRHS1(handle, &pIMX662Ctx->cur_rhs1);
        CHECK_RESULT_RET(result, "ReadRHS1");
        result = IMX662_ReadRHS2(handle, &pIMX662Ctx->cur_rhs2);
        CHECK_RESULT_RET(result, "ReadRHS2");
        result = IMX662_ReadHmax(handle, &hmax);
        CHECK_RESULT_RET(result, "ReadHmax");

        pIMX662Ctx->SensorMode.ae_info.one_line_exp_time_ns = HMAX_TO_ONE_LINE_EXP_NS(hmax);
        pIMX662Ctx->one_line_exp_time =
        (float)(pIMX662Ctx->SensorMode.ae_info.one_line_exp_time_ns) / 1000000000;

        if (pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
            //2exposure
            result = IMX662_Calculate2DOLExposures(handle, NewIntegrationTime, NewGain,
                                    &long_it, &short_it,
                                    &long_gain, &short_gain,
                                    hdr_ratio);
            CHECK_RESULT_RET(result, "Calculate2DOLExposures");
        } else {
            //3exposure
            result = IMX662_Calculate3DOLExposures(handle, NewIntegrationTime, NewGain,
                                        &long_it, &short_it, &very_short_it,
                                        &long_gain, &short_gain, &very_short_gain,
                                        hdr_ratio);
            CHECK_RESULT_RET(result, "Calculate3DOLExposures");
        }

		result = IMX662_IsiSetLEFIntegrationTimeIss(handle, long_it,
							pSetIntegrationTime,
							pNumberOfFramesToSkip,
							hdr_ratio);
		CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
		result = IMX662_IsiSetLEFGainIss(handle, long_gain, pSetGain, hdr_ratio);
		CHECK_RESULT_RET(result, "SetLEFGain");
		result = IMX662_IsiSetSEF1IntegrationTimeIss(
			handle, short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1IntegrationTime");
		result = IMX662_IsiSetSEF1GainIss(handle, NewIntegrationTime,
						  short_gain, pSetGain, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1Gain");
        if (pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            result = IMX662_IsiSetSEF2IntegrationTimeIss(
			    handle, very_short_it, pSetIntegrationTime,
			    pNumberOfFramesToSkip, hdr_ratio);
			CHECK_RESULT_RET(result, "SetSEF2IntegrationTime");
            result = IMX662_IsiSetSEF2GainIss(handle, NewIntegrationTime,
                            very_short_gain, pSetGain, hdr_ratio);
			CHECK_RESULT_RET(result, "SetSEF2Gain");
        }

        // Recalculate `io_hdr_ratio` according to the set values
        {
            float lef_sens = pIMX662Ctx->hcg_lef ? pIMX662Ctx->hcg_factor : 1.0f;
            float sef_sens = pIMX662Ctx->hcg_sef1 ? pIMX662Ctx->hcg_factor : 1.0f;
            hdr_ratio[0] = (long_it * long_gain * lef_sens) / (short_it * short_gain * sef_sens);
            if (pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
                float vs_sens = pIMX662Ctx->hcg_sef2 ? pIMX662Ctx->hcg_factor : 1.0f;
                hdr_ratio[1] = (short_it * short_gain * sef_sens) / (very_short_it * very_short_gain * vs_sens);
            }
        }

        // Set the output values to SEF1 values
        *pSetGain = short_gain;
        *pSetIntegrationTime = short_it;

        TRACE(IMX662_DEBUG, "%s: actual hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n",
            __func__, hdr_ratio[0], hdr_ratio[1]);
    } else {
        result = IMX662_IsiSetLEFIntegrationTimeIss(handle, NewIntegrationTime,
                                                pSetIntegrationTime,
                                                pNumberOfFramesToSkip, hdr_ratio);
        CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
        result = IMX662_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
        CHECK_RESULT_RET(result, "SetLEFGain");
    }
    return result;
}

RESULT IMX662_IsiExposureControlExpandedIss(
    IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime,
    float NewIris, uint8_t* pNumberOfFramesToSkip, float* pSetGain,
    float* pSetIntegrationTime, float* pSetIris, float* hdr_ratio) {

    RESULT result = RET_SUCCESS;
    if (pSetIris) {
        result = IMX662_IsiSetIrisIss(handle, NewIris);
        CHECK_RESULT_RET(result, "SetIris");
        *pSetIris = NewIris;
    }

    result = IMX662_IsiExposureControlIss(handle, NewGain, NewIntegrationTime,
                                        pNumberOfFramesToSkip, pSetGain,
                                        pSetIntegrationTime, hdr_ratio);
    CHECK_RESULT_RET(result, "ExposureControl");
    return result;
}

RESULT IMX662_IsiGetCurrentExposureIss(IsiSensorHandle_t handle,
                                       float* pSetGain,
                                       float* pSetIntegrationTime) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pSetGain == NULL) || (pSetIntegrationTime == NULL))
        return (RET_NULL_POINTER);

    if (pIMX662Ctx->enableHdr) {
		*pSetGain = pIMX662Ctx->AecCurGainSEF1;
		*pSetIntegrationTime = pIMX662Ctx->AecCurIntegrationTimeSEF1;
	} else {
		*pSetGain = pIMX662Ctx->AecCurGainLEF;
		*pSetIntegrationTime = pIMX662Ctx->AecCurIntegrationTimeLEF;
	}

    return (result);
}

RESULT IMX662_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t* pFps) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *pFps = pIMX662Ctx->CurrFps;

    return (result);
}

RESULT IMX662_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t Fps) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX662_IsiSetFlickerFpsIss(IsiSensorHandle_t handle, uint32_t flickerMode) {
    RESULT result = RET_SUCCESS;
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    uint32_t current_vmax = 0;
    uint32_t requested_vmax = 0;
    uint32_t requested_fsc = 0;
    uint32_t shr = 0;
    size_t dol = 0;
    uint32_t fsc = 0;
    uint32_t min_shr0 = (dol == 1) ? IMX662_MIN_SHR : IMX662_2DOL_SHR0_RHS1_GAP + pIMX662Ctx->cur_rhs1;
    int exp = 0;

    TRACE(IMX662_DEBUG, "%s: set sensor flickerMode = %d\n", __func__, flickerMode);

    if (!pIMX662Ctx) {
        return RET_NULL_POINTER;
    }
    if (pIMX662Ctx->flicker_fps_mode == flickerMode) {
        return RET_SUCCESS;
    }
    if (pIMX662Ctx->enableHdr && (pIMX662Ctx->SensorMode.stitching_mode != SENSOR_STITCHING_L_AND_S)) {
        return RET_SUCCESS;
    }
    result = IMX662_GetNumExposures(pIMX662Ctx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");
    if (flickerMode > ISI_AE_ANTIBANDING_MODE_AUTO) {
        TRACE(IMX662_INFO, "%s: Invalid flickerMode (%d), setting ISI_AE_ANTIBANDING_MODE_AUTO instead.\n", __func__, flickerMode);
        flickerMode = ISI_AE_ANTIBANDING_MODE_AUTO;
    }
    pIMX662Ctx->flicker_fps_mode = flickerMode;

    result = IMX662_ReadVmax(handle, &current_vmax);
    CHECK_RESULT_RET(result, "ReadVmax");
    if (pIMX662Ctx->original_vmax == 0) {
        pIMX662Ctx->original_vmax = current_vmax;
    }

    fsc = current_vmax * dol;

    exp = pIMX662Ctx->AecCurIntegrationTimeLEF / pIMX662Ctx->one_line_exp_time;
    shr = MAX((int)(fsc) - exp, min_shr0);

    if (current_vmax > pIMX662Ctx->original_vmax) {
        current_vmax = MAX((int)fsc - (int)shr + min_shr0, min_shr0);
        fsc = current_vmax * dol;
        shr = MAX((int)fsc - exp, min_shr0);
        pIMX662Ctx->unlimit_fps_vmax_changed = current_vmax > pIMX662Ctx->original_vmax && pIMX662Ctx->unlimit_fps;
    }

    result = IMX662_getNewVmaxAntiFlicker(pIMX662Ctx, current_vmax, &requested_vmax);
    CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
    requested_vmax = MAX( MIN(requested_vmax, IMX662_VMAX_MAX), 1);
    requested_fsc = requested_vmax * dol;
    
    if (current_vmax != requested_vmax) {
        shr = MAX( (int)requested_fsc - (int)fsc + (int)shr, min_shr0);
        TRACE(IMX662_DEBUG, "%s - writing 0x%x to VMAX, writing 0x%x to SHR0\n", __func__, requested_vmax, shr);

        result = IMX662_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        result = IMX662_WriteVmax(handle, requested_vmax);
        CHECK_RESULT_RET(result, "WriteVmax");
        result = IMX662_WriteShr0(handle, shr);
        CHECK_RESULT_RET(result, "WriteShr0");
        result = IMX662_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");
        result = IMX662_UpdateCurrLEFIntegrationTimeFromVmax(pIMX662Ctx, requested_vmax, shr);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromVmax");
    }
    
    // these 2 are being used only in SDR
    pIMX662Ctx->MaxIntegrationLine = MAX( MIN(requested_vmax - IMX662_MIN_SHR, IMX662_VMAX_MAX - IMX662_MIN_SHR), 1);
    pIMX662Ctx->AecMaxIntegrationTime = pIMX662Ctx->one_line_exp_time * pIMX662Ctx->MaxIntegrationLine;

    TRACE(IMX662_INFO, "%s: set sensor fps = %d\n", __func__,
          pIMX662Ctx->CurrFps);

    TRACE(IMX662_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX662_IsiGetAutoFpsInfoIss(IsiSensorHandle_t handle,
                                   IsiAutoFps_t* pAutoFpsInfo) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX662_IsiGetStartEvIss(IsiSensorHandle_t handle, uint64_t* pStartEv) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX662_IsiGetIspStatusIss(IsiSensorHandle_t handle,
                                 IsiIspStatus_t* pIspStatus) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (pIMX662Ctx == NULL || pIMX662Ctx->IsiCtx.HalHandle == NULL) {
        return RET_WRONG_HANDLE;
    }

    pIspStatus->useSensorAE = false;
    pIspStatus->useSensorBLC = false;
    pIspStatus->useSensorAWB = false;

    return RET_SUCCESS;
}

RESULT IMX662_IsiSetTpgIss(IsiSensorHandle_t handle, IsiTpg_t Tpg) {
    RESULT result = RET_SUCCESS;

    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (pIMX662Ctx == NULL || pIMX662Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX662Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (Tpg.enable == 0) {
        result = IMX662_IsiWriteRegIss(handle, 0x30E0, 0x00);
        CHECK_RESULT_RET(result, "disable test pattern");
    } else {
        result = IMX662_IsiWriteRegIss(handle, 0x30E0, 0x01);
        CHECK_RESULT_RET(result, "enable test pattern");
    }

    pIMX662Ctx->TestPattern = Tpg.enable;

    return (result);
}

RESULT IMX662_IsiGetTpgIss(IsiSensorHandle_t handle, IsiTpg_t* Tpg) {
    RESULT result = RET_SUCCESS;
    uint32_t value = 0;

    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (pIMX662Ctx == NULL || pIMX662Ctx->IsiCtx.HalHandle == NULL ||
        Tpg == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX662Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    result = IMX662_IsiReadRegIss(handle, 0x30E0, &value);
    CHECK_RESULT_RET(result, "read TPG register");

    Tpg->enable = ((value & 0x01) != 0) ? 1 : 0;
    if (Tpg->enable) {
        Tpg->pattern = (0xff & value);
    }
    pIMX662Ctx->TestPattern = Tpg->enable;

    return (result);
}

RESULT IMX662_IsiFocusCreateIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX662_IsiFocusReleaseIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX662_IsiFocusGetCalibrateIss(IsiSensorHandle_t handle,
                                      IsiFocusCalibAttr_t* pFocusCalib) {
    return RET_SUCCESS;
}

RESULT IMX662_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

RESULT IMX662_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}


RESULT IMX662_IsiSetAgainDgainIss(IsiSensorHandle_t handle,
                                  IsiUserGain_t Gain) {
    RESULT result = RET_SUCCESS;
    uint32_t Again = 0, Dgain = 0;
    TRACE(IMX662_INFO, "%s: (enter)\n", __func__);

    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((Gain.again < 1) | (Gain.again > 16)) {
        TRACE(IMX662_ERROR, "%s: Invalid sensor again (%f)\n", __func__, Gain.again);
        return (RET_OUTOFRANGE);
    }
    // Again = (uint32_t)(1024 - (1024/Gain.again));
    Again = (uint32_t)(((2048 * Gain.again) - 2048) / Gain.again);

    result = IMX662_IsiWriteRegIss(handle, 0x3094, (Again & 0x0000FF));
    CHECK_RESULT_RET(result, "write again low");
    result = IMX662_IsiWriteRegIss(handle, 0x3095, (Again & 0x00FF00) >> 8);
    CHECK_RESULT_RET(result, "write again high");

    if ((Gain.dgain < 1) | (Gain.dgain > 16)) {
        TRACE(IMX662_ERROR, "%s: Invalid sensor dgain (%f)\n", __func__, Gain.dgain);
        return (RET_OUTOFRANGE);
    }
    Dgain = Gain.dgain * 256;

    result = IMX662_IsiWriteRegIss(handle, 0x308c, (Dgain & 0x0000FF));
    CHECK_RESULT_RET(result, "write dgain low");
    result = IMX662_IsiWriteRegIss(handle, 0x308d, (Dgain & 0x00FF00) >> 8);
    CHECK_RESULT_RET(result, "write dgain high");

    TRACE(IMX662_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX662_IsiGetIrisIss( IsiSensorHandle_t handle,
                                     float *pSetIris ) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX662_INFO, "%s: (enter)\n", __func__);

    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    //TODO implement getting iris here!
    *pSetIris = 1.0;

    TRACE(IMX662_INFO, "%s: (exit)\n", __func__);
    return (result);
}

// TODO: Should this sensor actually support iris control?
RESULT IMX662_IsiSetIrisIss( IsiSensorHandle_t handle, 
                    const float NewIris) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX662_INFO, "%s: (enter)\n", __func__);

    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (NewIris < IMX662_IRIS_MIN_VAL || NewIris > IMX662_IRIS_MAX_VAL) {
        TRACE(IMX662_ERROR, "%s: Invalid Iris %f\n", __func__, NewIris);
    }

    //TODO implement setting iris here!

    TRACE(IMX662_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX662_IsiGetHCGIss( IsiSensorHandle_t handle,
                                     bool *phcg_lef, bool *phcg_sef1, bool *phcg_sef2 ) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX662_INFO, "%s: (enter)\n", __func__);

    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *phcg_lef = pIMX662Ctx->hcg_lef;
    *phcg_sef1 = pIMX662Ctx->hcg_sef1;
    *phcg_sef2 = pIMX662Ctx->hcg_sef2;

    TRACE(IMX662_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX662_IsiSetHCGIss(IsiSensorHandle_t handle, bool hcg_lef, bool hcg_sef1, bool hcg_sef2) {

    RESULT result = RET_SUCCESS;

    TRACE(IMX662_INFO, "%s: (enter)\n", __func__);

    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;

    if (pIMX662Ctx == NULL) {
        TRACE(IMX662_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    result = IMX662_IsiWriteRegIss(handle, 0x3030 , hcg_lef);
    CHECK_RESULT_RET(result, "write HCG LEF");
    pIMX662Ctx->hcg_lef = hcg_lef;

    if (pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S ||
        pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        result = IMX662_IsiWriteRegIss(handle, 0x3031 , hcg_sef1);
        CHECK_RESULT_RET(result, "write HCG SEF1");
        pIMX662Ctx->hcg_sef1 = hcg_sef1;
    }
    if (pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        result = IMX662_IsiWriteRegIss(handle, 0x3032 , hcg_sef2);
        CHECK_RESULT_RET(result, "write HCG SEF2");
        pIMX662Ctx->hcg_sef2 = hcg_sef2;
    }

    TRACE(IMX662_INFO, "%s: (exit)\n", __func__);
    return result;
}



static RESULT IMX662_CalculateHdrBlankingLines(IsiSensorHandle_t handle,
        uint32_t *pBlankingLines, uint32_t rhs1, uint32_t rhs2) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;

    /* The following formulas are taken directly from Sony's IMX662 datasheet.
     * They are located at the "AppNote_DOL_E" document,
     * at "Operating Mode" -> "Number of Blanking Lines" section ("Virtual Channel Mode"). */

    // 2DOL
    if (pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        pBlankingLines[0] = (rhs1 - 3) / 2 + 1;
        pBlankingLines[1] = 0; // Irrelevant in 2DOL
    // 3DOL
    } else if (pIMX662Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        pBlankingLines[0] = (rhs1 - 4) / 3 + 1;
        pBlankingLines[1] = (rhs2 - 5) / 3 + 1;
    } else {
        TRACE(IMX662_ERROR, "%s: Unsupported stitching mode %d\n",
              __func__, pIMX662Ctx->SensorMode.stitching_mode);
        return RET_NOTSUPP;
    }

    return RET_SUCCESS;
}

static RESULT IMX662_IsiGetHdrBlankingLinesIss(IsiSensorHandle_t handle,
        uint32_t *pBlankingLines, size_t elementCount) {
    IMX662_Context_t* pIMX662Ctx = (IMX662_Context_t*)handle;
    HalContext_t* pHalCtx = (HalContext_t*)pIMX662Ctx->IsiCtx.HalHandle;
    RESULT result = RET_SUCCESS;
    int32_t rhs1 = 0, rhs2 = 0;

    if (elementCount != 2) {
        TRACE(IMX662_ERROR, "%s: Invalid element count %zu, expected 2\n",
              __func__, elementCount);
        return RET_OUTOFRANGE;
    }

    result = IMX662_GetCtrl(pHalCtx->sensor_fd, IMX662_CID_RHS1, &rhs1);
    CHECK_RESULT_RET(result, "GetCtrl RHS1");
    result = IMX662_GetCtrl(pHalCtx->sensor_fd, IMX662_CID_RHS2, &rhs2);
    CHECK_RESULT_RET(result, "GetCtrl RHS2");

    result = IMX662_CalculateHdrBlankingLines(handle, pBlankingLines, rhs1, rhs2);
    CHECK_RESULT_RET(result, "CalculateHdrBlankingLines");
    return result;
}

RESULT IMX662_IsiGetSensorIss(IsiSensor_t* pIsiSensor) {
    RESULT result = RET_SUCCESS;
    static const char SensorName[16] = "IMX662";
    TRACE(IMX662_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor != NULL) {
        pIsiSensor->pszName = SensorName;
        pIsiSensor->pIsiCreateIss = IMX662_IsiCreateIss;
        pIsiSensor->pIsiReleaseIss = IMX662_IsiReleaseIss;
        pIsiSensor->pIsiReadRegIss = IMX662_IsiReadRegIss;
        pIsiSensor->pIsiWriteRegIss = IMX662_IsiWriteRegIss;
        pIsiSensor->pIsiGetModeIss = IMX662_IsiGetModeIss;
        pIsiSensor->pIsiSetModeIss = IMX662_IsiSetModeIss;
        pIsiSensor->pIsiEnumModeIss = IMX662_IsiEnumModeIss;
        pIsiSensor->pIsiGetCapsIss = IMX662_IsiGetCapsIss;
        pIsiSensor->pIsiSetupIss = IMX662_IsiSetupIss;
        pIsiSensor->pIsiCheckConnectionIss = IMX662_IsiCheckConnectionIss;
        pIsiSensor->pIsiGetRevisionIss = IMX662_IsiGetRevisionIss;
        pIsiSensor->pIsiSetStreamingIss = IMX662_IsiSetStreamingIss;

        /* AEC functions */
        pIsiSensor->pIsiGetGainLimitsIss = 					IMX662_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIrisLimitsIss = 					IMX662_IsiGetIrisLimitsIss;
        pIsiSensor->pIsiSetIrisLimitsIss = 					IMX662_IsiSetIrisLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss =		IMX662_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetAbsoluteIntegrationTimeLimitsIss =		IMX662_IsiGetAbsoluteIntegrationTimeLimitsIss;

        pIsiSensor->pIsiExposureControlIss =				IMX662_IsiExposureControlIss;
        pIsiSensor->pIsiExposureControlExpandedIss =		IMX662_IsiExposureControlExpandedIss;
        pIsiSensor->pIsiSetIntegrationTimeIss =				IMX662_IsiSetIntegrationTimeIss;

        pIsiSensor->pIsiGetLongIntegrationTimeIss =			IMX662_IsiGetLEFIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIss =				IMX662_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetShortIntegrationTimeIss =		IMX662_IsiGetSEF1IntegrationTimeIss;
        pIsiSensor->pIsiGetVSIntegrationTimeIss =			IMX662_IsiGetSEF2IntegrationTimeIss;

        pIsiSensor->pIsiGetLongGainIss = 					IMX662_IsiGetLEFGainIss;
        pIsiSensor->pIsiGetGainIss = 						IMX662_IsiGetGainIss;
        pIsiSensor->pIsiGetShortGainIss = 					IMX662_IsiGetSEF1GainIss;
        pIsiSensor->pIsiGetVSGainIss = 						IMX662_IsiGetSEF2GainIss;

        pIsiSensor->pIsiGetGainIncrementIss =				IMX662_IsiGetGainIncrementIss;
        pIsiSensor->pIsiGetIrisIncrementIss =				IMX662_IsiGetIrisIncrementIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss =	IMX662_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetGainIss = 						IMX662_IsiSetGainIss;
        pIsiSensor->pIsiGetFpsIss = 						IMX662_IsiGetFpsIss;
        pIsiSensor->pIsiSetFpsIss = 						IMX662_IsiSetFpsIss;
        pIsiSensor->pIsiSetFlickerFpsIss = 					IMX662_IsiSetFlickerFpsIss;
        pIsiSensor->pIsiUnlimitFpsIss =						IMX662_IsiUnlimitFpsIss;
        pIsiSensor->pIsiLimitFpsIss =						IMX662_IsiLimitFpsIss;
        pIsiSensor->pIsiGetAutoFpsInfoIss = 				IMX662_IsiGetAutoFpsInfoIss;
        pIsiSensor->pIsiGetStartEvIss = 					IMX662_IsiGetStartEvIss;
        pIsiSensor->pIsiGetIrisIss =						IMX662_IsiGetIrisIss;
        pIsiSensor->pIsiSetIrisIss =						IMX662_IsiSetIrisIss;
        pIsiSensor->pIsiGetHCGIss =                         IMX662_IsiGetHCGIss;
        pIsiSensor->pIsiSetHCGIss =                         IMX662_IsiSetHCGIss;

        /* SENSOR ISP */
        pIsiSensor->pIsiGetIspStatusIss = IMX662_IsiGetIspStatusIss;
        // pIsiSensor->pIsiSetBlcIss                       =
        // IMX662_IsiSetBlcIss; pIsiSensor->pIsiSetWBIss = IMX662_IsiSetWBIss;

        /* SENSOE OTHER FUNC*/
        pIsiSensor->pIsiSetPowerIss = IMX662_IsiSetPowerIss;
        pIsiSensor->pIsiSetTpgIss = IMX662_IsiSetTpgIss;
        pIsiSensor->pIsiGetTpgIss = IMX662_IsiGetTpgIss;
        // pIsiSensor->pIsiGetExpandCurveIss               =
        // IMX662_IsiGetExpandCurveIss; pIsiSensor->pIsiGetCompressCurveIss =
        // IMX662_IsiGetCompressCurveIss; pIsiSensor->pIsiExtendFuncIss =
        // IMX662_IsiExtendFuncIss; pIsiSensor->pIsiGetOtpDataIss =
        // IMX662_IsiGetOtpDataIss;
        pIsiSensor->pIsiGetHdrBlankingLinesIss = IMX662_IsiGetHdrBlankingLinesIss;

        /* AF */
        pIsiSensor->pIsiFocusCreateIss = IMX662_IsiFocusCreateIss;
        pIsiSensor->pIsiFocusReleaseIss = IMX662_IsiFocusReleaseIss;
        pIsiSensor->pIsiFocusGetCalibrateIss = IMX662_IsiFocusGetCalibrateIss;
        pIsiSensor->pIsiFocusSetIss = IMX662_IsiFocusSetIss;
        pIsiSensor->pIsiFocusGetIss = IMX662_IsiFocusGetIss;
        pIsiSensor->pIsiSetAgainDgainIss = IMX662_IsiSetAgainDgainIss;

    } else {
        result = RET_NULL_POINTER;
    }

    TRACE(IMX662_INFO, "%s (exit)\n", __func__);
    return (result);
}

/*****************************************************************************
 * each sensor driver need declare this struct for isi load
 *****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x9012,
    .pIsiHalEnumModeIss = IMX662_IsiHalEnumModeIss,
    .pIsiGetSensorIss = IMX662_IsiGetSensorIss,
};
