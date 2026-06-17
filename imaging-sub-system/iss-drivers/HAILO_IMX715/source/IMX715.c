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

#include "IMX715_priv.h"
#include "vvsensor.h"
#include <linux/i2c.h>

CREATE_TRACER(IMX715_INFO, "IMX715: ", INFO, 1)
CREATE_TRACER(IMX715_WARN, "IMX715: ", WARNING, 1)
CREATE_TRACER(IMX715_ERROR, "IMX715: ", ERROR, 1)
CREATE_TRACER(IMX715_DEBUG, "IMX715: ", INFO, 1)
CREATE_TRACER(IMX715_REG_INFO, "IMX715: ", INFO, 1)
CREATE_TRACER(IMX715_REG_DEBUG, "IMX715: ", INFO, 1)

#define LOG_ERROR(...) TRACE(IMX715_ERROR, __VA_ARGS__)

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
#define IMX715_FSC_3DOL_HDR 6750
#define IMX715_VMAX_2DOL_HDR 2250
#define IMX715_FSC_2DOL_HDR 4500
#define IMX715_VMAX_MAX ((1 << 20) - 2)
#define IMX715_MIN_SHR 8
#define IMX715_MIN_LINES 1
#define IMX715_MAX_GAIN_AEC                                                    \
    (32.0f) /**< max. gain used by the AEC (arbitrarily chosen, hardware limit \
               = 62.0, driver limit = 32.0 ) */
#define IMX715_VS_MAX_INTEGRATION_TIME (0.0018)
#define IMX715_TRANSFER_BUFFER_LENGTH 3
#define IMX715_TRANSFER_BUFFER_LENGTH 3
#define IMX715_MAX_GAIN 3981
/* Sensor returns EREMOTEIO while in runtime_suspend; retry briefly
 * until power-on completes (~20ms). */
#define IMX715_I2C_READ_MAX_ATTEMPTS 3
#define IMX715_I2C_READ_RETRY_INTERVAL_US 10000

#define IMX715_SHR0_RHS2_GAP 7
#define IMX715_2DOL_SHR0_RHS1_GAP 9
#define IMX715_SHR0_FSC_GAP 8
#define IMX715_2DOL_SHR0_FSC_GAP 8
#define IMX715_3DOL_SHR1_RHS1_GAP 13
#define IMX715_2DOL_SHR1_MIN_GAP 9
#define IMX715_2DOL_SHR1_RHS1_GAP 8
#define IMX715_SHR2_RHS1_GAP 7
#define IMX715_SHR2_RHS2_GAP 3

#define IMX715_PIXEL_CLK_RATE 74.25
#define MICRO_2_NANO 1000
#define IMX715_2DOL_NUM_EXP 2
#define IMX715_3DOL_NUM_EXP 3
/* When required long EV is within this many lines of max_long_it, use unity gain to avoid quantization noise. */
#define IMX715_2DOL_MAX_IT_NEAR_LINES 2

#define IMX715_RES_4K_HEIGHT 2160
#define IMX715_RES_4K_WIDTH 3840
#define IMX715_RES_FHD_HEIGHT 1080
#define IMX715_RES_FHD_WIDTH 1920

#define SPI_IOC_MAGIC   'k'
#define HAILO15_IOC_GET_IRIS    _IOR(SPI_IOC_MAGIC, 1, int)
#define HAILO15_IOC_SET_IRIS    _IOW(SPI_IOC_MAGIC, 2, int)


FlickerModePeaksPerSec flickerPeaksPerSecMap[] = {
    { ISI_AE_ANTIBANDING_MODE_OFF, 0 },
    { ISI_AE_ANTIBANDING_MODE_50HZ, 100 },
    { ISI_AE_ANTIBANDING_MODE_60HZ, 120 },
    { ISI_AE_ANTIBANDING_MODE_AUTO, 100 }
};

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
			.bounds_width  = IMX715_RES_4K_WIDTH,
			.bounds_height = IMX715_RES_4K_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX715_RES_4K_WIDTH,
			.height        = IMX715_RES_4K_HEIGHT,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_GBRG,
	.ae_info = {
		.one_line_exp_time_ns = 7407,
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
			.bounds_width  = IMX715_RES_FHD_WIDTH,
			.bounds_height = IMX715_RES_FHD_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX715_RES_FHD_WIDTH,
			.height        = IMX715_RES_FHD_HEIGHT,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_GBRG,
	.ae_info = {
		.one_line_exp_time_ns = 7407,
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
			.bounds_width  = IMX715_RES_FHD_WIDTH,
			.bounds_height = IMX715_RES_FHD_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX715_RES_FHD_WIDTH,
			.height        = IMX715_RES_FHD_HEIGHT,
		},
		.fps       = 20 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_3DOL,
		.bit_width = 12,
		.bayer_pattern = BAYER_GBRG,
        .ae_info = {
		.one_line_exp_time_ns = 7407,
		.max_integration_time = 1, /* TODO: change max/min when supporting 3dol */
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
			.bounds_width  = IMX715_RES_4K_WIDTH,
			.bounds_height = IMX715_RES_4K_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX715_RES_4K_WIDTH,
			.height        = IMX715_RES_4K_HEIGHT,
		},
		.fps       = 20 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_3DOL,
		.bit_width = 12,
		.bayer_pattern = BAYER_GBRG,
        .ae_info = {
		.one_line_exp_time_ns = 7407,
		.max_integration_time = 1, /* TODO: change max/min when supporting 3dol */
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX715_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 20
	}
    },
    {
        .index     = 4,
        .size      ={
			.bounds_width  = IMX715_RES_4K_WIDTH,
			.bounds_height = IMX715_RES_4K_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX715_RES_4K_WIDTH,
			.height        = IMX715_RES_4K_HEIGHT,
		},
		.fps       = 30 * ISI_FPS_ACCURACY,
		.hdr_mode  = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_L_AND_S,
		.bit_width = 12,
		.bayer_pattern = BAYER_GBRG,
        .ae_info = {
		.one_line_exp_time_ns = 7407,
		.max_integration_time = 1,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX715_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    }
};

static RESULT IMX715_IsiSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    RESULT result = RET_SUCCESS;

    return (result);
}

static RESULT IMX715_GetCtrl(int sensor_fd, unsigned int ctrl_cid, int32_t *value)
{
	struct v4l2_control control;
	memset(&control, 0, sizeof(control));
	control.id = ctrl_cid;

	int ret = ioctl(sensor_fd, VIDIOC_G_CTRL, &control);
	if (ret < 0) {
		TRACE(IMX715_ERROR, "%s: VIDIOC_G_CTRL ioctl failed with error %d (%s). ret = %d\n",
			__func__, errno, strerror(errno), ret);
		return RET_FAILURE;
	}

	*value = control.value;
	return RET_SUCCESS;
}

static uint32_t IMX715_CalculateModeMaxIntegrationTime(
        uint32_t hdr_mode, uint32_t stitching_mode, uint32_t rhs1) {
	if (hdr_mode == SENSOR_MODE_LINEAR) {
		return IMX715_VMAX_30FPS - IMX715_MIN_SHR; // SDR
	} else {
        uint32_t rhs1_gap = (stitching_mode == SENSOR_STITCHING_3DOL) ?
            IMX715_3DOL_SHR1_RHS1_GAP : IMX715_2DOL_SHR1_RHS1_GAP;
        return rhs1 - rhs1_gap; // HDR
	}
}

static RESULT IMX715_SetSensorModeData(IMX715_Context_t* pIMX715Ctx, size_t index) {
    RESULT result = RET_SUCCESS;
    int sensor_fd = 0;
    int32_t rhs1 = 0;

    if (pIMX715Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    sensor_fd = ((HalContext_t*)pIMX715Ctx->IsiCtx.HalHandle)->sensor_fd;

    if (index >= ARRAY_SIZE(pimx715_mode_info)) {
        return RET_OUTOFRANGE;
    }

    pIMX715Ctx->SensorMode.index = index;
    memcpy(&pIMX715Ctx->SensorMode, &pimx715_mode_info[index], sizeof(pIMX715Ctx->SensorMode));
    if (pIMX715Ctx->SensorMode.index != index) {
        TRACE(IMX715_ERROR, "%s: Sensor mode index mismatch: expected %zu, got %u\n",
              __func__, index, pIMX715Ctx->SensorMode.index);
        return RET_FAILURE;
    }

    // If the mode is HDR, we need to get RHS1 value from the sensor.
    if (pIMX715Ctx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        /* Get the RHS1 value via a control rather then reading the register,
         * because the register is not available until a stream is running. */
        result = IMX715_GetCtrl(sensor_fd, IMX715_CID_RHS1, &rhs1);
        if ((result != RET_SUCCESS) || rhs1 == 0) {
            TRACE(IMX715_ERROR, "%s: Failed to get RHS1 value from sensor (result=%d)\n",
                __func__, result);
            return result;
        }
    }

    pIMX715Ctx->SensorMode.ae_info.max_integration_time =
        IMX715_CalculateModeMaxIntegrationTime(
            pIMX715Ctx->SensorMode.hdr_mode, pIMX715Ctx->SensorMode.stitching_mode, rhs1);
    TRACE(IMX715_INFO, "%s: Mode %zu: hdr_mode=%d, stitching_mode=%d, rhs1=%u, max_integration_time=%u\n",
          __func__, index, pIMX715Ctx->SensorMode.hdr_mode,
          pIMX715Ctx->SensorMode.stitching_mode, rhs1,
          pIMX715Ctx->SensorMode.ae_info.max_integration_time);
    if (pIMX715Ctx->SensorMode.ae_info.max_integration_time == 0) {
        TRACE(IMX715_ERROR, "%s: Invalid max integration time calculated (%u) for HDR mode %d, index %zu\n",
              __func__, pIMX715Ctx->SensorMode.ae_info.max_integration_time,
              pIMX715Ctx->SensorMode.hdr_mode, index);
        return RET_FAILURE;
    }

    return RET_SUCCESS;
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
    pIMX715Ctx->i2c_fd = -1;
    pIMX715Ctx->i2c_af_fd = -1;

    result = HalAddRef(pConfig->HalHandle);
    if (result != RET_SUCCESS) {
        TRACE(IMX715_ERROR, "%s: HalAddRef failed, result=%d\n", __func__, result);
        goto error_free_ctx;
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

    // By default, until specified otherwise, all ratios are 1 (SDR)
    pIMX715Ctx->hdr_ratio[0] = 1.0f;
    pIMX715Ctx->hdr_ratio[1] = 1.0f;
    
    result = IMX715_SetSensorModeData(pIMX715Ctx, pConfig->SensorModeIndex);
    if (result != RET_SUCCESS) {
        TRACE(IMX715_ERROR, "%s: Set sensor mode data failed! (%d)\n", __func__, result);
        goto error_hal_del_ref;
    }

    pConfig->hSensor = (IsiSensorHandle_t)pIMX715Ctx;
    pIMX715Ctx->pattern = ISI_BPAT_RGRGGBGB;
    pIMX715Ctx->subdev = HalGetFdHandle(pConfig->HalHandle,
                                        HAL_MODULE_SENSOR);  // two sensors??
    pIMX715Ctx->KernelDriverFlag = 1;
    TRACE(IMX715_INFO, "%s - sensor i2c bus: %d, af i2c bus: %d, sensor i2c addr: 0x%x, af i2c addr: 0x%x\n", __func__,
           pConfig->I2cBusNum, pConfig->I2cAfBusNum, pConfig->SlaveAddr, pConfig->SlaveAfAddr);
    pIMX715Ctx->i2c_addr = pConfig->SlaveAddr;
    pIMX715Ctx->i2c_af_addr = pConfig->SlaveAfAddr;

    sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cBusNum);
    pIMX715Ctx->i2c_fd = open(i2c_file_path, O_RDWR);
    if (pIMX715Ctx->i2c_fd < 0) {
        TRACE(IMX715_ERROR, "%s: unable to open /dev/i2c-%d, errno=%d (%s)\n",
              __func__, pConfig->I2cBusNum, errno, strerror(errno));
        goto error_hal_del_ref;
    }

    if (ioctl(pIMX715Ctx->i2c_fd, I2C_SLAVE_FORCE, pIMX715Ctx->i2c_addr) < 0) {
        TRACE(IMX715_ERROR, "%s: unable to set I2C_SLAVE_FORCE on /dev/i2c-%d, errno=%d (%s)\n",
              __func__, pConfig->I2cBusNum, errno, strerror(errno));
        goto error_close_i2c_fd;
    }

    if (pConfig->I2cAfBusNum < 0) {
        TRACE(IMX715_INFO, "no af i2c bus\n");
        pIMX715Ctx->SensorMode.af_mode = ISI_SENSOR_AF_MODE_NOTSUPP;
    } else {
        sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cAfBusNum);
        pIMX715Ctx->i2c_af_fd = open(i2c_file_path, O_RDWR);
        if (pIMX715Ctx->i2c_af_fd < 0) {
            TRACE(IMX715_ERROR, "%s: unable to open /dev/i2c-%d for AF, errno=%d (%s)\n",
                  __func__, pConfig->I2cAfBusNum, errno, strerror(errno));
            goto error_close_i2c_fd;
        }
        pIMX715Ctx->SensorMode.af_mode = ISI_SENSOR_AF_MODE_CDAF;
    }

    return (result);

error_close_i2c_fd:
    close(pIMX715Ctx->i2c_fd);
error_hal_del_ref:
    (void)HalDelRef(pConfig->HalHandle);
error_free_ctx:
    free(pIMX715Ctx);
    pIMX715Ctx = NULL;
    return RET_FAILURE;
}

static RESULT IMX715_IsiReleaseIss(IsiSensorHandle_t handle) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX715Ctx == NULL) return (RET_WRONG_HANDLE);

    result = IMX715_IsiSetStreamingIss(pIMX715Ctx, BOOL_FALSE);
    CHECK_RESULT_RET(result, "SetStreaming off");
    result = IMX715_IsiSetPowerIss(pIMX715Ctx, BOOL_FALSE);
    CHECK_RESULT_RET(result, "SetPower off");
    (void)HalDelRef(pIMX715Ctx->IsiCtx.HalHandle);
    if (pIMX715Ctx->i2c_af_fd >= 0) {
        close(pIMX715Ctx->i2c_af_fd);
    }
    close(pIMX715Ctx->i2c_fd);
    MEMSET(pIMX715Ctx, 0, sizeof(IMX715_Context_t));
    free(pIMX715Ctx);
    pIMX715Ctx = NULL;
    return (result);
}

static RESULT IMX715_IsiReadRegIss(IsiSensorHandle_t handle,
                                   const uint32_t Addr, uint32_t* pValue) {
    RESULT result = RET_SUCCESS;    
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    struct i2c_rdwr_ioctl_data ioctl_data;
    unsigned char out[IMX715_TRANSFER_BUFFER_LENGTH];
    struct i2c_msg msgs[2];
    uint8_t addr_buf[2] = { (Addr >> 8) & 0xff, Addr & 0xff };

    if (pIMX715Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    msgs[0].addr = pIMX715Ctx->i2c_addr;
    msgs[0].flags = 0; // Write
    msgs[0].len = sizeof(addr_buf);
    msgs[0].buf = addr_buf;

    msgs[1].addr = pIMX715Ctx->i2c_addr;
    msgs[1].flags = I2C_M_RD; // Read
    msgs[1].len = 1;
    msgs[1].buf = out;

    ioctl_data.msgs = msgs;
    ioctl_data.nmsgs = 2;

    int ret = 0;
    int attempt;
    for (attempt = 0; attempt < IMX715_I2C_READ_MAX_ATTEMPTS; attempt++) {
        ret = ioctl(pIMX715Ctx->i2c_fd, I2C_RDWR, &ioctl_data);
        if (ret >= 0)
            break;
        if (errno != EREMOTEIO)
            break;
        usleep(IMX715_I2C_READ_RETRY_INTERVAL_US);
    }
    if (ret < 0) {
        TRACE(IMX715_ERROR, "%s: ioctl I2C_RDWR failed, errno=%d (%s) after %d attempt(s)\n",
              __func__, errno, strerror(errno), attempt + 1);
        return RET_FAILURE;
    }
    if (attempt > 0) {
        TRACE(IMX715_INFO, "%s: I2C_RDWR ioctl recovered after %d EREMOTEIO retries (addr 0x%04x)\n",
              __func__, attempt, Addr);
    }

    *pValue = out[0];

    return (result);
}

static RESULT IMX715_IsiWriteRegIss(IsiSensorHandle_t handle,
                                    const uint32_t Addr, const uint32_t Value) {
    RESULT result = RET_SUCCESS;
    char out[IMX715_TRANSFER_BUFFER_LENGTH];
    ssize_t write_ret;

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX715_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    out[2] = Value;
    write_ret = write(pIMX715Ctx->i2c_fd, out, sizeof(out));
    if (write_ret != sizeof(out)) {
        if (write_ret < 0) {
            TRACE(IMX715_ERROR, "%s: I2C write failed with error %d (%s)\n",
                  __func__, errno, strerror(errno));
        } else {
            TRACE(IMX715_ERROR, "%s: I2C write incomplete. Wrote %zd of %zu bytes\n",
                  __func__, write_ret, sizeof(out));
        }
        result = RET_FAILURE;
    }
    return (result);
}

static RESULT IMX715_UpdateFps(IMX715_Context_t *pIMX715Ctx, uint32_t vmax) {
    float frame_time = 0;
    frame_time = (vmax * pIMX715Ctx->one_line_exp_time);
    if (frame_time == 0) {
        TRACE(IMX715_ERROR, "%s: frame_time is 0, vmax=%u, one_line_exp_time=%f\n",
              __func__, vmax, pIMX715Ctx->one_line_exp_time);
        return RET_FAILURE;
    }

    pIMX715Ctx->CurrFps = (uint32_t)(ceil(1 / frame_time)) * ISI_FPS_ACCURACY;
    return RET_SUCCESS;
}

static RESULT IMX715_ReadVmax(IsiSensorHandle_t handle, uint32_t* vmax) {
    uint32_t vmax_low = 0, vmax_mid = 0, vmax_high = 0;
    RESULT result;

    result = IMX715_IsiReadRegIss(handle, 0x3024, &vmax_low);
    CHECK_RESULT_RET(result, "ReadVmax low");
    result = IMX715_IsiReadRegIss(handle, 0x3025, &vmax_mid);
    CHECK_RESULT_RET(result, "ReadVmax mid");
    result = IMX715_IsiReadRegIss(handle, 0x3026, &vmax_high);
    CHECK_RESULT_RET(result, "ReadVmax high");

    *vmax = (vmax_high << 16) | (vmax_mid << 8) | vmax_low;
    return RET_SUCCESS;
}

static RESULT IMX715_ReadHmax(IsiSensorHandle_t handle, uint32_t* hmax) {
    uint32_t hmax_low = 0, hmax_high = 0;
    RESULT result;

    result = IMX715_IsiReadRegIss(handle, 0x3028, &hmax_low);
    CHECK_RESULT_RET(result, "ReadHmax low");
    result = IMX715_IsiReadRegIss(handle, 0x3029, &hmax_high);
    CHECK_RESULT_RET(result, "ReadHmax high");

    *hmax = (hmax_high << 8) | hmax_low;
    return RET_SUCCESS;
}

static RESULT IMX715_WriteVmax(IsiSensorHandle_t handle, uint32_t vmax) {
    RESULT result;
    result = IMX715_IsiWriteRegIss(handle, 0x3024, vmax & 0xff);
    CHECK_RESULT_RET(result, "WriteVmax low");
    result = IMX715_IsiWriteRegIss(handle, 0x3025, (vmax >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteVmax mid");
    result = IMX715_IsiWriteRegIss(handle, 0x3026, (vmax >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteVmax high");

    result = IMX715_UpdateFps((IMX715_Context_t *)handle, vmax);
    CHECK_RESULT_RET(result, "UpdateFps");
    return result;
}

static RESULT IMX715_ReadRHS1(IsiSensorHandle_t handle, uint32_t* rhs1) {
    uint32_t rhs1_low = 0, rhs1_mid = 0, rhs1_high = 0;
    RESULT result;

    result = IMX715_IsiReadRegIss(handle, 0x3060, &rhs1_low);
    CHECK_RESULT_RET(result, "ReadRHS1 low");
    result = IMX715_IsiReadRegIss(handle, 0x3061, &rhs1_mid);
    CHECK_RESULT_RET(result, "ReadRHS1 mid");
    result = IMX715_IsiReadRegIss(handle, 0x3062, &rhs1_high);
    CHECK_RESULT_RET(result, "ReadRHS1 high");

    *rhs1 = (rhs1_high << 16) | (rhs1_mid << 8) | rhs1_low;
    return RET_SUCCESS;
}

static RESULT IMX715_ReadRHS2(IsiSensorHandle_t handle, uint32_t* rhs2) {
    uint32_t rhs2_low = 0, rhs2_mid = 0, rhs2_high = 0;
    RESULT result;

    result = IMX715_IsiReadRegIss(handle, 0x3064, &rhs2_low);
    CHECK_RESULT_RET(result, "ReadRHS2 low");
    result = IMX715_IsiReadRegIss(handle, 0x3065, &rhs2_mid);
    CHECK_RESULT_RET(result, "ReadRHS2 mid");
    result = IMX715_IsiReadRegIss(handle, 0x3066, &rhs2_high);
    CHECK_RESULT_RET(result, "ReadRHS2 high");

    *rhs2 = (rhs2_high << 16) | (rhs2_mid << 8) | rhs2_low;
    return RET_SUCCESS;
}

static RESULT IMX715_WriteShr0(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;
    result = IMX715_IsiWriteRegIss(handle, 0x3050, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr0 low");
    result = IMX715_IsiWriteRegIss(handle, 0x3051, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr0 mid");
    result = IMX715_IsiWriteRegIss(handle, 0x3052, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr0 high");

    return RET_SUCCESS;
}

static RESULT IMX715_WriteShr1(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3054, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr1 low");
    result = IMX715_IsiWriteRegIss(handle, 0x3055, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr1 mid");
    result = IMX715_IsiWriteRegIss(handle, 0x3056, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr1 high");

    return RET_SUCCESS;
}

static RESULT IMX715_WriteShr2(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3058, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr2 low");
    result = IMX715_IsiWriteRegIss(handle, 0x3059, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr2 mid");
    result = IMX715_IsiWriteRegIss(handle, 0x305a, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr2 high");

    return RET_SUCCESS;
}

static RESULT IMX715_WriteGain(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3090, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain low");
    result = IMX715_IsiWriteRegIss(handle, 0x3091, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain high");

    return RET_SUCCESS;
}

static RESULT IMX715_WriteGain1(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3092, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain1 low");
    result = IMX715_IsiWriteRegIss(handle, 0x3093, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain1 high");

    return RET_SUCCESS;
}

static RESULT IMX715_WriteGain2(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3094, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain2 low");
    result = IMX715_IsiWriteRegIss(handle, 0x3095, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain2 high");

    return RET_SUCCESS;
}

static RESULT IMX715_LockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3001, 0x1);
    CHECK_RESULT_RET(result, "LockRegHold");

    return RET_SUCCESS;
}

static RESULT IMX715_UnlockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX715_IsiWriteRegIss(handle, 0x3001, 0x0);
    CHECK_RESULT_RET(result, "UnlockRegHold");

    return RET_SUCCESS;
}

static RESULT IMX715_IsiGetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    TRACE(IMX715_DEBUG, "%s (enter)\n", __func__);
    
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    memcpy(pMode, &(pIMX715Ctx->SensorMode), sizeof(pIMX715Ctx->SensorMode));

    TRACE(IMX715_DEBUG, "%s (exit)\n", __func__);
    return (RET_SUCCESS);
}

static RESULT IMX715_IsiSetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    int ret = 0;
    TRACE(IMX715_DEBUG, "%s (enter)\n", __func__);

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX715Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, pMode);
    if (ret != 0) {
        TRACE(IMX715_ERROR, "%s: ioctl VVSENSORIOC_S_SENSOR_MODE failed, errno=%d (%s)\n",
              __func__, errno, strerror(errno));
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
            TRACE(IMX715_DEBUG, "%s (exit)\n", __func__);
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
    CHECK_RESULT_RET(result, "sensor enum mode");

    return result;
}

static RESULT IMX715_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t* pCaps) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;

    RESULT result = RET_SUCCESS;

    TRACE(IMX715_DEBUG, "%s (enter)\n", __func__);

    if (pIMX715Ctx == NULL) return (RET_WRONG_HANDLE);

    if (pCaps == NULL) {
        return (RET_NULL_POINTER);
    }

    if (!pIMX715Ctx->Configured) {
        result = IMX715_IsiSetupIss(handle, pCaps);
        CHECK_RESULT_RET(result, "Setup");
    }

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
    TRACE(IMX715_DEBUG, "%s (exit)\n", __func__);
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

    TRACE(IMX715_DEBUG, "%s (enter)\n", __func__);

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

    pIMX715Ctx->unlimit_fps_vmax_changed = 0;
    pIMX715Ctx->flicker_fps_mode = ISI_AE_ANTIBANDING_MODE_OFF;

    TRACE(IMX715_INFO, "%s - got caps - width %d height %d buswidth %d\n",
          __func__, pIMX715Ctx->SensorMode.size.width,
          pIMX715Ctx->SensorMode.size.height, pIMX715Ctx->SensorMode.bit_width);

    TRACE(IMX715_INFO, "%s - MinGain %f MaxGain %f\n", __func__, pIMX715Ctx->AecMinGain,
          pIMX715Ctx->AecMaxGain);

    memcpy(&pIMX715Ctx->CapsConfig, pCaps, sizeof(pIMX715Ctx->CapsConfig));

    /* SW reset comment preserved - no actual reset code was present,
     * and the 100ms sleep was unnecessary (no hardware access in this function). */

    result = IMX715_AecSetModeParameters(pIMX715Ctx, pCaps);
    CHECK_RESULT_RET(result, "AecSetModeParameters");

    pIMX715Ctx->Configured = BOOL_TRUE;
    TRACE(IMX715_DEBUG, "%s: (exit)\n", __func__);
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
        CHECK_RESULT_RET(result, "read sensor ID high");
        sensor_id = (reg_val & 0xff) << 8;

        reg_val = 0;
        result = IMX715_IsiReadRegIss(handle, 0x3a05, &reg_val);
        CHECK_RESULT_RET(result, "read sensor ID low");
        sensor_id |= (reg_val & 0xff);
    }

    *pValue = sensor_id;
    TRACE(IMX715_DEBUG, "%s (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX715_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t is_on) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX715_DEBUG, "%s (enter)\n", __func__);

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    pIMX715Ctx->Streaming = is_on;

    if (pIMX715Ctx->enableHdr)
        return result;

    TRACE(IMX715_DEBUG, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX715_IsiGetGainLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinGain, float* pMaxGain) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX715_DEBUG, "%s: (enter)\n", __func__);    

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

    TRACE(IMX715_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX715_GetNumExposures(IMX715_Context_t* pIMX715Ctx, size_t* num_exposures) {
    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    if (num_exposures == NULL) {
        TRACE(IMX715_ERROR, "%s: Invalid output parameter\n", __func__);
        return RET_NULL_POINTER;
    }

    if (pIMX715Ctx->SensorMode.hdr_mode == SENSOR_MODE_LINEAR) {
        *num_exposures = 1; // SDR
    } else if (pIMX715Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        *num_exposures = IMX715_2DOL_NUM_EXP;
    } else if (pIMX715Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        *num_exposures = IMX715_3DOL_NUM_EXP;
    } else {
        TRACE(IMX715_ERROR, "%s: Unsupported HDR mode %d with stitching mode %d\n",
            __func__, pIMX715Ctx->SensorMode.hdr_mode, pIMX715Ctx->SensorMode.stitching_mode);
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

static RESULT IMX715_UpdateCurrLEFIntegrationTimeFromFsc(IMX715_Context_t* pIMX715Ctx, uint32_t fsc, uint32_t shr0) {
    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    float configuredIntegrationTime = (fsc - shr0) * pIMX715Ctx->one_line_exp_time;

    pIMX715Ctx->OldIntegrationTime = configuredIntegrationTime;
    pIMX715Ctx->AecCurIntegrationTimeLEF = configuredIntegrationTime;

    TRACE(IMX715_DEBUG, "%s: Updated LEF Integration Time = (fsc[%u] - shr0[%u]) * one_line_exp_time[%f] = %f\n",
          __func__, fsc, shr0, pIMX715Ctx->one_line_exp_time, configuredIntegrationTime);
    return RET_SUCCESS;
}

static RESULT IMX715_UpdateCurrLEFIntegrationTimeFromVmax(IMX715_Context_t* pIMX715Ctx, uint32_t vmax, uint32_t shr0) {
    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    size_t dol = 0;
    RESULT result = IMX715_GetNumExposures(pIMX715Ctx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");
    uint32_t fsc = vmax * dol;
    TRACE(IMX715_DEBUG, "%s: fsc = vmax[%u] * dol[%zu] = %u\n", __func__, vmax, dol, fsc);
    result = IMX715_UpdateCurrLEFIntegrationTimeFromFsc(pIMX715Ctx, fsc, shr0);
    CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromFsc");
    return result;
}

static inline int IMX715_getFlickerPeaksPerSec(IsiSensorAntibandingMode_t mode) {
    int num_modes = sizeof(flickerPeaksPerSecMap) / sizeof(FlickerModePeaksPerSec);
    for (int i = 0; i < num_modes; i++) {
        if (flickerPeaksPerSecMap[i].mode == mode) {
            return flickerPeaksPerSecMap[i].value;
        }
    }
    return 0; // Defaults to 0 if mode not found
}

static inline RESULT IMX715_getNewVmaxAntiFlicker(IMX715_Context_t *pIMX715Ctx, uint32_t requestedVmax, uint32_t *outClosestVmax) {
    uint32_t closestVmax = requestedVmax;
    int peaks = 0;
    int difference = INT_MAX;
    int minDifference = INT_MAX;
    if (!pIMX715Ctx) {
        TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return RET_FAILURE;
    }
    peaks = IMX715_getFlickerPeaksPerSec(pIMX715Ctx->flicker_fps_mode);
    if (peaks == 0) {
        // No anti-flicker mode, return requested or original Vmax
        if (pIMX715Ctx->unlimit_fps && pIMX715Ctx->unlimit_fps_vmax_changed)
            *outClosestVmax = requestedVmax;
        else
            *outClosestVmax = pIMX715Ctx->original_vmax;
        return RET_SUCCESS;
    }

    for (int i = 1; i < peaks; ++i) {
        uint32_t candidateVmax = i / (peaks * pIMX715Ctx->one_line_exp_time);
        if (candidateVmax < pIMX715Ctx->SensorMode.size.height || candidateVmax < pIMX715Ctx->original_vmax)
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
    uint32_t new_vmax = 0;
    
    TRACE(IMX715_DEBUG, "%s (enter)\n", __func__);

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (!pIMX715Ctx) 
        return RET_NULL_POINTER;

    if (pIMX715Ctx->enableHdr)
        return result;
    
    // Working around overriding VMAX issue with reset(?) value
    if (pIMX715Ctx->Streaming == BOOL_FALSE)
        return result;

    result = IMX715_ReadVmax(handle, &current_vmax);
    CHECK_RESULT_RET(result, "ReadVmax");

    pIMX715Ctx->unlimit_fps = 0;
    pIMX715Ctx->unlimit_fps_vmax_changed = 0;
    if (pIMX715Ctx->original_vmax == 0) {
        pIMX715Ctx->original_vmax = current_vmax;
    }

    new_vmax = pIMX715Ctx->original_vmax;
    if (pIMX715Ctx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
        result = IMX715_getNewVmaxAntiFlicker(pIMX715Ctx, pIMX715Ctx->original_vmax, &new_vmax);
        CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
        TRACE(IMX715_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX715Ctx->flicker_fps_mode, new_vmax);
    }

    if (current_vmax != new_vmax) {
        result = IMX715_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        result = IMX715_WriteVmax(handle, new_vmax);
        CHECK_RESULT_RET(result, "WriteVmax");
        result = IMX715_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");


        int shr = MAX((int)current_vmax - (int)(pIMX715Ctx->AecCurIntegrationTimeLEF / pIMX715Ctx->one_line_exp_time), IMX715_MIN_SHR);
        result = IMX715_UpdateCurrLEFIntegrationTimeFromVmax(pIMX715Ctx, new_vmax, shr);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromVmax");
    }

    pIMX715Ctx->MaxIntegrationLine =
        MAX(new_vmax - IMX715_MIN_SHR, 1);
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

    TRACE(IMX715_DEBUG, "%s: (enter)\n", __func__);

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

    TRACE(IMX715_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX715_IsiSetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float minIris, float maxIris) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX715_DEBUG, "%s: (enter)\n", __func__);

    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    pIMX715Ctx->AecMinIris = minIris;
    pIMX715Ctx->AecMaxIris = maxIris;

    TRACE(IMX715_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX715_IsiGetIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float* pMinIntegrationTime,
    float* pMaxIntegrationTime) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    float max_short_it, min_short_it;
    RESULT result = RET_SUCCESS;
    int rhs1 = -1;
    HalContext_t* pHalCtx = NULL;

    TRACE(IMX715_DEBUG, "%s: (enter). prev values: min: %f, max: %f\n", __func__, pIMX715Ctx->AecMinIntegrationTime, pIMX715Ctx->AecMaxIntegrationTime);

    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX715_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMX715Ctx->enableHdr) {
        *pMinIntegrationTime = pIMX715Ctx->AecMinIntegrationTime;
        *pMaxIntegrationTime = pIMX715Ctx->AecMaxIntegrationTime;
    
        TRACE(IMX715_INFO, "%s: (exit - SDR) %f, %f\n", 
        __func__, *pMinIntegrationTime, *pMaxIntegrationTime);

        return RET_SUCCESS;
    }

    pHalCtx = (HalContext_t*)pIMX715Ctx->IsiCtx.HalHandle;
    if (!pHalCtx) {
        TRACE(IMX715_ERROR,
              "%s: Invalid HAL handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIMX715Ctx->cur_rhs1 == 0) {
        result = IMX715_GetCtrl(pHalCtx->sensor_fd, IMX715_CID_RHS1, &rhs1);
        CHECK_RESULT_RET(result, "GetCtrl RHS1");
        pIMX715Ctx->cur_rhs1 = (uint32_t)rhs1;
    }

    if (pIMX715Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        max_short_it = (pIMX715Ctx->cur_rhs1 - IMX715_2DOL_SHR1_MIN_GAP) * pIMX715Ctx->one_line_exp_time;
        min_short_it = IMX715_2DOL_SHR1_RHS1_GAP * pIMX715Ctx->one_line_exp_time;
    } else {
        /* TODO: implement 3dol exposure */
        max_short_it = (pIMX715Ctx->cur_rhs1 - IMX715_2DOL_SHR1_MIN_GAP) * pIMX715Ctx->one_line_exp_time;
        min_short_it = IMX715_2DOL_SHR1_RHS1_GAP * pIMX715Ctx->one_line_exp_time;
    }

    /*
     * Report the actual SEF1 hardware limits.  The ratio is enforced inside
     * Calculate2DOLExposures / Calculate3DOLExposures which will maximise the
     * LEF integration time and apply LEF gain when needed to meet the ratio.
     */
    pIMX715Ctx->AecMinIntegrationTime = min_short_it;
    pIMX715Ctx->AecMaxIntegrationTime = max_short_it;

    *pMinIntegrationTime = pIMX715Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMX715Ctx->AecMaxIntegrationTime;

    TRACE(IMX715_INFO, "%s: (exit - HDR). min: %f, max: %f\n",  __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

static RESULT IMX715_IsiGetAbsoluteIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float* pMinIntegrationTime,
    float* pMaxIntegrationTime) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX715_DEBUG, "%s: (enter)\n", __func__);
    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX715_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIntegrationTime = IMX715_MIN_LINES * pIMX715Ctx->one_line_exp_time;
    *pMaxIntegrationTime =  (IMX715_VMAX_MAX - IMX715_MIN_SHR) * pIMX715Ctx->one_line_exp_time;

    TRACE(IMX715_DEBUG, "%s: (exit) %f, %f\n", 
    __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

/* Gain get functions*/

RESULT IMX715_IsiGetGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	TRACE(IMX715_DEBUG, "%s: (enter)\n", __func__);

	if (pIMX715Ctx == NULL) {
		TRACE(IMX715_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	if (pIMX715Ctx->enableHdr) {
		RESULT result = IMX715_IsiGetSEF1GainIss(handle, pSetGain);
		CHECK_RESULT_RET(result, "GetSEF1Gain");
		return result;
	}

	RESULT result = IMX715_IsiGetLEFGainIss(handle, pSetGain);
	CHECK_RESULT_RET(result, "GetLEFGain");
	return result;
}

RESULT IMX715_IsiGetLEFGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	TRACE(IMX715_DEBUG, "%s: (enter)\n", __func__);

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
	TRACE(IMX715_DEBUG, "%s: (exit)\n", __func__);
	return (result);
}

// HDR has not been tested yet
RESULT IMX715_IsiGetSEF1GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX715_DEBUG, "%s: (enter)\n", __func__);

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

	TRACE(IMX715_DEBUG, "%s: (exit)\n", __func__);

	return (result);
}

// HDR has not been tested yet
RESULT IMX715_IsiGetSEF2GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX715_DEBUG, "%s: (enter)\n", __func__);

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

	TRACE(IMX715_DEBUG, "%s: (exit)\n", __func__);

	return (result);
}

RESULT IMX715_IsiGetGainIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX715_DEBUG, "%s: (enter)\n", __func__);

    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = pIMX715Ctx->AecGainIncrement;

    TRACE(IMX715_DEBUG, "%s: (exit)\n", __func__);

    return (result);
}

RESULT IMX715_IsiGetIrisIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX715_DEBUG, "%s: (enter)\n", __func__);
    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = 0.0001;

    TRACE(IMX715_DEBUG, "%s: (exit)\n", __func__);

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
		CHECK_RESULT_RET(result, "SetSEF1Gain");

		result = IMX715_IsiSetSEF2GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF2Gain");
	}
	result = IMX715_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
	CHECK_RESULT_RET(result, "SetLEFGain");
	return RET_SUCCESS;
}

static inline uint32_t _linear2sensorGain(float gain)
{    
    uint32_t db = 0;
    float log_gain = log10(gain);
    log_gain = (log_gain * 10 * 20) / 3;
    db = (uint32_t)(log_gain);
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
    // This will break the hdr ratios in this extreme case.
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

    result = IMX715_LockRegHold(handle);
    CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX715_WriteGain(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain");
	result = IMX715_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pIMX715Ctx->AecCurGainLEF = _sensorGain2linear(Gain);
	*pSetGain = pIMX715Ctx->AecCurGainLEF;
	TRACE(IMX715_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return RET_SUCCESS;
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

	result = IMX715_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX715_WriteGain1(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain1");
	result = IMX715_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pIMX715Ctx->AecCurGainSEF1 = _sensorGain2linear(Gain);
	*pSetGain = pIMX715Ctx->AecCurGainSEF1;

	TRACE(IMX715_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return RET_SUCCESS;
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

	result = IMX715_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX715_WriteGain2(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain2");
	result = IMX715_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pIMX715Ctx->AecCurGainSEF2 = _sensorGain2linear(Gain);
	*pSetGain = pIMX715Ctx->AecCurGainSEF2;

	TRACE(IMX715_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return RET_SUCCESS;
}

/* Integration Time get functions*/

RESULT IMX715_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;

	if (!pIMX715Ctx) {
		TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	
	TRACE(IMX715_DEBUG, "%s - enter\n", __func__);
	
	if (pIMX715Ctx->enableHdr) {
		RESULT result = IMX715_IsiGetSEF1IntegrationTimeIss(handle, pSetIntegrationTime);
		CHECK_RESULT_RET(result, "GetSEF1IntegrationTime");
		return result;
	}
	
	RESULT result = IMX715_IsiGetLEFIntegrationTimeIss(handle, pSetIntegrationTime);
	CHECK_RESULT_RET(result, "GetLEFIntegrationTime");
	return result;
}

RESULT IMX715_IsiGetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX715_Context_t *pIMX715Ctx = (IMX715_Context_t *)handle;
	RESULT result = RET_SUCCESS;


	if (!pIMX715Ctx) {
		TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
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
		TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
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
		TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
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
        TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
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
		TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX715Ctx->enableHdr) {
		RESULT result = IMX715_IsiSetSEF1IntegrationTimeIss(
			handle, NewIntegrationTime, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1IntegrationTime");
		return result;
	}

	RESULT result = IMX715_IsiSetLEFIntegrationTimeIss(
		handle, NewIntegrationTime, pSetIntegrationTime,
		pNumberOfFramesToSkip, hdr_ratio);
	CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
	return result;
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
    uint32_t rhs1;
    uint32_t rhs2;
    if (!pIMX715Ctx) {
        TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pSetIntegrationTime || !pNumberOfFramesToSkip) {
        TRACE(IMX715_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
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

            rhs1 = pIMX715Ctx->cur_rhs1;
            rhs2 = pIMX715Ctx->cur_rhs2;
            if (pIMX715Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
                result = IMX715_ReadVmax(pIMX715Ctx, &new_vmax);
                CHECK_RESULT_RET(result, "ReadVmax");
                new_vmax *= IMX715_2DOL_NUM_EXP;

                exp = new_vmax - exp;
                exp = exp > rhs1 + IMX715_2DOL_SHR0_RHS1_GAP ? exp : rhs1 + IMX715_2DOL_SHR0_RHS1_GAP;
                exp = exp < new_vmax - IMX715_2DOL_SHR0_FSC_GAP? exp : new_vmax - IMX715_2DOL_SHR0_FSC_GAP;
                shr = exp;
            } else {
                new_vmax = IMX715_FSC_3DOL_HDR;
                exp = new_vmax - exp;
                exp = exp > rhs2 + IMX715_SHR0_RHS2_GAP ? exp : rhs2 + IMX715_SHR0_RHS2_GAP;
                exp = exp < new_vmax - IMX715_SHR0_FSC_GAP? exp : new_vmax - IMX715_SHR0_FSC_GAP;
                shr = exp;
            }
		} else {
            if (exp > pIMX715Ctx->MaxIntegrationLine || exp == 0) {
                TRACE(IMX715_ERROR, "%s: Integration time %f (exp %u) out of range (%u)\n", __func__,
                    NewIntegrationTime, exp, pIMX715Ctx->MaxIntegrationLine);
                return RET_FAILURE;
            }
            result = IMX715_ReadVmax(handle, &current_vmax);
            CHECK_RESULT_RET(result, "ReadVmax");

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
            if (shr < IMX715_MIN_SHR || current_vmax > pIMX715Ctx->original_vmax) {
                if (shr < IMX715_MIN_SHR) {
                    // user asked for too big integration-line: more than (current_vmax - minimal_shutter)
                    // increase vmax and set new shutter to minimal value
                    new_vmax = MIN(exp + IMX715_MIN_SHR, pIMX715Ctx->MaxIntegrationLine + IMX715_MIN_SHR);
                } else {
                    // make sure that when we unlimit fps, stay below original fps.
                    // also, make sure we are not using high vmax + high shr more than needed (will slow fps for no reason)
                    new_vmax = MAX(current_vmax - shr + IMX715_MIN_SHR, pIMX715Ctx->original_vmax);
                }
                pIMX715Ctx->unlimit_fps_vmax_changed = new_vmax > pIMX715Ctx->original_vmax && pIMX715Ctx->unlimit_fps;

                if (pIMX715Ctx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
                    result = IMX715_getNewVmaxAntiFlicker(pIMX715Ctx, new_vmax, &new_vmax);
                    CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
                    TRACE(IMX715_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX715Ctx->flicker_fps_mode, new_vmax);
                }

                shr = MAX(IMX715_MIN_SHR, (int)new_vmax - (int)exp);
                vmax_updated = 1;

            } else {
                new_vmax = current_vmax;
            }
        }

        result = IMX715_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        if (vmax_updated && pIMX715Ctx->unlimit_fps && !pIMX715Ctx->enableHdr) {
            result = IMX715_WriteVmax(handle, new_vmax);
            CHECK_RESULT_RET(result, "WriteVmax");
        }

        TRACE(IMX715_DEBUG, "%s - writing 0x%x to SHR0\n", __func__, shr);
        result = IMX715_WriteShr0(handle, shr);
        CHECK_RESULT_RET(result, "WriteShr0");
        result = IMX715_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");

        // In this context, the "new_vmax" is actually the FSC (multiplied by DOL), not the VMAX.
        result = IMX715_UpdateCurrLEFIntegrationTimeFromFsc(pIMX715Ctx, new_vmax, shr);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromFsc");

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
	uint32_t shr1;
	uint32_t rhs1;

	if (!pIMX715Ctx) {
		TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX715Ctx->cur_rhs1 == 0 || pIMX715Ctx->cur_rhs2 == 0) {
		TRACE(IMX715_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX715Ctx->cur_rhs1;

	if (!pSetIntegrationTimeSEF1 || !pNumberOfFramesToSkip) {
		TRACE(IMX715_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX715_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = (NewIntegrationTime / pIMX715Ctx->one_line_exp_time);
	TRACE(IMX715_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX715Ctx->AecCurIntegrationTimeSEF1) > FLT_EPSILON) {
		shr1 = rhs1 - exp;
		shr1 = shr1 > IMX715_2DOL_SHR1_MIN_GAP ? shr1 : IMX715_2DOL_SHR1_MIN_GAP;
		shr1 = shr1 < rhs1 - IMX715_2DOL_SHR1_RHS1_GAP ? shr1 : rhs1 - IMX715_2DOL_SHR1_RHS1_GAP;
		TRACE(IMX715_DEBUG, "%s - writing 0x%x to SHR1\n", __func__, shr1);

		result = IMX715_LockRegHold(handle);
		CHECK_RESULT_RET(result, "LockRegHold");
		result = IMX715_WriteShr1(handle, shr1);
		CHECK_RESULT_RET(result, "WriteShr1");
		result = IMX715_UnlockRegHold(handle);
		CHECK_RESULT_RET(result, "UnlockRegHold");

		pIMX715Ctx->AecCurIntegrationTimeSEF1 = (rhs1 - shr1) * pIMX715Ctx->one_line_exp_time; // in sec
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
		TRACE(IMX715_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX715Ctx->cur_rhs1;
	rhs2 = pIMX715Ctx->cur_rhs2;

	if (!pIMX715Ctx) {
		TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTimeSEF2 || !pNumberOfFramesToSkip) {
		TRACE(IMX715_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
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

		result = IMX715_LockRegHold(handle);
		CHECK_RESULT_RET(result, "LockRegHold");
		result = IMX715_WriteShr2(handle, exp);
		CHECK_RESULT_RET(result, "WriteShr2");
		result = IMX715_UnlockRegHold(handle);
		CHECK_RESULT_RET(result, "UnlockRegHold");

		pIMX715Ctx->AecCurIntegrationTimeSEF2 = (rhs2 - exp) * pIMX715Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF2 = pIMX715Ctx->AecCurIntegrationTimeSEF2;

	TRACE(IMX715_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF2);
	return (result);
}

RESULT IMX715_Calculate3DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it, float *o_very_short_it,
                                    float *o_long_gain, float *o_short_gain, float *o_very_short_gain,
                                    float *hdr_ratio) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
	float very_short_it, very_short_exp_val;
	bool recalc_vs_gain = false;
	float very_short_gain = 1;
	uint32_t rhs1;
	uint32_t rhs2;

    if (pIMX715Ctx == NULL || o_long_it == NULL || o_short_it == NULL ||
        o_very_short_it == NULL || o_long_gain == NULL || o_short_gain == NULL ||
        o_very_short_gain == NULL || hdr_ratio == NULL) {
        TRACE(IMX715_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

	if (pIMX715Ctx->cur_rhs1 == 0 || pIMX715Ctx->cur_rhs2 == 0) {
		TRACE(IMX715_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

    if (hdr_ratio[1] <= 0.0f || hdr_ratio[0] <= 0.0f) {
        TRACE(IMX715_ERROR, "%s: Invalid parameter (hdr_ratio is invalid) hdr_ratio[0]=%f, hdr_ratio[1]= %f\n", __func__, hdr_ratio[0], hdr_ratio[1]);
        return (RET_WRONG_CONFIG);
    }   

	rhs1 = pIMX715Ctx->cur_rhs1;
	rhs2 = pIMX715Ctx->cur_rhs2;

    if(NewIntegrationTime < IMX715_SHR2_RHS2_GAP * pIMX715Ctx->one_line_exp_time * hdr_ratio[1]){
	    pIMX715Ctx->MinIntegrationLine =  IMX715_SHR2_RHS2_GAP * hdr_ratio[1];
	    pIMX715Ctx->AecMinIntegrationTime = pIMX715Ctx->MinIntegrationLine * pIMX715Ctx->one_line_exp_time;
    }

    very_short_it = NewIntegrationTime / hdr_ratio[1];
    very_short_exp_val = very_short_it / pIMX715Ctx->one_line_exp_time;
    very_short_gain = _sensorGain2linear(_linear2sensorGain(NewGain));

    if (very_short_exp_val < rhs1 + IMX715_SHR2_RHS1_GAP) {
        very_short_exp_val = rhs1 + IMX715_SHR2_RHS1_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX715Ctx->one_line_exp_time;
        recalc_vs_gain = true;
        TRACE(IMX715_DEBUG, "%s: very_short_exp_val is too long, set to %u, new very_short_it = %f\n",
        __func__, rhs2 + IMX715_SHR2_RHS1_GAP, very_short_it);
    } else if(very_short_exp_val > rhs2 - IMX715_SHR2_RHS2_GAP) {
        very_short_exp_val = rhs2 - IMX715_SHR2_RHS2_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX715Ctx->one_line_exp_time;
        recalc_vs_gain = true;
        TRACE(IMX715_DEBUG, "%s: very_short_exp_val is too short, set to %u, new very_short_it = %f\n",
        __func__, rhs2 - IMX715_SHR2_RHS2_GAP, very_short_it);
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

RESULT IMX715_Calculate2DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it,
                                    float *o_long_gain, float *o_short_gain,
                                    float *hdr_ratio) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
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

    TRACE(IMX715_DEBUG, "%s: enter with NewIntegrationTime=%.6f NewGain=%.4f ratio=%.1f\n",
        __func__, NewIntegrationTime, NewGain, hdr_ratio[0]);

    result = IMX715_ReadVmax(handle, &vmax);
    CHECK_RESULT_RET(result, "ReadVmax");

    result = IMX715_GetNumExposures(pIMX715Ctx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");

    fsc = vmax * (uint32_t)dol;
    rhs1 = pIMX715Ctx->cur_rhs1;
    one_line = pIMX715Ctx->one_line_exp_time;

    if (one_line <= 0.0f) {
        TRACE(IMX715_ERROR, "%s: Invalid one_line_exp_time (%.9f)\n", __func__, one_line);
        return (RET_WRONG_CONFIG);
    }

    if (fsc <= rhs1 + IMX715_2DOL_SHR0_RHS1_GAP) {
        TRACE(IMX715_ERROR, "%s: fsc(%u) <= rhs1(%u) + gap, invalid config\n",
              __func__, fsc, rhs1);
        return (RET_WRONG_CONFIG);
    }

    /* Hardware limits: SHR0 must satisfy  rhs1 + gap <= SHR0 <= fsc - gap. */
    max_long_it_lines = fsc - rhs1 - IMX715_2DOL_SHR0_RHS1_GAP;
    min_long_it_lines = IMX715_2DOL_SHR0_FSC_GAP;

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
     * Step 1b: When the required EV is within IMX715_2DOL_MAX_IT_NEAR_LINES of
     * max_long_it, the shortfall is a float-rounding artifact — accept unity
     * gain and the negligible ratio deviation instead of bumping a gain step.
     */
    if (long_gain_needed > 1.0f && long_it_lines == max_long_it_lines) {
        required_lines = required_long_ev / one_line;
        if (required_lines - (float)max_long_it_lines < (float)IMX715_2DOL_MAX_IT_NEAR_LINES) {
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

        max_gain_db = _linear2sensorGain(IMX715_MAX_GAIN_AEC);
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

    TRACE(IMX715_DEBUG, "%s: ratio=%.1f vmax=%u fsc=%u rhs1=%u max_long_it_lines=%u\n",
        __func__, hdr_ratio[0], vmax, fsc, rhs1, max_long_it_lines);
    TRACE(IMX715_DEBUG, "%s: required_long_ev=%.6f long_it=%.6f(%u lines) long_gain=%.4f\n",
        __func__, required_long_ev, long_it, long_it_lines, long_gain);
    TRACE(IMX715_DEBUG, "%s: short_it=%.6f short_gain=%.4f\n",
        __func__, *o_short_it, *o_short_gain);

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
        TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }        
    if ((pNumberOfFramesToSkip == NULL) || (pSetGain == NULL) ||
        (pSetIntegrationTime == NULL)) {
        TRACE(IMX715_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    /* Skip silently when PRE_STREAMOFF has paused 3A; trailing calls are expected. */
    if (!pIMX715Ctx->Streaming) {
        return RET_SUCCESS;
    }

    if (NewIntegrationTime == 0 || NewGain == 0) {
        TRACE(IMX715_ERROR, "%s: Invalid parameter (NewIntegrationTime or NewGain is 0)\n", __func__);
        return (RET_WRONG_CONFIG);
    }

    // Regardless of 2dol/3dol (or SDR) - we should get the up-to-date hdr_ratio (if 2dol/sdr, hdr_ratio[1] = 1)
    pIMX715Ctx->hdr_ratio[0] = hdr_ratio[0];
    pIMX715Ctx->hdr_ratio[1] = hdr_ratio[1];

    // HDR has not been tested yet
    if (pIMX715Ctx->enableHdr) {
        result = IMX715_ReadRHS1(handle, &pIMX715Ctx->cur_rhs1);
        CHECK_RESULT_RET(result, "ReadRHS1");
        result = IMX715_ReadRHS2(handle, &pIMX715Ctx->cur_rhs2);
        CHECK_RESULT_RET(result, "ReadRHS2");
        result = IMX715_ReadHmax(handle, &hmax);
        CHECK_RESULT_RET(result, "ReadHmax");

        pIMX715Ctx->SensorMode.ae_info.one_line_exp_time_ns = (uint32_t)(((float)hmax / IMX715_PIXEL_CLK_RATE) * MICRO_2_NANO);
        pIMX715Ctx->one_line_exp_time =
        (float)(pIMX715Ctx->SensorMode.ae_info.one_line_exp_time_ns) / 1000000000;

        TRACE(IMX715_INFO, "%s: HDR mode enabled, RHS1: %u, RHS2: %u, HMAX: %u, one_line_exp_time: %f\n",
        __func__, pIMX715Ctx->cur_rhs1, pIMX715Ctx->cur_rhs2, hmax, pIMX715Ctx->one_line_exp_time);

        if (pIMX715Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
            //2exposure
            result = IMX715_Calculate2DOLExposures(handle, NewIntegrationTime, NewGain,
                                    &long_it, &short_it,
                                    &long_gain, &short_gain,
                                    hdr_ratio);
            CHECK_RESULT_RET(result, "Calculate2DOLExposures");
        } else {
            //3exposure
            result = IMX715_Calculate3DOLExposures(handle, NewIntegrationTime, NewGain,
                                        &long_it, &short_it, &very_short_it,
                                        &long_gain, &short_gain, &very_short_gain,
                                        hdr_ratio);
            CHECK_RESULT_RET(result, "Calculate3DOLExposures");
        }

		result = IMX715_IsiSetLEFIntegrationTimeIss(handle, long_it,
							pSetIntegrationTime,
							pNumberOfFramesToSkip,
							hdr_ratio);
		CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
		result = IMX715_IsiSetLEFGainIss(handle, long_gain, pSetGain, hdr_ratio);
		CHECK_RESULT_RET(result, "SetLEFGain");
		result = IMX715_IsiSetSEF1IntegrationTimeIss(
			handle, short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1IntegrationTime");

		result = IMX715_IsiSetSEF1GainIss(handle, NewIntegrationTime,
						  short_gain, pSetGain, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1Gain");
		result = IMX715_IsiSetSEF2IntegrationTimeIss(
			handle, very_short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF2IntegrationTime");
		result = IMX715_IsiSetSEF2GainIss(handle, NewIntegrationTime,
						  very_short_gain, pSetGain, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF2Gain");

        // Recalculate `io_hdr_ratio` according to the set values
        hdr_ratio[0] = (long_it * long_gain) / (short_it * short_gain);
        if (pIMX715Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            hdr_ratio[1] = (short_it * short_gain) / (very_short_it * very_short_gain);
        }

        // Set the output values to SEF1 values
        *pSetGain = short_gain;
        *pSetIntegrationTime = short_it;

        TRACE(IMX715_DEBUG, "%s: actual hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n",
        __func__, hdr_ratio[0], hdr_ratio[1]);
    } else {
        result = IMX715_IsiSetLEFIntegrationTimeIss(handle, NewIntegrationTime,
                                                pSetIntegrationTime,
                                                pNumberOfFramesToSkip, hdr_ratio);
        CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
        result = IMX715_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
        CHECK_RESULT_RET(result, "SetLEFGain");
    }
    return RET_SUCCESS;
}

RESULT IMX715_IsiExposureControlExpandedIss(
    IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime,
    float NewIris, uint8_t* pNumberOfFramesToSkip, float* pSetGain,
    float* pSetIntegrationTime, float* pSetIris, float* hdr_ratio) {

    RESULT result = RET_SUCCESS;
    if (pSetIris) {
        result = IMX715_IsiSetIrisIss(handle, NewIris);
        CHECK_RESULT_RET(result, "SetIris");
        *pSetIris = NewIris;
    }

    result = IMX715_IsiExposureControlIss(handle, NewGain, NewIntegrationTime,
                                        pNumberOfFramesToSkip, pSetGain,
                                        pSetIntegrationTime, hdr_ratio);
    CHECK_RESULT_RET(result, "ExposureControl");
    return result;
}

RESULT IMX715_IsiGetCurrentExposureIss(IsiSensorHandle_t handle,
                                       float* pSetGain,
                                       float* pSetIntegrationTime) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
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
        TRACE(IMX715_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
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
    uint32_t current_vmax = 0;
    uint32_t requested_vmax = 0;
    uint32_t requested_fsc = 0;
    uint32_t shr = 0;
    size_t dol = 0;
    uint32_t fsc = 0;
    uint32_t min_shr0 = (dol == 1) ? IMX715_MIN_SHR : IMX715_2DOL_SHR0_RHS1_GAP + pIMX715Ctx->cur_rhs1;
    int exp = 0;

    TRACE(IMX715_DEBUG, "%s: set sensor flickerMode = %d\n", __func__, flickerMode);

    if (!pIMX715Ctx) {
        return RET_NULL_POINTER;
    }
    if (pIMX715Ctx->flicker_fps_mode == flickerMode) {
        return RET_SUCCESS;
    }
    if (pIMX715Ctx->enableHdr && (pIMX715Ctx->SensorMode.stitching_mode != SENSOR_STITCHING_L_AND_S)) {
        return RET_SUCCESS;
    }

    result = IMX715_GetNumExposures(pIMX715Ctx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");

    if (flickerMode > ISI_AE_ANTIBANDING_MODE_AUTO) {
        TRACE(IMX715_INFO, "%s: Invalid flickerMode (%d), setting ISI_AE_ANTIBANDING_MODE_AUTO instead.\n", __func__, flickerMode);
        flickerMode = ISI_AE_ANTIBANDING_MODE_AUTO;
    }
    pIMX715Ctx->flicker_fps_mode = flickerMode;

    result = IMX715_ReadVmax(handle, &current_vmax);
    CHECK_RESULT_RET(result, "ReadVmax");
    if (pIMX715Ctx->original_vmax == 0) {
        pIMX715Ctx->original_vmax = current_vmax;
    }

    fsc = current_vmax * dol;

    exp = pIMX715Ctx->AecCurIntegrationTimeLEF / pIMX715Ctx->one_line_exp_time;
    shr = MAX((int)(fsc) - exp, min_shr0);

    if (current_vmax > pIMX715Ctx->original_vmax) {
        current_vmax = MAX((int)fsc - (int)shr + min_shr0, min_shr0);
        fsc = current_vmax * dol;
        shr = MAX((int)fsc - exp, min_shr0);
        pIMX715Ctx->unlimit_fps_vmax_changed = current_vmax > pIMX715Ctx->original_vmax && pIMX715Ctx->unlimit_fps;
    }

    result = IMX715_getNewVmaxAntiFlicker(pIMX715Ctx, current_vmax, &requested_vmax);
    CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
    requested_vmax = MAX( MIN(requested_vmax, IMX715_VMAX_MAX), 1);
    requested_fsc = requested_vmax * dol;

    if (current_vmax != requested_vmax) {
        shr = MAX( (int)requested_fsc - (int)fsc + (int)shr, min_shr0);
        TRACE(IMX715_DEBUG, "%s - writing 0x%x to VMAX, writing 0x%x to SHR0\n", __func__, requested_vmax, shr);
        
        result = IMX715_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        result = IMX715_WriteVmax(handle, requested_vmax);
        CHECK_RESULT_RET(result, "WriteVmax");
        result = IMX715_WriteShr0(handle, shr);
        CHECK_RESULT_RET(result, "WriteShr0");
        result = IMX715_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");
        result = IMX715_UpdateCurrLEFIntegrationTimeFromVmax(pIMX715Ctx, requested_vmax, shr);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromVmax");
    }

    // these 2 are being used only in SDR
    pIMX715Ctx->MaxIntegrationLine = MAX( MIN(requested_vmax - IMX715_MIN_SHR, IMX715_VMAX_MAX - IMX715_MIN_SHR), 1);
    pIMX715Ctx->AecMaxIntegrationTime = pIMX715Ctx->one_line_exp_time * pIMX715Ctx->MaxIntegrationLine;

    TRACE(IMX715_INFO, "%s: set sensor fps = %d\n", __func__,
          pIMX715Ctx->CurrFps);

    TRACE(IMX715_DEBUG, "%s: (exit)\n", __func__);
    return (result);
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
        CHECK_RESULT_RET(result, "disable test pattern");
    } else {
        result = IMX715_IsiWriteRegIss(handle, 0x3253, 0x80);
        CHECK_RESULT_RET(result, "enable test pattern");
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

    result = IMX715_IsiReadRegIss(handle, 0x5081, &value);
    CHECK_RESULT_RET(result, "read TPG register");

    Tpg->enable = ((value & 0x80) != 0) ? 1 : 0;
    if (Tpg->enable) {
        Tpg->pattern = (0xff & value);
    }
    pIMX715Ctx->TestPattern = Tpg->enable;

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
    TRACE(IMX715_DEBUG, "%s: (enter)\n", __func__);

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((Gain.again < 1) | (Gain.again > 16)) {
        TRACE(IMX715_ERROR, "%s: Invalid sensor again (%f)\n", __func__, Gain.again);
        return (RET_OUTOFRANGE);
    }
    // Again = (uint32_t)(1024 - (1024/Gain.again));
    Again = (uint32_t)(((2048 * Gain.again) - 2048) / Gain.again);

    result = IMX715_IsiWriteRegIss(handle, 0x309c, (Again & 0x0000FF));
    CHECK_RESULT_RET(result, "write again low");
    result = IMX715_IsiWriteRegIss(handle, 0x309d, (Again & 0x00FF00) >> 8);
    CHECK_RESULT_RET(result, "write again high");

    if ((Gain.dgain < 1) | (Gain.dgain > 16)) {
        TRACE(IMX715_ERROR, "%s: Invalid sensor dgain (%f)\n", __func__, Gain.dgain);
        return (RET_OUTOFRANGE);
    }
    Dgain = Gain.dgain * 256;

    result = IMX715_IsiWriteRegIss(handle, 0x308c, (Dgain & 0x0000FF));
    CHECK_RESULT_RET(result, "write dgain low");
    result = IMX715_IsiWriteRegIss(handle, 0x308d, (Dgain & 0x00FF00) >> 8);
    CHECK_RESULT_RET(result, "write dgain high");

    TRACE(IMX715_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX715_IsiGetIrisIss( IsiSensorHandle_t handle, float *pSetIris ) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX715_DEBUG, "%s: (enter)\n", __func__);

    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    if (pIMX715Ctx == NULL) {
        TRACE(IMX715_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    //TODO implement getting iris here!
    *pSetIris = 1.0;

    TRACE(IMX715_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX715_IsiSetIrisIss( IsiSensorHandle_t handle, float NewIris) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX715_DEBUG, "%s: (enter)\n", __func__);

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

    TRACE(IMX715_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX715_CalculateHdrBlankingLines(IsiSensorHandle_t handle,
        uint32_t *pBlankingLines, uint32_t rhs1, uint32_t rhs2) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;

    /* The following formulas are taken directly from Sony's IMX715 datasheet.
     * They are located at the "AppNote_DOL_E" document,
     * at "Operating Mode" -> "Number of Blanking Lines" section ("Virtual Channel Mode").
     * The relevant lines for ISP configuration are '*Front* blanking' */

    /* The formula is different from AllPixel/Binning configurations,
     * so we will assume that FHD resolution will be binning */
    bool allPixelMode = (pIMX715Ctx->SensorMode.size.width == IMX715_RES_4K_WIDTH &&
                         pIMX715Ctx->SensorMode.size.height == IMX715_RES_4K_HEIGHT);

    if (!allPixelMode) {
        // Make sure the resolution matches a 2/2 binning mode
        if (pIMX715Ctx->SensorMode.size.width != IMX715_RES_FHD_WIDTH ||
            pIMX715Ctx->SensorMode.size.height != IMX715_RES_FHD_HEIGHT) {
            TRACE(IMX715_ERROR, "%s: Unsupported resolution %dx%d for HDR\n",
                  __func__, pIMX715Ctx->SensorMode.size.width,
                  pIMX715Ctx->SensorMode.size.height);
            return RET_NOTSUPP;
        }
    }

    // 2DOL
    if (pIMX715Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        if (allPixelMode) {
            pBlankingLines[0] = (rhs1 - 3) / 2;
        } else { // 2/2 Binning
            pBlankingLines[0] = ((rhs1 + 1) / 2 - 3) / 2;
        }
        pBlankingLines[1] = 0; // Irrelevant in 2DOL
    // 3DOL
    } else if (pIMX715Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        if (allPixelMode) {
            pBlankingLines[0] = (rhs1 - 4) / 3;
            pBlankingLines[1] = (rhs2 - 5) / 3;
        } else { // 2/2 Binning
            pBlankingLines[0] = ((rhs1 + 1) / 2 - 4) / 3;
            pBlankingLines[1] = ((rhs2 + 2) / 2 - 5) / 3;
        }
    } else {
        TRACE(IMX715_ERROR, "%s: Unsupported stitching mode %d\n",
              __func__, pIMX715Ctx->SensorMode.stitching_mode);
        return RET_NOTSUPP;
    }

    return RET_SUCCESS;
}

static RESULT IMX715_IsiGetHdrBlankingLinesIss(IsiSensorHandle_t handle,
        uint32_t *pBlankingLines, size_t elementCount) {
    IMX715_Context_t* pIMX715Ctx = (IMX715_Context_t*)handle;
    HalContext_t* pHalCtx = (HalContext_t*)pIMX715Ctx->IsiCtx.HalHandle;
    RESULT result = RET_SUCCESS;
    int32_t rhs1 = 0, rhs2 = 0;

    if (elementCount != 2) {
        TRACE(IMX715_ERROR, "%s: Invalid element count %zu, expected 2\n",
              __func__, elementCount);
        return RET_OUTOFRANGE;
    }

    result = IMX715_GetCtrl(pHalCtx->sensor_fd, IMX715_CID_RHS1, &rhs1);
    CHECK_RESULT_RET(result, "GetCtrl RHS1");
    result = IMX715_GetCtrl(pHalCtx->sensor_fd, IMX715_CID_RHS2, &rhs2);
    CHECK_RESULT_RET(result, "GetCtrl RHS2");

    result = IMX715_CalculateHdrBlankingLines(handle, pBlankingLines, rhs1, rhs2);
    CHECK_RESULT_RET(result, "CalculateHdrBlankingLines");
    return result;
}

RESULT IMX715_IsiGetSensorIss(IsiSensor_t* pIsiSensor) {
    RESULT result = RET_SUCCESS;
    static const char SensorName[16] = "IMX715";
    TRACE(IMX715_DEBUG, "%s (enter)\n", __func__);

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
		pIsiSensor->pIsiGetAbsoluteIntegrationTimeLimitsIss =		IMX715_IsiGetAbsoluteIntegrationTimeLimitsIss;

		pIsiSensor->pIsiExposureControlIss =				IMX715_IsiExposureControlIss;
		pIsiSensor->pIsiExposureControlExpandedIss =		IMX715_IsiExposureControlExpandedIss;
		pIsiSensor->pIsiSetIntegrationTimeIss =				IMX715_IsiSetIntegrationTimeIss;

		pIsiSensor->pIsiGetLongIntegrationTimeIss =			IMX715_IsiGetLEFIntegrationTimeIss;
		pIsiSensor->pIsiGetIntegrationTimeIss =				IMX715_IsiGetIntegrationTimeIss;
		pIsiSensor->pIsiGetShortIntegrationTimeIss =		IMX715_IsiGetSEF1IntegrationTimeIss;
		pIsiSensor->pIsiGetVSIntegrationTimeIss =			IMX715_IsiGetSEF2IntegrationTimeIss;

		pIsiSensor->pIsiGetLongGainIss = 					IMX715_IsiGetLEFGainIss;
		pIsiSensor->pIsiGetGainIss = 						IMX715_IsiGetGainIss;
		pIsiSensor->pIsiGetShortGainIss = 					IMX715_IsiGetSEF1GainIss;
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
        pIsiSensor->pIsiGetHdrBlankingLinesIss = IMX715_IsiGetHdrBlankingLinesIss;

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

    TRACE(IMX715_DEBUG, "%s (exit)\n", __func__);
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
