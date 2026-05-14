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

#include "IMX307_priv.h"
#include "vvsensor.h"
#include <linux/i2c.h>

CREATE_TRACER(IMX307_INFO, "IMX307: ", INFO, 1)
CREATE_TRACER(IMX307_WARN, "IMX307: ", WARNING, 1)
CREATE_TRACER(IMX307_ERROR, "IMX307: ", ERROR, 1)
CREATE_TRACER(IMX307_DEBUG, "IMX307: ", INFO, 1)
CREATE_TRACER(IMX307_REG_INFO, "IMX307: ", INFO, 1)
CREATE_TRACER(IMX307_REG_DEBUG, "IMX307: ", INFO, 1)

#include <fcntl.h>
#define LOG_ERROR(...) TRACE(IMX307_ERROR, __VA_ARGS__)

#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

/* I2C */
#define IMX307_I2C_ADDR 0x1a
#define IMX307_TRANSFER_BUFFER_LENGTH 3

/* AE */
#define IMX307_MIN_SHR 1 /* Called SHS1 in the manual  [1, number of lines - 2] */
#define IMX307_IRIS_MIN_VAL 1
#define IMX307_IRIS_MAX_VAL 1
#define IMX307_EXP_MIN_LINES 1
#define IMX307_MAX_GAIN 3981
#define IMX307_MIN_GAIN_STEP                                          \
    (0.035) /**< min gain step size used by GUI (hardware min = 1/16; \
               1/16..32/16 depending on actual gain ) */

#define IMX307_RES_FHD_HEIGHT 1080
#define IMX307_RES_FHD_WIDTH 1920

/* TODO: HDR - NOT IMPLEMENTED YET */
#define IMX307_2DOL_NUM_EXP 2
#define IMX307_3DOL_NUM_EXP 3
#define IMX307_VMAX_MAX ((1 << 20) - 2) // max even value of unsigned 20 bits
#define IMX307_HDR_VMAX_30FPS 0x804 // 2052
#define IMX307_2DOL_HMAX 0x25A // 602
#define IMX307_SDR_FSC  IMX307_SDR_VMAX_30FPS // Frame Set Count
#define IMX307_2DOL_FSC (IMX307_HDR_VMAX_30FPS * IMX307_2DOL_NUM_EXP)
#define IMX307_3DOL_FSC (IMX307_HDR_VMAX_30FPS * IMX307_3DOL_NUM_EXP)

#define IMX307_2DOL_SMALL_GAP 2
#define IMX307_2DOL_LARGE_GAP 5
#define IMX307_3DOL_SMALL_GAP 3
#define IMX307_3DOL_LARGE_GAP 7

#define IMX307_2DOL_SHR0_RHS1_GAP   IMX307_2DOL_LARGE_GAP
#define IMX307_2DOL_SHR0_FSC_GAP    IMX307_2DOL_SMALL_GAP
#define IMX307_2DOL_SHR1_MIN_GAP    IMX307_2DOL_LARGE_GAP
#define IMX307_2DOL_SHR1_RHS1_GAP   IMX307_2DOL_SMALL_GAP

#define IMX307_3DOL_SHR0_RHS2_GAP   IMX307_3DOL_LARGE_GAP
#define IMX307_3DOL_SHR0_FSC_GAP    IMX307_3DOL_SMALL_GAP
#define IMX307_3DOL_SHR1_MIN_GAP    IMX307_3DOL_LARGE_GAP
#define IMX307_3DOL_SHR1_RHS1_GAP   IMX307_3DOL_SMALL_GAP
#define IMX307_3DOL_SHR2_RHS1_GAP   IMX307_3DOL_LARGE_GAP
#define IMX307_3DOL_SHR2_RHS2_GAP   IMX307_3DOL_SMALL_GAP

#define IMX307_SDR_VMAX_30FPS 0x898 // 2200
#define IMX307_SDR_HMAX 0x465 // 1125
#define IMX307_PLL_PCLK 74250000 // 74.25 MHz

#define HMAX_TO_ONE_LINE_EXP_NS(hmax)       (((unsigned long long)(hmax) * 1000000000ULL) / IMX307_PLL_PCLK)
#define IMX307_SDR_ONE_LINE_EXP_TIME_NS     HMAX_TO_ONE_LINE_EXP_NS(IMX307_SDR_HMAX)
#define IMX307_2DOL_ONE_LINE_EXP_TIME_NS    HMAX_TO_ONE_LINE_EXP_NS(IMX307_2DOL_HMAX)

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
RESULT IMX307_IsiSetIrisIss( IsiSensorHandle_t handle,
                    const float NewIris);

/*****************************************************************************
 *Sensor Info
*****************************************************************************/

static struct vvsensor_mode_s pimx307_mode_info[] = {
    /* 1920x1080 @ 30fps, 12-bit */
    {
        .index     = 0,
        .size      ={
			.bounds_width  = IMX307_RES_FHD_WIDTH,
			.bounds_height = IMX307_RES_FHD_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX307_RES_FHD_WIDTH,
			.height        = IMX307_RES_FHD_HEIGHT,
	},
	.fps       = 30 * ISI_FPS_ACCURACY,
	.hdr_mode  = SENSOR_MODE_LINEAR,
	.bit_width = 12,
	.bayer_pattern = BAYER_RGGB,
	.ae_info = {
		.one_line_exp_time_ns = IMX307_SDR_ONE_LINE_EXP_TIME_NS,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = IMX307_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30,
		.max_integration_time = IMX307_SDR_VMAX_30FPS - IMX307_MIN_SHR,
	}
    },
};

static RESULT IMX307_IsiSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    RESULT result = RET_SUCCESS;
    return (result);
}

static RESULT IMX307_GetCtrl(int sensor_fd, unsigned int ctrl_cid, int32_t *value)
{
	struct v4l2_control control;
	memset(&control, 0, sizeof(control));
	control.id = ctrl_cid;

	int ret = ioctl(sensor_fd, VIDIOC_G_CTRL, &control);
	if (ret < 0) {
		TRACE(IMX307_ERROR, "%s: VIDIOC_G_CTRL ioctl failed with error %d (%s). ret = %d\n",
			__func__, errno, strerror(errno), ret);
		return RET_FAILURE;
	}

	*value = control.value;
	return RET_SUCCESS;
}

static uint32_t IMX307_CalculateModeMaxIntegrationTime(
        uint32_t hdr_mode, uint32_t stitching_mode, uint32_t rhs1) {
	if (hdr_mode == SENSOR_MODE_LINEAR) {
		return IMX307_SDR_FSC - IMX307_MIN_SHR; // SDR
	} else {
        uint32_t rhs1_gap = (stitching_mode == SENSOR_STITCHING_3DOL) ?
        IMX307_3DOL_SHR1_RHS1_GAP : IMX307_2DOL_SHR1_RHS1_GAP;
        return rhs1 - rhs1_gap; // HDR
    }
}

static RESULT IMX307_SetSensorModeData(IMX307_Context_t* pIMX307Ctx, size_t index) {
    int sensor_fd = 0;
    int32_t rhs1 = 0;
    RESULT result = RET_SUCCESS;

    if (pIMX307Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    sensor_fd = ((HalContext_t*)pIMX307Ctx->IsiCtx.HalHandle)->sensor_fd;

    if (index >= ARRAY_SIZE(pimx307_mode_info)) {
        return RET_OUTOFRANGE;
    }

    pIMX307Ctx->SensorMode.index = index;
    memcpy(&pIMX307Ctx->SensorMode, &pimx307_mode_info[index], sizeof(pIMX307Ctx->SensorMode));
    if (pIMX307Ctx->SensorMode.index != index) {
        TRACE(IMX307_ERROR, "%s: Sensor mode index mismatch: expected %zu, got %u\n",
              __func__, index, pIMX307Ctx->SensorMode.index);
        return RET_FAILURE;
    }

    // If the mode is HDR, we need to get RHS1 value from the sensor.
    if (pIMX307Ctx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        /* Get the RHS1 value via a control rather then reading the register,
         * because the register is not available until a stream is running. */
        result = IMX307_GetCtrl(sensor_fd, IMX307_CID_RHS1, &rhs1);
        if ((result != RET_SUCCESS) || rhs1 == 0) {
            TRACE(IMX307_ERROR, "%s: Failed to get RHS1 value from sensor (result=%d)\n",
                __func__, result);
            return result;
        }
    }

    pIMX307Ctx->SensorMode.ae_info.max_integration_time =
        IMX307_CalculateModeMaxIntegrationTime(
            pIMX307Ctx->SensorMode.hdr_mode, pIMX307Ctx->SensorMode.stitching_mode, rhs1);

    if (pIMX307Ctx->SensorMode.ae_info.max_integration_time == 0) {
        TRACE(IMX307_ERROR, "%s: Invalid max integration time calculated (%u) for HDR mode %d, index %zu\n",
              __func__, pIMX307Ctx->SensorMode.ae_info.max_integration_time,
              pIMX307Ctx->SensorMode.hdr_mode, index);
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

static RESULT IMX307_IsiCreateIss(IsiSensorInstanceConfig_t* pConfig) {
    RESULT result = RET_SUCCESS;
    IMX307_Context_t* pIMX307Ctx;
    char i2c_file_path[PATH_MAX];

    memset(i2c_file_path, 0, PATH_MAX);

    if (!pConfig || !pConfig->pSensor) return (RET_NULL_POINTER);

    pIMX307Ctx = (IMX307_Context_t*)malloc(sizeof(IMX307_Context_t));
    if (!pIMX307Ctx) {
        return (RET_OUTOFMEM);
    }

    MEMSET(pIMX307Ctx, 0, sizeof(IMX307_Context_t));
    pIMX307Ctx->i2c_fd = -1;
    pIMX307Ctx->i2c_af_fd = -1;

    result = HalAddRef(pConfig->HalHandle);
    if (result != RET_SUCCESS) {
        goto error_free_ctx;
    }

    pIMX307Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pIMX307Ctx->IsiCtx.pSensor = pConfig->pSensor;
    pIMX307Ctx->GroupHold = BOOL_FALSE;
    pIMX307Ctx->OldGain = 1.0;
    pIMX307Ctx->OldIntegrationTime = 0.01;
    pIMX307Ctx->Configured = BOOL_FALSE;
    pIMX307Ctx->Streaming = BOOL_FALSE;
    pIMX307Ctx->TestPattern = BOOL_FALSE;
    pIMX307Ctx->isAfpsRun = BOOL_FALSE;
    
    // By default, until specified otherwise, all ratios are 1 (SDR)
    pIMX307Ctx->hdr_ratio[0] = 1.0f;
    pIMX307Ctx->hdr_ratio[1] = 1.0f;
    pIMX307Ctx->hcg_factor = 2.0f; /* Rcg typical, IMX307 Datasheet, Image Sensor Characteristics table */

    result = IMX307_SetSensorModeData(pIMX307Ctx, pConfig->SensorModeIndex);
    if (result != RET_SUCCESS) {
        TRACE(IMX307_ERROR, "%s: Set sensor mode data failed! (%d)\n", __func__, result);
        goto error_hal_del_ref;
    }
    
    pConfig->hSensor = (IsiSensorHandle_t)pIMX307Ctx;
    pIMX307Ctx->pattern = ISI_BPAT_RGRGGBGB;
    pIMX307Ctx->subdev = HalGetFdHandle(pConfig->HalHandle,
                                        HAL_MODULE_SENSOR);  // two sensors??
    pIMX307Ctx->KernelDriverFlag = 1;
    TRACE(IMX307_INFO, "%s - sensor i2c bus: %d, af i2c bus: %d, sensor i2c addr: 0x%x, af i2c addr: 0x%x\n", __func__,
           pConfig->I2cBusNum, pConfig->I2cAfBusNum, pConfig->SlaveAddr, pConfig->SlaveAfAddr);
    pIMX307Ctx->i2c_addr = pConfig->SlaveAddr;
    pIMX307Ctx->i2c_af_addr = pConfig->SlaveAfAddr;

    sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cBusNum);
    pIMX307Ctx->i2c_fd = open(i2c_file_path, O_RDWR);
    if (pIMX307Ctx->i2c_fd < 0) {
        TRACE(IMX307_ERROR, "%s: unable to open /dev/i2c-%d, error %d (%s)\n",
              __func__, pConfig->I2cBusNum, errno, strerror(errno));
        result = RET_FAILURE;
        goto error_hal_del_ref;
    }

    if (ioctl(pIMX307Ctx->i2c_fd, I2C_SLAVE_FORCE, pIMX307Ctx->i2c_addr) < 0) {
        TRACE(IMX307_ERROR, "%s: unable to set I2C_SLAVE_FORCE on /dev/i2c-%d, error %d (%s)\n",
              __func__, pConfig->I2cBusNum, errno, strerror(errno));
        result = RET_FAILURE;
        goto error_close_i2c_fd;
    }

    if (pConfig->I2cAfBusNum < 0) {
        TRACE(IMX307_INFO, "no af i2c bus\n");
        pIMX307Ctx->SensorMode.af_mode = ISI_SENSOR_AF_MODE_NOTSUPP;
    } else {
        sprintf(i2c_file_path, "/dev/i2c-%d", pConfig->I2cAfBusNum);
        pIMX307Ctx->i2c_af_fd = open(i2c_file_path, O_RDWR);
        if (pIMX307Ctx->i2c_af_fd < 0) {
            TRACE(IMX307_ERROR, "%s: unable to open /dev/i2c-%d for af, error %d (%s)\n",
                  __func__, pConfig->I2cAfBusNum, errno, strerror(errno));
            result = RET_FAILURE;
            goto error_close_i2c_fd;
        }
        pIMX307Ctx->SensorMode.af_mode = ISI_SENSOR_AF_MODE_CDAF;
    }

    return (result);

error_close_i2c_fd:
    close(pIMX307Ctx->i2c_fd);
error_hal_del_ref:
    (void)HalDelRef(pConfig->HalHandle);
error_free_ctx:
    free(pIMX307Ctx);
    pIMX307Ctx = NULL;
    return (result);
}

static RESULT IMX307_IsiReleaseIss(IsiSensorHandle_t handle) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    RESULT cur_result = RET_SUCCESS;

    if (pIMX307Ctx == NULL) return (RET_WRONG_HANDLE);

    cur_result = IMX307_IsiSetStreamingIss(pIMX307Ctx, BOOL_FALSE);
    UPDATE_RESULT_LOG(result, cur_result, "SetStreaming off");
    cur_result = IMX307_IsiSetPowerIss(pIMX307Ctx, BOOL_FALSE);
    UPDATE_RESULT_LOG(result, cur_result, "SetPower off");
    (void)HalDelRef(pIMX307Ctx->IsiCtx.HalHandle);
    if (pIMX307Ctx->i2c_af_fd >= 0) {
        close(pIMX307Ctx->i2c_af_fd);
    }
    close(pIMX307Ctx->i2c_fd);
    MEMSET(pIMX307Ctx, 0, sizeof(IMX307_Context_t));
    free(pIMX307Ctx);
    pIMX307Ctx = NULL;
    return (result);
}

static RESULT IMX307_IsiReadRegIss(IsiSensorHandle_t handle,
                                   const uint32_t Addr, uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    struct i2c_rdwr_ioctl_data ioctl_data;
    unsigned char out[IMX307_TRANSFER_BUFFER_LENGTH];
    struct i2c_msg msgs[2];
    uint8_t addr_buf[2] = { (Addr >> 8) & 0xff, Addr & 0xff };

    if (pIMX307Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    msgs[0].addr = pIMX307Ctx->i2c_addr;
    msgs[0].flags = 0; // Write
    msgs[0].len = sizeof(addr_buf);
    msgs[0].buf = addr_buf;

    msgs[1].addr = pIMX307Ctx->i2c_addr;
    msgs[1].flags = I2C_M_RD; // Read
    msgs[1].len = 1;
    msgs[1].buf = out;

    ioctl_data.msgs = msgs;
    ioctl_data.nmsgs = 2;

    if (ioctl(pIMX307Ctx->i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        TRACE(IMX307_ERROR, "%s: I2C_RDWR ioctl failed for addr 0x%04x, error %d (%s)\n",
              __func__, Addr, errno, strerror(errno));
        return RET_FAILURE;
    }

    *pValue = out[0];

    return (result);
}

static RESULT IMX307_IsiWriteRegIss(IsiSensorHandle_t handle,
                                    const uint32_t Addr, const uint32_t Value) {
    char out[IMX307_TRANSFER_BUFFER_LENGTH];
    ssize_t write_ret;

    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (pIMX307Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memset(out, 0, IMX307_TRANSFER_BUFFER_LENGTH);
    out[0] = (Addr >> 8) & 0xff;
    out[1] = Addr & 0xff;
    out[2] = Value;
    write_ret = write(pIMX307Ctx->i2c_fd, out, sizeof(out));
    if (write_ret != sizeof(out)) {
        if (write_ret < 0) {
            TRACE(IMX307_ERROR, "%s: I2C write failed with error %d (%s)\n",
                  __func__, errno, strerror(errno));
        } else {
            TRACE(IMX307_ERROR, "%s: I2C write incomplete. Wrote %zd of %zu bytes\n",
                  __func__, write_ret, sizeof(out));
        }
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

static RESULT IMX307_UpdateFps(IMX307_Context_t *pIMX307Ctx, uint32_t vmax) {
    float frame_time = 0;
    frame_time = (vmax * pIMX307Ctx->one_line_exp_time);

    if (frame_time == 0) return RET_FAILURE;

    pIMX307Ctx->CurrFps = (uint32_t)(ceil(1 / frame_time)) * ISI_FPS_ACCURACY;
    return RET_SUCCESS;
}

static RESULT IMX307_ReadVmax(IsiSensorHandle_t handle, uint32_t* vmax) {
    uint32_t vmax_low = 0, vmax_mid = 0, vmax_high = 0;
    RESULT result;

    result = IMX307_IsiReadRegIss(handle, 0x3018, &vmax_low);
    CHECK_RESULT_RET(result, "ReadVmax low");
    result = IMX307_IsiReadRegIss(handle, 0x3019, &vmax_mid);
    CHECK_RESULT_RET(result, "ReadVmax mid");
    result = IMX307_IsiReadRegIss(handle, 0x301a, &vmax_high);
    CHECK_RESULT_RET(result, "ReadVmax high");

    *vmax = (vmax_high << 16) | (vmax_mid << 8) | vmax_low;
    return RET_SUCCESS;
}

static RESULT IMX307_ReadHmax(IsiSensorHandle_t handle, uint32_t* hmax) {
    uint32_t hmax_low = 0, hmax_high = 0;
    RESULT result;

    result = IMX307_IsiReadRegIss(handle, 0x301c, &hmax_low);
    CHECK_RESULT_RET(result, "ReadHmax low");
    result = IMX307_IsiReadRegIss(handle, 0x301d, &hmax_high);
    CHECK_RESULT_RET(result, "ReadHmax high");

    *hmax = (hmax_high << 8) | hmax_low;
    return RET_SUCCESS;
}

static RESULT IMX307_WriteVmax(IsiSensorHandle_t handle, uint32_t vmax) {
    RESULT result;

    result = IMX307_IsiWriteRegIss(handle, 0x3018, vmax & 0xff);
    CHECK_RESULT_RET(result, "WriteVmax low");
    result = IMX307_IsiWriteRegIss(handle, 0x3019, (vmax >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteVmax mid");
    result = IMX307_IsiWriteRegIss(handle, 0x301a, (vmax >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteVmax high");

    result = IMX307_UpdateFps((IMX307_Context_t *)handle, vmax);
    CHECK_RESULT_RET(result, "UpdateFps");
    return RET_SUCCESS;
}

// TODO: HDR
static RESULT IMX307_ReadRHS1(IsiSensorHandle_t handle, uint32_t* rhs1) {
    uint32_t rhs1_low = 0, rhs1_mid = 0, rhs1_high = 0;
    RESULT result;

    result = IMX307_IsiReadRegIss(handle, 0x3030, &rhs1_low);
    CHECK_RESULT_RET(result, "ReadRHS1 low");
    result = IMX307_IsiReadRegIss(handle, 0x3031, &rhs1_mid);
    CHECK_RESULT_RET(result, "ReadRHS1 mid");
    result = IMX307_IsiReadRegIss(handle, 0x3032, &rhs1_high);
    CHECK_RESULT_RET(result, "ReadRHS1 high");

    *rhs1 = (rhs1_high << 16) | (rhs1_mid << 8) | rhs1_low;
    return RET_SUCCESS;
}

// TODO: HDR
static RESULT IMX307_ReadRHS2(IsiSensorHandle_t handle, uint32_t* rhs2) {
    uint32_t rhs2_low = 0, rhs2_mid = 0, rhs2_high = 0;
    RESULT result;

    result = IMX307_IsiReadRegIss(handle, 0x3064, &rhs2_low);
    CHECK_RESULT_RET(result, "ReadRHS2 low");
    result = IMX307_IsiReadRegIss(handle, 0x3065, &rhs2_mid);
    CHECK_RESULT_RET(result, "ReadRHS2 mid");
    result = IMX307_IsiReadRegIss(handle, 0x3066, &rhs2_high);
    CHECK_RESULT_RET(result, "ReadRHS2 high");

    *rhs2 = (rhs2_high << 16) | (rhs2_mid << 8) | rhs2_low;
    return RET_SUCCESS;
}

static RESULT IMX307_WriteShr0(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX307_IsiWriteRegIss(handle, 0x3020, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr0 low");
    result = IMX307_IsiWriteRegIss(handle, 0x3021, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr0 mid");
    result = IMX307_IsiWriteRegIss(handle, 0x3022, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr0 high");

    return RET_SUCCESS;
}

// TODO: HDR
static RESULT IMX307_WriteShr1(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX307_IsiWriteRegIss(handle, 0x3054, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr1 low");
    result = IMX307_IsiWriteRegIss(handle, 0x3055, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr1 mid");
    result = IMX307_IsiWriteRegIss(handle, 0x3056, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr1 high");

    return RET_SUCCESS;
}

// TODO: HDR
static RESULT IMX307_WriteShr2(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = IMX307_IsiWriteRegIss(handle, 0x3058, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr2 low");
    result = IMX307_IsiWriteRegIss(handle, 0x3059, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr2 mid");
    result = IMX307_IsiWriteRegIss(handle, 0x305a, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr2 high");

    return RET_SUCCESS;
}

static RESULT IMX307_WriteGain(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX307_IsiWriteRegIss(handle, 0x3014, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain low");

    return RET_SUCCESS;
}

// TODO: HDR
static RESULT IMX307_WriteGain1(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX307_IsiWriteRegIss(handle, 0x3072, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain1 low");
    result = IMX307_IsiWriteRegIss(handle, 0x3073, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain1 high");

    return RET_SUCCESS;
}

// TODO: HDR
static RESULT IMX307_WriteGain2(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = IMX307_IsiWriteRegIss(handle, 0x3074, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain2 low");
    result = IMX307_IsiWriteRegIss(handle, 0x3075, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain2 high");

    return RET_SUCCESS;
}

static RESULT IMX307_LockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX307_IsiWriteRegIss(handle, 0x3001, 0x1);
    CHECK_RESULT_RET(result, "LockRegHold");

    return result;
}

static RESULT IMX307_UnlockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = IMX307_IsiWriteRegIss(handle, 0x3001, 0x0);
    CHECK_RESULT_RET(result, "UnlockRegHold");

    return result;
}

static RESULT IMX307_IsiGetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    TRACE(IMX307_INFO, "%s (enter)\n", __func__);
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (pIMX307Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    memcpy(pMode, &(pIMX307Ctx->SensorMode), sizeof(pIMX307Ctx->SensorMode));

    TRACE(IMX307_INFO, "%s (exit)\n", __func__);
    return (RET_SUCCESS);
}

static RESULT IMX307_IsiSetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    int ret = 0;
    TRACE(IMX307_INFO, "%s (enter)\n", __func__);

    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR, "%s: Invalid sensor handle\n", __func__);
        return (RET_WRONG_HANDLE);
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX307Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, pMode);
    if (ret != 0) {
        TRACE(IMX307_ERROR, "%s: VVSENSORIOC_S_SENSOR_MODE ioctl failed with error %d (%s). ret = %d\n",
            __func__, errno, strerror(errno), ret);
        return (RET_FAILURE);
    }

    return (RET_SUCCESS);
}

static RESULT IMX307_IsiHalEnumModeIss(HalHandle_t HalHandle,
                                       IsiEnumMode_t* pEnumMode) {
    HalContext_t* pHalCtx = HalHandle;
    if (pHalCtx == NULL) {
        return RET_NULL_POINTER;
    }

    if (pEnumMode->index >=
        (sizeof(pimx307_mode_info) / sizeof(pimx307_mode_info[0])))
        return RET_OUTOFRANGE;

    for (uint32_t i = 0;
         i < (sizeof(pimx307_mode_info) / sizeof(pimx307_mode_info[0])); i++) {
        if (pimx307_mode_info[i].index == pEnumMode->index) {
            memcpy(&pEnumMode->mode, &pimx307_mode_info[i], sizeof(IsiMode_t));
            TRACE(IMX307_INFO, "%s (exit)\n", __func__);
            return RET_SUCCESS;
        }
    }

    return RET_NOTSUPP;
}

static RESULT IMX307_IsiEnumModeIss(IsiSensorHandle_t handle,
                                    IsiEnumMode_t* pEnumMode) {
    RESULT result = RET_SUCCESS;
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (pIMX307Ctx == NULL || pIMX307Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    HalContext_t* pHalCtx = (HalContext_t*)pIMX307Ctx->IsiCtx.HalHandle;
    result = IMX307_IsiHalEnumModeIss(pHalCtx, pEnumMode);
    CHECK_RESULT_RET(result, "sensor enum mode");

    return result;
}

static RESULT IMX307_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t* pCaps) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;

    RESULT result = RET_SUCCESS;

    TRACE(IMX307_INFO, "%s (enter)\n", __func__);

    if (pIMX307Ctx == NULL) return (RET_WRONG_HANDLE);

    if (pCaps == NULL) {
        return (RET_NULL_POINTER);
    }

    if (!pIMX307Ctx->Configured) {
        result = IMX307_IsiSetupIss(handle, pCaps);
        CHECK_RESULT_RET(result, "Setup");
    }

    pCaps->BusWidth = pIMX307Ctx->SensorMode.bit_width;
    pCaps->Mode = ISI_MODE_BAYER;
    pCaps->FieldSelection = ISI_FIELDSEL_BOTH;
    pCaps->YCSequence = ISI_YCSEQ_YCBYCR;
    pCaps->Conv422 = ISI_CONV422_COSITED;
    pCaps->BPat = pIMX307Ctx->SensorMode.bayer_pattern;
    pCaps->HPol = ISI_HPOL_REFPOS;
    pCaps->VPol = ISI_VPOL_POS;
    pCaps->Edge = ISI_EDGE_RISING;
    pCaps->Resolution.width = pIMX307Ctx->SensorMode.size.width;
    pCaps->Resolution.height = pIMX307Ctx->SensorMode.size.height;
    pCaps->SmiaMode = ISI_SMIA_OFF;
    pCaps->MipiLanes = ISI_MIPI_4LANES;

    if (pCaps->BusWidth == 10) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_10;
    } else if (pCaps->BusWidth == 12) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_12;
    } else {
        pCaps->MipiMode = ISI_MIPI_OFF;
    }
    TRACE(IMX307_INFO, "got caps - width %d height %d buswidth %d\n",
          pCaps->Resolution.width, pCaps->Resolution.height, pCaps->BusWidth);
    TRACE(IMX307_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX307_AecSetModeParameters(IMX307_Context_t* pIMX307Ctx,
                                          const IsiCaps_t* pConfig) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX307_INFO, "%s%s: (enter)\n", __func__,
          pIMX307Ctx->isAfpsRun ? "(AFPS)" : "");

    pIMX307Ctx->AecIntegrationTimeIncrement = pIMX307Ctx->one_line_exp_time;
    pIMX307Ctx->AecMinIntegrationTime =
        pIMX307Ctx->one_line_exp_time * pIMX307Ctx->MinIntegrationLine;
    pIMX307Ctx->AecMaxIntegrationTime =
        pIMX307Ctx->one_line_exp_time * pIMX307Ctx->MaxIntegrationLine;

    TRACE(IMX307_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __func__,
          pIMX307Ctx->isAfpsRun ? "(AFPS)" : "",
          pIMX307Ctx->AecMaxIntegrationTime);

    pIMX307Ctx->AecGainIncrement = IMX307_MIN_GAIN_STEP;

    // reflects the state of the sensor registers, must equal default settings
    pIMX307Ctx->AecCurGainLEF = pIMX307Ctx->AecMinGain;
	pIMX307Ctx->AecCurGainSEF1 = pIMX307Ctx->AecMinGain;
	pIMX307Ctx->AecCurGainSEF2 = pIMX307Ctx->AecMinGain;
	pIMX307Ctx->AecCurIntegrationTimeLEF = pIMX307Ctx->AecMaxIntegrationTime;
	pIMX307Ctx->AecCurIntegrationTimeSEF1 = pIMX307Ctx->AecMaxIntegrationTime;
	pIMX307Ctx->AecCurIntegrationTimeSEF2 = pIMX307Ctx->AecMaxIntegrationTime;
    pIMX307Ctx->OldGain = 1;
    pIMX307Ctx->OldIntegrationTime = 0.0f;

    TRACE(IMX307_INFO, "%s%s: (exit)\n", __func__,
          pIMX307Ctx->isAfpsRun ? "(AFPS)" : "");

    return (result);
}

static RESULT IMX307_IsiSetupIss(IsiSensorHandle_t handle,
                                 const IsiCaps_t* pCaps) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX307_INFO, "%s (enter)\n", __func__);

    if (!pIMX307Ctx) {
        TRACE(IMX307_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pCaps == NULL) {
        TRACE(IMX307_ERROR,
              "%s: Invalid configuration (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pIMX307Ctx->Streaming != BOOL_FALSE) {
        return RET_WRONG_STATE;
    }

    if (pIMX307Ctx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        pIMX307Ctx->enableHdr = true;
    } else {
        pIMX307Ctx->enableHdr = false;
    }

    pIMX307Ctx->one_line_exp_time =
        (float)(pIMX307Ctx->SensorMode.ae_info.one_line_exp_time_ns) /
        1000000000;
    pIMX307Ctx->MaxIntegrationLine =
        pIMX307Ctx->SensorMode.ae_info.max_integration_time;
    TRACE(IMX307_INFO, "%s: MaxIntegrationLine %u\n", __func__,
          pIMX307Ctx->MaxIntegrationLine);
    pIMX307Ctx->MinIntegrationLine =
        pIMX307Ctx->SensorMode.ae_info.min_integration_time;
    pIMX307Ctx->gain_accuracy = pIMX307Ctx->SensorMode.ae_info.gain_accuracy;
    pIMX307Ctx->AecMaxGain = (float)(pIMX307Ctx->SensorMode.ae_info.max_gain) /
                             pIMX307Ctx->gain_accuracy;
    pIMX307Ctx->AecMinGain = (float)(pIMX307Ctx->SensorMode.ae_info.min_gain) /
                             pIMX307Ctx->gain_accuracy;

    pIMX307Ctx->AecMinIris = IMX307_IRIS_MIN_VAL;
    pIMX307Ctx->AecMaxIris = IMX307_IRIS_MAX_VAL;

    pIMX307Ctx->original_vmax = 0;
    pIMX307Ctx->unlimit_fps = 0;
    pIMX307Ctx->unlimit_fps_vmax_changed = 0;
    pIMX307Ctx->flicker_fps_mode = ISI_AE_ANTIBANDING_MODE_OFF;

    pIMX307Ctx->MaxFps = pIMX307Ctx->SensorMode.fps;
    pIMX307Ctx->CurrFps = pIMX307Ctx->MaxFps;
    TRACE(IMX307_INFO, "%s - got caps - width %d height %d buswidth %d\n",
          __func__, pIMX307Ctx->SensorMode.size.width,
          pIMX307Ctx->SensorMode.size.height, pIMX307Ctx->SensorMode.bit_width);

    TRACE(IMX307_INFO, "%s - MinGain %f MaxGain %f\n", __func__, pIMX307Ctx->AecMinGain,
          pIMX307Ctx->AecMaxGain);

    memcpy(&pIMX307Ctx->CapsConfig, pCaps, sizeof(pIMX307Ctx->CapsConfig));

    /* SW reset comment preserved - no actual reset code was present,
     * and the 100ms sleep was unnecessary (no hardware access in this function). */

    result = IMX307_AecSetModeParameters(pIMX307Ctx, pCaps);
    CHECK_RESULT_RET(result, "AecSetModeParameters");

    pIMX307Ctx->Configured = BOOL_TRUE;
    TRACE(IMX307_INFO, "%s: (exit)\n", __func__);
    return 0;
}

static RESULT IMX307_IsiCheckConnectionIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    return (result);
}

static RESULT IMX307_IsiGetRevisionIss(IsiSensorHandle_t handle,
                                       uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    int ret = 0;
    uint32_t reg_val;
    uint32_t sensor_id;

    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (pIMX307Ctx == NULL || pIMX307Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }
    HalContext_t* pHalCtx = (HalContext_t*)pIMX307Ctx->IsiCtx.HalHandle;

    if (!pValue) return (RET_NULL_POINTER);

    if (pIMX307Ctx->KernelDriverFlag) {
        ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, &sensor_id);
        if (ret != 0) {
            TRACE(IMX307_ERROR, "%s: Read Sensor ID Error! \n", __func__);
            return (RET_FAILURE);
        }
    } else {
        reg_val = 0;
        result = IMX307_IsiReadRegIss(handle, 0x301e, &reg_val);
        CHECK_RESULT_RET(result, "read sensor ID high");
        sensor_id = (reg_val & 0xff) << 8;

        reg_val = 0;
        result = IMX307_IsiReadRegIss(handle, 0x301f, &reg_val);
        CHECK_RESULT_RET(result, "read sensor ID low");
        sensor_id |= (reg_val & 0xff);
    }

    *pValue = sensor_id;
    TRACE(IMX307_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX307_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t is_on) {
    RESULT result = RET_SUCCESS;
    TRACE(IMX307_INFO, "%s (enter)\n", __func__);

    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (pIMX307Ctx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    pIMX307Ctx->Streaming = is_on;

    if (pIMX307Ctx->enableHdr)
        return result;

    TRACE(IMX307_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT IMX307_IsiGetGainLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinGain, float* pMaxGain) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX307_INFO, "%s: (enter)\n", __func__);

    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinGain == NULL) || (pMaxGain == NULL)) {
        TRACE(IMX307_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinGain = pIMX307Ctx->AecMinGain;
    *pMaxGain = pIMX307Ctx->AecMaxGain;

    TRACE(IMX307_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static inline int IMX307_getFlickerPeaksPerSec(IsiSensorAntibandingMode_t mode) {
    int num_modes = sizeof(flickerPeaksPerSecMap) / sizeof(FlickerModePeaksPerSec);
    for (int i = 0; i < num_modes; i++) {
        if (flickerPeaksPerSecMap[i].mode == mode) {
            return flickerPeaksPerSecMap[i].value;
        }
    }
    return 0; // Defaults to 0 if mode not found
}

static RESULT IMX307_GetNumExposures(IMX307_Context_t* pIMX307Ctx, size_t* num_exposures) {
    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    if (num_exposures == NULL) {
        TRACE(IMX307_ERROR, "%s: Invalid output parameter\n", __func__);
        return RET_NULL_POINTER;
    }

    if (pIMX307Ctx->SensorMode.hdr_mode == SENSOR_MODE_LINEAR) {
        *num_exposures = 1; // SDR
    } else if (pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        *num_exposures = IMX307_2DOL_NUM_EXP;
    } else if (pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        *num_exposures = IMX307_3DOL_NUM_EXP;
    } else {
        TRACE(IMX307_ERROR, "%s: Unsupported HDR mode %d with stitching mode %d\n",
            __func__, pIMX307Ctx->SensorMode.hdr_mode, pIMX307Ctx->SensorMode.stitching_mode);
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

static RESULT IMX307_UpdateCurrLEFIntegrationTimeFromFsc(IMX307_Context_t* pIMX307Ctx, uint32_t fsc, uint32_t shr0) {
    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    float configuredIntegrationTime = (fsc - shr0) * pIMX307Ctx->one_line_exp_time;

    pIMX307Ctx->OldIntegrationTime = configuredIntegrationTime;
    pIMX307Ctx->AecCurIntegrationTimeLEF = configuredIntegrationTime;

    TRACE(IMX307_DEBUG, "%s: Updated LEF Integration Time = (fsc[%u] - shr0[%u]) * one_line_exp_time[%f] = %f\n",
          __func__, fsc, shr0, pIMX307Ctx->one_line_exp_time, configuredIntegrationTime);
    return RET_SUCCESS;
}

static RESULT IMX307_UpdateCurrLEFIntegrationTimeFromVmax(IMX307_Context_t* pIMX307Ctx, uint32_t vmax, uint32_t shr0) {
    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    size_t dol = 0;
    RESULT result = IMX307_GetNumExposures(pIMX307Ctx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");
    uint32_t fsc = vmax * dol;
    TRACE(IMX307_DEBUG, "%s: fsc = vmax[%u] * dol[%zu] = %u\n", __func__, vmax, dol, fsc);
    result = IMX307_UpdateCurrLEFIntegrationTimeFromFsc(pIMX307Ctx, fsc, shr0);
    CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromFsc");
    return result;
}

static inline RESULT IMX307_getNewVmaxAntiFlicker(IMX307_Context_t *pIMX307Ctx, uint32_t requestedVmax, uint32_t *outClosestVmax) {
    uint32_t closestVmax = requestedVmax;
    int peaks = 0;
    int difference = INT_MAX;
    int minDifference = INT_MAX;
    if (!pIMX307Ctx) {
        TRACE(IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return RET_FAILURE;
    }
    peaks = IMX307_getFlickerPeaksPerSec(pIMX307Ctx->flicker_fps_mode);
    if (peaks == 0) {
        // No anti-flicker mode, return requested or original Vmax
        if (pIMX307Ctx->unlimit_fps && pIMX307Ctx->unlimit_fps_vmax_changed)
            *outClosestVmax = requestedVmax;
        else
            *outClosestVmax = pIMX307Ctx->original_vmax;
        return RET_SUCCESS;
    }

    for (int i = 1; i < peaks; ++i) {
        uint32_t candidateVmax = i / (peaks * pIMX307Ctx->one_line_exp_time);
        if (candidateVmax < pIMX307Ctx->SensorMode.size.height || candidateVmax < pIMX307Ctx->original_vmax)
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

static RESULT IMX307_IsiUnlimitFpsIss(IsiSensorHandle_t handle,
                                      float maxIntegrationTime) {
    RESULT result = RET_SUCCESS;
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;

    if (!pIMX307Ctx) {
        return RET_NULL_POINTER;
    }

    if (pIMX307Ctx->enableHdr) {
        return result;
    }
    pIMX307Ctx->unlimit_fps = 1;

    if (maxIntegrationTime < 0)
        pIMX307Ctx->MaxIntegrationLine = IMX307_VMAX_MAX - IMX307_MIN_SHR;
    else
        pIMX307Ctx->MaxIntegrationLine =
            MIN((uint32_t)(maxIntegrationTime / pIMX307Ctx->one_line_exp_time),
                IMX307_VMAX_MAX - IMX307_MIN_SHR);
    TRACE(IMX307_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
        pIMX307Ctx->MaxIntegrationLine);
    pIMX307Ctx->AecMaxIntegrationTime =
        pIMX307Ctx->one_line_exp_time * pIMX307Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX307_IsiLimitFpsIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    uint32_t current_vmax = 0;
    uint32_t new_vmax = 0;
    TRACE(IMX307_INFO, "%s (enter)\n", __func__);
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (!pIMX307Ctx) {
        return RET_NULL_POINTER;
    }

    if (pIMX307Ctx->enableHdr)
        return result;

    result = IMX307_ReadVmax(handle, &current_vmax);
    CHECK_RESULT_RET(result, "ReadVmax");
    if (current_vmax == 0) {
        TRACE(IMX307_INFO, "%s - exit because current_vmax is 0\n", __func__);
        return result;
    }

    pIMX307Ctx->unlimit_fps = 0;
    pIMX307Ctx->unlimit_fps_vmax_changed = 0;
    if (pIMX307Ctx->original_vmax == 0) {
        pIMX307Ctx->original_vmax = current_vmax;
    }

    new_vmax = pIMX307Ctx->original_vmax;
    if (pIMX307Ctx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
        result = IMX307_getNewVmaxAntiFlicker(pIMX307Ctx, pIMX307Ctx->original_vmax, &new_vmax);
        CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
        TRACE(IMX307_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX307Ctx->flicker_fps_mode, new_vmax);
    }
    if (current_vmax != new_vmax) {
        result = IMX307_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        result = IMX307_WriteVmax(handle, new_vmax);
        CHECK_RESULT_RET(result, "WriteVmax");
        result = IMX307_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");

        int shr = MAX((int)current_vmax - (int)(pIMX307Ctx->AecCurIntegrationTimeLEF / pIMX307Ctx->one_line_exp_time), IMX307_MIN_SHR);
        result = IMX307_UpdateCurrLEFIntegrationTimeFromVmax(pIMX307Ctx, new_vmax, shr);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromVmax");
    }

    pIMX307Ctx->MaxIntegrationLine =
        MAX(new_vmax - IMX307_MIN_SHR, 1);
    TRACE(IMX307_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
          pIMX307Ctx->MaxIntegrationLine);
    pIMX307Ctx->AecMaxIntegrationTime =
        pIMX307Ctx->one_line_exp_time * pIMX307Ctx->MaxIntegrationLine;
    return result;
}

static RESULT IMX307_IsiGetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinIris, float* pMaxIris) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX307_INFO, "%s: (enter)\n", __func__);

    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIris == NULL) || (pMaxIris == NULL)) {
        TRACE(IMX307_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIris = pIMX307Ctx->AecMinIris;
    *pMaxIris = pIMX307Ctx->AecMaxIris;

    TRACE(IMX307_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX307_IsiSetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float minIris, float maxIris) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX307_INFO, "%s: (enter)\n", __func__);

    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    pIMX307Ctx->AecMinIris = minIris;
    pIMX307Ctx->AecMaxIris = maxIris;

    TRACE(IMX307_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX307_IsiGetIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float* pMinIntegrationTime,
    float* pMaxIntegrationTime) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    float max_short_it, min_short_it;
    RESULT result = RET_SUCCESS;
    int rhs1 = -1;
    HalContext_t* pHalCtx = NULL;

    TRACE(IMX307_INFO, "%s: (enter). prev values: min: %f, max: %f\n", __func__, pIMX307Ctx->AecMinIntegrationTime, pIMX307Ctx->AecMaxIntegrationTime);

    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX307_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMX307Ctx->enableHdr) {
    *pMinIntegrationTime = pIMX307Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMX307Ctx->AecMaxIntegrationTime;

        TRACE(IMX307_INFO, "%s: (exit - SDR) %f, %f\n", 
    __func__, *pMinIntegrationTime, *pMaxIntegrationTime);

        return RET_SUCCESS;
    }

    pHalCtx = (HalContext_t*)pIMX307Ctx->IsiCtx.HalHandle;
    if (!pHalCtx) {
        TRACE(IMX307_ERROR,
              "%s: Invalid HAL handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIMX307Ctx->cur_rhs1 == 0) {
        result = IMX307_GetCtrl(pHalCtx->sensor_fd, IMX307_CID_RHS1, &rhs1);
        CHECK_RESULT_RET(result, "GetCtrl RHS1");
        pIMX307Ctx->cur_rhs1 = (uint32_t)rhs1;
    }

    if (pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        max_short_it = (pIMX307Ctx->cur_rhs1 - IMX307_2DOL_SHR1_MIN_GAP) * pIMX307Ctx->one_line_exp_time;
        min_short_it = IMX307_2DOL_SHR1_RHS1_GAP * pIMX307Ctx->one_line_exp_time;
    } else {
        /* TODO: implement 3dol exposure */
        max_short_it = (pIMX307Ctx->cur_rhs1 - IMX307_2DOL_SHR1_MIN_GAP) * pIMX307Ctx->one_line_exp_time;
        min_short_it = IMX307_2DOL_SHR1_RHS1_GAP * pIMX307Ctx->one_line_exp_time;
    }

    /*
     * Report the actual SEF1 hardware limits.  The ratio is enforced inside
     * Calculate2DOLExposures / Calculate3DOLExposures which will maximise the
     * LEF integration time and apply LEF gain when needed to meet the ratio.
     */
    pIMX307Ctx->AecMinIntegrationTime = min_short_it;
    pIMX307Ctx->AecMaxIntegrationTime = max_short_it;

    *pMinIntegrationTime = pIMX307Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMX307Ctx->AecMaxIntegrationTime;

    TRACE(IMX307_INFO, "%s: (exit - HDR). min: %f, max: %f\n",  __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

static RESULT IMX307_IsiGetAbsoluteIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float* pMinIntegrationTime,
    float* pMaxIntegrationTime) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(IMX307_INFO, "%s: (enter)\n", __func__);
    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(IMX307_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIntegrationTime = IMX307_EXP_MIN_LINES * pIMX307Ctx->one_line_exp_time;
    *pMaxIntegrationTime = (IMX307_VMAX_MAX - IMX307_MIN_SHR) * pIMX307Ctx->one_line_exp_time;

    TRACE(IMX307_INFO, "%s: (exit) %f, %f\n", 
    __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

/* Gain get functions*/

RESULT IMX307_IsiGetGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;
	TRACE(IMX307_INFO, "%s: (enter)\n", __func__);

	if (pIMX307Ctx == NULL) {
		TRACE(IMX307_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	if (pIMX307Ctx->enableHdr) {
		RESULT result = IMX307_IsiGetSEF1GainIss(handle, pSetGain);
		CHECK_RESULT_RET(result, "GetSEF1Gain");
		return result;
	}

	RESULT result = IMX307_IsiGetLEFGainIss(handle, pSetGain);
	CHECK_RESULT_RET(result, "GetLEFGain");
	return result;
}

RESULT IMX307_IsiGetLEFGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	TRACE(IMX307_INFO, "%s: (enter)\n", __func__);

	if (pIMX307Ctx == NULL) {
		TRACE(IMX307_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}
	*pSetGain = pIMX307Ctx->AecCurGainLEF;
	TRACE(IMX307_DEBUG, "%s - returning %f\n", __func__, pIMX307Ctx->AecCurGainLEF);
	TRACE(IMX307_INFO, "%s: (exit)\n", __func__);
	return (result);
}

RESULT IMX307_IsiGetSEF1GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX307_INFO, "%s: (enter)\n", __func__);

	if (pIMX307Ctx == NULL) {
		TRACE(IMX307_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX307Ctx->AecCurGainSEF1;
	TRACE(IMX307_DEBUG, "%s - returning %f\n", __func__, pIMX307Ctx->AecCurGainSEF1);

	TRACE(IMX307_INFO, "%s: (exit)\n", __func__);

	return (result);
}

RESULT IMX307_IsiGetSEF2GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX307_INFO, "%s: (enter)\n", __func__);

	if (pIMX307Ctx == NULL) {
		TRACE(IMX307_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pIMX307Ctx->AecCurGainSEF2;
	TRACE(IMX307_DEBUG, "%s - returning %f\n", __func__, pIMX307Ctx->AecCurGainSEF2);

	TRACE(IMX307_INFO, "%s: (exit)\n", __func__);

	return (result);
}


RESULT IMX307_IsiGetGainIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX307_INFO, "%s: (enter)\n", __func__);

    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = pIMX307Ctx->AecGainIncrement;

    TRACE(IMX307_INFO, "%s: (exit)\n", __func__);

    return (result);
}

RESULT IMX307_IsiGetIrisIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(IMX307_INFO, "%s: (enter)\n", __func__);
    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = 1;

    TRACE(IMX307_INFO, "%s: (exit)\n", __func__);

    return (result);
}

/* Gain set functions*/

RESULT IMX307_IsiSetGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;
	if (pIMX307Ctx == NULL || pIMX307Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	if (pIMX307Ctx->enableHdr) {
		result = IMX307_IsiSetSEF1GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);
        CHECK_RESULT_RET(result, "SetSEF1Gain");

        if (pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            result = IMX307_IsiSetSEF2GainIss(
                handle, 0, NewGain, pSetGain, hdr_ratio);
            CHECK_RESULT_RET(result, "SetSEF2Gain");
        }
	}

	result = IMX307_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
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

RESULT IMX307_IsiSetLEFGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;
	if (pIMX307Ctx == NULL || pIMX307Ctx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	TRACE(IMX307_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX307_DEBUG, "%s: writting 0x%x to GAIN\n", __func__, Gain);

	result = IMX307_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX307_WriteGain(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain");
	result = IMX307_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pIMX307Ctx->AecCurGainLEF = _sensorGain2linear(Gain);
	*pSetGain = pIMX307Ctx->AecCurGainLEF;
	TRACE(IMX307_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

RESULT IMX307_IsiSetSEF1GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX307Ctx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);

	TRACE(IMX307_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX307_DEBUG, "%s: writting 0x%x to GAIN1\n", __func__, Gain);

	result = IMX307_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX307_WriteGain1(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain1");
	result = IMX307_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pIMX307Ctx->AecCurGainSEF1 = _sensorGain2linear(Gain);
	*pSetGain = pIMX307Ctx->AecCurGainSEF1;

	TRACE(IMX307_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

RESULT IMX307_IsiSetSEF2GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX307Ctx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);
	
	TRACE(IMX307_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(IMX307_DEBUG, "%s: writting 0x%x to GAIN2\n", __func__, Gain);

	result = IMX307_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = IMX307_WriteGain2(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain2");
	result = IMX307_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pIMX307Ctx->AecCurGainSEF2 = _sensorGain2linear(Gain);
	*pSetGain = pIMX307Ctx->AecCurGainSEF2;

	TRACE(IMX307_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return (result);
}

/* Integration Time get functions*/

RESULT IMX307_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;

	if (!pIMX307Ctx) {
		TRACE(IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	
	TRACE(IMX307_DEBUG, "%s - enter\n", __func__);
	
	if (pIMX307Ctx->enableHdr) {
		RESULT result = IMX307_IsiGetSEF1IntegrationTimeIss(handle, pSetIntegrationTime);
		CHECK_RESULT_RET(result, "GetSEF1IntegrationTime");
		return result;
	}
	
	RESULT result = IMX307_IsiGetLEFIntegrationTimeIss(handle, pSetIntegrationTime);
	CHECK_RESULT_RET(result, "GetLEFIntegrationTime");
	return result;
}

RESULT IMX307_IsiGetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;
	RESULT result = RET_SUCCESS;


	if (!pIMX307Ctx) {
		TRACE(IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX307Ctx->AecCurIntegrationTimeLEF;
	TRACE(IMX307_DEBUG, "%s - returning %f\n", __func__, pIMX307Ctx->AecCurIntegrationTimeLEF);
	return (result);
}

RESULT IMX307_IsiGetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX307Ctx) {
		TRACE(IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pIMX307Ctx->AecCurIntegrationTimeSEF1;
	TRACE(IMX307_DEBUG, "%s - returning %f\n", __func__, pIMX307Ctx->AecCurIntegrationTimeSEF1);
	return (result);
}

RESULT IMX307_IsiGetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX307Ctx) {
		TRACE(IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);

	*pSetIntegrationTime = pIMX307Ctx->AecCurIntegrationTimeSEF2;
	TRACE(IMX307_DEBUG, "%s - returning %f\n", __func__, pIMX307Ctx->AecCurIntegrationTimeSEF2);
	return (result);
}

RESULT IMX307_IsiGetIntegrationTimeIncrementIss(IsiSensorHandle_t handle,
                                                float* pIncr) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (!pIMX307Ctx) {
        TRACE(IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pIncr) return (RET_NULL_POINTER);

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders
    //in
    // the application)
    *pIncr = pIMX307Ctx->AecIntegrationTimeIncrement;
    return (result);
}

/* Integration Time set functions*/

RESULT IMX307_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;

	if (!pIMX307Ctx) {
		TRACE(IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX307Ctx->enableHdr) {
		RESULT result = IMX307_IsiSetSEF1IntegrationTimeIss(
			handle, NewIntegrationTime, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1IntegrationTime");
		return result;
	}

	RESULT result = IMX307_IsiSetLEFIntegrationTimeIss(
		handle, NewIntegrationTime, pSetIntegrationTime,
		pNumberOfFramesToSkip, hdr_ratio);
	CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
	return result;
}

RESULT IMX307_IsiSetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;

    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;

    int shr = 0;
    uint32_t exp = 0;
    uint32_t new_vmax = 0;
    uint32_t vmax_updated = 0;
    uint32_t current_vmax = 0;
    uint32_t rhs1;
    uint32_t rhs2;

    if (!pIMX307Ctx) {
        TRACE(IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pSetIntegrationTime || !pNumberOfFramesToSkip) {
        TRACE(IMX307_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pIMX307Ctx->Streaming) {
        TRACE(IMX307_ERROR, "%s: sensor not streaming\n", __func__);
        return RET_FAILURE;
    }
    exp = roundf(NewIntegrationTime / pIMX307Ctx->one_line_exp_time);

    TRACE(IMX307_DEBUG, "%s: set AEC_PK_EXPO=0x%05x\n", __func__, exp);

    if (fabs(NewIntegrationTime - pIMX307Ctx->AecCurIntegrationTimeLEF) > FLT_EPSILON) {

        if (pIMX307Ctx->enableHdr){
            if (pIMX307Ctx->cur_rhs1 == 0 || pIMX307Ctx->cur_rhs2 == 0) {
                TRACE(IMX307_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
                return (RET_WRONG_CONFIG);
            }

            rhs1 = pIMX307Ctx->cur_rhs1;
            rhs2 = pIMX307Ctx->cur_rhs2;
            if (pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
                result = IMX307_ReadVmax(pIMX307Ctx, &new_vmax);
                CHECK_RESULT_RET(result, "ReadVmax");
                new_vmax *= IMX307_2DOL_NUM_EXP;

                exp = new_vmax - exp;
                exp = exp > rhs1 + IMX307_2DOL_SHR0_RHS1_GAP ? exp : rhs1 + IMX307_2DOL_SHR0_RHS1_GAP;
                exp = exp < new_vmax - IMX307_2DOL_SHR0_FSC_GAP? exp : new_vmax - IMX307_2DOL_SHR0_FSC_GAP;
                shr = exp;
            } else {
                new_vmax = IMX307_3DOL_FSC;
                exp = new_vmax - exp;
                exp = exp > rhs2 + IMX307_3DOL_SHR0_RHS2_GAP ? exp : rhs2 + IMX307_3DOL_SHR0_RHS2_GAP;
                exp = exp < new_vmax - IMX307_3DOL_SHR0_FSC_GAP? exp : new_vmax - IMX307_3DOL_SHR0_FSC_GAP;
                shr = exp;
            }
		} else {
            if (exp > pIMX307Ctx->MaxIntegrationLine || exp == 0) {
                TRACE(IMX307_ERROR, "%s: Integration time %f (exp %u) out of range (%u)\n", __func__,
                    NewIntegrationTime, exp, pIMX307Ctx->MaxIntegrationLine);
                return RET_FAILURE;
            }
            result = IMX307_ReadVmax(handle, &current_vmax);
            CHECK_RESULT_RET(result, "ReadVmax");

            if (pIMX307Ctx->original_vmax == 0) {
                pIMX307Ctx->original_vmax = current_vmax;
            }

            if (pIMX307Ctx->original_vmax - IMX307_MIN_SHR >
                pIMX307Ctx->MaxIntegrationLine) {
                pIMX307Ctx->MaxIntegrationLine =
                    pIMX307Ctx->original_vmax - IMX307_MIN_SHR;
                TRACE(IMX307_DEBUG, "%s: set MaxIntegrationLine to %u\n", __func__,
                      pIMX307Ctx->MaxIntegrationLine);
                pIMX307Ctx->AecMaxIntegrationTime =
                    pIMX307Ctx->one_line_exp_time * pIMX307Ctx->MaxIntegrationLine;
                TRACE(IMX307_DEBUG, "%s: set AecMaxIntegrationTime to %f\n", __func__,
                      pIMX307Ctx->AecMaxIntegrationTime);
            }

            shr = current_vmax - exp;

            if (shr < IMX307_MIN_SHR || current_vmax > pIMX307Ctx->original_vmax) {
                if (shr < IMX307_MIN_SHR) {
                    // user asked for too big integration-line: more than (current_vmax - minimal_shutter)
                    // increase vmax and set new shutter to minimal value
                    new_vmax = MIN(exp + IMX307_MIN_SHR, pIMX307Ctx->MaxIntegrationLine + IMX307_MIN_SHR);
                } else {
                    // make sure that when we unlimit fps, stay below original fps.
                    // also, make sure we are not using high vmax + high shr more than needed (will slow fps for no reason)
                    new_vmax = MAX(current_vmax - shr + IMX307_MIN_SHR, pIMX307Ctx->original_vmax);
                }
                pIMX307Ctx->unlimit_fps_vmax_changed = new_vmax > pIMX307Ctx->original_vmax && pIMX307Ctx->unlimit_fps;
        
                if (pIMX307Ctx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
                    result = IMX307_getNewVmaxAntiFlicker(pIMX307Ctx, new_vmax, &new_vmax);
                    CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
                    TRACE(IMX307_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pIMX307Ctx->flicker_fps_mode, new_vmax);
                }
                
                shr = MAX(IMX307_MIN_SHR, (int)new_vmax - (int)exp);
                vmax_updated = 1;

            } else {
                new_vmax = current_vmax;
            }
        }

        result = IMX307_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        if (vmax_updated && pIMX307Ctx->unlimit_fps && !pIMX307Ctx->enableHdr) {
            result = IMX307_WriteVmax(handle, new_vmax);
            CHECK_RESULT_RET(result, "WriteVmax");
        }

        TRACE(IMX307_DEBUG, "%s - writing 0x%x to SHR0\n", __func__, shr);
        result = IMX307_WriteShr0(handle, shr);
        CHECK_RESULT_RET(result, "WriteShr0");
        result = IMX307_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");

        // In this context, the "new_vmax" is actually the FSC (multiplied by DOL), not the VMAX.
        result = IMX307_UpdateCurrLEFIntegrationTimeFromFsc(pIMX307Ctx, new_vmax, shr);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromFsc");

        *pNumberOfFramesToSkip = 1U;
    } else {
        *pNumberOfFramesToSkip = 0U;  // no frame skip
    }

    *pSetIntegrationTime = pIMX307Ctx->AecCurIntegrationTimeLEF;
    TRACE(IMX307_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTime);
    return (result);
}

RESULT IMX307_IsiSetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF1,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;

	if (!pIMX307Ctx) {
		TRACE(IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pIMX307Ctx->cur_rhs1 == 0 || pIMX307Ctx->cur_rhs2 == 0) {
		TRACE(IMX307_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX307Ctx->cur_rhs1;

	if (!pSetIntegrationTimeSEF1 || !pNumberOfFramesToSkip) {
		TRACE(IMX307_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX307_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = roundf(NewIntegrationTime / pIMX307Ctx->one_line_exp_time);
	TRACE(IMX307_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX307Ctx->AecCurIntegrationTimeSEF1) > FLT_EPSILON) {
        if (pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
			exp = rhs1 - exp;
			exp = exp > IMX307_2DOL_SHR1_RHS1_GAP ? exp : IMX307_2DOL_SHR1_RHS1_GAP;
			exp = exp < rhs1 - IMX307_2DOL_SHR1_RHS1_GAP ? exp : rhs1 - IMX307_2DOL_SHR1_RHS1_GAP;
		} else {
			exp = rhs1 - exp;
			exp = exp > IMX307_3DOL_SHR1_RHS1_GAP ? exp : IMX307_3DOL_SHR1_RHS1_GAP;
			exp = exp < rhs1 - IMX307_3DOL_SHR1_RHS1_GAP ? exp : rhs1 - IMX307_3DOL_SHR1_RHS1_GAP;
		}
		TRACE(IMX307_DEBUG, "%s - writing 0x%x to SHR1\n", __func__, exp);

		result = IMX307_LockRegHold(handle);
		CHECK_RESULT_RET(result, "LockRegHold");
		result = IMX307_WriteShr1(handle, exp);
		CHECK_RESULT_RET(result, "WriteShr1");
		result = IMX307_UnlockRegHold(handle);
		CHECK_RESULT_RET(result, "UnlockRegHold");

		pIMX307Ctx->AecCurIntegrationTimeSEF1 = (rhs1 - exp) * pIMX307Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF1 = pIMX307Ctx->AecCurIntegrationTimeSEF1;

	TRACE(IMX307_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF1);
	return (result);
}

RESULT IMX307_IsiSetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF2,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	IMX307_Context_t *pIMX307Ctx = (IMX307_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;
	uint32_t rhs2;

	if (pIMX307Ctx->cur_rhs1 == 0 || pIMX307Ctx->cur_rhs2 == 0) {
		TRACE(IMX307_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX307Ctx->cur_rhs1;
	rhs2 = pIMX307Ctx->cur_rhs2;

	if (!pIMX307Ctx) {
		TRACE(IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTimeSEF2 || !pNumberOfFramesToSkip) {
		TRACE(IMX307_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(IMX307_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = roundf(NewIntegrationTime / pIMX307Ctx->one_line_exp_time);
	TRACE(IMX307_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pIMX307Ctx->AecCurIntegrationTimeSEF2) > FLT_EPSILON) {
		exp = rhs2 - exp;
		exp = exp > rhs1 + IMX307_3DOL_SHR2_RHS1_GAP ? exp : rhs1 + IMX307_3DOL_SHR2_RHS1_GAP;
		exp = exp < rhs2 - IMX307_3DOL_SHR2_RHS2_GAP ? exp : rhs2 - IMX307_3DOL_SHR2_RHS2_GAP;
		TRACE(IMX307_DEBUG, "%s - writing 0x%x to SHR2\n", __func__, exp);

		result = IMX307_LockRegHold(handle);
		CHECK_RESULT_RET(result, "LockRegHold");
		result = IMX307_WriteShr2(handle, exp);
		CHECK_RESULT_RET(result, "WriteShr2");
		result = IMX307_UnlockRegHold(handle);
		CHECK_RESULT_RET(result, "UnlockRegHold");

		pIMX307Ctx->AecCurIntegrationTimeSEF2 = (rhs2 - exp) * pIMX307Ctx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF2 = pIMX307Ctx->AecCurIntegrationTimeSEF2;

	TRACE(IMX307_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF2);
	return (result);
}

RESULT IMX307_Calculate3DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it, float *o_very_short_it,
                                    float *o_long_gain, float *o_short_gain, float *o_very_short_gain,
                                    float *hdr_ratio) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
	float very_short_it, very_short_exp_val;
	bool recalc_vs_gain = false;
	float very_short_gain = 1;
	uint32_t rhs1;
	uint32_t rhs2;

	if (pIMX307Ctx == NULL || o_long_it == NULL || o_short_it == NULL ||
        o_very_short_it == NULL || o_long_gain == NULL || o_short_gain == NULL ||
        o_very_short_gain == NULL || hdr_ratio == NULL) {
        TRACE(IMX307_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (NewIntegrationTime == 0 || NewGain == 0) {
        TRACE(IMX307_ERROR, "%s: Invalid parameter (NewIntegrationTime or NewGain is 0)\n", __func__);
        return (RET_WRONG_CONFIG);
    }

	if (pIMX307Ctx->cur_rhs1 == 0 || pIMX307Ctx->cur_rhs2 == 0) {
		TRACE(IMX307_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pIMX307Ctx->cur_rhs1;
	rhs2 = pIMX307Ctx->cur_rhs2;

    {
        float lef_sens = pIMX307Ctx->hcg_lef ? pIMX307Ctx->hcg_factor : 1.0f;
        float sef_sens = pIMX307Ctx->hcg_sef1 ? pIMX307Ctx->hcg_factor : 1.0f;
        float vs_sens  = pIMX307Ctx->hcg_sef2 ? pIMX307Ctx->hcg_factor : 1.0f;
        float ls_adjusted = hdr_ratio[0] * sef_sens / lef_sens;
        float sv_adjusted = hdr_ratio[1] * vs_sens / sef_sens;

	if(NewIntegrationTime < IMX307_3DOL_SHR2_RHS2_GAP * pIMX307Ctx->one_line_exp_time * sv_adjusted){
		pIMX307Ctx->MinIntegrationLine =  IMX307_3DOL_SHR2_RHS2_GAP * sv_adjusted;
		pIMX307Ctx->AecMinIntegrationTime = pIMX307Ctx->MinIntegrationLine * pIMX307Ctx->one_line_exp_time;
    }

    very_short_it = NewIntegrationTime / sv_adjusted;
    very_short_exp_val = very_short_it / pIMX307Ctx->one_line_exp_time;
    very_short_gain = _sensorGain2linear(_linear2sensorGain(NewGain));

    if (very_short_exp_val < rhs1 + IMX307_3DOL_SHR2_RHS1_GAP) {
        very_short_exp_val = rhs1 + IMX307_3DOL_SHR2_RHS1_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX307Ctx->one_line_exp_time;
        recalc_vs_gain = true;
        TRACE(IMX307_DEBUG, "%s: very_short_exp_val is too long, set to %u, new very_short_it = %f\n",
        __func__, rhs2 + IMX307_3DOL_SHR2_RHS1_GAP, very_short_it);
    } else if(very_short_exp_val > rhs2 - IMX307_3DOL_SHR2_RHS2_GAP) {
        very_short_exp_val = rhs2 - IMX307_3DOL_SHR2_RHS2_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pIMX307Ctx->one_line_exp_time;
        recalc_vs_gain = true;
        TRACE(IMX307_DEBUG, "%s: very_short_exp_val is too short, set to %u, new very_short_it = %f\n",
        __func__, rhs2 - IMX307_3DOL_SHR2_RHS2_GAP, very_short_it);
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


RESULT IMX307_Calculate2DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it,
                                    float *o_long_gain, float *o_short_gain,
                                    float *hdr_ratio) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;

	if (pIMX307Ctx->cur_rhs1 == 0) {
		TRACE(IMX307_ERROR, "%s: Invalid parameter (RHS1 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

    *o_short_gain = _sensorGain2linear(_linear2sensorGain(NewGain));

    {
        float lef_sens = pIMX307Ctx->hcg_lef ? pIMX307Ctx->hcg_factor : 1.0f;
        float sef_sens = pIMX307Ctx->hcg_sef1 ? pIMX307Ctx->hcg_factor : 1.0f;
        float adjusted_ratio = hdr_ratio[0] * sef_sens / lef_sens;
        *o_long_it = NewIntegrationTime * adjusted_ratio;
    }

    *o_long_gain = _sensorGain2linear(_linear2sensorGain(NewGain));
    *o_short_it = NewIntegrationTime;

    return RET_SUCCESS;
}

RESULT IMX307_IsiExposureControlIss(IsiSensorHandle_t handle, float NewGain,
                                    float NewIntegrationTime,
                                    uint8_t* pNumberOfFramesToSkip,
                                    float* pSetGain, float* pSetIntegrationTime,
                                    float* hdr_ratio) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;

    RESULT result = RET_SUCCESS;
    float long_it = 0.0;
	float short_it = 0.0;
	float very_short_it = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	float very_short_gain = 1;
    uint32_t hmax;

    TRACE(IMX307_INFO, "%s: enter with NewIntegrationTime: %f, NewGain: %f\n",
        __func__, NewIntegrationTime, NewGain);

    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pNumberOfFramesToSkip == NULL) || (pSetGain == NULL) ||
        (pSetIntegrationTime == NULL)) {
        TRACE(IMX307_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    // Regardless of 2dol/3dol (or SDR) - we should get the up-to-date hdr_ratio (if 2dol/sdr, hdr_ratio[1] = 1)
    pIMX307Ctx->hdr_ratio[0] = hdr_ratio[0];
    pIMX307Ctx->hdr_ratio[1] = hdr_ratio[1];

    if (pIMX307Ctx->enableHdr) {
        result = IMX307_ReadRHS1(handle, &pIMX307Ctx->cur_rhs1);
        CHECK_RESULT_RET(result, "ReadRHS1");
        result = IMX307_ReadRHS2(handle, &pIMX307Ctx->cur_rhs2);
        CHECK_RESULT_RET(result, "ReadRHS2");
        result = IMX307_ReadHmax(handle, &hmax);
        CHECK_RESULT_RET(result, "ReadHmax");

        pIMX307Ctx->SensorMode.ae_info.one_line_exp_time_ns = HMAX_TO_ONE_LINE_EXP_NS(hmax);
        pIMX307Ctx->one_line_exp_time =
        (float)(pIMX307Ctx->SensorMode.ae_info.one_line_exp_time_ns) / 1000000000;

        if (pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
            //2exposure
            result = IMX307_Calculate2DOLExposures(handle, NewIntegrationTime, NewGain,
                                    &long_it, &short_it,
                                    &long_gain, &short_gain,
                                    hdr_ratio);
            CHECK_RESULT_RET(result, "Calculate2DOLExposures");
        } else {
            //3exposure
            result = IMX307_Calculate3DOLExposures(handle, NewIntegrationTime, NewGain,
                                        &long_it, &short_it, &very_short_it,
                                        &long_gain, &short_gain, &very_short_gain,
                                        hdr_ratio);
            CHECK_RESULT_RET(result, "Calculate3DOLExposures");
        }

		result = IMX307_IsiSetLEFIntegrationTimeIss(handle, long_it,
							pSetIntegrationTime,
							pNumberOfFramesToSkip,
							hdr_ratio);
		CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
		result = IMX307_IsiSetLEFGainIss(handle, long_gain, pSetGain, hdr_ratio);
		CHECK_RESULT_RET(result, "SetLEFGain");
		result = IMX307_IsiSetSEF1IntegrationTimeIss(
			handle, short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1IntegrationTime");
		result = IMX307_IsiSetSEF1GainIss(handle, NewIntegrationTime,
						  short_gain, pSetGain, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1Gain");
        if (pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            result = IMX307_IsiSetSEF2IntegrationTimeIss(
			    handle, very_short_it, pSetIntegrationTime,
			    pNumberOfFramesToSkip, hdr_ratio);
			CHECK_RESULT_RET(result, "SetSEF2IntegrationTime");
            result = IMX307_IsiSetSEF2GainIss(handle, NewIntegrationTime,
                            very_short_gain, pSetGain, hdr_ratio);
			CHECK_RESULT_RET(result, "SetSEF2Gain");
        }

        // Recalculate `io_hdr_ratio` according to the set values
        {
            float lef_sens = pIMX307Ctx->hcg_lef ? pIMX307Ctx->hcg_factor : 1.0f;
            float sef_sens = pIMX307Ctx->hcg_sef1 ? pIMX307Ctx->hcg_factor : 1.0f;
            hdr_ratio[0] = (long_it * long_gain * lef_sens) / (short_it * short_gain * sef_sens);
            if (pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
                float vs_sens = pIMX307Ctx->hcg_sef2 ? pIMX307Ctx->hcg_factor : 1.0f;
                hdr_ratio[1] = (short_it * short_gain * sef_sens) / (very_short_it * very_short_gain * vs_sens);
            }
        }

        // Set the output values to SEF1 values
        *pSetGain = short_gain;
        *pSetIntegrationTime = short_it;

        TRACE(IMX307_DEBUG, "%s: actual hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n",
            __func__, hdr_ratio[0], hdr_ratio[1]);
    } else {
        result = IMX307_IsiSetLEFIntegrationTimeIss(handle, NewIntegrationTime,
                                                pSetIntegrationTime,
                                                pNumberOfFramesToSkip, hdr_ratio);
        CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
        result = IMX307_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
        CHECK_RESULT_RET(result, "SetLEFGain");
    }
    return result;
}

RESULT IMX307_IsiExposureControlExpandedIss(
    IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime,
    float NewIris, uint8_t* pNumberOfFramesToSkip, float* pSetGain,
    float* pSetIntegrationTime, float* pSetIris, float* hdr_ratio) {

    RESULT result = RET_SUCCESS;
    if (pSetIris) {
        result = IMX307_IsiSetIrisIss(handle, NewIris);
        CHECK_RESULT_RET(result, "SetIris");
        *pSetIris = NewIris;
    }

    result = IMX307_IsiExposureControlIss(handle, NewGain, NewIntegrationTime,
                                        pNumberOfFramesToSkip, pSetGain,
                                        pSetIntegrationTime, hdr_ratio);
    CHECK_RESULT_RET(result, "ExposureControl");
    return result;
}

RESULT IMX307_IsiGetCurrentExposureIss(IsiSensorHandle_t handle,
                                       float* pSetGain,
                                       float* pSetIntegrationTime) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pSetGain == NULL) || (pSetIntegrationTime == NULL))
        return (RET_NULL_POINTER);

    if (pIMX307Ctx->enableHdr) {
		*pSetGain = pIMX307Ctx->AecCurGainSEF1;
		*pSetIntegrationTime = pIMX307Ctx->AecCurIntegrationTimeSEF1;
	} else {
		*pSetGain = pIMX307Ctx->AecCurGainLEF;
		*pSetIntegrationTime = pIMX307Ctx->AecCurIntegrationTimeLEF;
	}

    return (result);
}

RESULT IMX307_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t* pFps) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *pFps = pIMX307Ctx->CurrFps;

    return (result);
}

RESULT IMX307_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t Fps) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX307_IsiSetFlickerFpsIss(IsiSensorHandle_t handle, uint32_t flickerMode) {
    RESULT result = RET_SUCCESS;
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    uint32_t current_vmax = 0;
    uint32_t requested_vmax = 0;
    uint32_t requested_fsc = 0;
    uint32_t shr = 0;
    size_t dol = 0;
    uint32_t fsc = 0;
    uint32_t min_shr0 = (dol == 1) ? IMX307_MIN_SHR : IMX307_2DOL_SHR0_RHS1_GAP + pIMX307Ctx->cur_rhs1;
    int exp = 0;

    TRACE(IMX307_DEBUG, "%s: set sensor flickerMode = %d\n", __func__, flickerMode);

    if (!pIMX307Ctx) {
        return RET_NULL_POINTER;
    }
    if (pIMX307Ctx->flicker_fps_mode == flickerMode) {
        return RET_SUCCESS;
    }
    if (pIMX307Ctx->enableHdr && (pIMX307Ctx->SensorMode.stitching_mode != SENSOR_STITCHING_L_AND_S)) {
        return RET_SUCCESS;
    }
    result = IMX307_GetNumExposures(pIMX307Ctx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");
    if (flickerMode > ISI_AE_ANTIBANDING_MODE_AUTO) {
        TRACE(IMX307_INFO, "%s: Invalid flickerMode (%d), setting ISI_AE_ANTIBANDING_MODE_AUTO instead.\n", __func__, flickerMode);
        flickerMode = ISI_AE_ANTIBANDING_MODE_AUTO;
    }
    pIMX307Ctx->flicker_fps_mode = flickerMode;

    result = IMX307_ReadVmax(handle, &current_vmax);
    CHECK_RESULT_RET(result, "ReadVmax");
    if (pIMX307Ctx->original_vmax == 0) {
        pIMX307Ctx->original_vmax = current_vmax;
    }

    fsc = current_vmax * dol;

    exp = pIMX307Ctx->AecCurIntegrationTimeLEF / pIMX307Ctx->one_line_exp_time;
    shr = MAX((int)(fsc) - exp, min_shr0);

    if (current_vmax > pIMX307Ctx->original_vmax) {
        current_vmax = MAX((int)fsc - (int)shr + min_shr0, min_shr0);
        fsc = current_vmax * dol;
        shr = MAX((int)fsc - exp, min_shr0);
        pIMX307Ctx->unlimit_fps_vmax_changed = current_vmax > pIMX307Ctx->original_vmax && pIMX307Ctx->unlimit_fps;
    }

    result = IMX307_getNewVmaxAntiFlicker(pIMX307Ctx, current_vmax, &requested_vmax);
    CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
    requested_vmax = MAX( MIN(requested_vmax, IMX307_VMAX_MAX), 1);
    requested_fsc = requested_vmax * dol;
    
    if (current_vmax != requested_vmax) {
        shr = MAX( (int)requested_fsc - (int)fsc + (int)shr, min_shr0);
        TRACE(IMX307_DEBUG, "%s - writing 0x%x to VMAX, writing 0x%x to SHR0\n", __func__, requested_vmax, shr);

        result = IMX307_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        result = IMX307_WriteVmax(handle, requested_vmax);
        CHECK_RESULT_RET(result, "WriteVmax");
        result = IMX307_WriteShr0(handle, shr);
        CHECK_RESULT_RET(result, "WriteShr0");
        result = IMX307_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");
        result = IMX307_UpdateCurrLEFIntegrationTimeFromVmax(pIMX307Ctx, requested_vmax, shr);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromVmax");
    }
    
    // these 2 are being used only in SDR
    pIMX307Ctx->MaxIntegrationLine = MAX( MIN(requested_vmax - IMX307_MIN_SHR, IMX307_VMAX_MAX - IMX307_MIN_SHR), 1);
    pIMX307Ctx->AecMaxIntegrationTime = pIMX307Ctx->one_line_exp_time * pIMX307Ctx->MaxIntegrationLine;

    TRACE(IMX307_INFO, "%s: set sensor fps = %d\n", __func__,
          pIMX307Ctx->CurrFps);

    TRACE(IMX307_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX307_IsiGetAutoFpsInfoIss(IsiSensorHandle_t handle,
                                   IsiAutoFps_t* pAutoFpsInfo) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX307_IsiGetStartEvIss(IsiSensorHandle_t handle, uint64_t* pStartEv) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT IMX307_IsiGetIspStatusIss(IsiSensorHandle_t handle,
                                 IsiIspStatus_t* pIspStatus) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (pIMX307Ctx == NULL || pIMX307Ctx->IsiCtx.HalHandle == NULL) {
        return RET_WRONG_HANDLE;
    }

    pIspStatus->useSensorAE = false;
    pIspStatus->useSensorBLC = false;
    pIspStatus->useSensorAWB = false;

    return RET_SUCCESS;
}

RESULT IMX307_IsiSetTpgIss(IsiSensorHandle_t handle, IsiTpg_t Tpg) {
    RESULT result = RET_SUCCESS;

    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (pIMX307Ctx == NULL || pIMX307Ctx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX307Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (Tpg.enable == 0) {
        result = IMX307_IsiWriteRegIss(handle, 0x3253, 0x00);
        CHECK_RESULT_RET(result, "disable test pattern");
    } else {
        result = IMX307_IsiWriteRegIss(handle, 0x3253, 0x80);
        CHECK_RESULT_RET(result, "enable test pattern");
    }

    pIMX307Ctx->TestPattern = Tpg.enable;

    return (result);
}

RESULT IMX307_IsiGetTpgIss(IsiSensorHandle_t handle, IsiTpg_t* Tpg) {
    RESULT result = RET_SUCCESS;
    uint32_t value = 0;

    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (pIMX307Ctx == NULL || pIMX307Ctx->IsiCtx.HalHandle == NULL ||
        Tpg == NULL) {
        return RET_NULL_POINTER;
    }

    if (pIMX307Ctx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    result = IMX307_IsiReadRegIss(handle, 0x5081, &value);
    CHECK_RESULT_RET(result, "read TPG register");

    Tpg->enable = ((value & 0x80) != 0) ? 1 : 0;
    if (Tpg->enable) {
        Tpg->pattern = (0xff & value);
    }
    pIMX307Ctx->TestPattern = Tpg->enable;

    return (result);
}

RESULT IMX307_IsiFocusCreateIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX307_IsiFocusReleaseIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT IMX307_IsiFocusGetCalibrateIss(IsiSensorHandle_t handle,
                                      IsiFocusCalibAttr_t* pFocusCalib) {
    return RET_SUCCESS;
}

RESULT IMX307_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

RESULT IMX307_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

// Taken from the IMX681 driver
RESULT IMX307_IsiSetAgainDgainIss(IsiSensorHandle_t handle,
                                  IsiUserGain_t Gain) {
    RESULT result = RET_SUCCESS;
    uint32_t Again = 0, Dgain = 0;
    TRACE(IMX307_INFO, "%s: (enter)\n", __func__);

    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((Gain.again < 1) | (Gain.again > 16)) {
        TRACE(IMX307_ERROR, "%s: Invalid sensor again (%f)\n", __func__, Gain.again);
        return (RET_OUTOFRANGE);
    }
    // Again = (uint32_t)(1024 - (1024/Gain.again));
    Again = (uint32_t)(((2048 * Gain.again) - 2048) / Gain.again);

    result = IMX307_IsiWriteRegIss(handle, 0x309c, (Again & 0x0000FF));
    CHECK_RESULT_RET(result, "write again low");
    result = IMX307_IsiWriteRegIss(handle, 0x309d, (Again & 0x00FF00) >> 8);
    CHECK_RESULT_RET(result, "write again high");

    if ((Gain.dgain < 1) | (Gain.dgain > 16)) {
        TRACE(IMX307_ERROR, "%s: Invalid sensor dgain (%f)\n", __func__, Gain.dgain);
        return (RET_OUTOFRANGE);
    }
    Dgain = Gain.dgain * 256;

    result = IMX307_IsiWriteRegIss(handle, 0x308c, (Dgain & 0x0000FF));
    CHECK_RESULT_RET(result, "write dgain low");
    result = IMX307_IsiWriteRegIss(handle, 0x308d, (Dgain & 0x00FF00) >> 8);
    CHECK_RESULT_RET(result, "write dgain high");

    TRACE(IMX307_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX307_IsiGetIrisIss( IsiSensorHandle_t handle,
                                     float *pSetIris ) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX307_INFO, "%s: (enter)\n", __func__);

    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    //TODO implement getting iris here!
    *pSetIris = 1.0;

    TRACE(IMX307_INFO, "%s: (exit)\n", __func__);
    return (result);
}

// TODO: Should this sensor actually support iris control?
RESULT IMX307_IsiSetIrisIss( IsiSensorHandle_t handle, 
                    const float NewIris) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX307_INFO, "%s: (enter)\n", __func__);

    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (NewIris < IMX307_IRIS_MIN_VAL || NewIris > IMX307_IRIS_MAX_VAL) {
        TRACE(IMX307_ERROR, "%s: Invalid Iris %f\n", __func__, NewIris);
    }

    //TODO implement setting iris here!

    TRACE(IMX307_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT IMX307_IsiGetHCGIss( IsiSensorHandle_t handle,
                                     bool *phcg_lef, bool *phcg_sef1, bool *phcg_sef2 ) {
    RESULT result = RET_SUCCESS;

    TRACE(IMX307_INFO, "%s: (enter)\n", __func__);

    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *phcg_lef = pIMX307Ctx->hcg_lef;
    *phcg_sef1 = pIMX307Ctx->hcg_sef1;
    *phcg_sef2 = pIMX307Ctx->hcg_sef2;

    TRACE(IMX307_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT IMX307_IsiSetHCGIss(IsiSensorHandle_t handle, bool hcg_lef, bool hcg_sef1, bool hcg_sef2) {

    RESULT result = RET_SUCCESS;

    TRACE(IMX307_INFO, "%s: (enter)\n", __func__);

    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;

    if (pIMX307Ctx == NULL) {
        TRACE(IMX307_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    result = IMX307_IsiWriteRegIss(handle, 0x3030 , hcg_lef);
    CHECK_RESULT_RET(result, "write HCG LEF");
    pIMX307Ctx->hcg_lef = hcg_lef;

    if (pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S ||
        pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        result = IMX307_IsiWriteRegIss(handle, 0x3031 , hcg_sef1);
        CHECK_RESULT_RET(result, "write HCG SEF1");
        pIMX307Ctx->hcg_sef1 = hcg_sef1;
    }
    if (pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        result = IMX307_IsiWriteRegIss(handle, 0x3032 , hcg_sef2);
        CHECK_RESULT_RET(result, "write HCG SEF2");
        pIMX307Ctx->hcg_sef2 = hcg_sef2;
    }

    TRACE(IMX307_INFO, "%s: (exit)\n", __func__);
    return result;
}



static RESULT IMX307_CalculateHdrBlankingLines(IsiSensorHandle_t handle,
        uint32_t *pBlankingLines, uint32_t rhs1, uint32_t rhs2) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;

    /* The following formulas are taken directly from Sony's IMX307 datasheet.
     * They are located at the "AppNote_DOL_E" document,
     * at "Operating Mode" -> "Number of Blanking Lines" section ("Virtual Channel Mode"). */

    // 2DOL
    if (pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        pBlankingLines[0] = (rhs1 - 3) / 2 + 1;
        pBlankingLines[1] = 0; // Irrelevant in 2DOL
    // 3DOL
    } else if (pIMX307Ctx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        pBlankingLines[0] = (rhs1 - 4) / 3 + 1;
        pBlankingLines[1] = (rhs2 - 5) / 3 + 1;
    } else {
        TRACE(IMX307_ERROR, "%s: Unsupported stitching mode %d\n",
              __func__, pIMX307Ctx->SensorMode.stitching_mode);
        return RET_NOTSUPP;
    }

    return RET_SUCCESS;
}

static RESULT IMX307_IsiGetHdrBlankingLinesIss(IsiSensorHandle_t handle,
        uint32_t *pBlankingLines, size_t elementCount) {
    IMX307_Context_t* pIMX307Ctx = (IMX307_Context_t*)handle;
    HalContext_t* pHalCtx = (HalContext_t*)pIMX307Ctx->IsiCtx.HalHandle;
    RESULT result = RET_SUCCESS;
    int32_t rhs1 = 0, rhs2 = 0;

    if (elementCount != 2) {
        TRACE(IMX307_ERROR, "%s: Invalid element count %zu, expected 2\n",
              __func__, elementCount);
        return RET_OUTOFRANGE;
    }

    result = IMX307_GetCtrl(pHalCtx->sensor_fd, IMX307_CID_RHS1, &rhs1);
    CHECK_RESULT_RET(result, "GetCtrl RHS1");
    result = IMX307_GetCtrl(pHalCtx->sensor_fd, IMX307_CID_RHS2, &rhs2);
    CHECK_RESULT_RET(result, "GetCtrl RHS2");

    result = IMX307_CalculateHdrBlankingLines(handle, pBlankingLines, rhs1, rhs2);
    CHECK_RESULT_RET(result, "CalculateHdrBlankingLines");
    return result;
}

RESULT IMX307_IsiGetSensorIss(IsiSensor_t* pIsiSensor) {
    RESULT result = RET_SUCCESS;
    static const char SensorName[16] = "IMX307";
    TRACE(IMX307_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor != NULL) {
        pIsiSensor->pszName = SensorName;
        pIsiSensor->pIsiCreateIss = IMX307_IsiCreateIss;
        pIsiSensor->pIsiReleaseIss = IMX307_IsiReleaseIss;
        pIsiSensor->pIsiReadRegIss = IMX307_IsiReadRegIss;
        pIsiSensor->pIsiWriteRegIss = IMX307_IsiWriteRegIss;
        pIsiSensor->pIsiGetModeIss = IMX307_IsiGetModeIss;
        pIsiSensor->pIsiSetModeIss = IMX307_IsiSetModeIss;
        pIsiSensor->pIsiEnumModeIss = IMX307_IsiEnumModeIss;
        pIsiSensor->pIsiGetCapsIss = IMX307_IsiGetCapsIss;
        pIsiSensor->pIsiSetupIss = IMX307_IsiSetupIss;
        pIsiSensor->pIsiCheckConnectionIss = IMX307_IsiCheckConnectionIss;
        pIsiSensor->pIsiGetRevisionIss = IMX307_IsiGetRevisionIss;
        pIsiSensor->pIsiSetStreamingIss = IMX307_IsiSetStreamingIss;

        /* AEC functions */
        pIsiSensor->pIsiGetGainLimitsIss = 					IMX307_IsiGetGainLimitsIss;
		pIsiSensor->pIsiGetIrisLimitsIss = 					IMX307_IsiGetIrisLimitsIss;
		pIsiSensor->pIsiSetIrisLimitsIss = 					IMX307_IsiSetIrisLimitsIss;
		pIsiSensor->pIsiGetIntegrationTimeLimitsIss =		IMX307_IsiGetIntegrationTimeLimitsIss;
		 pIsiSensor->pIsiGetAbsoluteIntegrationTimeLimitsIss =		IMX307_IsiGetAbsoluteIntegrationTimeLimitsIss;

		pIsiSensor->pIsiExposureControlIss =				IMX307_IsiExposureControlIss;
		pIsiSensor->pIsiExposureControlExpandedIss =		IMX307_IsiExposureControlExpandedIss;
		pIsiSensor->pIsiSetIntegrationTimeIss =				IMX307_IsiSetIntegrationTimeIss;

		pIsiSensor->pIsiGetLongIntegrationTimeIss =			IMX307_IsiGetLEFIntegrationTimeIss;
		pIsiSensor->pIsiGetIntegrationTimeIss =				IMX307_IsiGetIntegrationTimeIss;
		pIsiSensor->pIsiGetShortIntegrationTimeIss =		IMX307_IsiGetSEF1IntegrationTimeIss;
		pIsiSensor->pIsiGetVSIntegrationTimeIss =			IMX307_IsiGetSEF2IntegrationTimeIss;

		pIsiSensor->pIsiGetLongGainIss = 					IMX307_IsiGetLEFGainIss;
		pIsiSensor->pIsiGetGainIss = 						IMX307_IsiGetGainIss;
		pIsiSensor->pIsiGetShortGainIss = 					IMX307_IsiGetSEF1GainIss;
		pIsiSensor->pIsiGetVSGainIss = 						IMX307_IsiGetSEF2GainIss;

		pIsiSensor->pIsiGetGainIncrementIss =				IMX307_IsiGetGainIncrementIss;
		pIsiSensor->pIsiGetIrisIncrementIss =				IMX307_IsiGetIrisIncrementIss;
		pIsiSensor->pIsiGetIntegrationTimeIncrementIss =	IMX307_IsiGetIntegrationTimeIncrementIss;
		pIsiSensor->pIsiSetGainIss = 						IMX307_IsiSetGainIss;
		pIsiSensor->pIsiGetFpsIss = 						IMX307_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss = 						IMX307_IsiSetFpsIss;
        pIsiSensor->pIsiSetFlickerFpsIss = 					IMX307_IsiSetFlickerFpsIss;
        pIsiSensor->pIsiUnlimitFpsIss =						IMX307_IsiUnlimitFpsIss;
        pIsiSensor->pIsiLimitFpsIss =						IMX307_IsiLimitFpsIss;
		pIsiSensor->pIsiGetAutoFpsInfoIss = 				IMX307_IsiGetAutoFpsInfoIss;
		pIsiSensor->pIsiGetStartEvIss = 					IMX307_IsiGetStartEvIss;
        pIsiSensor->pIsiGetIrisIss =						IMX307_IsiGetIrisIss;
        pIsiSensor->pIsiSetIrisIss =						IMX307_IsiSetIrisIss;
        pIsiSensor->pIsiGetHCGIss =                         IMX307_IsiGetHCGIss;
        pIsiSensor->pIsiSetHCGIss =                         IMX307_IsiSetHCGIss;

        /* SENSOR ISP */
        pIsiSensor->pIsiGetIspStatusIss = IMX307_IsiGetIspStatusIss;
        // pIsiSensor->pIsiSetBlcIss                       =
        // IMX307_IsiSetBlcIss; pIsiSensor->pIsiSetWBIss = IMX307_IsiSetWBIss;

        /* SENSOE OTHER FUNC*/
        pIsiSensor->pIsiSetPowerIss = IMX307_IsiSetPowerIss;
        pIsiSensor->pIsiSetTpgIss = IMX307_IsiSetTpgIss;
        pIsiSensor->pIsiGetTpgIss = IMX307_IsiGetTpgIss;
        // pIsiSensor->pIsiGetExpandCurveIss               =
        // IMX307_IsiGetExpandCurveIss; pIsiSensor->pIsiGetCompressCurveIss =
        // IMX307_IsiGetCompressCurveIss; pIsiSensor->pIsiExtendFuncIss =
        // IMX307_IsiExtendFuncIss; pIsiSensor->pIsiGetOtpDataIss =
        // IMX307_IsiGetOtpDataIss;
        pIsiSensor->pIsiGetHdrBlankingLinesIss = IMX307_IsiGetHdrBlankingLinesIss;

        /* AF */
        pIsiSensor->pIsiFocusCreateIss = IMX307_IsiFocusCreateIss;
        pIsiSensor->pIsiFocusReleaseIss = IMX307_IsiFocusReleaseIss;
        pIsiSensor->pIsiFocusGetCalibrateIss = IMX307_IsiFocusGetCalibrateIss;
        pIsiSensor->pIsiFocusSetIss = IMX307_IsiFocusSetIss;
        pIsiSensor->pIsiFocusGetIss = IMX307_IsiFocusGetIss;
        pIsiSensor->pIsiSetAgainDgainIss = IMX307_IsiSetAgainDgainIss;

    } else {
        result = RET_NULL_POINTER;
    }

    TRACE(IMX307_INFO, "%s (exit)\n", __func__);
    return (result);
}

/*****************************************************************************
 * each sensor driver need declare this struct for isi load
 *****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x9012,
    .pIsiHalEnumModeIss = IMX307_IsiHalEnumModeIss,
    .pIsiGetSensorIss = IMX307_IsiGetSensorIss,
};
