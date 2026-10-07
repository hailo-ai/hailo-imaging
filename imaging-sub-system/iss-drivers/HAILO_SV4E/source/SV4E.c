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

#include "sv4e_priv.h"
#include "vvsensor.h"
#include "sv4e_mode_loader.h"
#include <pthread.h>

static struct vvsensor_mode_s *g_modes = NULL;
static size_t g_n_modes = 0;
static pthread_once_t g_modes_once = PTHREAD_ONCE_INIT;

CREATE_TRACER(SV4E_INFO, "SV4E: ", INFO, 1)
CREATE_TRACER(SV4E_WARN, "SV4E: ", WARNING, 1)
CREATE_TRACER(SV4E_ERROR, "SV4E: ", ERROR, 1)
CREATE_TRACER(SV4E_DEBUG, "SV4E: ", INFO, 1)
CREATE_TRACER(SV4E_REG_INFO, "SV4E: ", INFO, 1)
CREATE_TRACER(SV4E_REG_DEBUG, "SV4E: ", INFO, 1)

#define LOG_ERROR(...) TRACE(SV4E_ERROR, __VA_ARGS__)

#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

/** @defgroup group_defines  API definitions
 *  @{
 */

#define SV4E_I2C_ADDR 0x1a
#define SV4E_IRIS_MIN_VAL 1

/** @} */ // end of group_defines

#define SV4E_IRIS_MAX_VAL 1
#define SV4E_MIN_GAIN_STEP                                          \
    (0.035) /**< min gain step size used by GUI (hardware min = 1/16; \
               1/16..32/16 depending on actual gain ) */
#define SV4E_PLL_PCLK 74250000
// HMAX is in raw PCLK ticks (74.25 MHz). 0x0226 = 550 reproduces the
// 1H period of 7407 ns that every mode in psv4e_mode_info[] declares
// (550 / 74.25 MHz ~= 7407.07 ns). Older revisions used 0xaec=2796,
// which derived to ~37.66 us/line on the HDR re-derivation path at
// Sv4e_ReadHmax (~5.08x too slow) -- latent because SDR doesn't go
// through that path. See `fps_unit_story.md` in the MSW-14708 docs.
#define SV4E_HMAX 0x0226
#define SV4E_VMAX_30FPS 4500
#define SV4E_VMAX_3DOL_HDR 6750
#define SV4E_VMAX_2DOL_HDR 4500
#define SV4E_VMAX_MAX 135000
#define SV4E_MIN_SHR 3
#define SV4E_MAX_GAIN_AEC                                                    \
    (32.0f) /**< max. gain used by the AEC (arbitrarily chosen, hardware limit \
               = 62.0, driver limit = 32.0 ) */
#define SV4E_VS_MAX_INTEGRATION_TIME (0.0018)
#define SV4E_TRANSFER_BUFFER_LENGTH 3
#define SV4E_MAX_GAIN 3981
#define SV4E_SHR0_RHS2_GAP 7
#define SV4E_2DOL_SHR0_RHS1_GAP 5
#define SV4E_SHR0_FSC_GAP 3
#define SV4E_2DOL_SHR0_FSC_GAP 2
#define SV4E_3DOL_SHR1_RHS1_GAP 7
#define SV4E_2DOL_SHR1_MIN_GAP 5
#define SV4E_SHR1_RHS1_GAP 3
#define SV4E_2DOL_SHR1_RHS1_GAP 5
#define SV4E_SHR2_RHS1_GAP 7
#define SV4E_SHR2_RHS2_GAP 3
#define SV4E_PIXEL_CLK_RATE 74.25
#define DEFAULT_RHS1_3DOL 0x1F3
#define DEFAULT_RHS2_3DOL 0x230
#define DEFAULT_RHS1_2DOL 0xa3
#define MICRO_2_NANO 1000
#define SV4E_2DOL_NUM_EXP 2
#define SV4E_3DOL_NUM_EXP 3
/* When required long EV is within this many lines of max_long_it, use unity gain to avoid quantization noise. */
#define SV4E_2DOL_MAX_IT_NEAR_LINES 2

FlickerModePeaksPerSec flickerPeaksPerSecMap[] = {
    { ISI_AE_ANTIBANDING_MODE_OFF, 0 },
    { ISI_AE_ANTIBANDING_MODE_50HZ, 100 },
    { ISI_AE_ANTIBANDING_MODE_60HZ, 120 },
    { ISI_AE_ANTIBANDING_MODE_AUTO, 100 }
};

/*****************************************************************************
 *Forward Declarations
*****************************************************************************/
RESULT Sv4e_IsiSetIrisIss( IsiSensorHandle_t handle,
                    const float NewIris);

/*****************************************************************************
 *Sensor Info
*****************************************************************************/

static struct vvsensor_mode_s psv4e_mode_info_default[] = {
    /* idx 0: 4K30 RGGB12 (existing) */
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
		.max_integration_time = SV4E_VMAX_30FPS - SV4E_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = SV4E_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
    /* idx 1: 1080p30 RGGB12 (new) */
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
		.max_integration_time = SV4E_VMAX_30FPS - SV4E_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = SV4E_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
    /* idx 2: 4K30 GBRG12 (new — imx715 fidelity) */
    {
        .index     = 2,
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
		.max_integration_time = SV4E_VMAX_30FPS - SV4E_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = SV4E_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
    /* idx 3: 1080p30 GBRG12 (new — imx715 fidelity) */
    {
        .index     = 3,
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
	.bayer_pattern = BAYER_GBRG,
	.ae_info = {
		.one_line_exp_time_ns = 7407,
		.max_integration_time = SV4E_VMAX_30FPS - SV4E_MIN_SHR,
		.min_integration_time = 1,
		.integration_accuracy = 1,
		.max_gain = SV4E_MAX_GAIN,
		.min_gain = 1,
		.gain_accuracy = 1,
		.cur_fps = 30
	}
    },
};

static RESULT Sv4e_IsiSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    RESULT result = RET_SUCCESS;

    return (result);
}

static void sv4e_modes_init(void)
{
    struct vvsensor_mode_s *loaded = NULL;
    size_t loaded_n = 0;
    int rc = sv4e_load_modes_from_json(&loaded, &loaded_n);
    if (rc == 0 && loaded && loaded_n) {
        g_modes = loaded;
        g_n_modes = loaded_n;
        TRACE(SV4E_WARN,
              "sv4e: loaded %zu modes from JSON\n", g_n_modes);
        return;
    }
    /* Fallback: static defaults (fail-closed). */
    g_modes = psv4e_mode_info_default;
    g_n_modes = sizeof(psv4e_mode_info_default) /
                sizeof(psv4e_mode_info_default[0]);
    if (rc != -ENOENT) {
        TRACE(SV4E_ERROR,
              "sv4e: JSON load failed (%d); using %zu static modes\n",
              rc, g_n_modes);
    }
}

static RESULT Sv4e_IsiCreateIss(IsiSensorInstanceConfig_t* pConfig) {
    RESULT result = RET_SUCCESS;
    Sv4e_Context_t* pSv4eCtx;
    char i2c_file_path[PATH_MAX];

    memset(i2c_file_path, 0, PATH_MAX);

    if (!pConfig || !pConfig->pSensor) return (RET_NULL_POINTER);

    pSv4eCtx = (Sv4e_Context_t*)malloc(sizeof(Sv4e_Context_t));
    if (!pSv4eCtx) {
        return (RET_OUTOFMEM);
    }

    MEMSET(pSv4eCtx, 0, sizeof(Sv4e_Context_t));
    pSv4eCtx->i2c_fd = -1;
    pSv4eCtx->i2c_af_fd = -1;

    result = HalAddRef(pConfig->HalHandle);
    if (result != RET_SUCCESS) {
        goto error_free_ctx;
    }

    pSv4eCtx->IsiCtx.HalHandle = pConfig->HalHandle;
    pSv4eCtx->IsiCtx.pSensor = pConfig->pSensor;
    pSv4eCtx->GroupHold = BOOL_FALSE;
    pSv4eCtx->OldGain = 1.0;
    pSv4eCtx->OldIntegrationTime = 0.01;
    pSv4eCtx->Configured = BOOL_FALSE;
    pSv4eCtx->Streaming = BOOL_FALSE;
    pSv4eCtx->TestPattern = BOOL_FALSE;
    pSv4eCtx->isAfpsRun = BOOL_FALSE;
    pthread_once(&g_modes_once, sv4e_modes_init);

    /* Fix prior off-by-one: when idx >= count, clamp to last valid (count-1). */
    if (g_n_modes == 0) {
        TRACE(SV4E_ERROR, "%s: no SV4E modes available\n", __func__);
        return RET_OUTOFRANGE;
    }
    if (pConfig->SensorModeIndex >= g_n_modes)
        pConfig->SensorModeIndex = g_n_modes - 1;
    memcpy(&pSv4eCtx->SensorMode,
           &g_modes[pConfig->SensorModeIndex],
           sizeof(pSv4eCtx->SensorMode));
    pConfig->hSensor = (IsiSensorHandle_t)pSv4eCtx;
    pSv4eCtx->pattern = ISI_BPAT_RGRGGBGB;
    pSv4eCtx->subdev = HalGetFdHandle(pConfig->HalHandle,
                                        HAL_MODULE_SENSOR);  // two sensors??
    pSv4eCtx->KernelDriverFlag = 1;
    TRACE(SV4E_INFO, "%s - sensor i2c bus: %d, af i2c bus: %d, sensor i2c addr: 0x%x, af i2c addr: 0x%x\n", __func__,
           pConfig->I2cBusNum, pConfig->I2cAfBusNum, pConfig->SlaveAddr, pConfig->SlaveAfAddr);
    pSv4eCtx->i2c_addr = pConfig->SlaveAddr;
    pSv4eCtx->i2c_af_addr = pConfig->SlaveAfAddr;

    /* SV4E is a synthetic MIPI source with no I2C device. Skip the i2c bus
     * open entirely; reg R/W are stubs and never dereference these fds. */
    (void)i2c_file_path;
    pSv4eCtx->SensorMode.af_mode = ISI_SENSOR_AF_MODE_NOTSUPP;

    return (result);

error_free_ctx:
    free(pSv4eCtx);
    pSv4eCtx = NULL;
    return RET_FAILURE;
}

static RESULT Sv4e_IsiReleaseIss(IsiSensorHandle_t handle) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pSv4eCtx == NULL) return (RET_WRONG_HANDLE);

    result = Sv4e_IsiSetStreamingIss(pSv4eCtx, BOOL_FALSE);
    CHECK_RESULT_RET(result, "SetStreaming off");
    result = Sv4e_IsiSetPowerIss(pSv4eCtx, BOOL_FALSE);
    CHECK_RESULT_RET(result, "SetPower off");
    (void)HalDelRef(pSv4eCtx->IsiCtx.HalHandle);
    if (pSv4eCtx->i2c_af_fd >= 0) {
        close(pSv4eCtx->i2c_af_fd);
    }
    if (pSv4eCtx->i2c_fd >= 0) {
        close(pSv4eCtx->i2c_fd);
    }
    MEMSET(pSv4eCtx, 0, sizeof(Sv4e_Context_t));
    free(pSv4eCtx);
    pSv4eCtx = NULL;
    return (result);
}

static RESULT Sv4e_IsiReadRegIss(IsiSensorHandle_t handle,
                                   const uint32_t Addr, uint32_t* pValue) {

    if (pValue)
        *pValue = 0;
    return RET_SUCCESS;
}

static RESULT Sv4e_IsiWriteRegIss(IsiSensorHandle_t handle,
                                    const uint32_t Addr, const uint32_t Value) {
    return RET_SUCCESS;
}

static RESULT Sv4e_UpdateFps(Sv4e_Context_t *pSv4eCtx, uint32_t vmax) {
    float frame_time = 0;
    frame_time = (vmax * pSv4eCtx->one_line_exp_time);
    if (frame_time == 0) {
        TRACE(SV4E_ERROR, "%s: frame_time is 0, cannot update FPS\n", __func__);
        return RET_FAILURE;
    }

    pSv4eCtx->CurrFps = (uint32_t)(ceil(1 / frame_time));
    return RET_SUCCESS;
}

static RESULT Sv4e_ReadVmax(IsiSensorHandle_t handle, uint32_t* vmax) {

    *vmax = SV4E_VMAX_30FPS;
    return RET_SUCCESS;
}

static RESULT Sv4e_ReadHmax(IsiSensorHandle_t handle, uint32_t* hmax) {

    *hmax = SV4E_HMAX;
    return RET_SUCCESS;
}

static RESULT Sv4e_WriteVmax(IsiSensorHandle_t handle, uint32_t vmax) {
    RESULT result;

    result = Sv4e_IsiWriteRegIss(handle, 0x3028, vmax & 0xff);
    CHECK_RESULT_RET(result, "WriteVmax low");
    result = Sv4e_IsiWriteRegIss(handle, 0x3029, (vmax >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteVmax mid");
    result = Sv4e_IsiWriteRegIss(handle, 0x302a, (vmax >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteVmax high");

    result = Sv4e_UpdateFps((Sv4e_Context_t *)handle, vmax);
    CHECK_RESULT_RET(result, "UpdateFps");
    return result;
}

static RESULT Sv4e_ReadRHS1(IsiSensorHandle_t handle, uint32_t* rhs1) {

    *rhs1 = 0;
    return RET_SUCCESS;
}

static RESULT Sv4e_ReadRHS2(IsiSensorHandle_t handle, uint32_t* rhs2) {

    *rhs2 = 0;
    return RET_SUCCESS;
}


static RESULT Sv4e_WriteShr0(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = Sv4e_IsiWriteRegIss(handle, 0x3050, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr0 low");
    result = Sv4e_IsiWriteRegIss(handle, 0x3051, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr0 mid");
    result = Sv4e_IsiWriteRegIss(handle, 0x3052, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr0 high");

    return RET_SUCCESS;
}

static RESULT Sv4e_WriteShr1(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = Sv4e_IsiWriteRegIss(handle, 0x3054, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr1 low");
    result = Sv4e_IsiWriteRegIss(handle, 0x3055, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr1 mid");
    result = Sv4e_IsiWriteRegIss(handle, 0x3056, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr1 high");

    return RET_SUCCESS;
}

static RESULT Sv4e_WriteShr2(IsiSensorHandle_t handle, uint32_t shr) {
    RESULT result;

    result = Sv4e_IsiWriteRegIss(handle, 0x3058, shr & 0xff);
    CHECK_RESULT_RET(result, "WriteShr2 low");
    result = Sv4e_IsiWriteRegIss(handle, 0x3059, (shr >> 8) & 0xff);
    CHECK_RESULT_RET(result, "WriteShr2 mid");
    result = Sv4e_IsiWriteRegIss(handle, 0x305a, (shr >> 16) & 0x0f);
    CHECK_RESULT_RET(result, "WriteShr2 high");

    return RET_SUCCESS;
}

static RESULT Sv4e_WriteGain(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = Sv4e_IsiWriteRegIss(handle, 0x3070, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain low");
    result = Sv4e_IsiWriteRegIss(handle, 0x3071, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain high");

    return RET_SUCCESS;
}

static RESULT Sv4e_WriteGain1(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = Sv4e_IsiWriteRegIss(handle, 0x3072, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain1 low");
    result = Sv4e_IsiWriteRegIss(handle, 0x3073, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain1 high");

    return RET_SUCCESS;
}

static RESULT Sv4e_WriteGain2(IsiSensorHandle_t handle, uint32_t gain) {
    RESULT result;

    result = Sv4e_IsiWriteRegIss(handle, 0x3074, (gain & 0x00ff));
    CHECK_RESULT_RET(result, "WriteGain2 low");
    result = Sv4e_IsiWriteRegIss(handle, 0x3075, (gain & 0x0700) >> 8);
    CHECK_RESULT_RET(result, "WriteGain2 high");

    return RET_SUCCESS;
}

static RESULT Sv4e_LockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = Sv4e_IsiWriteRegIss(handle, 0x3001, 0x1);
    CHECK_RESULT_RET(result, "LockRegHold");

    return RET_SUCCESS;
}

static RESULT Sv4e_UnlockRegHold(IsiSensorHandle_t handle) {
    RESULT result;

    result = Sv4e_IsiWriteRegIss(handle, 0x3001, 0x0);
    CHECK_RESULT_RET(result, "UnlockRegHold");

    return RET_SUCCESS;
}

static RESULT Sv4e_IsiGetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    TRACE(SV4E_INFO, "%s (enter)\n", __func__);
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    if (pSv4eCtx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    memcpy(pMode, &(pSv4eCtx->SensorMode), sizeof(pSv4eCtx->SensorMode));

    TRACE(SV4E_INFO, "%s (exit)\n", __func__);
    return (RET_SUCCESS);
}

static RESULT Sv4e_IsiSetModeIss(IsiSensorHandle_t handle, IsiMode_t* pMode) {
    int ret = 0;
    TRACE(SV4E_INFO, "%s (enter)\n", __func__);

    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }
    HalContext_t* pHalCtx = (HalContext_t*)pSv4eCtx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, pMode);
    if (ret != 0) {
        TRACE(SV4E_ERROR, "%s: ioctl VVSENSORIOC_S_SENSOR_MODE failed, errno=%d (%s)\n",
              __func__, errno, strerror(errno));
        return (RET_FAILURE);
    }

    return (RET_SUCCESS);
}

static RESULT Sv4e_IsiHalEnumModeIss(HalHandle_t HalHandle,
                                       IsiEnumMode_t* pEnumMode) {
    HalContext_t* pHalCtx = HalHandle;
    if (pHalCtx == NULL)
        return RET_NULL_POINTER;

    pthread_once(&g_modes_once, sv4e_modes_init);

    if (pEnumMode->index >= g_n_modes)
        return RET_OUTOFRANGE;

    for (size_t i = 0; i < g_n_modes; i++) {
        if (g_modes[i].index == pEnumMode->index) {
            memcpy(&pEnumMode->mode, &g_modes[i], sizeof(IsiMode_t));
            TRACE(SV4E_INFO, "%s (exit)\n", __func__);
            return RET_SUCCESS;
        }
    }
    return RET_NOTSUPP;
}

static RESULT Sv4e_IsiEnumModeIss(IsiSensorHandle_t handle,
                                    IsiEnumMode_t* pEnumMode) {
    RESULT result = RET_SUCCESS;
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    if (pSv4eCtx == NULL || pSv4eCtx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    HalContext_t* pHalCtx = (HalContext_t*)pSv4eCtx->IsiCtx.HalHandle;
    result = Sv4e_IsiHalEnumModeIss(pHalCtx, pEnumMode);
    CHECK_RESULT_RET(result, "sensor enum mode");

    return result;
}

static RESULT Sv4e_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t* pCaps) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;

    RESULT result = RET_SUCCESS;

    TRACE(SV4E_INFO, "%s (enter)\n", __func__);

    if (pSv4eCtx == NULL) return (RET_WRONG_HANDLE);

    if (pCaps == NULL) {
        return (RET_NULL_POINTER);
    }

    if (!pSv4eCtx->Configured) {
        result = Sv4e_IsiSetupIss(handle, pCaps);
        CHECK_RESULT_RET(result, "Setup");
    }

    pCaps->BusWidth = pSv4eCtx->SensorMode.bit_width;
    pCaps->Mode = ISI_MODE_BAYER;
    pCaps->FieldSelection = ISI_FIELDSEL_BOTH;
    pCaps->YCSequence = ISI_YCSEQ_YCBYCR;
    pCaps->Conv422 = ISI_CONV422_COSITED;
    pCaps->BPat = pSv4eCtx->SensorMode.bayer_pattern;
    pCaps->HPol = ISI_HPOL_REFPOS;
    pCaps->VPol = ISI_VPOL_POS;
    pCaps->Edge = ISI_EDGE_RISING;
    pCaps->Resolution.width = pSv4eCtx->SensorMode.size.width;
    pCaps->Resolution.height = pSv4eCtx->SensorMode.size.height;
    pCaps->SmiaMode = ISI_SMIA_OFF;
    pCaps->MipiLanes = ISI_MIPI_4LANES;

    if (pCaps->BusWidth == 10) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_10;
    } else if (pCaps->BusWidth == 12) {
        pCaps->MipiMode = ISI_MIPI_MODE_RAW_12;
    } else {
        pCaps->MipiMode = ISI_MIPI_OFF;
    }
    TRACE(SV4E_INFO, "got caps - width %d height %d buswidth %d\n",
          pCaps->Resolution.width, pCaps->Resolution.height, pCaps->BusWidth);
    TRACE(SV4E_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT Sv4e_AecSetModeParameters(Sv4e_Context_t* pSv4eCtx,
                                          const IsiCaps_t* pConfig) {
    RESULT result = RET_SUCCESS;
    TRACE(SV4E_INFO, "%s%s: (enter)\n", __func__,
          pSv4eCtx->isAfpsRun ? "(AFPS)" : "");

    pSv4eCtx->AecIntegrationTimeIncrement = pSv4eCtx->one_line_exp_time;
    pSv4eCtx->AecMinIntegrationTime =
        pSv4eCtx->one_line_exp_time * pSv4eCtx->MinIntegrationLine;
    pSv4eCtx->AecMaxIntegrationTime =
        pSv4eCtx->one_line_exp_time * pSv4eCtx->MaxIntegrationLine;

    TRACE(SV4E_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __func__,
          pSv4eCtx->isAfpsRun ? "(AFPS)" : "",
          pSv4eCtx->AecMaxIntegrationTime);

    pSv4eCtx->AecGainIncrement = SV4E_MIN_GAIN_STEP;

    // reflects the state of the sensor registers, must equal default settings
    pSv4eCtx->AecCurGainLEF = pSv4eCtx->AecMinGain;
	pSv4eCtx->AecCurGainSEF1 = pSv4eCtx->AecMinGain;
	pSv4eCtx->AecCurGainSEF2 = pSv4eCtx->AecMinGain;
	pSv4eCtx->AecCurIntegrationTimeLEF = pSv4eCtx->AecMaxIntegrationTime;
	pSv4eCtx->AecCurIntegrationTimeSEF1 = pSv4eCtx->AecMaxIntegrationTime;
	pSv4eCtx->AecCurIntegrationTimeSEF2 = pSv4eCtx->AecMaxIntegrationTime;
    pSv4eCtx->OldGain = 1;
    pSv4eCtx->OldIntegrationTime = 0.0f;

    TRACE(SV4E_INFO, "%s%s: (exit)\n", __func__,
          pSv4eCtx->isAfpsRun ? "(AFPS)" : "");

    return (result);
}

static RESULT Sv4e_IsiSetupIss(IsiSensorHandle_t handle,
                                 const IsiCaps_t* pCaps) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(SV4E_INFO, "%s (enter)\n", __func__);

    if (!pSv4eCtx) {
        TRACE(SV4E_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pCaps == NULL) {
        TRACE(SV4E_ERROR,
              "%s: Invalid configuration (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pSv4eCtx->Streaming != BOOL_FALSE) {
        return RET_WRONG_STATE;
    }

    if (pSv4eCtx->SensorMode.hdr_mode != SENSOR_MODE_LINEAR) {
        pSv4eCtx->enableHdr = true;
    } else {
        pSv4eCtx->enableHdr = false;
    }

    pSv4eCtx->one_line_exp_time =
        (float)(pSv4eCtx->SensorMode.ae_info.one_line_exp_time_ns) /
        1000000000;
    pSv4eCtx->MaxIntegrationLine =
        pSv4eCtx->SensorMode.ae_info.max_integration_time;
    TRACE(SV4E_INFO, "%s: MaxIntegrationLine %u\n", __func__,
          pSv4eCtx->MaxIntegrationLine);
    pSv4eCtx->MinIntegrationLine =
        pSv4eCtx->SensorMode.ae_info.min_integration_time;
    pSv4eCtx->gain_accuracy = pSv4eCtx->SensorMode.ae_info.gain_accuracy;
    pSv4eCtx->AecMaxGain = (float)(pSv4eCtx->SensorMode.ae_info.max_gain) /
                             pSv4eCtx->gain_accuracy;
    pSv4eCtx->AecMinGain = (float)(pSv4eCtx->SensorMode.ae_info.min_gain) /
                             pSv4eCtx->gain_accuracy;

    pSv4eCtx->AecMinIris = SV4E_IRIS_MIN_VAL;
    pSv4eCtx->AecMaxIris = SV4E_IRIS_MAX_VAL;

    pSv4eCtx->original_vmax = 0;
    pSv4eCtx->unlimit_fps = 0;
    pSv4eCtx->unlimit_fps_vmax_changed = 0;
    pSv4eCtx->flicker_fps_mode = ISI_AE_ANTIBANDING_MODE_OFF;

    pSv4eCtx->MaxFps = pSv4eCtx->SensorMode.fps;
    pSv4eCtx->CurrFps = pSv4eCtx->MaxFps;
    TRACE(SV4E_INFO, "%s - got caps - width %d height %d buswidth %d\n",
          __func__, pSv4eCtx->SensorMode.size.width,
          pSv4eCtx->SensorMode.size.height, pSv4eCtx->SensorMode.bit_width);

    TRACE(SV4E_INFO, "%s - MinGain %f MaxGain %f\n", __func__, pSv4eCtx->AecMinGain,
          pSv4eCtx->AecMaxGain);

    memcpy(&pSv4eCtx->CapsConfig, pCaps, sizeof(pSv4eCtx->CapsConfig));

    /* SW reset comment preserved - no actual reset code was present,
     * and the 100ms sleep was unnecessary (no hardware access in this function). */

    result = Sv4e_AecSetModeParameters(pSv4eCtx, pCaps);
    CHECK_RESULT_RET(result, "AecSetModeParameters");

    pSv4eCtx->Configured = BOOL_TRUE;
    TRACE(SV4E_INFO, "%s: (exit)\n", __func__);
    return 0;
}

static RESULT Sv4e_IsiCheckConnectionIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    return (result);
}

static RESULT Sv4e_IsiGetRevisionIss(IsiSensorHandle_t handle,
                                       uint32_t* pValue) {
    RESULT result = RET_SUCCESS;
    int ret = 0;
    uint32_t reg_val;
    uint32_t sensor_id;

    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    if (pSv4eCtx == NULL || pSv4eCtx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }
    HalContext_t* pHalCtx = (HalContext_t*)pSv4eCtx->IsiCtx.HalHandle;

    if (!pValue) return (RET_NULL_POINTER);

    if (pSv4eCtx->KernelDriverFlag) {
        ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, &sensor_id);
        if (ret != 0) {
            TRACE(SV4E_ERROR, "%s: Read Sensor ID Error! \n", __func__);
            return (RET_FAILURE);
        }
    } else {
        reg_val = 0;
        result = Sv4e_IsiReadRegIss(handle, 0x3a04, &reg_val);
        CHECK_RESULT_RET(result, "read sensor ID high");
        sensor_id = (reg_val & 0xff) << 8;

        reg_val = 0;
        result = Sv4e_IsiReadRegIss(handle, 0x3a05, &reg_val);
        CHECK_RESULT_RET(result, "read sensor ID low");
        sensor_id |= (reg_val & 0xff);
    }

    *pValue = sensor_id;
    TRACE(SV4E_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT Sv4e_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t is_on) {
    RESULT result = RET_SUCCESS;
    TRACE(SV4E_INFO, "%s (enter)\n", __func__);

    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    if (pSv4eCtx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    pSv4eCtx->Streaming = is_on;

    if (pSv4eCtx->enableHdr)
        return result;

    TRACE(SV4E_INFO, "%s (exit)\n", __func__);
    return (result);
}

static RESULT Sv4e_IsiGetGainLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinGain, float* pMaxGain) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(SV4E_INFO, "%s: (enter)\n", __func__);

    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinGain == NULL) || (pMaxGain == NULL)) {
        TRACE(SV4E_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinGain = pSv4eCtx->AecMinGain;
    *pMaxGain = pSv4eCtx->AecMaxGain;

    TRACE(SV4E_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT Sv4e_GetNumExposures(Sv4e_Context_t* pSv4eCtx, size_t* num_exposures) {
    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    if (num_exposures == NULL) {
        TRACE(SV4E_ERROR, "%s: Invalid output parameter\n", __func__);
        return RET_NULL_POINTER;
    }

    if (pSv4eCtx->SensorMode.hdr_mode == SENSOR_MODE_LINEAR) {
        *num_exposures = 1; // SDR
    } else if (pSv4eCtx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
        *num_exposures = SV4E_2DOL_NUM_EXP;
    } else if (pSv4eCtx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
        *num_exposures = SV4E_3DOL_NUM_EXP;
    } else {
        TRACE(SV4E_ERROR, "%s: Unsupported HDR mode %d with stitching mode %d\n",
            __func__, pSv4eCtx->SensorMode.hdr_mode, pSv4eCtx->SensorMode.stitching_mode);
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

static RESULT Sv4e_UpdateCurrLEFIntegrationTimeFromFsc(Sv4e_Context_t* pSv4eCtx, uint32_t fsc, uint32_t shr0) {
    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    float configuredIntegrationTime = (fsc - shr0) * pSv4eCtx->one_line_exp_time;

    pSv4eCtx->OldIntegrationTime = configuredIntegrationTime;
    pSv4eCtx->AecCurIntegrationTimeLEF = configuredIntegrationTime;

    TRACE(SV4E_DEBUG, "%s: Updated LEF Integration Time = (fsc[%u] - shr0[%u]) * one_line_exp_time[%f] = %f\n",
          __func__, fsc, shr0, pSv4eCtx->one_line_exp_time, configuredIntegrationTime);
    return RET_SUCCESS;
}

static RESULT Sv4e_UpdateCurrLEFIntegrationTimeFromVmax(Sv4e_Context_t* pSv4eCtx, uint32_t vmax, uint32_t shr0) {
    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR, "%s: Invalid sensor context\n", __func__);
        return RET_NULL_POINTER;
    }

    size_t dol = 0;
    RESULT result = Sv4e_GetNumExposures(pSv4eCtx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");
    uint32_t fsc = vmax * dol;
    TRACE(SV4E_DEBUG, "%s: fsc = vmax[%u] * dol[%zu] = %u\n", __func__, vmax, dol, fsc);
    result = Sv4e_UpdateCurrLEFIntegrationTimeFromFsc(pSv4eCtx, fsc, shr0);
    CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromFsc");
    return RET_SUCCESS;
}

static inline int Sv4e_getFlickerPeaksPerSec(IsiSensorAntibandingMode_t mode) {
    int num_modes = sizeof(flickerPeaksPerSecMap) / sizeof(FlickerModePeaksPerSec);
    for (int i = 0; i < num_modes; i++) {
        if (flickerPeaksPerSecMap[i].mode == mode) {
            return flickerPeaksPerSecMap[i].value;
        }
    }
    return 0; // Default to 0 if mode not found
}

static inline RESULT Sv4e_getNewVmaxAntiFlicker(Sv4e_Context_t *pSv4eCtx, uint32_t requestedVmax, uint32_t *outClosestVmax) {
    uint32_t closestVmax = requestedVmax;
    int peaks = 0;
    int difference = INT_MAX;
    int minDifference = INT_MAX;
    if (!pSv4eCtx) {
        TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return RET_FAILURE;
    }
    peaks = Sv4e_getFlickerPeaksPerSec(pSv4eCtx->flicker_fps_mode);
    if (peaks == 0) {
        // No anti-flicker mode, return requested or original Vmax
        if (pSv4eCtx->unlimit_fps && pSv4eCtx->unlimit_fps_vmax_changed)
            *outClosestVmax = requestedVmax;
        else
            *outClosestVmax = pSv4eCtx->original_vmax;
        return RET_SUCCESS;
    }

    for (int i = 1; i < peaks; ++i) {
        uint32_t candidateVmax = i / (peaks * pSv4eCtx->one_line_exp_time);
        if (candidateVmax < pSv4eCtx->SensorMode.size.height || candidateVmax < pSv4eCtx->original_vmax)
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

static RESULT Sv4e_IsiUnlimitFpsIss(IsiSensorHandle_t handle,
                                      float maxIntegrationTime) {
    RESULT result = RET_SUCCESS;
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;

    if (!pSv4eCtx) {
        return RET_NULL_POINTER;
    }

    if (pSv4eCtx->enableHdr) {
        return result;
    }
    pSv4eCtx->unlimit_fps = 1;

    if (maxIntegrationTime < 0)
        pSv4eCtx->MaxIntegrationLine = SV4E_VMAX_MAX - SV4E_MIN_SHR;
    else
        pSv4eCtx->MaxIntegrationLine =
            MIN((uint32_t)(maxIntegrationTime / pSv4eCtx->one_line_exp_time),
                SV4E_VMAX_MAX - SV4E_MIN_SHR);
    TRACE(SV4E_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
        pSv4eCtx->MaxIntegrationLine);
    pSv4eCtx->AecMaxIntegrationTime =
        pSv4eCtx->one_line_exp_time * pSv4eCtx->MaxIntegrationLine;
    return result;
}

static RESULT Sv4e_IsiLimitFpsIss(IsiSensorHandle_t handle) {
    RESULT result = RET_SUCCESS;
    uint32_t current_vmax = 0;
    uint32_t new_vmax = 0;
    TRACE(SV4E_INFO, "%s (enter)\n", __func__);
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    if (!pSv4eCtx) {
        return RET_NULL_POINTER;
    }

    if (pSv4eCtx->enableHdr)
        return result;

    result = Sv4e_ReadVmax(handle, &current_vmax);
    CHECK_RESULT_RET(result, "ReadVmax");
    if (current_vmax == 0) {
        TRACE(SV4E_INFO, "%s - exit because current_vmax is 0\n", __func__);
        return result;
    }

    pSv4eCtx->unlimit_fps = 0;
    pSv4eCtx->unlimit_fps_vmax_changed = 0;
    if (pSv4eCtx->original_vmax == 0) {
        pSv4eCtx->original_vmax = current_vmax;
    }

    new_vmax = pSv4eCtx->original_vmax;
    if (pSv4eCtx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
        result = Sv4e_getNewVmaxAntiFlicker(pSv4eCtx, pSv4eCtx->original_vmax, &new_vmax);
        CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
        TRACE(SV4E_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pSv4eCtx->flicker_fps_mode, new_vmax);
    }
    if (current_vmax != new_vmax) {
        result = Sv4e_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        result = Sv4e_WriteVmax(handle, new_vmax);
        CHECK_RESULT_RET(result, "WriteVmax");
        result = Sv4e_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");

        int shr = MAX((int)current_vmax - (int)(pSv4eCtx->AecCurIntegrationTimeLEF / pSv4eCtx->one_line_exp_time), SV4E_MIN_SHR);
        float configuredIntegrationTime = (new_vmax - shr) * pSv4eCtx->one_line_exp_time;
        pSv4eCtx->OldIntegrationTime = configuredIntegrationTime;
        pSv4eCtx->AecCurIntegrationTimeLEF = configuredIntegrationTime;
    }

    pSv4eCtx->MaxIntegrationLine =
        MAX(new_vmax - SV4E_MIN_SHR, 1);
    TRACE(SV4E_INFO, "%s: set MaxIntegrationLine to %u\n", __func__,
        pSv4eCtx->MaxIntegrationLine);
    pSv4eCtx->AecMaxIntegrationTime =
        pSv4eCtx->one_line_exp_time * pSv4eCtx->MaxIntegrationLine;
    return result;
}

static RESULT Sv4e_IsiGetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float* pMinIris, float* pMaxIris) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(SV4E_INFO, "%s: (enter)\n", __func__);

    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIris == NULL) || (pMaxIris == NULL)) {
        TRACE(SV4E_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIris = pSv4eCtx->AecMinIris;
    *pMaxIris = pSv4eCtx->AecMaxIris;

    TRACE(SV4E_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT Sv4e_IsiSetIrisLimitsIss(IsiSensorHandle_t handle,
                                         float minIris, float maxIris) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(SV4E_INFO, "%s: (enter)\n", __func__);

    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    pSv4eCtx->AecMinIris = minIris;
    pSv4eCtx->AecMaxIris = maxIris;

    TRACE(SV4E_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT Sv4e_IsiGetIntegrationTimeLimitsIss(
    IsiSensorHandle_t handle, float* pMinIntegrationTime,
    float* pMaxIntegrationTime) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    TRACE(SV4E_INFO, "%s: (enter)\n", __func__);
    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL)) {
        TRACE(SV4E_ERROR, "%s: NULL pointer received!!\n", __func__);
        return (RET_NULL_POINTER);
    }

    *pMinIntegrationTime = pSv4eCtx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pSv4eCtx->AecMaxIntegrationTime;

    TRACE(SV4E_INFO, "%s: (exit) %f, %f\n", 
    __func__, *pMinIntegrationTime, *pMaxIntegrationTime);
    return (result);
}

/* Gain get functions*/

RESULT Sv4e_IsiGetGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;
	TRACE(SV4E_INFO, "%s: (enter)\n", __func__);

	if (pSv4eCtx == NULL) {
		TRACE(SV4E_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	if (pSv4eCtx->enableHdr) {
		RESULT result = Sv4e_IsiGetSEF1GainIss(handle, pSetGain);
		CHECK_RESULT_RET(result, "GetSEF1Gain");
		return result;
	}

	RESULT result = Sv4e_IsiGetLEFGainIss(handle, pSetGain);
	CHECK_RESULT_RET(result, "GetLEFGain");
	return result;
}

RESULT Sv4e_IsiGetLEFGainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	TRACE(SV4E_INFO, "%s: (enter)\n", __func__);

	if (pSv4eCtx == NULL) {
		TRACE(SV4E_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}
	*pSetGain = pSv4eCtx->AecCurGainLEF;
	TRACE(SV4E_DEBUG, "%s - returning %f\n", __func__, pSv4eCtx->AecCurGainLEF);
	TRACE(SV4E_INFO, "%s: (exit)\n", __func__);
	return (result);
}

RESULT Sv4e_IsiGetSEF1GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(SV4E_INFO, "%s: (enter)\n", __func__);

	if (pSv4eCtx == NULL) {
		TRACE(SV4E_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pSv4eCtx->AecCurGainSEF1;
	TRACE(SV4E_DEBUG, "%s - returning %f\n", __func__, pSv4eCtx->AecCurGainSEF1);

	TRACE(SV4E_INFO, "%s: (exit)\n", __func__);

	return (result);
}

RESULT Sv4e_IsiGetSEF2GainIss(IsiSensorHandle_t handle, float *pSetGain)
{
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(SV4E_INFO, "%s: (enter)\n", __func__);

	if (pSv4eCtx == NULL) {
		TRACE(SV4E_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSetGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSetGain = pSv4eCtx->AecCurGainSEF2;
	TRACE(SV4E_DEBUG, "%s - returning %f\n", __func__, pSv4eCtx->AecCurGainSEF2);

	TRACE(SV4E_INFO, "%s: (exit)\n", __func__);

	return (result);
}

RESULT Sv4e_IsiGetGainIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(SV4E_INFO, "%s: (enter)\n", __func__);

    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = pSv4eCtx->AecGainIncrement;

    TRACE(SV4E_INFO, "%s: (exit)\n", __func__);

    return (result);
}

RESULT Sv4e_IsiGetIrisIncrementIss(IsiSensorHandle_t handle, float* pIncr) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    RESULT result = RET_SUCCESS;
    TRACE(SV4E_INFO, "%s: (enter)\n", __func__);
    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (pIncr == NULL) return (RET_NULL_POINTER);

    *pIncr = 1;

    TRACE(SV4E_INFO, "%s: (exit)\n", __func__);

    return (result);
}

/* Gain set functions*/

RESULT Sv4e_IsiSetGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;
	if (pSv4eCtx == NULL || pSv4eCtx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	if (pSv4eCtx->enableHdr) {
		result = Sv4e_IsiSetSEF1GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1Gain");

		result = Sv4e_IsiSetSEF2GainIss(handle, 0, NewGain, pSetGain,
						hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF2Gain");
	}

	result = Sv4e_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
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

RESULT Sv4e_IsiSetLEFGainIss(IsiSensorHandle_t handle, float NewGain,
			    float *pSetGain, float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;
	if (pSv4eCtx == NULL || pSv4eCtx->IsiCtx.HalHandle == NULL) {
		return RET_NULL_POINTER;
	}

	TRACE(SV4E_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(SV4E_DEBUG, "%s: writting 0x%x to GAIN\n", __func__, Gain);

	result = Sv4e_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = Sv4e_WriteGain(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain");
	result = Sv4e_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pSv4eCtx->AecCurGainLEF = _sensorGain2linear(Gain);
	*pSetGain = pSv4eCtx->AecCurGainLEF;
	TRACE(SV4E_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return RET_SUCCESS;
}

RESULT Sv4e_IsiSetSEF1GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pSv4eCtx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);

	TRACE(SV4E_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(SV4E_DEBUG, "%s: writting 0x%x to GAIN1\n", __func__, Gain);

	result = Sv4e_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = Sv4e_WriteGain1(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain1");
	result = Sv4e_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pSv4eCtx->AecCurGainSEF1 = _sensorGain2linear(Gain);
	*pSetGain = pSv4eCtx->AecCurGainSEF1;

	TRACE(SV4E_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return RET_SUCCESS;
}

RESULT Sv4e_IsiSetSEF2GainIss(IsiSensorHandle_t handle,
				float NewIntegrationTime, float NewGain,
				float *pSetGain, float *hdr_ratio)
{
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pSv4eCtx) {
		return (RET_WRONG_HANDLE);
	}

	if (!pSetGain || !hdr_ratio)
		return (RET_NULL_POINTER);
	
	TRACE(SV4E_DEBUG, "%s: got NewGain %f\n",
	__func__, NewGain);

	uint32_t Gain = _linear2sensorGain(NewGain);
	TRACE(SV4E_DEBUG, "%s: writting 0x%x to GAIN2\n", __func__, Gain);

	result = Sv4e_LockRegHold(handle);
	CHECK_RESULT_RET(result, "LockRegHold");
	result = Sv4e_WriteGain2(handle, Gain);
	CHECK_RESULT_RET(result, "WriteGain2");
	result = Sv4e_UnlockRegHold(handle);
	CHECK_RESULT_RET(result, "UnlockRegHold");

	pSv4eCtx->AecCurGainSEF2 = _sensorGain2linear(Gain);
	*pSetGain = pSv4eCtx->AecCurGainSEF2;

	TRACE(SV4E_DEBUG, "%s: g=%f\n", __func__, *pSetGain);
	return RET_SUCCESS;
}

/* Integration Time get functions*/

RESULT Sv4e_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;

	if (!pSv4eCtx) {
		TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	
	TRACE(SV4E_DEBUG, "%s - enter\n", __func__);
	
	if (pSv4eCtx->enableHdr) {
		RESULT result = Sv4e_IsiGetSEF1IntegrationTimeIss(handle, pSetIntegrationTime);
		CHECK_RESULT_RET(result, "GetSEF1IntegrationTime");
		return result;
	}
	
	RESULT result = Sv4e_IsiGetLEFIntegrationTimeIss(handle, pSetIntegrationTime);
	CHECK_RESULT_RET(result, "GetLEFIntegrationTime");
	return result;
}

RESULT Sv4e_IsiGetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float *pSetIntegrationTime)
{
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;
	RESULT result = RET_SUCCESS;


	if (!pSv4eCtx) {
		TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pSv4eCtx->AecCurIntegrationTimeLEF;
	TRACE(SV4E_DEBUG, "%s - returning %f\n", __func__, pSv4eCtx->AecCurIntegrationTimeLEF);
	return (result);
}

RESULT Sv4e_IsiGetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pSv4eCtx) {
		TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);
	*pSetIntegrationTime = pSv4eCtx->AecCurIntegrationTimeSEF1;
	TRACE(SV4E_DEBUG, "%s - returning %f\n", __func__, pSv4eCtx->AecCurIntegrationTimeSEF1);
	return (result);
}

RESULT Sv4e_IsiGetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float *pSetIntegrationTime)
{
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (!pSv4eCtx) {
		TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}
	if (!pSetIntegrationTime)
		return (RET_NULL_POINTER);

	*pSetIntegrationTime = pSv4eCtx->AecCurIntegrationTimeSEF2;
	TRACE(SV4E_DEBUG, "%s - returning %f\n", __func__, pSv4eCtx->AecCurIntegrationTimeSEF2);
	return (result);
}

RESULT Sv4e_IsiGetIntegrationTimeIncrementIss(IsiSensorHandle_t handle,
                                                float* pIncr) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (!pSv4eCtx) {
        TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pIncr) return (RET_NULL_POINTER);

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders
    //in
    // the application)
    *pIncr = pSv4eCtx->AecIntegrationTimeIncrement;
    return (result);
}

/* Integration Time set functions*/

RESULT Sv4e_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;

	if (!pSv4eCtx) {
		TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSv4eCtx->enableHdr) {
		RESULT result = Sv4e_IsiSetSEF1IntegrationTimeIss(
			handle, NewIntegrationTime, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1IntegrationTime");
		return result;
	}

	RESULT result = Sv4e_IsiSetLEFIntegrationTimeIss(
		handle, NewIntegrationTime, pSetIntegrationTime,
		pNumberOfFramesToSkip, hdr_ratio);
	CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
	return result;
}

RESULT Sv4e_IsiSetLEFIntegrationTimeIss(IsiSensorHandle_t handle,
				       float NewIntegrationTime,
				       float *pSetIntegrationTime,
				       uint8_t *pNumberOfFramesToSkip,
				       float *hdr_ratio)
{
	RESULT result = RET_SUCCESS;

    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;

    int shr = 0;
    uint32_t exp = 0;
    uint32_t new_vmax = 0;
    uint32_t vmax_updated = 0;
    uint32_t current_vmax = 0;
    uint32_t rhs1;
    uint32_t rhs2;
    if (!pSv4eCtx) {
        TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (!pSetIntegrationTime || !pNumberOfFramesToSkip) {
        TRACE(SV4E_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (!pSv4eCtx->Streaming) {
        TRACE(SV4E_ERROR, "%s: sensor not streaming\n", __func__);
        return RET_FAILURE;
    }
    exp = roundf(NewIntegrationTime / pSv4eCtx->one_line_exp_time);

    TRACE(SV4E_DEBUG, "%s: set AEC_PK_EXPO=0x%05x\n", __func__, exp);

    if (fabs(NewIntegrationTime - pSv4eCtx->AecCurIntegrationTimeLEF) > FLT_EPSILON) {

        if (pSv4eCtx->enableHdr){
            if (pSv4eCtx->cur_rhs1 == 0 || pSv4eCtx->cur_rhs2 == 0) {
                TRACE(SV4E_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
                return (RET_WRONG_CONFIG);
            }
	

            rhs1 = pSv4eCtx->cur_rhs1;
            rhs2 = pSv4eCtx->cur_rhs2;
            if (pSv4eCtx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
                result = Sv4e_ReadVmax(pSv4eCtx, &new_vmax);
                CHECK_RESULT_RET(result, "ReadVmax");
                new_vmax *= SV4E_2DOL_NUM_EXP;

                exp = new_vmax - exp;
                exp = exp > rhs1 + SV4E_2DOL_SHR0_RHS1_GAP ? exp : rhs1 + SV4E_2DOL_SHR0_RHS1_GAP;
                exp = exp < new_vmax - SV4E_2DOL_SHR0_FSC_GAP? exp : new_vmax - SV4E_2DOL_SHR0_FSC_GAP;
                shr = exp;
            } else {
                new_vmax = SV4E_VMAX_3DOL_HDR;
                exp = new_vmax - exp;
                exp = exp > rhs2 + SV4E_SHR0_RHS2_GAP ? exp : rhs2 + SV4E_SHR0_RHS2_GAP;
                exp = exp < new_vmax - SV4E_SHR0_FSC_GAP? exp : new_vmax - SV4E_SHR0_FSC_GAP;
                shr = exp;
            }
            
		} else {
            if (exp > pSv4eCtx->MaxIntegrationLine || exp == 0) {
                TRACE(SV4E_ERROR, "%s: Integration time %f (exp %u) out of range (%u)\n", __func__,
                    NewIntegrationTime, exp, pSv4eCtx->MaxIntegrationLine);
                return RET_FAILURE;
            }
            result = Sv4e_ReadVmax(handle, &current_vmax);
            CHECK_RESULT_RET(result, "ReadVmax");

            if (pSv4eCtx->original_vmax == 0) {
                pSv4eCtx->original_vmax = current_vmax;
            }

            if (pSv4eCtx->original_vmax - SV4E_MIN_SHR >
                pSv4eCtx->MaxIntegrationLine) {
                pSv4eCtx->MaxIntegrationLine =
                    pSv4eCtx->original_vmax - SV4E_MIN_SHR;
                TRACE(SV4E_DEBUG, "%s: set MaxIntegrationLine to %u\n", __func__,
                      pSv4eCtx->MaxIntegrationLine);
                pSv4eCtx->AecMaxIntegrationTime =
                    pSv4eCtx->one_line_exp_time * pSv4eCtx->MaxIntegrationLine;
                TRACE(SV4E_DEBUG, "%s: set AecMaxIntegrationTime to %f\n", __func__,
                      pSv4eCtx->AecMaxIntegrationTime);
            }

            shr = current_vmax - exp;

            if (shr < SV4E_MIN_SHR || current_vmax > pSv4eCtx->original_vmax) {
                if (shr < SV4E_MIN_SHR) {
                    // user asked for too big integration-line: more than (current_vmax - minimal_shutter)
                    // increase vmax and set new shutter to minimal value
                    new_vmax = MIN(exp + SV4E_MIN_SHR, pSv4eCtx->MaxIntegrationLine + SV4E_MIN_SHR);
                } else {
                    // make sure that when we unlimit fps, stay below original fps.
                    // also, make sure we are not using high vmax + high shr more than needed (will slow fps for no reason)
                    new_vmax = MAX(current_vmax - shr + SV4E_MIN_SHR, pSv4eCtx->original_vmax);
                }
                pSv4eCtx->unlimit_fps_vmax_changed = new_vmax > pSv4eCtx->original_vmax && pSv4eCtx->unlimit_fps;
        
                if (pSv4eCtx->flicker_fps_mode != ISI_AE_ANTIBANDING_MODE_OFF) {
                    result = Sv4e_getNewVmaxAntiFlicker(pSv4eCtx, new_vmax, &new_vmax);
                    CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
                    TRACE(SV4E_DEBUG, "%s -Anti Flicker Fps mode %d, set new vmax %u\n", __func__, pSv4eCtx->flicker_fps_mode, new_vmax);
                }
                
                shr = MAX(SV4E_MIN_SHR, (int)new_vmax - (int)exp);
                vmax_updated = 1;

            } else {
                new_vmax = current_vmax;
            }
        }

        result = Sv4e_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        if (vmax_updated && pSv4eCtx->unlimit_fps && !pSv4eCtx->enableHdr) {
            result = Sv4e_WriteVmax(handle, new_vmax);
            CHECK_RESULT_RET(result, "WriteVmax");
        }

        TRACE(SV4E_DEBUG, "%s - writing 0x%x to SHR0\n", __func__, shr);
        result = Sv4e_WriteShr0(handle, shr);
        CHECK_RESULT_RET(result, "WriteShr0");
        result = Sv4e_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");

        float configuredIntegrationTime =
            (new_vmax - shr) * pSv4eCtx->one_line_exp_time;
        pSv4eCtx->OldIntegrationTime = configuredIntegrationTime;
        pSv4eCtx->AecCurIntegrationTimeLEF = configuredIntegrationTime;

        *pNumberOfFramesToSkip = 1U;
    } else {
        *pNumberOfFramesToSkip = 0U;  // no frame skip
    }

    *pSetIntegrationTime = pSv4eCtx->AecCurIntegrationTimeLEF;
    TRACE(SV4E_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTime);
    return (result);
}

RESULT Sv4e_IsiSetSEF1IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF1,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;

	if (!pSv4eCtx) {
		TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSv4eCtx->cur_rhs1 == 0 || pSv4eCtx->cur_rhs2 == 0) {
		TRACE(SV4E_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pSv4eCtx->cur_rhs1;

	if (!pSetIntegrationTimeSEF1 || !pNumberOfFramesToSkip) {
		TRACE(SV4E_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(SV4E_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = roundf(NewIntegrationTime / pSv4eCtx->one_line_exp_time);

	if (fabs(NewIntegrationTime - pSv4eCtx->AecCurIntegrationTimeSEF1) > FLT_EPSILON) {
		if (pSv4eCtx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
			exp = rhs1 - exp;
			exp = exp > SV4E_2DOL_SHR1_RHS1_GAP ? exp : SV4E_2DOL_SHR1_RHS1_GAP;
			exp = exp < rhs1 - SV4E_SHR1_RHS1_GAP ? exp : rhs1 - SV4E_SHR1_RHS1_GAP;
		} else {
			exp = rhs1 - exp;
			exp = exp > SV4E_3DOL_SHR1_RHS1_GAP ? exp : SV4E_3DOL_SHR1_RHS1_GAP;
			exp = exp < rhs1 - SV4E_SHR1_RHS1_GAP ? exp : rhs1 - SV4E_SHR1_RHS1_GAP;
		}

		TRACE(SV4E_DEBUG, "%s - writing 0x%x to SHR1\n", __func__, exp);

		result = Sv4e_LockRegHold(handle);
		CHECK_RESULT_RET(result, "LockRegHold");
		result = Sv4e_WriteShr1(handle, exp);
		CHECK_RESULT_RET(result, "WriteShr1");
		result = Sv4e_UnlockRegHold(handle);
		CHECK_RESULT_RET(result, "UnlockRegHold");

		pSv4eCtx->AecCurIntegrationTimeSEF1 = (rhs1 - exp) * pSv4eCtx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF1 = pSv4eCtx->AecCurIntegrationTimeSEF1;

	TRACE(SV4E_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF1);
	return (result);
}

RESULT Sv4e_IsiSetSEF2IntegrationTimeIss(IsiSensorHandle_t handle,
					   float NewIntegrationTime,
					   float *pSetIntegrationTimeSEF2,
					   uint8_t *pNumberOfFramesToSkip,
					   float *hdr_ratio)
{
	Sv4e_Context_t *pSv4eCtx = (Sv4e_Context_t *)handle;
	RESULT result = RET_SUCCESS;
	uint32_t exp = 0;
	uint32_t rhs1;
	uint32_t rhs2;

	if (pSv4eCtx->cur_rhs1 == 0 || pSv4eCtx->cur_rhs2 == 0) {
		TRACE(SV4E_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pSv4eCtx->cur_rhs1;
	rhs2 = pSv4eCtx->cur_rhs2;

	if (!pSv4eCtx) {
		TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		       __func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSetIntegrationTimeSEF2 || !pNumberOfFramesToSkip) {
		TRACE(SV4E_ERROR, "%s: Invalid parameter (NULL pointer detected)\n",
		       __func__);
		return (RET_NULL_POINTER);
	}
	TRACE(SV4E_DEBUG, "%s: NewIntegrationTime = %f\n", __func__, NewIntegrationTime);

	exp = roundf(NewIntegrationTime / pSv4eCtx->one_line_exp_time);
	TRACE(SV4E_DEBUG, "%s - calculated IT in rows = 0x%x\n", __func__, exp);

	if (fabs(NewIntegrationTime - pSv4eCtx->AecCurIntegrationTimeSEF2) > FLT_EPSILON) {
		exp = rhs2 - exp;
		exp = exp > rhs1 + SV4E_SHR2_RHS1_GAP ? exp : rhs1 + SV4E_SHR2_RHS1_GAP;
		exp = exp < rhs2 - SV4E_SHR2_RHS2_GAP ? exp : rhs2 - SV4E_SHR2_RHS2_GAP;
		TRACE(SV4E_DEBUG, "%s - writing 0x%x to SHR2\n", __func__, exp);

		result = Sv4e_LockRegHold(handle);
		CHECK_RESULT_RET(result, "LockRegHold");
		result = Sv4e_WriteShr2(handle, exp);
		CHECK_RESULT_RET(result, "WriteShr2");
		result = Sv4e_UnlockRegHold(handle);
		CHECK_RESULT_RET(result, "UnlockRegHold");

		pSv4eCtx->AecCurIntegrationTimeSEF2 = (rhs2 - exp) * pSv4eCtx->one_line_exp_time; // in sec
		*pNumberOfFramesToSkip = 1U;
	} else {
		*pNumberOfFramesToSkip = 0U;
	}

	*pSetIntegrationTimeSEF2 = pSv4eCtx->AecCurIntegrationTimeSEF2;

	TRACE(SV4E_DEBUG, "%s: Ti=%f\n", __func__, *pSetIntegrationTimeSEF2);
	return (result);
}

RESULT Sv4e_Calculate3DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it, float *o_very_short_it,
                                    float *o_long_gain, float *o_short_gain, float *o_very_short_gain,
                                    float *hdr_ratio) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
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

    if (pSv4eCtx == NULL || o_long_it == NULL || o_short_it == NULL ||
        o_very_short_it == NULL || o_long_gain == NULL || o_short_gain == NULL ||
        o_very_short_gain == NULL || hdr_ratio == NULL) {
        TRACE(SV4E_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

	if (pSv4eCtx->cur_rhs1 == 0 || pSv4eCtx->cur_rhs2 == 0) {
		TRACE(SV4E_ERROR, "%s: Invalid parameter (RHS1 or RHS2 not set)\n", __func__);
		return (RET_WRONG_CONFIG);
	}

	rhs1 = pSv4eCtx->cur_rhs1;
	rhs2 = pSv4eCtx->cur_rhs2;

    TRACE(SV4E_DEBUG, "%s: hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n", 
    __func__, hdr_ratio[0], hdr_ratio[1]);
    
    // Sometimes there is no actual input gain. In that case, we will read it from the sensor
    if (NewGain == 0) {
        TRACE(SV4E_DEBUG, "%s: Input NewGain is 0, reading gain from sensor\n", __func__);
        result = Sv4e_IsiGetSEF1GainIss(handle, &NewGain);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    // Same for integration time
    if (NewIntegrationTime == 0) {
        TRACE(SV4E_DEBUG, "%s: Input NewIntegrationTime is 0, reading integration time from sensor\n", __func__);
        result = Sv4e_IsiGetSEF1IntegrationTimeIss(handle, &NewIntegrationTime);
        if (result != RET_SUCCESS) {
            return result;
        }
        calculate_gain = true;
    }

    if(NewIntegrationTime < SV4E_SHR2_RHS2_GAP * pSv4eCtx->one_line_exp_time * hdr_ratio[1]){
	    pSv4eCtx->MinIntegrationLine =  SV4E_SHR2_RHS2_GAP * hdr_ratio[1];
	    pSv4eCtx->AecMinIntegrationTime = pSv4eCtx->MinIntegrationLine * pSv4eCtx->one_line_exp_time;
    }

    // assume gain is 1 and see if ratio can be achieved with integration time
    long_it 		= NewIntegrationTime * hdr_ratio[0];
    short_it 		= NewIntegrationTime;
    very_short_it 	= NewIntegrationTime / hdr_ratio[1];
    
    TRACE(SV4E_DEBUG, "%s: requested IT long: %f, short: %f, very_short: %f\n", 
    __func__, long_it, short_it, very_short_it);
    long_exp_val 		= long_it / pSv4eCtx->one_line_exp_time;
    short_exp_val 		= short_it / pSv4eCtx->one_line_exp_time;
    very_short_exp_val 	= very_short_it / pSv4eCtx->one_line_exp_time;

    TRACE(SV4E_DEBUG, "%s: requested IT in lines long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    long_exp_val 		= SV4E_VMAX_3DOL_HDR - long_exp_val;
    short_exp_val 		= rhs1 - short_exp_val;
    very_short_exp_val 	= rhs2 - very_short_exp_val;

    TRACE(SV4E_DEBUG, "%s: requested IT in shr long: %f, short: %f, very_short: %f\n", 
    __func__, long_exp_val, short_exp_val, very_short_exp_val);
    if(long_exp_val < rhs2 + SV4E_SHR0_RHS2_GAP) {
        long_exp_val = rhs2 + SV4E_SHR0_RHS2_GAP;
        long_it = (SV4E_VMAX_3DOL_HDR - long_exp_val) * pSv4eCtx->one_line_exp_time;
        calculate_gain = true;
        optimize_long_gain = true;
        TRACE(SV4E_DEBUG, "%s: long_exp_val is too long, set to %u, new long_it = %f\n",
        __func__, rhs2 + SV4E_SHR0_RHS2_GAP, long_it);
    } else if(long_exp_val > SV4E_VMAX_3DOL_HDR - SV4E_SHR0_FSC_GAP) {
        long_exp_val = SV4E_VMAX_3DOL_HDR - SV4E_SHR0_FSC_GAP;
        long_it = (SV4E_VMAX_3DOL_HDR - long_exp_val) * pSv4eCtx->one_line_exp_time;
        calculate_gain = true;
        TRACE(SV4E_DEBUG, "%s: long_exp_val is too short, set to %u, new long_it = %f\n",
        __func__, SV4E_VMAX_3DOL_HDR - SV4E_SHR0_FSC_GAP, long_it);
    }
    if(short_exp_val < SV4E_3DOL_SHR1_RHS1_GAP) {
        short_exp_val = SV4E_3DOL_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pSv4eCtx->one_line_exp_time;
        calculate_gain = true;
        TRACE(SV4E_DEBUG, "%s: short_exp_val is too long, set to %u, new short_it = %f\n",
        __func__, SV4E_3DOL_SHR1_RHS1_GAP, short_it);
    } else if(short_exp_val > rhs1 - SV4E_SHR1_RHS1_GAP) {
        short_exp_val = rhs1 - SV4E_SHR1_RHS1_GAP;
        short_it = (rhs1 - short_exp_val) * pSv4eCtx->one_line_exp_time;
        calculate_gain = true;
        TRACE(SV4E_DEBUG, "%s: short_exp_val is too short, set to %u, new short_it = %f\n",
        __func__, rhs1 - SV4E_SHR1_RHS1_GAP, short_it);
    }
    if(very_short_exp_val < rhs1 + SV4E_SHR2_RHS1_GAP) {
        very_short_exp_val = rhs1 + SV4E_SHR2_RHS1_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pSv4eCtx->one_line_exp_time;
        calculate_gain = true;
        optimize_short_gain = true;
        TRACE(SV4E_DEBUG, "%s: very_short_exp_val is too long, set to %u, new very_short_it = %f\n",
        __func__, rhs2 + SV4E_SHR2_RHS1_GAP, very_short_it);
    } else if(very_short_exp_val > rhs2 - SV4E_SHR2_RHS2_GAP) {
        very_short_exp_val = rhs2 - SV4E_SHR2_RHS2_GAP;
        very_short_it = (rhs2 - very_short_exp_val) * pSv4eCtx->one_line_exp_time;
        calculate_gain = true;
        TRACE(SV4E_DEBUG, "%s: very_short_exp_val is too short, set to %u, new very_short_it = %f\n",
        __func__, rhs2 - SV4E_SHR2_RHS2_GAP, very_short_it);
    }

    // need to use gain to achive ratio / requested gain update
    if(calculate_gain || NewGain != pSv4eCtx->AecCurGainSEF1) {
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
        TRACE(SV4E_DEBUG, "%s: calculated gain: long: %f, short: %f, very_short: %f\n",
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

RESULT Sv4e_Calculate2DOLExposures(IsiSensorHandle_t handle, float NewIntegrationTime, float NewGain,
                                    float *o_long_it, float *o_short_it,
                                    float *o_long_gain, float *o_short_gain,
                                    float *hdr_ratio) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
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

    TRACE(SV4E_DEBUG, "%s: enter with NewIntegrationTime=%.6f NewGain=%.4f ratio=%.1f\n",
        __func__, NewIntegrationTime, NewGain, hdr_ratio[0]);

    result = Sv4e_ReadVmax(handle, &vmax);
    CHECK_RESULT_RET(result, "ReadVmax");

    result = Sv4e_GetNumExposures(pSv4eCtx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");

    fsc = vmax * (uint32_t)dol;
    rhs1 = pSv4eCtx->cur_rhs1;
    one_line = pSv4eCtx->one_line_exp_time;

    if (one_line <= 0.0f) {
        TRACE(SV4E_ERROR, "%s: Invalid one_line_exp_time (%.9f)\n", __func__, one_line);
        return (RET_WRONG_CONFIG);
    }

    if (fsc <= rhs1 + SV4E_2DOL_SHR0_RHS1_GAP) {
        TRACE(SV4E_ERROR, "%s: fsc(%u) <= rhs1(%u) + gap, invalid config\n",
              __func__, fsc, rhs1);
        return (RET_WRONG_CONFIG);
    }

    /* Hardware limits: SHR0 must satisfy  rhs1 + gap <= SHR0 <= fsc - gap. */
    max_long_it_lines = fsc - rhs1 - SV4E_2DOL_SHR0_RHS1_GAP;
    min_long_it_lines = SV4E_2DOL_SHR0_FSC_GAP;

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
     * Step 1b: When the required EV is within SV4E_2DOL_MAX_IT_NEAR_LINES of
     * max_long_it, the shortfall is a float-rounding artifact — accept unity
     * gain and the negligible ratio deviation instead of bumping a gain step.
     */
    if (long_gain_needed > 1.0f && long_it_lines == max_long_it_lines) {
        required_lines = required_long_ev / one_line;
        if (required_lines - (float)max_long_it_lines < (float)SV4E_2DOL_MAX_IT_NEAR_LINES) {
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

        max_gain_db = _linear2sensorGain(SV4E_MAX_GAIN_AEC);
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

    TRACE(SV4E_DEBUG, "%s: ratio=%.1f vmax=%u fsc=%u rhs1=%u max_long_it_lines=%u\n",
        __func__, hdr_ratio[0], vmax, fsc, rhs1, max_long_it_lines);
    TRACE(SV4E_DEBUG, "%s: required_long_ev=%.6f long_it=%.6f(%u lines) long_gain=%.4f\n",
        __func__, required_long_ev, long_it, long_it_lines, long_gain);
    TRACE(SV4E_DEBUG, "%s: short_it=%.6f short_gain=%.4f\n",
        __func__, *o_short_it, *o_short_gain);

    return RET_SUCCESS;
}

RESULT Sv4e_IsiExposureControlIss(IsiSensorHandle_t handle, float NewGain,
                                    float NewIntegrationTime,
                                    uint8_t* pNumberOfFramesToSkip,
                                    float* pSetGain, float* pSetIntegrationTime,
                                    float* hdr_ratio) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;

    RESULT result = RET_SUCCESS;
    float long_it = 0.0;
	float short_it = 0.0;
	float very_short_it = 0.0;
	float long_gain = 1;
	float short_gain = 1;
	float very_short_gain = 1;
    uint32_t hmax;

    TRACE(SV4E_INFO, "%s: enter with NewIntegrationTime: %f, NewGain: %f\n",
        __func__, NewIntegrationTime, NewGain);

    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pNumberOfFramesToSkip == NULL) || (pSetGain == NULL) ||
        (pSetIntegrationTime == NULL)) {
        TRACE(SV4E_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __func__);
        return (RET_NULL_POINTER);
    }

    if (pSv4eCtx->enableHdr) {
        result = Sv4e_ReadRHS1(handle, &pSv4eCtx->cur_rhs1);
        CHECK_RESULT_RET(result, "ReadRHS1");
        result = Sv4e_ReadRHS2(handle, &pSv4eCtx->cur_rhs2);
        CHECK_RESULT_RET(result, "ReadRHS2");
        result = Sv4e_ReadHmax(handle, &hmax);
        CHECK_RESULT_RET(result, "ReadHmax");

        pSv4eCtx->SensorMode.ae_info.one_line_exp_time_ns = (uint32_t)(((float)hmax / SV4E_PIXEL_CLK_RATE) * MICRO_2_NANO);
        pSv4eCtx->one_line_exp_time =
        (float)(pSv4eCtx->SensorMode.ae_info.one_line_exp_time_ns) / 1000000000;

        if (pSv4eCtx->SensorMode.stitching_mode == SENSOR_STITCHING_L_AND_S) {
            //2exposure
            result = Sv4e_Calculate2DOLExposures(handle, NewIntegrationTime, NewGain,
                                    &long_it, &short_it,
                                    &long_gain, &short_gain,
                                    hdr_ratio);
            CHECK_RESULT_RET(result, "Calculate2DOLExposures");
        } else {
            //3exposure
            result = Sv4e_Calculate3DOLExposures(handle, NewIntegrationTime, NewGain,
                                        &long_it, &short_it, &very_short_it,
                                        &long_gain, &short_gain, &very_short_gain,
                                        hdr_ratio);
            CHECK_RESULT_RET(result, "Calculate3DOLExposures");
        }

		result = Sv4e_IsiSetLEFIntegrationTimeIss(handle, long_it,
							pSetIntegrationTime,
							pNumberOfFramesToSkip,
							hdr_ratio);
		CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
		result = Sv4e_IsiSetLEFGainIss(handle, long_gain, pSetGain, hdr_ratio);
		CHECK_RESULT_RET(result, "SetLEFGain");
		result = Sv4e_IsiSetSEF1IntegrationTimeIss(
			handle, short_it, pSetIntegrationTime,
			pNumberOfFramesToSkip, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1IntegrationTime");
		result = Sv4e_IsiSetSEF1GainIss(handle, NewIntegrationTime,
						  short_gain, pSetGain, hdr_ratio);
		CHECK_RESULT_RET(result, "SetSEF1Gain");
		if (pSv4eCtx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
			result = Sv4e_IsiSetSEF2IntegrationTimeIss(
			    handle, very_short_it, pSetIntegrationTime,
			    pNumberOfFramesToSkip, hdr_ratio);
			CHECK_RESULT_RET(result, "SetSEF2IntegrationTime");
            result = Sv4e_IsiSetSEF2GainIss(handle, NewIntegrationTime,
                            very_short_gain, pSetGain, hdr_ratio);
            CHECK_RESULT_RET(result, "SetSEF2Gain");
        }

        // Recalculate `io_hdr_ratio` according to the set values
        hdr_ratio[0] = (long_it * long_gain) / (short_it * short_gain);
        if (pSv4eCtx->SensorMode.stitching_mode == SENSOR_STITCHING_3DOL) {
            hdr_ratio[1] = (short_it * short_gain) / (very_short_it * very_short_gain);
        }

        // Set the output values to SEF1 values
        *pSetGain = short_gain;
        *pSetIntegrationTime = short_it;

        TRACE(SV4E_DEBUG, "%s: actual hdr_ratio[0] = LS Ratio = %f, hdr_ratio[1] = VS Ratio = %f\n",
            __func__, hdr_ratio[0], hdr_ratio[1]);
    } else {
        result = Sv4e_IsiSetLEFIntegrationTimeIss(handle, NewIntegrationTime,
                                                pSetIntegrationTime,
                                                pNumberOfFramesToSkip, hdr_ratio);
        CHECK_RESULT_RET(result, "SetLEFIntegrationTime");
        result = Sv4e_IsiSetLEFGainIss(handle, NewGain, pSetGain, hdr_ratio);
        CHECK_RESULT_RET(result, "SetLEFGain");
    }
    return RET_SUCCESS;
}

RESULT Sv4e_IsiExposureControlExpandedIss(
    IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime,
    float NewIris, uint8_t* pNumberOfFramesToSkip, float* pSetGain,
    float* pSetIntegrationTime, float* pSetIris, float* hdr_ratio) {

    RESULT result = RET_SUCCESS;
    if (pSetIris) {
        result = Sv4e_IsiSetIrisIss(handle, NewIris);
        CHECK_RESULT_RET(result, "SetIris");
        *pSetIris = NewIris;
    }

    result = Sv4e_IsiExposureControlIss(handle, NewGain, NewIntegrationTime,
                                        pNumberOfFramesToSkip, pSetGain,
                                        pSetIntegrationTime, hdr_ratio);
    CHECK_RESULT_RET(result, "ExposureControl");
    return result;
}

RESULT Sv4e_IsiGetCurrentExposureIss(IsiSensorHandle_t handle,
                                       float* pSetGain,
                                       float* pSetIntegrationTime) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((pSetGain == NULL) || (pSetIntegrationTime == NULL))
        return (RET_NULL_POINTER);

    if (pSv4eCtx->enableHdr) {
		*pSetGain = pSv4eCtx->AecCurGainSEF1;
		*pSetIntegrationTime = pSv4eCtx->AecCurIntegrationTimeSEF1;
	} else {
		*pSetGain = pSv4eCtx->AecCurGainLEF;
		*pSetIntegrationTime = pSv4eCtx->AecCurIntegrationTimeLEF;
	}

    return (result);
}

RESULT Sv4e_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t* pFps) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    RESULT result = RET_SUCCESS;

    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *pFps = pSv4eCtx->CurrFps;

    return (result);
}

RESULT Sv4e_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t Fps) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT Sv4e_IsiSetFlickerFpsIss(IsiSensorHandle_t handle, uint32_t flickerMode) {
    RESULT result = RET_SUCCESS;
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    uint32_t current_vmax = 0;
    uint32_t requested_vmax = 0;
    uint32_t requested_fsc = 0;
    uint32_t shr = 0;
    size_t dol = 0;
    uint32_t fsc = 0;
    uint32_t min_shr0 = (dol == 1) ? SV4E_MIN_SHR : SV4E_2DOL_SHR0_RHS1_GAP + pSv4eCtx->cur_rhs1;
    int exp = 0;

    TRACE(SV4E_DEBUG, "%s: set sensor flickerMode = %d\n", __func__, flickerMode);

    if (!pSv4eCtx) {
        return RET_NULL_POINTER;
    }
    if (pSv4eCtx->flicker_fps_mode == flickerMode) {
        return RET_SUCCESS;
    }
    if (pSv4eCtx->enableHdr && (pSv4eCtx->SensorMode.stitching_mode != SENSOR_STITCHING_L_AND_S)) {
        return RET_SUCCESS;
    }

    result = Sv4e_GetNumExposures(pSv4eCtx, &dol);
    CHECK_RESULT_RET(result, "GetNumExposures");

    if (flickerMode > ISI_AE_ANTIBANDING_MODE_AUTO) {
        TRACE(SV4E_INFO, "%s: Invalid flickerMode (%d), setting ISI_AE_ANTIBANDING_MODE_AUTO instead.\n", __func__, flickerMode);
        flickerMode = ISI_AE_ANTIBANDING_MODE_AUTO;
    }
    pSv4eCtx->flicker_fps_mode = flickerMode;

    result = Sv4e_ReadVmax(handle, &current_vmax);
    CHECK_RESULT_RET(result, "ReadVmax");
    if (pSv4eCtx->original_vmax == 0) {
        pSv4eCtx->original_vmax = current_vmax;
    }

    fsc = current_vmax * dol;

    exp = pSv4eCtx->AecCurIntegrationTimeLEF / pSv4eCtx->one_line_exp_time;
    shr = MAX((int)(fsc) - exp, min_shr0);

    if (current_vmax > pSv4eCtx->original_vmax) {
        current_vmax = MAX((int)fsc - (int)shr + min_shr0, min_shr0);
        fsc = current_vmax * dol;
        shr = MAX((int)fsc - exp, min_shr0);
        pSv4eCtx->unlimit_fps_vmax_changed = current_vmax > pSv4eCtx->original_vmax && pSv4eCtx->unlimit_fps;
    }

    result = Sv4e_getNewVmaxAntiFlicker(pSv4eCtx, current_vmax, &requested_vmax);
    CHECK_RESULT_RET(result, "getNewVmaxAntiFlicker");
    requested_vmax = MAX( MIN(requested_vmax, SV4E_VMAX_MAX), 1);
    requested_fsc = requested_vmax * dol;
    
    if (current_vmax != requested_vmax) {
        shr = MAX( (int)requested_fsc - (int)fsc + (int)shr, min_shr0);
        TRACE(SV4E_DEBUG, "%s - writing 0x%x to VMAX, writing 0x%x to SHR0\n", __func__, requested_vmax, shr);
        
        result = Sv4e_LockRegHold(handle);
        CHECK_RESULT_RET(result, "LockRegHold");
        result = Sv4e_WriteVmax(handle, requested_vmax);
        CHECK_RESULT_RET(result, "WriteVmax");
        result = Sv4e_WriteShr0(handle, shr);
        CHECK_RESULT_RET(result, "WriteShr0");
        result = Sv4e_UnlockRegHold(handle);
        CHECK_RESULT_RET(result, "UnlockRegHold");
        result = Sv4e_UpdateCurrLEFIntegrationTimeFromVmax(pSv4eCtx, requested_vmax, shr);
        CHECK_RESULT_RET(result, "UpdateCurrLEFIntegrationTimeFromVmax");
    }
    
    // these 2 are being used only in SDR
    pSv4eCtx->MaxIntegrationLine = MAX( MIN(requested_vmax - SV4E_MIN_SHR, SV4E_VMAX_MAX - SV4E_MIN_SHR), 1);
    pSv4eCtx->AecMaxIntegrationTime = pSv4eCtx->one_line_exp_time * pSv4eCtx->MaxIntegrationLine;

    TRACE(SV4E_INFO, "%s: set sensor fps = %d\n", __func__,
          pSv4eCtx->CurrFps);

    TRACE(SV4E_DEBUG, "%s: (exit)\n", __func__);
    return (result);
}

RESULT Sv4e_IsiGetAutoFpsInfoIss(IsiSensorHandle_t handle,
                                   IsiAutoFps_t* pAutoFpsInfo) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT Sv4e_IsiGetStartEvIss(IsiSensorHandle_t handle, uint64_t* pStartEv) {
    RESULT result = RET_SUCCESS;
    return (result);
}

RESULT Sv4e_IsiGetIspStatusIss(IsiSensorHandle_t handle,
                                 IsiIspStatus_t* pIspStatus) {
    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    if (pSv4eCtx == NULL || pSv4eCtx->IsiCtx.HalHandle == NULL) {
        return RET_WRONG_HANDLE;
    }

    pIspStatus->useSensorAE = false;
    pIspStatus->useSensorBLC = false;
    pIspStatus->useSensorAWB = false;

    return RET_SUCCESS;
}

RESULT Sv4e_IsiSetTpgIss(IsiSensorHandle_t handle, IsiTpg_t Tpg) {
    RESULT result = RET_SUCCESS;

    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    if (pSv4eCtx == NULL || pSv4eCtx->IsiCtx.HalHandle == NULL) {
        return RET_NULL_POINTER;
    }

    if (pSv4eCtx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    if (Tpg.enable == 0) {
        result = Sv4e_IsiWriteRegIss(handle, 0x3253, 0x00);
        CHECK_RESULT_RET(result, "disable test pattern");
    } else {
        result = Sv4e_IsiWriteRegIss(handle, 0x3253, 0x80);
        CHECK_RESULT_RET(result, "enable test pattern");
    }

    pSv4eCtx->TestPattern = Tpg.enable;

    return (result);
}

RESULT Sv4e_IsiGetTpgIss(IsiSensorHandle_t handle, IsiTpg_t* Tpg) {
    RESULT result = RET_SUCCESS;
    uint32_t value = 0;

    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    if (pSv4eCtx == NULL || pSv4eCtx->IsiCtx.HalHandle == NULL ||
        Tpg == NULL) {
        return RET_NULL_POINTER;
    }

    if (pSv4eCtx->Configured != BOOL_TRUE) return RET_WRONG_STATE;

    result = Sv4e_IsiReadRegIss(handle, 0x5081, &value);
    CHECK_RESULT_RET(result, "read TPG register");

    Tpg->enable = ((value & 0x80) != 0) ? 1 : 0;
    if (Tpg->enable) {
        Tpg->pattern = (0xff & value);
    }
    pSv4eCtx->TestPattern = Tpg->enable;

    return (result);
}

RESULT Sv4e_IsiFocusCreateIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT Sv4e_IsiFocusReleaseIss(IsiSensorHandle_t handle) {
    return RET_SUCCESS;
}

RESULT Sv4e_IsiFocusGetCalibrateIss(IsiSensorHandle_t handle,
                                      IsiFocusCalibAttr_t* pFocusCalib) {
    return RET_SUCCESS;
}

RESULT Sv4e_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

RESULT Sv4e_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t* pPos) {
    return RET_SUCCESS;
}

// Taken from the IMX681 driver
RESULT Sv4e_IsiSetAgainDgainIss(IsiSensorHandle_t handle,
                                  IsiUserGain_t Gain) {
    RESULT result = RET_SUCCESS;
    uint32_t Again = 0, Dgain = 0;
    TRACE(SV4E_INFO, "%s: (enter)\n", __func__);

    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if ((Gain.again < 1) | (Gain.again > 16)) {
        TRACE(SV4E_ERROR, "%s: Invalid sensor again (%f)\n", __func__, Gain.again);
        return (RET_OUTOFRANGE);
    }
    // Again = (uint32_t)(1024 - (1024/Gain.again));
    Again = (uint32_t)(((2048 * Gain.again) - 2048) / Gain.again);

    result = Sv4e_IsiWriteRegIss(handle, 0x309c, (Again & 0x0000FF));
    CHECK_RESULT_RET(result, "write again low");
    result = Sv4e_IsiWriteRegIss(handle, 0x309d, (Again & 0x00FF00) >> 8);
    CHECK_RESULT_RET(result, "write again high");

    if ((Gain.dgain < 1) | (Gain.dgain > 16)) {
        TRACE(SV4E_ERROR, "%s: Invalid sensor dgain (%f)\n", __func__, Gain.dgain);
        return (RET_OUTOFRANGE);
    }
    Dgain = Gain.dgain * 256;

    result = Sv4e_IsiWriteRegIss(handle, 0x308c, (Dgain & 0x0000FF));
    CHECK_RESULT_RET(result, "write dgain low");
    result = Sv4e_IsiWriteRegIss(handle, 0x308d, (Dgain & 0x00FF00) >> 8);
    CHECK_RESULT_RET(result, "write dgain high");

    // pSv4eCtx->CurAgain = Gain.again;
    // pSv4eCtx->CurDgain = Gain.dgain;

    TRACE(SV4E_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT Sv4e_IsiGetIrisIss( IsiSensorHandle_t handle,
                                     float *pSetIris ) {
    RESULT result = RET_SUCCESS;

    TRACE(SV4E_INFO, "%s: (enter)\n", __func__);

    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    //TODO implement getting iris here!
    *pSetIris = 1.0;

    TRACE(SV4E_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT Sv4e_IsiSetIrisIss( IsiSensorHandle_t handle, 
                    const float NewIris) {
    RESULT result = RET_SUCCESS;

    TRACE(SV4E_INFO, "%s: (enter)\n", __func__);

    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    if (NewIris < SV4E_IRIS_MIN_VAL || NewIris > SV4E_IRIS_MAX_VAL) {
        TRACE(SV4E_ERROR, "%s: Invalid Iris %f\n", __func__, NewIris);
    }

    //TODO implement setting iris here!

    TRACE(SV4E_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT Sv4e_IsiGetHCGIss( IsiSensorHandle_t handle,
                                     bool *phcg_lef, bool *phcg_sef1, bool *phcg_sef2 ) {
    RESULT result = RET_SUCCESS;

    TRACE(SV4E_INFO, "%s: (enter)\n", __func__);

    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;
    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    *phcg_lef = pSv4eCtx->hcg_lef;
    *phcg_sef1 = pSv4eCtx->hcg_sef1;
    *phcg_sef2 = pSv4eCtx->hcg_sef2;

    TRACE(SV4E_INFO, "%s: (exit)\n", __func__);
    return (result);
}

static RESULT Sv4e_IsiSetHCGIss(IsiSensorHandle_t handle, bool hcg_lef, bool hcg_sef1, bool hcg_sef2) {

    RESULT result = RET_SUCCESS;

    TRACE(SV4E_INFO, "%s: (enter)\n", __func__);

    Sv4e_Context_t* pSv4eCtx = (Sv4e_Context_t*)handle;

    if (pSv4eCtx == NULL) {
        TRACE(SV4E_ERROR,
              "%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
        return (RET_WRONG_HANDLE);
    }

    result = Sv4e_IsiWriteRegIss(handle, 0x3030 , hcg_lef);
    CHECK_RESULT_RET(result, "write HCG LEF");
    pSv4eCtx->hcg_lef = hcg_lef;
    pSv4eCtx->hcg_sef1 = hcg_sef1;
    pSv4eCtx->hcg_sef2 = hcg_sef2;

    TRACE(SV4E_INFO, "%s: (exit)\n", __func__);
    return result;

}

RESULT Sv4e_IsiGetSensorIss(IsiSensor_t* pIsiSensor) {
    RESULT result = RET_SUCCESS;
    static const char SensorName[16] = "SV4E";
    TRACE(SV4E_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor != NULL) {
        pIsiSensor->pszName = SensorName;
        pIsiSensor->pIsiCreateIss = Sv4e_IsiCreateIss;
        pIsiSensor->pIsiReleaseIss = Sv4e_IsiReleaseIss;
        pIsiSensor->pIsiReadRegIss = Sv4e_IsiReadRegIss;
        pIsiSensor->pIsiWriteRegIss = Sv4e_IsiWriteRegIss;
        pIsiSensor->pIsiGetModeIss = Sv4e_IsiGetModeIss;
        pIsiSensor->pIsiSetModeIss = Sv4e_IsiSetModeIss;
        pIsiSensor->pIsiEnumModeIss = Sv4e_IsiEnumModeIss;
        pIsiSensor->pIsiGetCapsIss = Sv4e_IsiGetCapsIss;
        pIsiSensor->pIsiSetupIss = Sv4e_IsiSetupIss;
        pIsiSensor->pIsiCheckConnectionIss = Sv4e_IsiCheckConnectionIss;
        pIsiSensor->pIsiGetRevisionIss = Sv4e_IsiGetRevisionIss;
        pIsiSensor->pIsiSetStreamingIss = Sv4e_IsiSetStreamingIss;

        /* AEC functions */
        pIsiSensor->pIsiGetGainLimitsIss = 					Sv4e_IsiGetGainLimitsIss;
		pIsiSensor->pIsiGetIrisLimitsIss = 					Sv4e_IsiGetIrisLimitsIss;
		pIsiSensor->pIsiSetIrisLimitsIss = 					Sv4e_IsiSetIrisLimitsIss;
		pIsiSensor->pIsiGetIntegrationTimeLimitsIss =		Sv4e_IsiGetIntegrationTimeLimitsIss;

		pIsiSensor->pIsiExposureControlIss =				Sv4e_IsiExposureControlIss;
		pIsiSensor->pIsiExposureControlExpandedIss =		Sv4e_IsiExposureControlExpandedIss;
		pIsiSensor->pIsiSetIntegrationTimeIss =				Sv4e_IsiSetIntegrationTimeIss;

		pIsiSensor->pIsiGetLongIntegrationTimeIss =			Sv4e_IsiGetLEFIntegrationTimeIss;
		pIsiSensor->pIsiGetIntegrationTimeIss =				Sv4e_IsiGetIntegrationTimeIss;
		pIsiSensor->pIsiGetVSIntegrationTimeIss =			Sv4e_IsiGetSEF2IntegrationTimeIss;

		pIsiSensor->pIsiGetLongGainIss = 					Sv4e_IsiGetLEFGainIss;
		pIsiSensor->pIsiGetGainIss = 						Sv4e_IsiGetGainIss;
		pIsiSensor->pIsiGetVSGainIss = 						Sv4e_IsiGetSEF2GainIss;

		pIsiSensor->pIsiGetGainIncrementIss =				Sv4e_IsiGetGainIncrementIss;
		pIsiSensor->pIsiGetIrisIncrementIss =				Sv4e_IsiGetIrisIncrementIss;
		pIsiSensor->pIsiGetIntegrationTimeIncrementIss =	Sv4e_IsiGetIntegrationTimeIncrementIss;
		pIsiSensor->pIsiSetGainIss = 						Sv4e_IsiSetGainIss;
		pIsiSensor->pIsiGetFpsIss = 						Sv4e_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss = 						Sv4e_IsiSetFpsIss;
        pIsiSensor->pIsiSetFlickerFpsIss = 					Sv4e_IsiSetFlickerFpsIss;
        pIsiSensor->pIsiUnlimitFpsIss =						Sv4e_IsiUnlimitFpsIss;
        pIsiSensor->pIsiLimitFpsIss =						Sv4e_IsiLimitFpsIss;
		pIsiSensor->pIsiGetAutoFpsInfoIss = 				Sv4e_IsiGetAutoFpsInfoIss;
		pIsiSensor->pIsiGetStartEvIss = 					Sv4e_IsiGetStartEvIss;
        pIsiSensor->pIsiGetIrisIss =						Sv4e_IsiGetIrisIss;
        pIsiSensor->pIsiSetIrisIss =						Sv4e_IsiSetIrisIss;
        pIsiSensor->pIsiGetHCGIss =                         Sv4e_IsiGetHCGIss;
        pIsiSensor->pIsiSetHCGIss =                         Sv4e_IsiSetHCGIss;

        /* SENSOR ISP */
        pIsiSensor->pIsiGetIspStatusIss = Sv4e_IsiGetIspStatusIss;
        // pIsiSensor->pIsiSetBlcIss                       =
        // Sv4e_IsiSetBlcIss; pIsiSensor->pIsiSetWBIss = Sv4e_IsiSetWBIss;

        /* SENSOE OTHER FUNC*/
        pIsiSensor->pIsiSetPowerIss = Sv4e_IsiSetPowerIss;
        pIsiSensor->pIsiSetTpgIss = Sv4e_IsiSetTpgIss;
        pIsiSensor->pIsiGetTpgIss = Sv4e_IsiGetTpgIss;
        // pIsiSensor->pIsiGetExpandCurveIss               =
        // Sv4e_IsiGetExpandCurveIss; pIsiSensor->pIsiGetCompressCurveIss =
        // Sv4e_IsiGetCompressCurveIss; pIsiSensor->pIsiExtendFuncIss =
        // Sv4e_IsiExtendFuncIss; pIsiSensor->pIsiGetOtpDataIss =
        // Sv4e_IsiGetOtpDataIss;

        /* AF */
        pIsiSensor->pIsiFocusCreateIss = Sv4e_IsiFocusCreateIss;
        pIsiSensor->pIsiFocusReleaseIss = Sv4e_IsiFocusReleaseIss;
        pIsiSensor->pIsiFocusGetCalibrateIss = Sv4e_IsiFocusGetCalibrateIss;
        pIsiSensor->pIsiFocusSetIss = Sv4e_IsiFocusSetIss;
        pIsiSensor->pIsiFocusGetIss = Sv4e_IsiFocusGetIss;
        pIsiSensor->pIsiSetAgainDgainIss = Sv4e_IsiSetAgainDgainIss;

    } else {
        result = RET_NULL_POINTER;
    }

    TRACE(SV4E_INFO, "%s (exit)\n", __func__);
    return (result);
}

/*****************************************************************************
 * each sensor driver need declare this struct for isi load
 *****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x9012,
    .pIsiHalEnumModeIss = Sv4e_IsiHalEnumModeIss,
    .pIsiGetSensorIss = Sv4e_IsiGetSensorIss,
};
