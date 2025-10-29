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

/* VeriSilicon 2022 */

/**
 * @file isi_iss.h
 *
 * @brief Interface description for image sensor specific implementation (iss).
 *
 *****************************************************************************/
/**
 * @page module_name_page Module Name
 * Describe here what this module does.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref module_name
 *
 * @defgroup isi_iss CamerIc Driver API
 * @{
 *
 */
#ifndef __ISI_ISS_H__
#define __ISI_ISS_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include <hal/hal_api.h>

#include "isi.h"

#ifdef __cplusplus
extern "C"
{
#endif



/*****************************************************************************/
/**
 *          IsiRegisterFlags_t
 *
 * @brief   Register permission enumeration type
 */
/*****************************************************************************/
typedef enum IsiRegisterFlags_e
{
    // basic features
    eTableEnd           = 0x00,                                                 /**< special flag for end of register table */
    eReadable           = 0x01,
    eWritable           = 0x02,
    eVolatile           = 0x04,                                                 /**< register can change even if not written by I2C */
    eDelay              = 0x08,                                                 /**< wait n ms */
    eReserved           = 0x10,
    eNoDefault          = 0x20,                                                 /**< no default value specified */
    eTwoBytes           = 0x40,                                                 /**< SMIA sensors use 8-, 16- and 32-bit registers */
    eFourBytes          = 0x80,                                                 /**< SMIA sensors use 8-, 16- and 32-bit registers */

    // combined features
    eReadOnly           = eReadable,
    eWriteOnly          = eWritable,
    eReadWrite          = eReadable | eWritable,
    eReadWriteDel       = eReadable | eWritable | eDelay,
    eReadWriteVolatile  = eReadable | eWritable | eVolatile,
    eReadWriteNoDef     = eReadable | eWritable | eNoDefault,
    eReadWriteVolNoDef  = eReadable | eWritable | eVolatile | eNoDefault,
    eReadVolNoDef       = eReadable | eVolatile | eNoDefault,
    eReadOnlyVolNoDef   = eReadOnly | eVolatile | eNoDefault,

    // additional SMIA features
    eReadOnly_16            = eReadOnly          | eTwoBytes,
    eReadWrite_16           = eReadWrite         | eTwoBytes,
    eReadWriteDel_16        = eReadWriteDel      | eTwoBytes,
    eReadWriteVolatile_16   = eReadWriteVolatile | eTwoBytes,
    eReadWriteNoDef_16      = eReadWriteNoDef    | eTwoBytes,
    eReadWriteVolNoDef_16   = eReadWriteVolNoDef | eTwoBytes,
    eReadOnlyVolNoDef_16    = eReadOnly_16 | eVolatile | eNoDefault,
    eReadOnly_32            = eReadOnly          | eFourBytes,
    eReadWrite_32           = eReadWrite         | eFourBytes,
    eReadWriteVolatile_32   = eReadWriteVolatile | eFourBytes,
    eReadWriteNoDef_32      = eReadWriteNoDef    | eFourBytes,
    eReadWriteVolNoDef_32   = eReadWriteVolNoDef | eFourBytes
} IsiRegisterFlags_t;



/*****************************************************************************/
/**
 *          IsiRegDescription_t
 *
 * @brief   Sensor register description struct
 */
/*****************************************************************************/
typedef struct IsiRegisterFlags_s
{
    uint32_t    Addr;
    uint32_t    DefaultValue;
    const char* pName;
    uint32_t    Flags;
} IsiRegDescription_t;


typedef RESULT (*IsiCreateIss_t)                     ( IsiSensorInstanceConfig_t *pConfig );
typedef RESULT (*IsiReleaseIss_t)                    ( IsiSensorHandle_t handle );
typedef RESULT (*IsiReadRegIss_t)                    ( IsiSensorHandle_t handle, const uint32_t Addr, uint32_t *pValue );
typedef RESULT (*IsiWriteRegIss_t)                   ( IsiSensorHandle_t handle, const uint32_t Addr, const uint32_t Value );
typedef RESULT (*IsiGetModeIss_t)                    ( IsiSensorHandle_t handle, IsiMode_t *pMode);
typedef RESULT (*IsiSetModeIss_t)                    ( IsiSensorHandle_t handle, IsiMode_t *pMode);
typedef RESULT (*IsiEnumModeIss_t)                   ( IsiSensorHandle_t handle, IsiEnumMode_t *pEnumMode );
typedef RESULT (*IsiGetCapsIss_t)                    ( IsiSensorHandle_t handle, IsiCaps_t *pCaps );
typedef RESULT (*IsiSetupIss_t)                      ( IsiSensorHandle_t handle, const IsiCaps_t *pCaps );
typedef RESULT (*IsiCheckConnectionIss_t)            ( IsiSensorHandle_t handle );
typedef RESULT (*IsiGetRevisionIss_t)                ( IsiSensorHandle_t handle, uint32_t *pRevision );
typedef RESULT (*IsiSetStreamingIss_t)               ( IsiSensorHandle_t handle, bool_t on );

/* AEC */
typedef RESULT (*IsiExposureControlIss_t)            ( IsiSensorHandle_t handle, const float NewGain, const float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime, float* hdr_ratio);
typedef RESULT (*IsiExposureControlExpandedIss_t)            ( IsiSensorHandle_t handle, const float NewGain, const float NewIntegrationTime, const float NewIris, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime, float* pSetIris, float* hdr_ratio);
typedef RESULT (*IsiSetIrisIss_t)                ( IsiSensorHandle_t handle, const float NewIris);
typedef RESULT (*IsiUnlimitFpsIss_t)              ( IsiSensorHandle_t handle, float maxIntegrationTime);
typedef RESULT (*IsiLimitFpsIss_t)              ( IsiSensorHandle_t handle);
typedef RESULT (*IsiGetGainLimitsIss_t)              ( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
typedef RESULT (*IsiGetIrisLimitsIss_t)              ( IsiSensorHandle_t handle, float *pMinIris, float *pMaxIris);
typedef RESULT (*IsiSetIrisLimitsIss_t)              ( IsiSensorHandle_t handle, float minIris, float maxIris);
typedef RESULT (*IsiGetIntegrationTimeLimitsIss_t)   ( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
typedef RESULT (*IsiGetAbsoluteIntegrationTimeLimitsIss_t)   ( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
typedef RESULT (*IsiGetCurrentExposureIss_t)         ( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime, float *pCurHdrRatio);
typedef RESULT (*IsiGetIrisIss_t)                    ( IsiSensorHandle_t handle, float *pSetIris );
typedef RESULT (*IsiGetGainIss_t)                    ( IsiSensorHandle_t handle, float *pSetGain );
typedef RESULT (*IsiGetVSGainIss_t)                  ( IsiSensorHandle_t handle, float *pSetGain );
typedef RESULT (*IsiGetShortGainIss_t)               ( IsiSensorHandle_t handle, float *pSetGain );
typedef RESULT (*IsiGetLongGainIss_t)                ( IsiSensorHandle_t handle, float *pSetGain );
typedef RESULT (*IsiGetGainIncrementIss_t)           ( IsiSensorHandle_t handle, float *pIncr );
typedef RESULT (*IsiGetIrisIncrementIss_t)           ( IsiSensorHandle_t handle, float *pIncr );
typedef RESULT (*IsiSetGainIss_t)                    ( IsiSensorHandle_t handle, float NewGain, float *pSetGain, float *hdr_ratio);
typedef RESULT (*IsiGetIntegrationTimeIss_t)         ( IsiSensorHandle_t handle, float *pSetIntegrationTime );
typedef RESULT (*IsiGetVSIntegrationTimeIss_t)       ( IsiSensorHandle_t handle, float *pSetIntegrationTime );
typedef RESULT (*IsiGetShortIntegrationTimeIss_t)    ( IsiSensorHandle_t handle, float *pSetIntegrationTime );
typedef RESULT (*IsiGetLongIntegrationTimeIss_t)     ( IsiSensorHandle_t handle, float *pSetIntegrationTime );
typedef RESULT (*IsiGetIntegrationTimeIncrementIss_t)( IsiSensorHandle_t handle, float *pIncr );
typedef RESULT (*IsiSetIntegrationTimeIss_t)         ( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *hdr_ratio);
typedef RESULT (*IsiGetFpsIss_t)                     ( IsiSensorHandle_t handle, uint32_t *pFps );
typedef RESULT (*IsiSetFpsIss_t)                     ( IsiSensorHandle_t handle, uint32_t Fps );
typedef RESULT (*IsiSetFlickerFpsIss_t)              ( IsiSensorHandle_t handle, uint32_t flickerMode );
typedef RESULT (*IsiGetAutoFpsInfoIss_t)             ( IsiSensorHandle_t handle, IsiAutoFps_t *pAutoFpsInfo );
typedef RESULT (*IsiGetStartEvIss_t)                 ( IsiSensorHandle_t handle, uint64_t *pStartEv );
typedef RESULT (*IsiGetAgainDgainIss_t)              ( IsiSensorHandle_t handle, IsiUserGain_t *pGain );
typedef RESULT (*IsiSetAgainDgainIss_t)              ( IsiSensorHandle_t handle, IsiUserGain_t Gain );
typedef RESULT (*IsiSetHCGIss_t)                     ( IsiSensorHandle_t handle, bool hcg );
typedef RESULT (*IsiGetHCGIss_t)                     ( IsiSensorHandle_t handle, bool *phcg );


/* SENSOR ISP */
typedef RESULT (*IsiGetIspStatusIss_t)               ( IsiSensorHandle_t handle, IsiIspStatus_t *pIspStatus );
typedef RESULT (*IsiSetBlcIss_t)                     ( IsiSensorHandle_t handle, IsiBlc_t *pBlc );
typedef RESULT (*IsiSetWBIss_t)                      ( IsiSensorHandle_t handle, IsiWB_t *pWb );

/* SENSOE OTHER FUNC*/
typedef RESULT (*IsiSetPowerIss_t)                   ( IsiSensorHandle_t handle, bool_t on );
typedef RESULT (*IsiSetTpgIss_t)                     ( IsiSensorHandle_t handle, IsiTpg_t Tpg );
typedef RESULT (*IsiGetTpgIss_t)                     ( IsiSensorHandle_t handle, IsiTpg_t *Tpg );
typedef RESULT (*IsiGetExpandCurveIss_t)             ( IsiSensorHandle_t handle, IsiCompandCurve_t *pCurve );
typedef RESULT (*IsiGetCompressCurveIss_t)           ( IsiSensorHandle_t handle, IsiCompandCurve_t *pCurve );
typedef RESULT (*IsiExtendFuncIss_t)                 ( IsiSensorHandle_t handle, void *pUserData );
typedef RESULT (*IsiGetOtpDataIss_t)                 ( IsiSensorHandle_t handle, uint32_t *length,void *pOtpData );
typedef RESULT (*IsiGetHdrBlankingLinesIss_t)        ( IsiSensorHandle_t handle, uint32_t *pBlankingLines, size_t elementCount ); 

/* AF */
typedef RESULT (*IsiFocusCreateIss_t)                ( IsiSensorHandle_t handle );
typedef RESULT (*IsiFocusReleaseIss_t)               ( IsiSensorHandle_t handle );
typedef RESULT (*IsiFocusGetCalibrateIss_t)          ( IsiSensorHandle_t handle, IsiFocusCalibAttr_t *pFocusCalib );
typedef RESULT (*IsiFocusSetIss_t)                   ( IsiSensorHandle_t handle, IsiFocusPos_t *pPos );
typedef RESULT (*IsiFocusGetIss_t)                   ( IsiSensorHandle_t handle, IsiFocusPos_t *pPos );



/*****************************************************************************/
/**
 *          IsiSensor_t
 *
 * @brief
 *
 */
/*****************************************************************************/
struct IsiSensor_s
{
    const char                          *pszName;                       /**< name of the camera-sensor */
    const IsiRegDescription_t           *pRegisterTable;                /**< pointer to register table */
    const IsiCaps_t                     *pIsiCaps;                      /**< pointer to sensor capabilities */

    IsiCreateIss_t                      pIsiCreateIss;     	            /**< create a sensor handle */
    IsiReleaseIss_t                     pIsiReleaseIss;                 /**< release a sensor handle */
    IsiReadRegIss_t                     pIsiReadRegIss;                 /**< read sensor register */
    IsiWriteRegIss_t                    pIsiWriteRegIss;                /**< write sensor register */
    IsiGetModeIss_t                     pIsiGetModeIss;
    IsiSetModeIss_t                     pIsiSetModeIss;
	IsiEnumModeIss_t                    pIsiEnumModeIss;
    IsiGetCapsIss_t                     pIsiGetCapsIss;                 /**< get sensor capabilities */
    IsiSetupIss_t                       pIsiSetupIss;                   /**< setup sensor capabilities */
    IsiCheckConnectionIss_t             pIsiCheckConnectionIss;
    IsiGetRevisionIss_t                 pIsiGetRevisionIss;             /**< read sensor revision register (if available) */
    IsiSetStreamingIss_t                pIsiSetStreamingIss;            /**< enable/disable streaming of data once sensor is configured */

    /* AEC */
    IsiExposureControlIss_t             pIsiExposureControlIss;
    IsiExposureControlExpandedIss_t     pIsiExposureControlExpandedIss;
    IsiUnlimitFpsIss_t                  pIsiUnlimitFpsIss;
    IsiLimitFpsIss_t                    pIsiLimitFpsIss;
    IsiSetIrisIss_t                     pIsiSetIrisIss;
    IsiGetIrisLimitsIss_t               pIsiGetIrisLimitsIss;
    IsiSetIrisLimitsIss_t               pIsiSetIrisLimitsIss;
    IsiGetGainLimitsIss_t               pIsiGetGainLimitsIss;
    IsiGetIntegrationTimeLimitsIss_t    pIsiGetIntegrationTimeLimitsIss;
    IsiGetAbsoluteIntegrationTimeLimitsIss_t pIsiGetAbsoluteIntegrationTimeLimitsIss;
    IsiGetCurrentExposureIss_t          pIsiGetCurrentExposureIss;      /**< get the currenntly adjusted ae values (gain and integration time) */
    IsiGetIrisIss_t                     pIsiGetIrisIss;
    IsiGetGainIss_t                     pIsiGetGainIss;
    IsiGetVSGainIss_t                   pIsiGetVSGainIss;
    IsiGetShortGainIss_t                pIsiGetShortGainIss;
    IsiGetLongGainIss_t                 pIsiGetLongGainIss;
    IsiGetGainIncrementIss_t            pIsiGetGainIncrementIss;
    IsiGetIrisIncrementIss_t            pIsiGetIrisIncrementIss;
    IsiSetGainIss_t                     pIsiSetGainIss;
    IsiGetIntegrationTimeIss_t          pIsiGetIntegrationTimeIss;
    IsiGetVSIntegrationTimeIss_t        pIsiGetVSIntegrationTimeIss;
    IsiGetShortIntegrationTimeIss_t     pIsiGetShortIntegrationTimeIss;
    IsiGetLongIntegrationTimeIss_t      pIsiGetLongIntegrationTimeIss;
    IsiGetIntegrationTimeIncrementIss_t pIsiGetIntegrationTimeIncrementIss;
    IsiSetIntegrationTimeIss_t          pIsiSetIntegrationTimeIss;
    IsiGetFpsIss_t                      pIsiGetFpsIss;
    IsiSetFpsIss_t                      pIsiSetFpsIss;
    IsiSetFlickerFpsIss_t               pIsiSetFlickerFpsIss;
    IsiGetAutoFpsInfoIss_t              pIsiGetAutoFpsInfoIss;
    IsiGetStartEvIss_t                  pIsiGetStartEvIss;
    IsiGetAgainDgainIss_t               pIsiGetAgainDgainIss;
    IsiSetAgainDgainIss_t               pIsiSetAgainDgainIss;
    IsiSetHCGIss_t                      pIsiSetHCGIss;
    IsiGetHCGIss_t                      pIsiGetHCGIss;

    /* SENSOR ISP */
    IsiGetIspStatusIss_t                pIsiGetIspStatusIss;
    IsiSetBlcIss_t                      pIsiSetBlcIss;
    IsiSetWBIss_t                       pIsiSetWBIss;

    /* SENSOE OTHER FUNC*/
    IsiSetPowerIss_t                    pIsiSetPowerIss;         /**< turn sensor power on/off */
    IsiSetTpgIss_t                      pIsiSetTpgIss;
    IsiGetTpgIss_t                      pIsiGetTpgIss;
    IsiGetExpandCurveIss_t              pIsiGetExpandCurveIss;
    IsiGetCompressCurveIss_t            pIsiGetCompressCurveIss;
    IsiExtendFuncIss_t                  pIsiExtendFuncIss;
    IsiGetOtpDataIss_t                  pIsiGetOtpDataIss;
    IsiGetHdrBlankingLinesIss_t         pIsiGetHdrBlankingLinesIss;

    /* AF */
    IsiFocusCreateIss_t                 pIsiFocusCreateIss;
    IsiFocusReleaseIss_t                pIsiFocusReleaseIss;
    IsiFocusGetCalibrateIss_t           pIsiFocusGetCalibrateIss;
    IsiFocusSetIss_t                    pIsiFocusSetIss;
    IsiFocusGetIss_t                    pIsiFocusGetIss;
};


typedef RESULT (*IsiHalEnumModeIss_t)                ( HalHandle_t  HalHandle, IsiEnumMode_t *pEnumMode );
/*****************************************************************************/
/**
 *          IsiGetSensorIss_t
 *
 * @brief   Only exported function of sensor specific code: fills in
 *          sensor decription struct
 *
 */
/*****************************************************************************/
typedef RESULT (*IsiGetSensorIss_t) ( IsiSensor_t *pIsiSensor );



/*****************************************************************************/
/**
 *          IsiCamDrvConfig_t
 *
 * @brief   Camera sensor driver specific data
 *
 */
/*****************************************************************************/
typedef struct IsiCamDrvConfig_s
{
    uint32_t                     CameraDriverID;
    IsiHalEnumModeIss_t          pIsiHalEnumModeIss;
    IsiGetSensorIss_t            pIsiGetSensorIss;
    IsiSensor_t                  IsiSensor;
} IsiCamDrvConfig_t;

typedef struct IsiSccbInfo_s
{
    uint8_t slave_addr;
    uint8_t addr_byte;
    uint8_t data_byte;
} IsiSccbInfo_t;


#ifdef __cplusplus
}
#endif

#endif /* __ISI_ISS_H__ */

