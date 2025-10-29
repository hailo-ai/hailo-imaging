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
#ifndef __ISI_OTP_H__
#define __ISI_OTP_H__
#include <ebase/types.h>
#include <hal/hal_api.h>
#include <isi/isi_common.h>
#define ISI_OTP_LSC_TABLE_NUM 17

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct IsiOTPLSC_s{
   uint16_t r[ISI_OTP_LSC_TABLE_NUM][ISI_OTP_LSC_TABLE_NUM];
   uint16_t gr[ISI_OTP_LSC_TABLE_NUM][ISI_OTP_LSC_TABLE_NUM];
   uint16_t gb[ISI_OTP_LSC_TABLE_NUM][ISI_OTP_LSC_TABLE_NUM];
   uint16_t b[ISI_OTP_LSC_TABLE_NUM][ISI_OTP_LSC_TABLE_NUM];
}IsiOTPLSC_t;
typedef struct IsiOTPAWB_s{
   uint16_t r;
   uint16_t gr;
   uint16_t gb;
   uint16_t b;
   uint16_t rg_ratio;
   uint16_t bg_ratio;
}IsiOTPAWB_t;

typedef struct IsiOTPLightSource_s{
  uint16_t CIE_x;
  uint16_t CIE_y;
  uint16_t Intensity;
}IsiOTPLightSource_t;

typedef struct IsiOTPFocus_s{
  uint16_t Macro;
  uint16_t Inif;
}IsiOTPFocus_t;
typedef struct IsiOTPModuleInformation_s{
  uint16_t HWVersion;
  uint16_t EEPROMRevision;
  uint16_t SensorRevision;
  uint16_t TLensRevision;
  uint16_t IRCFRevision;
  uint16_t LensRevision;
  uint16_t CARevision;
  uint16_t ModuleInteID;
  uint16_t FactoryID;
  uint16_t MirrorFlip;
  uint16_t TLensSlaveID;
  uint16_t EEPROMSlaveID;
  uint16_t SensorSlaveID;
  uint8_t  SensorID[11];
  uint16_t ManuDateYear;
  uint16_t ManuDateMonth;
  uint16_t ManuDateDay;
  uint8_t  BarcodeModuleSN[12];
  uint16_t MapTotalSize;

}IsiOTPModuleInformation_t;

typedef struct IsiOTPV1_s{
   uint32_t  OTP_version;
   IsiOTPModuleInformation_t OTPInformation;
   bool      OTPLSCEnable;
   bool      OTPAwbEnable;
   bool      OTPLightSourceEnable;
   bool      OTPFocusEnable;
   IsiOTPLSC_t LSC_4000K;
   IsiOTPAWB_t Awb_3100K;
   IsiOTPAWB_t Awb_4000K;
   IsiOTPAWB_t Awb_5800K;
   IsiOTPAWB_t GoldenAwb_3100K;
   IsiOTPAWB_t GoldenAwb_4000K;
   IsiOTPAWB_t GoldenAwb_5800K; 
   IsiOTPLightSource_t LightSource_3100K;
   IsiOTPLightSource_t LightSource_4000K;
   IsiOTPLightSource_t LightSource_5800K;
   IsiOTPFocus_t  Focus;
   bool DataCheckResult;

} IsiOTPV1_t;

#ifdef __cplusplus
}
#endif

#endif
