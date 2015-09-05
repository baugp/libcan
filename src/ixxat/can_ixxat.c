/***************************************************************************
 *   Copyright (C) 2008 by Ralf Kaestner                                   *
 *   ralf.kaestner@gmail.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#define _ISOC99_SOURCE

#include <sys/types.h>
#include <sys/dir.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/file.h>
#include <sys/time.h>
#include <signal.h>
#include <termios.h>
#include <math.h>

#include "ECI005.h"
#include "ECI109.h"
#include "EciDemoCommon.h"

#include <string/string.h>
//#include <timer/timer.h>

#include "can_ixxat.h"

const char* can_ixxat_errors[] = {
  "Success",
  "Failed to open CAN-IXXAT device",
  "Failed to close CAN-IXXAT device",
  "Failed to set CAN-IXXAT device parameters",
  "CAN-IXXAT device timeout",
  "Failed to send to CAN-IXXAT device",
  "Failed to receive from CAN-IXXAT device",
};

const char* can_device_name = "CAN-IXXAT";

config_param_t can_ixxat_default_parameters[] = {
  {CAN_IXXAT_PARAMETER_DEVICE,
    config_param_type_string,
    "pci0",
    "",
    "CAN-IXXAT device interface <pci0|usb0>"},
  {CAN_IXXAT_PARAMETER_BIT_RATE,
    config_param_type_int,
    "1000",
    "[10, 1000]",
    "The requested bit rate of the CAN bus in [kbit/s]"},
  {CAN_IXXAT_PARAMETER_TIMEOUT,
    config_param_type_float,
    "0.01",
    "",
    "The CAN bus communication timeout in [s]"},
  {CAN_IXXAT_PARAMETER_VERBOSE,
    config_param_type_bool,
    "false",
    "false|true",
    "Verbose mode"},
};

const config_default_t can_default_config = {
  can_ixxat_default_parameters,
  sizeof(can_ixxat_default_parameters)/sizeof(config_param_t),
};

void can_ixxat_device_init(can_ixxat_device_t* dev);
void can_ixxat_device_destroy(can_ixxat_device_t* dev);

int can_device_open(can_device_t* dev) {
  error_clear(&dev->error);

  if (!dev->num_references) {
    dev->comm_dev = malloc(sizeof(can_ixxat_device_t));
    can_ixxat_device_init(dev->comm_dev);

    dev->num_sent = 0;
    dev->num_received = 0;

    if (can_ixxat_device_open(dev->comm_dev,
        config_get_string(&dev->config, CAN_IXXAT_PARAMETER_DEVICE),
        config_get_bool(&dev->config, CAN_IXXAT_PARAMETER_VERBOSE)) ||
      can_ixxat_device_setup(dev->comm_dev,
        config_get_int(&dev->config, CAN_IXXAT_PARAMETER_BIT_RATE),
        config_get_float(&dev->config, CAN_IXXAT_PARAMETER_TIMEOUT))) {
      error_blame(&dev->error, &((can_ixxat_device_t*)dev->comm_dev)->error,
        CAN_ERROR_OPEN);

      can_ixxat_device_destroy(dev->comm_dev);

      free(dev->comm_dev);
      dev->comm_dev = 0;

      return dev->error.code;
    }
  }
  ++dev->num_references;

  return dev->error.code;
}

int can_device_close(can_device_t* dev) {
  error_clear(&dev->error);

  if (dev->num_references) {
    --dev->num_references;

    if (!dev->num_references) {
      if (!can_ixxat_device_close(dev->comm_dev)) {
        can_ixxat_device_destroy(dev->comm_dev);

        free(dev->comm_dev);
        dev->comm_dev = 0;
      }
      else
        error_blame(&dev->error, &((can_ixxat_device_t*)dev->comm_dev)->error,
          CAN_ERROR_CLOSE);
    }
  }
  else
    error_setf(&dev->error, CAN_ERROR_CLOSE, "Non-zero reference count");

  return dev->error.code;
}

int can_device_send_message(can_device_t* dev, const can_message_t* message) {
  error_clear(&dev->error);

  if (dev->comm_dev) {
    if (can_ixxat_device_send(dev->comm_dev, message))
      error_blame(&dev->error, &((can_ixxat_device_t*)dev->comm_dev)->error,
        CAN_ERROR_SEND);
    else
      ++dev->num_sent;
  }
  else
    error_setf(&dev->error, CAN_ERROR_SEND,
      "Communication device unavailable");

  return dev->error.code;
}

int can_device_receive_message(can_device_t* dev, can_message_t* message) {
  error_clear(&dev->error);

  if (dev->comm_dev) {
    if (can_ixxat_device_receive(dev->comm_dev, message))
      error_blame(&dev->error, &((can_ixxat_device_t*)dev->comm_dev)->error,
        CAN_ERROR_RECEIVE);
    else
      ++dev->num_received;
  }
  else
    error_setf(&dev->error, CAN_ERROR_RECEIVE,
      "Communication device unavailable");

  return dev->error.code;
}

void can_ixxat_device_init(can_ixxat_device_t* dev) {
  dev->name = 0;
  dev->interface = IXXAT_PCI;
  dev->dwHwIndex = 0;
  dev->dwCtrlIndex = 0;
  dev->dwCtrlHandle = 0;

  dev->bitrate = 0;
  dev->timeout = 0.0;
  dev->verbose = 0;

  memset(&dev->msg_received, 0, sizeof(can_message_t));

  error_init(&dev->error, can_ixxat_errors);
}

void can_ixxat_device_destroy(can_ixxat_device_t* dev) {
  string_destroy(&dev->name);
  error_destroy(&dev->error);
}

//ECI Functions Mapping
static ECI_RESULT (*ECI_Initialize[])(const DWORD,const ECI_HW_PARA[])                            = {ECI005_Initialize, ECI109_Initialize};
static ECI_RESULT (*ECI_Release[])(void)                                                          = {ECI005_Release, ECI109_Release};
static ECI_RESULT (*ECI_GetInfo[])(const DWORD,ECI_HW_INFO*)                                      = {ECI005_GetInfo, ECI109_GetInfo};
static ECI_RESULT (*ECI_CtrlOpen[])(ECI_CTRL_HDL*,const DWORD,const DWORD,const ECI_CTRL_CONFIG*) = {ECI005_CtrlOpen, ECI109_CtrlOpen};
static ECI_RESULT (*ECI_CtrlClose[])(const ECI_CTRL_HDL)                                          = {ECI005_CtrlClose, ECI109_CtrlClose};
static ECI_RESULT (*ECI_CtrlStart[])(const ECI_CTRL_HDL)                                          = {ECI005_CtrlStart, ECI109_CtrlStart};
static ECI_RESULT (*ECI_CtrlStop[])(const ECI_CTRL_HDL,const DWORD)                               = {ECI005_CtrlStop, ECI109_CtrlStop};
static ECI_RESULT (*ECI_CtrlGetCapabilities[])(const ECI_CTRL_HDL,ECI_CTRL_CAPABILITIES*)         = {ECI005_CtrlGetCapabilities, ECI109_CtrlGetCapabilities};
static ECI_RESULT (*ECI_CtrlGetStatus[])(const ECI_CTRL_HDL,ECI_CTRL_STATUS)                      = {ECI005_CtrlGetStatus, ECI109_CtrlGetStatus};
static ECI_RESULT (*ECI_CtrlSend[])(const ECI_CTRL_HDL,const ECI_CTRL_MESSAGE*,const DWORD)       = {ECI005_CtrlSend, ECI109_CtrlSend};
static ECI_RESULT (*ECI_CtrlReceive[])(const ECI_CTRL_HDL,DWORD*,ECI_CTRL_MESSAGE*,const DWORD)   = {ECI005_CtrlReceive, ECI109_CtrlReceive};
static const char* (*ECI_GetErrorString[])(const ECI_RESULT)                                      = {ECI005_GetErrorString, ECI109_GetErrorString};

static can_ixxat_device_t* ixxat_can_dev = NULL;
void ixxat_exit() {
  if(ixxat_can_dev)
    can_ixxat_device_close(ixxat_can_dev);
}

int can_ixxat_device_open(can_ixxat_device_t* dev, const char* name, int verbose) {

  error_clear(&dev->error);

//  if ((result = IXXAT_OpenChannel((char*)name)) >= 0) {
//    dev->handle = result;
//    dev->fd = 0;
//    string_copy(&dev->name, name);

//    IXXAT_AddHandlerEx(dev->handle, can_ixxat_device_handle, dev);
//  }
//  else
//    error_setf(&dev->error, CAN_IXXAT_ERROR_OPEN, IXXAT_DecodeErrorMsg(result));

  ECI_RESULT  hResult       = ECI_OK;
  ECI_HW_PARA astcHwPara[4] = {{0}};
  ECI_HW_INFO stcHwInfo     = {0};
  DWORD       dwIndex       = 0;

  if(string_starts_with_ignore_case(name, "pci"))
    dev->interface = IXXAT_PCI;
  else if(string_starts_with_ignore_case(name, "usb"))
    dev->interface = IXXAT_USB;
  else
  {
    error_setf(&dev->error, CAN_IXXAT_ERROR_OPEN, "Invalid CAN-IXXAT device interface type");
    return dev->error.code;
  }

  if((name[3]-'0')>=0 && (name[3]-'0')<=4)
    dev->dwHwIndex = name[3]-'0';
  else
  {
    error_setf(&dev->error, CAN_IXXAT_ERROR_OPEN, "Invalid CAN-IXXAT device index");
    return dev->error.code;
  }

  string_copy(&dev->name, name);
  dev->verbose = verbose;

  //*** Prepare Hardware parameter structure for multiple boards
  for(dwIndex=0; dwIndex < _countof(astcHwPara); dwIndex++)
  {
    if(dev->interface==IXXAT_PCI)
      astcHwPara[dwIndex].wHardwareClass = ECI_HW_PCI;
    else if(dev->interface==IXXAT_USB)
      astcHwPara[dwIndex].wHardwareClass = ECI_HW_USB;
    #ifdef HWUSEPOLLINGMODE
      astcHwPara[dwIndex].dwFlags = ECI_SETTINGS_FLAG_POLLING_MODE;
    #endif //HWUSEPOLLINGMODE
  }

  //*** At first call Initialize to prepare ECI driver
  hResult = ECI_Initialize[dev->interface](_countof(astcHwPara), astcHwPara);
  ECIDRV_CHECKERROR(ECIDRV_Initialize);

  //*** Retrieve hardware info
  if(ECI_OK == hResult)
  {
    //Save ixxat device handle, add it to exit() handler to close the port when programme terminates
    ixxat_can_dev = dev;
    atexit(ixxat_exit);

    //*** Retrieve hardware info
    hResult = ECI_GetInfo[dev->interface](dev->dwHwIndex, &stcHwInfo);
    ECIDRV_CHECKERROR(ECIDRV_GetInfo);
    if(dev->verbose) {
      if(ECI_OK == hResult)
        EciPrintHwInfo(&stcHwInfo);
    }
  }

  //*** Find first CAN Controller of Board
  if(ECI_OK == hResult)
  {
    hResult = EciGetNthCtrlOfClass(&stcHwInfo,
                                   ECI_CTRL_CAN,
                                   0, //first relative controller
                                   &dev->dwCtrlIndex);
  }

  if(hResult != ECI_OK)
    error_setf(&dev->error, CAN_IXXAT_ERROR_OPEN, ECI_GetErrorString[dev->interface](hResult));

  return dev->error.code;
}

int can_ixxat_device_close(can_ixxat_device_t* dev) {

  error_clear(&dev->error);

//  if (!(result = IXXAT_CANExit(dev->handle, 0)) &&
//    !(result = IXXAT_CloseChannel(dev->handle))) {
//    dev->name[0] = 0;
//    dev->fd = 0;
//  }
//  else
//    error_setf(&dev->error, CAN_IXXAT_ERROR_CLOSE, IXXAT_DecodeErrorMsg(result));

  ECI_RESULT            hResult         = ECI_OK;

  //*** Stop Controller
  if(ECI_OK == hResult)
  {
    hResult = ECI_CtrlStop[dev->interface](dev->dwCtrlHandle, ECI_STOP_FLAG_NONE);
    ECIDRV_CHECKERROR(ECIDRV_CtrlStop);
  }

  //*** Wait some time to ensure bus idle
  OS_Sleep(250);

  //*** Reset Controller
  if(ECI_OK == hResult)
  {
    hResult = ECI_CtrlStop[dev->interface](dev->dwCtrlHandle, ECI_STOP_FLAG_RESET_CTRL);
    ECIDRV_CHECKERROR(ECIDRV_CtrlStop);
  }

  //*** Close ECI Controller
  if(ECI_OK == hResult)
  {
    hResult = ECI_CtrlClose[dev->interface](dev->dwCtrlHandle);
    ECIDRV_CHECKERROR(ECIDRV_CtrlClose);
    dev->dwCtrlHandle = ECI_INVALID_HANDLE;
    ixxat_can_dev = NULL;
  }
  if(hResult != ECI_OK)
  {
      error_setf(&dev->error, CAN_IXXAT_ERROR_CLOSE, ECI_GetErrorString[dev->interface](hResult));
      OS_Printf(ECI_GetErrorString[dev->interface](hResult));
  }

  //*** Clean up ECI driver
  hResult = ECI_Release[dev->interface]();
  ECIDRV_CHECKERROR(ECIDRV_Release);
  if(hResult != ECI_OK)
      error_setf(&dev->error, CAN_IXXAT_ERROR_CLOSE, ECI_GetErrorString[dev->interface](hResult));

  return dev->error.code;
}

int can_ixxat_device_setup(can_ixxat_device_t* dev, int bitrate, double timeout) {

  error_clear(&dev->error);

  ECI_RESULT            hResult         = ECI_OK;
  ECI_CTRL_CAPABILITIES stcCtrlCaps     = {0};
  dev->dwCtrlHandle    = ECI_INVALID_HANDLE;
  DWORD                 dwCtrlFeatures  = 0;

  dev->bitrate = bitrate;
  dev->timeout = timeout;

  //*** Open given controller of given board
  ECI_CTRL_CONFIG stcCtrlConfig = {0};

  //*** Use Basic settings to open controller
  stcCtrlConfig.wCtrlClass                 = ECI_CTRL_CAN;
  stcCtrlConfig.u.sCanConfig.dwVer         = ECI_STRUCT_VERSION_V0;
  stcCtrlConfig.u.sCanConfig.u.V0.bBtReg0  = ECI_CAN_BT0_1000KB;
  stcCtrlConfig.u.sCanConfig.u.V0.bBtReg1  = ECI_CAN_BT1_1000KB;
  stcCtrlConfig.u.sCanConfig.u.V0.bOpMode  = ECI_CAN_OPMODE_STANDARD;

  //*** Open and Initialize given Controller of given board
  hResult = ECI_CtrlOpen[dev->interface](&dev->dwCtrlHandle, dev->dwHwIndex, dev->dwCtrlIndex, &stcCtrlConfig);
  ECIDRV_CHECKERROR(ECIDRV_CtrlOpen);

  //*** Get Controller Capabilities
  if(ECI_OK == hResult)
  {
    //*** Enable ECI structure Version 1 support
    stcCtrlCaps.wCtrlClass       = ECI_CTRL_CAN;
    stcCtrlCaps.u.sCanCaps.dwVer = ECI_STRUCT_VERSION_V1;
    hResult = ECI_CtrlGetCapabilities[dev->interface](dev->dwCtrlHandle, &stcCtrlCaps);
    ECIDRV_CHECKERROR(ECIDRV_CtrlGetCapabilities);
    if(ECI_OK == hResult)
    {
      if(dev->verbose)
        EciPrintCtrlCapabilities(&stcCtrlCaps);

      //*** Check if CAN Controller and if Struct version up to V1 is supported
      if(ECI_CTRL_CAN == stcCtrlCaps.wCtrlClass)
      {
        if(ECI_STRUCT_VERSION_V0 == stcCtrlCaps.u.sCanCaps.dwVer)
          { dwCtrlFeatures = stcCtrlCaps.u.sCanCaps.u.V0.dwFeatures; }
        else
        if(ECI_STRUCT_VERSION_V1 == stcCtrlCaps.u.sCanCaps.dwVer)
          { dwCtrlFeatures = stcCtrlCaps.u.sCanCaps.u.V1.dwFeatures; }
      }
    }
  }

  //*** Re-configure given controller of given board
  if(ECI_OK == hResult)
  {
    ECI_CTRL_CONFIG stcCtrlConfig = {0};
    ECI_CANBTP      stcBtpSdr     = ECI_CAN_SDR_BTP_1000KB;
//    ECI_CANBTP      stcBtpFdr     = ECI_CAN_FDR_BTP_8000KB;

    //predefined baud rate settings
    ECI_CANBTP      stcBtp_10KB = ECI_CAN_BTP_10KB,
                    stcBtp_20KB = ECI_CAN_BTP_20KB,
                    stcBtp_50KB = ECI_CAN_BTP_50KB,
                    stcBtp_100KB = ECI_CAN_BTP_100KB,
                    stcBtp_125KB = ECI_CAN_BTP_125KB,
                    stcBtp_250KB = ECI_CAN_BTP_250KB,
                    stcBtp_500KB = ECI_CAN_BTP_500KB,
                    stcBtp_800KB = ECI_CAN_BTP_800KB,
                    stcBtp_1000KB = {0,  1000000,  110, 16, 16, 0}, //SP 87.4%, Recommended by CANOpen
                    stcBtpSdr_500KB = ECI_CAN_SDR_BTP_500KB,
                    stcBtpSdr_1000KB = ECI_CAN_SDR_BTP_1000KB;

    switch(dev->bitrate)
    {
      case 10:
        stcBtpSdr = stcBtp_10KB;
        break;
      case 20:
        stcBtpSdr = stcBtp_20KB;
        break;
      case 50:
        stcBtpSdr = stcBtp_50KB;
        break;
      case 100:
        stcBtpSdr = stcBtp_100KB;
        break;
      case 125:
        stcBtpSdr = stcBtp_125KB;
        break;
      case 250:
        stcBtpSdr = stcBtp_250KB;
        break;
      case 500:
        stcBtpSdr = stcBtp_500KB;
        break;
      case 800:
        stcBtpSdr = stcBtp_800KB;
        break;
      case 1000:
        stcBtpSdr = stcBtp_1000KB;
        break;
      default:
        error_setf(&dev->error, CAN_IXXAT_ERROR_SETUP, "Invalid baud rate");
        return dev->error.code;
    }

    //*** Prepare Config struct
    stcCtrlConfig.wCtrlClass                       = ECI_CTRL_CAN;
    stcCtrlConfig.u.sCanConfig.dwVer               = ECI_STRUCT_VERSION_V1;
    stcCtrlConfig.u.sCanConfig.u.V1.bOpMode        = ECI_CAN_OPMODE_STANDARD | ECI_CAN_OPMODE_EXTENDED;
    stcCtrlConfig.u.sCanConfig.u.V1.bOpMode       |= (dwCtrlFeatures & ECI_CAN_FEATURE_ERRFRAME) ? ECI_CAN_OPMODE_ERRFRAME : 0;
    stcCtrlConfig.u.sCanConfig.u.V1.bExMode       |= (dwCtrlFeatures & ECI_CAN_FEATURE_EXTDATA)  ? ECI_CAN_EXMODE_EXTDATA  : 0;
//    stcCtrlConfig.u.sCanConfig.u.V1.bExMode       |= (dwCtrlFeatures & ECI_CAN_FEATURE_FASTDATA) ? ECI_CAN_EXMODE_FASTDATA : 0;
    stcCtrlConfig.u.sCanConfig.u.V1.sBtpSdr        = stcBtpSdr;
//    if(dwCtrlFeatures & ECI_CAN_FEATURE_FASTDATA)
//      stcCtrlConfig.u.sCanConfig.u.V1.sBtpFdr      = stcBtpFdr;

    //*** Re-configure given Controller of given board
    hResult = ECI_CtrlOpen[dev->interface](&dev->dwCtrlHandle, dev->dwHwIndex, dev->dwCtrlIndex, &stcCtrlConfig);
    ECIDRV_CHECKERROR(ECIDRV_CtrlOpen);
  }

  //*** Start Controller
  if(ECI_OK == hResult)
  {
    hResult = ECI_CtrlStart[dev->interface](dev->dwCtrlHandle);
    ECIDRV_CHECKERROR(ECIDRV_CtrlStart);
  }

  if(ECI_OK != hResult)
    error_setf(&dev->error, CAN_IXXAT_ERROR_SETUP, ECI_GetErrorString[dev->interface](hResult));

  return dev->error.code;
}

int can_ixxat_device_send(can_ixxat_device_t* dev, const can_message_t* message) {

  error_clear(&dev->error);

//  time.tv_sec = 0;
//  time.tv_usec = dev->timeout*1e6;

//  FD_ZERO(&set);
//  FD_SET(dev->fd, &set);

//  result = select(dev->fd+1, NULL, &set, NULL, &time);
//  if (result == 0) {
//    error_set(&dev->error, CAN_IXXAT_ERROR_TIMEOUT);
//    return dev->error.code;
//  }

//  while ((result = IXXAT_SendMsg(dev->handle, 0, &msg)) ==
//      IXXAT_ERR_CAN_NO_TRANSMIT_BUF)
//    timer_sleep(1e-5);
//  if (result)
//    error_setf(&dev->error, CAN_IXXAT_ERROR_SEND, IXXAT_DecodeErrorMsg(result));

  ECI_RESULT       hResult      = ECI_OK;
  ECI_CTRL_MESSAGE stcCtrlMsg   = {0};

  //*** Prepare CAN Message to send
  stcCtrlMsg.wCtrlClass                            = ECI_CTRL_CAN;
  stcCtrlMsg.u.sCanMessage.dwVer                   = ECI_STRUCT_VERSION_V1;
  stcCtrlMsg.u.sCanMessage.u.V1.dwMsgId            = message->id;
  stcCtrlMsg.u.sCanMessage.u.V1.uMsgInfo.Bits.rtr  = message->rtr;
  stcCtrlMsg.u.sCanMessage.u.V1.uMsgInfo.Bits.dlc  = message->length;
  memcpy(stcCtrlMsg.u.sCanMessage.u.V1.abData, message->content, message->length);

  if(dev->verbose) {
    OS_Printf("CAN Tx> ");
    EciPrintCtrlMessage(&stcCtrlMsg);
    OS_Printf("\n");
  }

  //*** Send one message
  hResult = ECI_CtrlSend[dev->interface]( dev->dwCtrlHandle, &stcCtrlMsg, dev->timeout*1000);
  if(ECI_OK != hResult)
  {
    if(dev->verbose)
      OS_Printf("Error while sending CAN Messages\n");
    ECIDRV_CHECKERROR(ECIDRV_CtrlSend);
    error_setf(&dev->error, CAN_IXXAT_ERROR_SEND, ECI_GetErrorString[dev->interface](hResult));
  }

  return dev->error.code;
}

const char *ixxat_status_frame[] =
{
  "Transmission pending",
  "Data overrun occurred",
  "Error warning limit exceeded",
  "Bus off status",
  "Init mode active",
  "Bus coupling error",
  "Acknowledge error"
};

const char *ixxat_error_frame[] =
{
  "Unknown or no error",
  "Stuff error",
  "Form error",
  "Acknowledgment error",
  "Bit error",
  "Fast data bit error (CAN FD)",
  "CRC error",
  "Other (unspecified) error"
};

int can_ixxat_device_receive(can_ixxat_device_t* dev, can_message_t* message) {

  error_clear(&dev->error);

//  time.tv_sec = 0;
//  time.tv_usec = dev->timeout*1e6;

//  FD_ZERO(&set);
//  FD_SET(dev->fd, &set);

//  result = select(dev->fd+1, &set, NULL, NULL, &time);
//  if (result == 0) {
//    error_set(&dev->error, CAN_IXXAT_ERROR_TIMEOUT);
//    return dev->error.code;
//  }

//  while (IXXAT_Handle(dev->handle))
//    timer_sleep(1e-5);
//  *message = dev->msg_received;

  ECI_RESULT       hResult      = ECI_OK;

  //*** Try to receive some CAN Messages
  ECI_CTRL_MESSAGE stcCtrlMsg    = {0};
  DWORD            dwCount       = 0;

  dwCount = 1;
  hResult = ECI_CtrlReceive[dev->interface]( dev->dwCtrlHandle, &dwCount, &stcCtrlMsg, dev->timeout*1000);
  if((ECI_OK == hResult) && (dwCount > 0))
  {
    if(dev->verbose) {
      OS_Printf("CAN Rx> ");
      EciPrintCtrlMessage(&stcCtrlMsg);
      OS_Printf("\n");
    }
    if(stcCtrlMsg.u.sCanMessage.u.V1.uMsgInfo.Bits.type != ECI_CAN_MSGTYPE_DATA)
    {
      switch(stcCtrlMsg.u.sCanMessage.u.V1.uMsgInfo.Bits.type)
      {
        case ECI_CAN_MSGTYPE_INFO:
          if(dev->verbose)
            OS_Printf("Info frame received\n");
          error_setf(&dev->error, CAN_IXXAT_ERROR_RECEIVE, "Info frame received");
          break;

        case ECI_CAN_MSGTYPE_ERROR:
          if(dev->verbose) {
            OS_Printf("Error frame received: ");
            OS_Printf(ixxat_error_frame[stcCtrlMsg.u.sCanMessage.u.V1.abData[0]]);
            OS_Printf("\n");
          }
          error_setf(&dev->error, CAN_IXXAT_ERROR_RECEIVE, ixxat_error_frame[stcCtrlMsg.u.sCanMessage.u.V1.abData[0]]);
          break;

        case ECI_CAN_MSGTYPE_STATUS:
          OS_Printf("Status frame received: ");
          if(stcCtrlMsg.u.sCanMessage.u.V1.abData[0] & ECI_CAN_STATUS_TXPEND)
          {
            if(dev->verbose)
              OS_Printf(ixxat_status_frame[0]);
            error_setf(&dev->error, CAN_IXXAT_ERROR_RECEIVE, ixxat_status_frame[0]);
          }
          if(stcCtrlMsg.u.sCanMessage.u.V1.abData[0] & ECI_CAN_STATUS_OVRRUN)
          {
            if(dev->verbose)
              OS_Printf(ixxat_status_frame[1]);
            error_setf(&dev->error, CAN_IXXAT_ERROR_RECEIVE, ixxat_status_frame[1]);
          }
          if(stcCtrlMsg.u.sCanMessage.u.V1.abData[0] & ECI_CAN_STATUS_ERRLIM)
          {
            if(dev->verbose)
              OS_Printf(ixxat_status_frame[2]);
            error_setf(&dev->error, CAN_IXXAT_ERROR_RECEIVE, ixxat_status_frame[2]);
          }
          if(stcCtrlMsg.u.sCanMessage.u.V1.abData[0] & ECI_CAN_STATUS_BUSOFF)
          {
            if(dev->verbose)
              OS_Printf(ixxat_status_frame[3]);
            error_setf(&dev->error, CAN_IXXAT_ERROR_RECEIVE, ixxat_status_frame[3]);
          }
          if(stcCtrlMsg.u.sCanMessage.u.V1.abData[0] & ECI_CAN_STATUS_ININIT)
          {
            if(dev->verbose)
              OS_Printf(ixxat_status_frame[4]);
            error_setf(&dev->error, CAN_IXXAT_ERROR_RECEIVE, ixxat_status_frame[4]);
          }
          if(stcCtrlMsg.u.sCanMessage.u.V1.abData[0] & ECI_CAN_STATUS_BUSCERR)
          {
            if(dev->verbose)
              OS_Printf(ixxat_status_frame[5]);
            error_setf(&dev->error, CAN_IXXAT_ERROR_RECEIVE, ixxat_status_frame[5]);
          }
          if(stcCtrlMsg.u.sCanMessage.u.V1.abData[0] & ECI_CAN_STATUS_ACKERR)
          {
            if(dev->verbose)
              OS_Printf(ixxat_status_frame[6]);
            error_setf(&dev->error, CAN_IXXAT_ERROR_RECEIVE, ixxat_status_frame[6]);
          }
          OS_Printf("\n");
          break;
      }
    }

    dev->msg_received.id = stcCtrlMsg.u.sCanMessage.u.V1.dwMsgId;
    memcpy(dev->msg_received.content, stcCtrlMsg.u.sCanMessage.u.V1.abData, stcCtrlMsg.u.sCanMessage.u.V1.uMsgInfo.Bits.dlc);
    dev->msg_received.length = stcCtrlMsg.u.sCanMessage.u.V1.uMsgInfo.Bits.dlc;
    *message = dev->msg_received;
  }
  else
  {
    if(dev->verbose)
      OS_Printf("Error while receiving CAN Messages\n");
    ECIDRV_CHECKERROR(ECIDRV_CtrlReceive);
    error_setf(&dev->error, CAN_IXXAT_ERROR_RECEIVE, ECI_GetErrorString[dev->interface](hResult));
  }

  return dev->error.code;
}

//void can_ixxat_device_handle(int handle, const ECI_CTRL_MESSAGE* msg, void* custom) {
//  can_ixxat_device_t* dev = custom;

//  dev->msg_received.id = msg->msg.canmsg.id;

//  memcpy(dev->msg_received.content, msg->msg.canmsg.msg,
//    msg->msg.canmsg.length);
//  dev->msg_received.length = msg->msg.canmsg.length;
//}
