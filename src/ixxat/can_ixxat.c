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

//#include <libixxat/ixxat.h>
//#include <libixxat/ixxatlib.h>
#include "ECI005.h"

#include <string/string.h>

#include <timer/timer.h>

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
    "/dev/ixxat_usb0",
    "",
    "Path to the special file of the connected CAN-IXXAT device"},
  {CAN_IXXAT_PARAMETER_BIT_RATE,
    config_param_type_int,
    "1000",
    "[10, 1000]",
    "The requested bit rate of the CAN bus in [kbit/s]"},
  {CAN_IXXAT_PARAMETER_QUANTA_PER_BIT,
    config_param_type_int,
    "8",
    "[8, 16]",
    "The requested number of time quanta per bit of the CAN bus"},
  {CAN_IXXAT_PARAMETER_SAMPLING_POINT,
    config_param_type_float,
    "0.75",
    "[0.75, 0.875]",
    "The requested synchronization sample point of the CAN bus, "
    "expressed as a ratio of the second phase buffer segment's "
    "start time quantum and the number of time quanta per bit"},
  {CAN_IXXAT_PARAMETER_TIMEOUT,
    config_param_type_float,
    "0.01",
    "",
    "The CAN bus communication timeout in [s]"},
};

const config_default_t can_default_config = {
  can_ixxat_default_parameters,
  sizeof(can_ixxat_default_parameters)/sizeof(config_param_t),
};

void can_ixxat_device_init(can_ixxat_device_t* dev);
void can_ixxat_device_destroy(can_ixxat_device_t* dev);
void can_ixxat_device_handle(int handle, const ECI_CTRL_MESSAGE* msg, void* custom);

int can_device_open(can_device_t* dev) {
  error_clear(&dev->error);

  if (!dev->num_references) {
    dev->comm_dev = malloc(sizeof(can_ixxat_device_t));
    can_ixxat_device_init(dev->comm_dev);

    dev->num_sent = 0;
    dev->num_received = 0;

    if (can_ixxat_device_open(dev->comm_dev,
        config_get_string(&dev->config, CAN_IXXAT_PARAMETER_DEVICE)) ||
      can_ixxat_device_setup(dev->comm_dev,
        config_get_int(&dev->config, CAN_IXXAT_PARAMETER_BIT_RATE),
        config_get_int(&dev->config, CAN_IXXAT_PARAMETER_QUANTA_PER_BIT),
        config_get_float(&dev->config, CAN_IXXAT_PARAMETER_SAMPLING_POINT),
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
  dev->handle = 0;
  dev->fd = 0;
  dev->name = 0;

  dev->bitrate = 0;
  dev->quanta_per_bit = 0;
  dev->sampling_point = 0.0;
  dev->timeout = 0.0;

  memset(&dev->msg_received, 0, sizeof(can_message_t));

  error_init(&dev->error, can_ixxat_errors);
}

void can_ixxat_device_destroy(can_ixxat_device_t* dev) {
  string_destroy(&dev->name);
  error_destroy(&dev->error);
}

#include "EciDemoCommon.h"
/** ECI Demo error check macro @ingroup EciDemo */
#define ECIDEMO_CHECKERROR(FuncName) \
{\
  if(ECI_OK == hResult)\
  {\
    OS_Printf(#FuncName "...succeeded.\n"); \
  }\
  else\
  {\
    OS_Printf( #FuncName "...failed with error code: 0x%08X. %s\n", \
               hResult, \
               ECI005_GetErrorString(hResult)); \
  }\
}
#define ECIDEMO_CHECKERROR(FuncName) {}

int can_ixxat_device_open(can_ixxat_device_t* dev, const char* name) {
//  int result;

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
  dev->dwHwIndex            = 0;
  dev->dwCtrlIndex          = 0;

//  OS_Printf("\n>> ECI Demo for CAN-IB1x0 / PCIe (Mini), (104) <<\n");

  //*** Prepare Hardware parameter structure for multiple boards
  for(dwIndex=0; dwIndex < _countof(astcHwPara); dwIndex++)
  {
    astcHwPara[dwIndex].wHardwareClass = ECI_HW_PCI;
    #ifdef ECIDEMO_HWUSEPOLLINGMODE
      astcHwPara[dwIndex].dwFlags = ECI_SETTINGS_FLAG_POLLING_MODE;
    #endif //ECIDEMO_HWUSEPOLLINGMODE
  }

  //*** At first call Initialize to prepare ECI driver
  hResult = ECI005_Initialize(_countof(astcHwPara), astcHwPara);
  ECIDEMO_CHECKERROR(ECI005_Initialize);

  //*** Retrieve hardware info
  if(ECI_OK == hResult)
  {
    //*** Retrieve hardware info
    hResult = ECI005_GetInfo(dev->dwHwIndex, &stcHwInfo);
    ECIDEMO_CHECKERROR(ECI005_GetInfo);
//    if(ECI_OK == hResult)
//      {EciPrintHwInfo(&stcHwInfo);}
  }

  //*** Find first CAN Controller of Board
  if(ECI_OK == hResult)
  {
    hResult = EciGetNthCtrlOfClass(&stcHwInfo,
                                   ECI_CTRL_CAN,
                                   0, //first relative controller
                                   &dev->dwCtrlIndex);
    if(ECI_OK == hResult)
    {
      //*** Start CAN Demo
//      hResult =  EciCanDemo005(dwHwIndex, dwCtrlIndex);
//      ECIDEMO_CHECKERROR(EciCanDemo005);
    }
    else
    {
      //*** Ignore if no controller was found
//      hResult = ECI_OK;
    }
  }

  if(hResult != ECI_OK)
    error_setf(&dev->error, CAN_IXXAT_ERROR_OPEN, ECI005_GetErrorString(hResult));

  return dev->error.code;
}

int can_ixxat_device_close(can_ixxat_device_t* dev) {
//  int result;

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
    hResult = ECI005_CtrlStop(dev->dwCtrlHandle, ECI_STOP_FLAG_NONE);
    ECIDEMO_CHECKERROR(ECI005_CtrlStop);
  }

  //*** Wait some time to ensure bus idle
  OS_Sleep(250);

  //*** Reset Controller
  if(ECI_OK == hResult)
  {
    hResult = ECI005_CtrlStop(dev->dwCtrlHandle, ECI_STOP_FLAG_RESET_CTRL);
    ECIDEMO_CHECKERROR(ECI005_CtrlStop);
  }

  //*** Close ECI Controller
  if(ECI_OK == hResult)
  {
    ECI005_CtrlClose(dev->dwCtrlHandle);
    ECIDEMO_CHECKERROR(ECI005_CtrlClose);
    dev->dwCtrlHandle = ECI_INVALID_HANDLE;
  }

  //*** Clean up ECI driver
  hResult = ECI005_Release();
  ECIDEMO_CHECKERROR(ECI005_Release);
  if(hResult != ECI_OK)
      error_setf(&dev->error, CAN_IXXAT_ERROR_CLOSE, ECI005_GetErrorString(hResult));

//  OS_Printf("-> Returning from ECI Demo for CAN-IB1x0 / PCIe (Mini), (104) <-\n");

  return dev->error.code;
}

int can_ixxat_device_setup(can_ixxat_device_t* dev, int bitrate, int
  quanta_per_bit, double sampling_point, double timeout) {
//  int result;
//  IXXAT_INIT_PARAMS_T* parameters;

  error_clear(&dev->error);

//  double t = 1.0/(8*bitrate*1e3);
//  int brp = round(4*t*CAN_IXXAT_CLOCK_FREQUENCY/quanta_per_bit);
//  int tseg1 = round(quanta_per_bit*sampling_point);
//  int tseg2 = quanta_per_bit-tseg1;

//  parameters = IXXAT_GetInitParamsPtr(dev->handle);
//  parameters->canparams.cc_type = SJA1000;
//  parameters->canparams.cc_params.sja1000.btr0 =
//    ((CAN_IXXAT_SYNC_JUMP_WIDTH-1) << 6)+(brp-1);
//  parameters->canparams.cc_params.sja1000.btr1 =
//    (CAN_IXXAT_TRIPLE_SAMPLING << 7)+((tseg2-1) << 4)+(tseg1-2);
//  parameters->canparams.cc_params.sja1000.outp_contr = 0xda;

//  parameters->canparams.cc_params.sja1000.acc_code1 = 0xff;
//  parameters->canparams.cc_params.sja1000.acc_code2 = 0xff;
//  parameters->canparams.cc_params.sja1000.acc_code3 = 0xff;
//  parameters->canparams.cc_params.sja1000.acc_mask0 = 0xff;
//  parameters->canparams.cc_params.sja1000.acc_mask1 = 0xff;
//  parameters->canparams.cc_params.sja1000.acc_mask2 = 0xff;
//  parameters->canparams.cc_params.sja1000.acc_mask3 = 0xff;
//  parameters->canparams.cc_params.sja1000.mode = 0;

//  if (!(result = IXXAT_CANInit(dev->handle, 0))) {
//    dev->fd = IXXAT_GetFdByHandle(dev->handle);

//    dev->bitrate = bitrate;
//    dev->quanta_per_bit = quanta_per_bit;
//    dev->sampling_point = sampling_point;
//    dev->timeout = timeout;

//    if ((result = IXXAT_Control(dev->handle, CONTR_CAN_Message |
//        CONTR_CONT_ON)))
//    error_setf(&dev->error, CAN_IXXAT_ERROR_SETUP, IXXAT_DecodeErrorMsg(result));
//  }
//  else
//    error_setf(&dev->error, CAN_IXXAT_ERROR_SETUP, IXXAT_DecodeErrorMsg(result));

  ECI_RESULT            hResult         = ECI_OK;
  ECI_CTRL_CAPABILITIES stcCtrlCaps     = {0};
  dev->dwCtrlHandle    = ECI_INVALID_HANDLE;
  DWORD                 dwCtrlFeatures  = 0;

//  OS_Printf("\n>> ECI CAN Demo <<\n");

  dev->bitrate = bitrate;
  dev->quanta_per_bit = quanta_per_bit;
  dev->sampling_point = sampling_point;
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
  hResult = ECI005_CtrlOpen(&dev->dwCtrlHandle, dev->dwHwIndex, dev->dwCtrlIndex, &stcCtrlConfig);
  ECIDEMO_CHECKERROR(ECI005_CtrlOpen);

  //*** Get Controller Capabilities
  if(ECI_OK == hResult)
  {
    //*** Enable ECI structure Version 1 support
    stcCtrlCaps.wCtrlClass       = ECI_CTRL_CAN;
    stcCtrlCaps.u.sCanCaps.dwVer = ECI_STRUCT_VERSION_V1;
    hResult = ECI005_CtrlGetCapabilities(dev->dwCtrlHandle, &stcCtrlCaps);
    ECIDEMO_CHECKERROR(ECI005_CtrlGetCapabilities);
    if(ECI_OK == hResult)
    {
//      EciPrintCtrlCapabilities(&stcCtrlCaps);

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
    ECI_CANBTP      stcBtpFdr     = ECI_CAN_FDR_BTP_8000KB;

    //*** Prepare Config struct
    stcCtrlConfig.wCtrlClass                       = ECI_CTRL_CAN;
    stcCtrlConfig.u.sCanConfig.dwVer               = ECI_STRUCT_VERSION_V1;
    stcCtrlConfig.u.sCanConfig.u.V1.bOpMode        = ECI_CAN_OPMODE_STANDARD | ECI_CAN_OPMODE_EXTENDED;
    stcCtrlConfig.u.sCanConfig.u.V1.bOpMode       |= (dwCtrlFeatures & ECI_CAN_FEATURE_ERRFRAME) ? ECI_CAN_OPMODE_ERRFRAME : 0;
    stcCtrlConfig.u.sCanConfig.u.V1.bExMode       |= (dwCtrlFeatures & ECI_CAN_FEATURE_EXTDATA)  ? ECI_CAN_EXMODE_EXTDATA  : 0;
    stcCtrlConfig.u.sCanConfig.u.V1.bExMode       |= (dwCtrlFeatures & ECI_CAN_FEATURE_FASTDATA) ? ECI_CAN_EXMODE_FASTDATA : 0;
    stcCtrlConfig.u.sCanConfig.u.V1.sBtpSdr        = stcBtpSdr;
    if(dwCtrlFeatures & ECI_CAN_FEATURE_FASTDATA)
    {
      stcCtrlConfig.u.sCanConfig.u.V1.sBtpFdr      = stcBtpFdr;
    }

    //*** Re-configure given Controller of given board
    hResult = ECI005_CtrlOpen(&dev->dwCtrlHandle, dev->dwHwIndex, dev->dwCtrlIndex, &stcCtrlConfig);
    ECIDEMO_CHECKERROR(ECI005_CtrlOpen);
  }

  //*** Start Controller
  if(ECI_OK == hResult)
  {
    hResult = ECI005_CtrlStart(dev->dwCtrlHandle);
    ECIDEMO_CHECKERROR(ECI005_CtrlStart);
  }

  return dev->error.code;
}

int can_ixxat_device_send(can_ixxat_device_t* dev, const can_message_t* message) {
//  IXXAT_CAN_MSG_T msg = {0x00L, 0, {0, 0, 0, 0, 0, 0, 0, 0}};
//  struct timeval time;
//  fd_set set;
//  int result;

  error_clear(&dev->error);

//  msg.id = message->id;
//  msg.length = message->length;
//  memcpy(msg.msg, message->content, message->length);

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

//  OS_Printf("Now, sending a CAN Message\n");

  //*** Prepare CAN Message to send
  stcCtrlMsg.wCtrlClass                            = ECI_CTRL_CAN;
  stcCtrlMsg.u.sCanMessage.dwVer                   = ECI_STRUCT_VERSION_V1;
  stcCtrlMsg.u.sCanMessage.u.V1.dwMsgId            = message->id;
  stcCtrlMsg.u.sCanMessage.u.V1.uMsgInfo.Bits.dlc  = message->length;
  memcpy(stcCtrlMsg.u.sCanMessage.u.V1.abData, message->content, message->length);

//  OS_Printf("\n");
//  EciPrintCtrlMessage(&stcCtrlMsg);
//  OS_Printf("\n");
//  OS_Fflush(stdout);

  //*** Send one message
  hResult = ECI005_CtrlSend( dev->dwCtrlHandle, &stcCtrlMsg, dev->timeout*1000);
  if(ECI_OK != hResult)
  {
    OS_Printf("Error while sending CAN Messages\n");
    ECIDEMO_CHECKERROR(ECI005_CtrlSend);
    hResult = ECI_OK;
  }

  return dev->error.code;
}

int can_ixxat_device_receive(can_ixxat_device_t* dev, can_message_t* message) {
//  struct timeval time;
//  fd_set set;
//  int result;

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
  DWORD            dwStartTime   = 0;
  DWORD            dwCurrentTime = 0;
  DWORD            dwCount       = 0;

  //*** Receive Messages
//  OS_Printf("Now, receiving CAN Messages for %0.3f seconds\n", dev->timeout);

  dwCount = 1;
  hResult = ECI005_CtrlReceive( dev->dwCtrlHandle, &dwCount, &stcCtrlMsg, dev->timeout*1000);
  if((ECI_OK == hResult) && (dwCount > 0))
  {
//    OS_Printf("\n");
//    EciPrintCtrlMessage(&stcCtrlMsg);
//    OS_Fflush(stdout);

    dev->msg_received.id = stcCtrlMsg.u.sCanMessage.u.V1.dwMsgId;
    memcpy(dev->msg_received.content, stcCtrlMsg.u.sCanMessage.u.V1.abData, stcCtrlMsg.u.sCanMessage.u.V1.uMsgInfo.Bits.dlc);
    dev->msg_received.length = stcCtrlMsg.u.sCanMessage.u.V1.uMsgInfo.Bits.dlc;
    *message = dev->msg_received;
  }//endif
  else
  {
//    OS_Printf(".");
//    OS_Fflush(stdout);
  }
//  OS_Printf("\n");

  return dev->error.code;
}

void can_ixxat_device_handle(int handle, const ECI_CTRL_MESSAGE* msg, void* custom) {
//  can_ixxat_device_t* dev = custom;

//  dev->msg_received.id = msg->msg.canmsg.id;

//  memcpy(dev->msg_received.content, msg->msg.canmsg.msg,
//    msg->msg.canmsg.length);
//  dev->msg_received.length = msg->msg.canmsg.length;
}
