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

#include <string/string.h>

#include "can_ixxat_stub.h"

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

static can_ixxat_device_t* ixxat_can_dev = NULL;
void ixxat_exit() {
  if(ixxat_can_dev)
    can_ixxat_device_close(ixxat_can_dev);
}

int can_ixxat_device_open(can_ixxat_device_t* dev, const char* name, int verbose) {

  error_clear(&dev->error);

  uint32_t  hResult       = 0;
  uint32_t       dwIndex       = 0;

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

  //*** Retrieve hardware info
  if(0 == hResult)
  {
    //Save ixxat device handle, add it to exit() handler to close the port when programme terminates
    ixxat_can_dev = dev;
    atexit(ixxat_exit);
  }

  return dev->error.code;
}

int can_ixxat_device_close(can_ixxat_device_t* dev) {

  error_clear(&dev->error);

  uint32_t            hResult         = 0;

  //*** Wait some time to ensure bus idle
  usleep(250000);

  //*** Close ECI Controller
  if(0 == hResult)
  {
    dev->dwCtrlHandle = -1;
    ixxat_can_dev = NULL;
  }

  return dev->error.code;
}

int can_ixxat_device_setup(can_ixxat_device_t* dev, int bitrate, double timeout) {

  error_clear(&dev->error);

  uint32_t            hResult         = 0;
  dev->dwCtrlHandle    = -1;

  dev->bitrate = bitrate;
  dev->timeout = timeout;

  return dev->error.code;
}

int can_ixxat_device_send(can_ixxat_device_t* dev, const can_message_t* message) {

  error_clear(&dev->error);

  uint32_t       hResult      = 0;

  if(dev->verbose) {
    printf("CAN Tx> ");
    printf("\n");
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

  uint32_t       hResult      = 0;

  //*** Try to receive some CAN Messages
  uint32_t            dwCount       = 0;

  dwCount = 1;
  if((0 == hResult)/* && (dwCount > 0)*/)
  {
    if(dev->verbose) {
      printf("CAN Rx> ");
      printf("\n");
    }

    can_message_t c_msg = {0};
    *message = c_msg;
  }

  return dev->error.code;
}
