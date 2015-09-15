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

#ifndef CAN_IXXAT_H
#define CAN_IXXAT_H

/**
  *  \file can_ixxat.h
  *  \brief CAN communication over CAN-IXXAT
  *  \author Ralf Kaestner
  * 
  *  This layer provides low-level mechanisms for CANopen communication via
  *  CAN-IXXAT hardware.
  */

#include "can.h"

/** \name Parameters
  * \brief Predefined CAN-IXXAT parameters
  */
//@{
#define CAN_IXXAT_PARAMETER_DEVICE           "ixxat-dev"
#define CAN_IXXAT_PARAMETER_BIT_RATE         "ixxat-bit-rate"
//#define CAN_IXXAT_PARAMETER_QUANTA_PER_BIT   "ixxat-quanta-per-bit"
//#define CAN_IXXAT_PARAMETER_SAMPLING_POINT   "ixxat-sampling-point"
#define CAN_IXXAT_PARAMETER_TIMEOUT          "ixxat-timeout"
#define CAN_IXXAT_PARAMETER_VERBOSE          "ixxat-verbose"
//@}

/** \name Constants
  * \brief Predefined CAN-IXXAT constants
  */
//@{
//#define CAN_IXXAT_CLOCK_FREQUENCY            40e6
//#define CAN_IXXAT_SYNC_JUMP_WIDTH            1
//#define CAN_IXXAT_TRIPLE_SAMPLING            0
//@}

/** \name Error Codes
  * \brief Predefined CAN-IXXAT error codes
  */
//@{
#define CAN_IXXAT_ERROR_NONE                 0
//!< Success
#define CAN_IXXAT_ERROR_OPEN                 1
//!< Failed to open CAN-IXXAT device
#define CAN_IXXAT_ERROR_CLOSE                2
//!< Failed to close CAN-IXXAT device
#define CAN_IXXAT_ERROR_SETUP                3
//!< Failed to set CAN-IXXAT device parameters
#define CAN_IXXAT_ERROR_TIMEOUT              4
//!< CAN-IXXAT device timeout
#define CAN_IXXAT_ERROR_SEND                 5
//!< Failed to send to CAN-IXXAT device
#define CAN_IXXAT_ERROR_RECEIVE              6
//!< Failed to receive from CAN-IXXAT device
//@}

/** \brief Predefined CAN-IXXAT error descriptions
  */
extern const char* can_ixxat_errors[];

typedef enum {
  IXXAT_PCI = 0,
  IXXAT_USB
}IXXAT_INTERFACE;

/** \brief CAN-IXXAT device structure
  */
typedef struct can_ixxat_device_t {
  char* name;                   //!< Device name.
  IXXAT_INTERFACE interface;
  uint32_t dwHwIndex;
  uint32_t dwCtrlIndex;
  uint32_t dwCtrlHandle;    //!< Device handle.

  int bitrate;                  //!< Device bitrate in [kbit/s].
  double timeout;               //!< Device select timeout in [s].
  int verbose;                  //!< Verbose mode.

  can_message_t msg_received;   //!< The most recent message received.
  
  error_t error;                //!< The most recent device error.
} can_ixxat_device_t;

/** \brief Open the CAN-IXXAT device with the specified name
  * \param[in] dev The CAN-IXXAT device to be opened.
  * \param[in] name The name of the CAN-IXXAT to be opened.
  * \param[in] verbose Debugging mode.
  * \return The resulting error code.
  */
int can_ixxat_device_open(can_ixxat_device_t* dev,
  const char* name, int verbose);

/** \brief Close an open CAN-IXXAT device
  * \param[in] dev The open CAN-IXXAT device to be closed.
  * \return The resulting error code.
  */
int can_ixxat_device_close(
  can_ixxat_device_t* dev);

/** \brief Setup an already opened CAN-IXXAT device
  * \param[in] dev The open serial CAN-IXXAT to be set up.
  * \param[in] bitrate The device bitrate to be set in [kbit/s].
  * \param[in] timeout The device select timeout to be set in [s].
  * \return The resulting error code.
  */
int can_ixxat_device_setup(can_ixxat_device_t* dev,
  int bitrate, double timeout);

/** \brief Send a CANopen SDO message over an open CAN-IXXAT device
  * \param[in] dev The open CAN-IXXAT device to send the message over.
  * \param[in] message The CANopen SDO message to be sent over the device.
  * \return The resulting error code.
  */
int can_ixxat_device_send(
  can_ixxat_device_t* dev,
  const can_message_t* message);

/** \brief Receive a CANopen SDO message on an open CAN-IXXAT device
  * \param[in] dev The open CAN-IXXAT device to receive the message on.
  * \param[out] message The CANopen SDO message received on the device.
  * \return The resulting error code.
  */
int can_ixxat_device_receive(
  can_ixxat_device_t* dev,
  can_message_t* message);

#endif
