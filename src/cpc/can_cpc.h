/*
Copyright 2008 Ralf Kaestner <ralf.kaestner@gmail.com>
ASL-ETHZ http://www.asl.ethz.ch/

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef CAN_CPC_H
#define CAN_CPC_H

/**
  *  \file can_cpc.h
  *  \brief CAN communication over CAN-CPC
  *  \author Ralf Kaestner
  *  This layer provides low-level mechanisms for CAN communication via
  *  CAN-CPC hardware.
  */

#include "can.h"

/** Predefined CAN-CPC parameters
  */
#define CAN_CPC_PARAMETER_DEVICE           "usb-dev"
#define CAN_CPC_PARAMETER_BITRATE          "bitrate"
#define CAN_CPC_PARAMETER_QUANTA_PER_BIT   "quanta-per-bit"
#define CAN_CPC_PARAMETER_SAMPLING_POINT   "sampling-point"
#define CAN_CPC_PARAMETER_TIMEOUT          "timeout"

/** Predefined CAN-CPC constants
  */
#define CAN_CPC_CLOCK_FREQUENCY            16e6
#define CAN_CPC_SYNC_JUMP_WIDTH            1
#define CAN_CPC_TRIPLE_SAMPLING            0

/** Predefined CAN-CPC error codes
  */
#define CAN_CPC_ERROR_NONE                 0
#define CAN_CPC_ERROR_OPEN                 1
#define CAN_CPC_ERROR_CLOSE                2
#define CAN_CPC_ERROR_SETUP                3
#define CAN_CPC_ERROR_TIMEOUT              4
#define CAN_CPC_ERROR_SEND                 5
#define CAN_CPC_ERROR_RECEIVE              6

/** \brief Predefined CAN-CPC error descriptions
  */
extern const char* can_cpc_errors[];

/** \brief CAN-CPC device structure
  */
typedef struct can_cpc_device_t {
  int handle;                   //!< Device handle.
  int fd;                       //!< File descriptor.
  char name[256];               //!< Device name.

  int bitrate;                  //!< Device bitrate in [kbit/s].
  int quanta_per_bit;           //!< Number of quanta per bit.
  double sampling_point;        //!< Sampling point in the range [0, 1].
  double timeout;               //!< Device select timeout in [s].

  can_message_t msg_received;   //!< The most recent message received.
  ssize_t num_sent;             //!< Number of messages sent from device.
  ssize_t num_received;         //!< Number of messages received by device.
} can_cpc_device_t, *can_cpc_device_p;

/** \brief Open the CAN-CPC device with the specified name
  * \param[in] dev The CAN-CPC device to be opened.
  * \param[in] name The name of the CAN-CPC to be opened.
  * \return The resulting error code.
  */
int can_cpc_open(
  can_cpc_device_p dev,
  const char* name);

/** \brief Close an open CAN-CPC device
  * \param[in] dev The open CAN-CPC device to be closed.
  * \return The resulting error code.
  */
int can_cpc_close(
  can_cpc_device_p dev);

/** \brief Setup an already opened CAN-CPC device
  * \param[in] dev The open serial CAN-CPC to be set up.
  * \param[in] bitrate The device bitrate to be set in [kbit/s].
  * \param[in] quanta_per_bit The device's number of quanta per bit.
  * \param[in] sampling_point The sampling point in the range [0, 1].
  * \param[in] timeout The device select timeout to be set in [s].
  * \return The resulting error code.
  */
int can_cpc_setup(
  can_cpc_device_p dev,
  int bitrate,
  int quanta_per_bit,
  double sampling_point,
  double timeout);

/** \brief Send message over open CAN-CPC device
  * \param[in] dev The open CAN-CPC device to send the message over.
  * \param[in] message The CAN message to be sent over the device.
  * \return The resulting error code.
  */
int can_cpc_send(
  can_cpc_device_p dev,
  can_message_p message);

/** \brief Receive message on open CAN-CPC device
  * \param[in] dev The open CAN-CPC device to receive the message on.
  * \param[out] message The CAN message received on the device.
  * \return The resulting error code.
  */
int can_cpc_receive(
  can_cpc_device_p dev,
  can_message_p message);

#endif
