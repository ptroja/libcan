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


#ifndef CAN_USB_H
#define CAN_USB_H

/**
  *  \file can_usb.h
  *  \brief CAN communication over EPOS USB
  *  \author Piotr Trojanek
  *  This layer provides low-level mechanisms for CAN communication via EPOS
  *  controllers over a USB connection.
  */

#include "can.h"

#define CAN_SERIAL_OPCODE_READ                0x10
#define CAN_SERIAL_OPCODE_WRITE               0x11

#define CAN_SERIAL_OKAY 		                  0x4F
#define CAN_SERIAL_FAILED		                  0x46
#define CAN_SERIAL_RESPONSE	                  0x00

/** Predefined CAN serial constants
  */

#define CAN_SERIAL_SEND_ID                    0x0600
#define CAN_SERIAL_RECEIVE_ID                 0x0580

#define CAN_SERIAL_WRITE_SEND_1_BYTE          0x2F
#define CAN_SERIAL_WRITE_SEND_2_BYTE          0x2B
#define CAN_SERIAL_WRITE_SEND_4_BYTE          0x23
#define CAN_SERIAL_WRITE_RECEIVE              0x60

#define CAN_SERIAL_READ_RECEIVE_UNDEFINED     0x42
#define CAN_SERIAL_READ_SEND                  0x40

#define CAN_SERIAL_ABORT                      0xC0

/** Predefined CAN serial parameters
  */
#define CAN_SERIAL_PARAMETER_DEVICE           "serial-dev"
#define CAN_SERIAL_PARAMETER_BAUDRATE         "serial-baudrate"
#define CAN_SERIAL_PARAMETER_DATABITS         "serial-databits"
#define CAN_SERIAL_PARAMETER_STOPBITS         "serial-stopbits"
#define CAN_SERIAL_PARAMETER_PARITY           "serial-parity"
#define CAN_SERIAL_PARAMETER_TIMEOUT          "timeout"

/** Predefined CAN serial error codes
  */
#define CAN_SERIAL_ERROR_NONE                 0
#define CAN_SERIAL_ERROR_CONVERT              1
#define CAN_SERIAL_ERROR_SEND                 2
#define CAN_SERIAL_ERROR_RECEIVE              3
#define CAN_SERIAL_ERROR_READ                 4
#define CAN_SERIAL_ERROR_WRITE                5
#define CAN_SERIAL_ERROR_NO_RESPONSE          6
#define CAN_SERIAL_ERROR_UNEXPECTED_RESPONSE  7
#define CAN_SERIAL_ERROR_CRC                  8

/** \brief Predefined CAN serial error descriptions
  */
extern const char* can_serial_errors[];

/** Convert an EPOS message into serial data.
  * \note This conversion is necessary to make EPOS messages compliant
  *   with the CAN protocol.
  * \param[in] dev The sending CAN device for which to convert the message.
  * \param[in] message The CAN message to be converted.
  * \param[out] data An array to store the converted data frame.
  * \return The number of bytes in the data frame to be sent or the
  *   negative error code.
  */
int can_serial_from_epos(
  can_device_p dev,
  can_message_p message,
  unsigned char* data);

/** Convert serial data to an EPOS message.
  * \note This conversion is necessary in order to provide CAN protocol
  *   messages to libepos.
  * \param[in] dev The receiving CAN device for which to convert the message.
  * \param[in] data An array containing the data frame sent to EPOS.
  * \param[in,out] message The CAN message to be converted.
  * \return The resulting negative error code.
  */
int can_serial_to_epos(
  can_device_p dev,
  unsigned char* data,
  can_message_p message);

/** Send serial data frame to EPOS.
  * \param[in] dev The open CAN serial device to send data to.
  * \param[in] data An array containing the serial data frame to be sent.
  * \param[in] num The size of the serial data frame to be sent.
  * \return The number of bytes sent to the CAN serial device or the
  *   negative error code.
  */
int can_serial_send(
  can_device_p dev,
  unsigned char* data,
  ssize_t num);

/** Receive serial data frame from EPOS.
  * \param[in] dev The open CAN serial device to reveice data from.
  * \param[out] data An array representing the serial data frame received.
  * \return The number of bytes received from the CAN serial device or the
  *   negative error code.
  */
int can_serial_receive(
  can_device_p dev,
  unsigned char* data);

/** Change the order of bytes in the data frame. The first two characters
  * will be ignored, the following characters will be reordered.
  * \note This is necessary according to the EPOS Communication guide.
  * \param[in,out] data An array of bytes for which to change the order.
  * \param[in] num The number of bytes in the array.
  * \return The number of reordered bytes within the data frame.
  */
ssize_t can_serial_change_byte_order(
  unsigned char* data,
  ssize_t num);

/** Change the order of words in the data frame. The first two characters will
  * be ignored, the following characters will be reordered in groups of two.
  * \note This is necessary according to the EPOS Communication guide.
  * \param[in,out] data An array of words for which to change order.
  * \param[in] num The number of bytes in the word array.
  * \return The number of reordered bytes within the data frame.
  */
ssize_t can_serial_change_word_order(
  unsigned char* data,
  ssize_t num);

/** Calculate a 16-bit CRC checksum using CRC-CCITT algorithm.
  * \note Calculation has to include all bytes in the data frame. Internally,
  *   the array is transformed to an array of words in order to calculate the
  *   CRC. The CRC word is then tranformed back to an array of characters.
  * \param[in] data An array of bytes representing the data frame.
  * \param[in] num The number of bytes in the data frame.
  * \param[out] crc_value An array of two bytes to store the CRC-word.
  * \return The number of words built from the array.
  */
ssize_t can_serial_calc_crc(
  unsigned char* data,
  ssize_t num,
  unsigned char* crc_value);

/** Implementation of the CRC-CCITT algorithm.
  * \param[in] data An array of words containing the data frame.
  * \param[in] num The number of words in the data frame.
  * \return The calculated CRC-value.
  */
unsigned short can_usb_crc_alg(
  unsigned short* data,
  ssize_t num);

#endif
