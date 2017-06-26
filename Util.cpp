// Copyright - Jared Gaillard - 2016
// MIT License
//
// Arduino CZII Project

#include "Util.h"

//
//  Compute CRC checksum from a RingBuffer
//
uint16_t ModRTU_CRC(RingBuffer& ringBuffer, byte length)
{
  uint16_t crc = 0x0000;

  for (int pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)(ringBuffer.peek(pos)); // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // NOTE, this number has low and high bytes swapped, so use it accordingly
  return crc;
}

//
//  Compute CRC checksum from a byte array
//
uint16_t ModRTU_CRC(byte values[], byte length)
{
  uint16_t crc = 0x0000;

  for (int pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)(values[pos]); // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // NOTE, this number has low and high bytes swapped, so use it accordingly
  return crc;
}

