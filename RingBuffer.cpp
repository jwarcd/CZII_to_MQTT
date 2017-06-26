// Copyright - Jared Gaillard - 2016
// MIT License
//
// Arduino CZII Project

#include "Arduino.h"
#include "RingBuffer.h"

RingBuffer::RingBuffer()
{
}

/**
   Adds byte to the head of the buffer
*/
bool RingBuffer::add(byte value) {
  if ((bufferHeadPos + 1) % MAX_BUFFER_SIZE == bufferTailPos) {
    return false;
  }

  buffer[bufferHeadPos] = value;
  bufferHeadPos = (bufferHeadPos + 1) % MAX_BUFFER_SIZE;
  return true;
}

/**
   Returns the byte from the specified buffer position
*/
byte RingBuffer::peek(short position) {
  if ( bufferHeadPos == bufferTailPos)
    return -1;

  return buffer[(bufferTailPos + position) % MAX_BUFFER_SIZE];
}

/**
   sets the value for the specified position
*/
void RingBuffer::set(short position, byte value) {
  buffer[position] = value;
}

/**
   Returns the byte from the specified buffer position
*/
byte RingBuffer::read() {
  if ( bufferHeadPos == bufferTailPos)
    return -1;

  byte value = buffer[bufferTailPos];
  shift(1);
  return value;
}

/**
  Shift the buffer by quantity specified.
*/
void RingBuffer::shift(short quantity) {
  bufferTailPos = (bufferTailPos + quantity) % MAX_BUFFER_SIZE;
}

short RingBuffer::length() {
  return (MAX_BUFFER_SIZE + bufferHeadPos - bufferTailPos) % MAX_BUFFER_SIZE;
}

void RingBuffer::reset() {
  bufferHeadPos = 0;
  bufferTailPos = 0;
}

/**
   debug dump of the entire buffer
*/
void RingBuffer::dump(short bufferLength) {
  Serial.print("BUFFER: ");
  for (int pos = 0; pos < bufferLength; pos++) {
    byte value = peek(pos);
    Serial.print(String(value) + ".");
  }
  Serial.println();
}
