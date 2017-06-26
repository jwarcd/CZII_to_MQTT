// Copyright - Jared Gaillard - 2016
// MIT License
//
// Arduino CZII Project

#ifndef RingBuffer_h
#define RingBuffer_h

#include "Arduino.h"

class RingBuffer
{
  public:
    RingBuffer();
    bool add(byte value);
    byte peek(short position);
    void set(short position, byte value);
    byte read();
    void shift(short quantity);
    short length();
    void dump(short bufferLength);
    void reset();

    // Fixed size for now
    static const short MAX_BUFFER_SIZE = 256;

  private:
    byte buffer[MAX_BUFFER_SIZE];
    short bufferTailPos = 0;
    short bufferHeadPos = 0;
};

#endif
