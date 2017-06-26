// Copyright - Jared Gaillard - 2016
// MIT License
//
// Arduino CZII Project

#ifndef Util_h
#define Util_h

#include <Arduino.h>
#include "RingBuffer.h"

//#define DEBUG_LOG;
#define INFO_LOG;

#ifdef DEBUG_LOG
#define debug_print(x)   Serial.print (x)
#define debug_println(x)  Serial.println (x)
#else
#define debug_print(x)
#define debug_println(x)
#endif

#ifdef INFO_LOG
#define info_print(x)   Serial.print (x)
#define info_println(x)  Serial.println (x)
#else
#define info_print(x)
#define info_println(x)
#endif

#define array_len( x )  ( sizeof( x ) / sizeof( x[0] ) )

#define FLOAT_MIN_VALUE -1000.0f  // We shouldn't have values less than this 

uint16_t ModRTU_CRC(RingBuffer& ringBuffer, byte length);
uint16_t ModRTU_CRC(byte values[], byte length);

#endif
