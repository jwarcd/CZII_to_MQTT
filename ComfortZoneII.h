// Copyright - Jared Gaillard - 2016
// MIT License
//
// Arduino CZII Project

#ifndef ComfortZoneII_h
#define ComfortZoneII_h

#include <ArduinoJson.h>
#include <Arduino.h>
#include <TimeLib.h>
#include "Zone.hpp"
#include "RingBuffer.h"
#include "Util.h"

#define TABLE1_TEMPS_ROW   		16
#define TABLE1_TIME_ROW   		18

class ComfortZoneII
{
  public:
    ComfortZoneII(byte numberZones);
    Zone* getZone(byte zoneIndex);
    bool update(RingBuffer& ringBuffer);
    bool isZoneModified();
    void clearZoneModified();
    bool isStatusModified();
    void clearStatusModified();
    String toZoneJson();
    String toStatusJson();

    // CZII frame
    static const byte DEST_ADDRESS_POS     = 0;
    static const byte SOURCE_ADDRESS_POS   = 2;
    static const byte DATA_LENGTH_POS      = 4;
    static const byte FUNCTION_POS         = 7;
    static const byte DATA_START_POS       = 8;
    static const byte MIN_MESSAGE_SIZE     = 11;

    // CZII Function Codes
    static const byte RESPONSE_FUNCTION    = 6;
    static const byte READ_FUNCTION        = 11;
    static const byte WRITE_FUNCTION       = 12;

  private:

    Zone* zones[8];
    byte NUMBER_ZONES;
    bool statusModified;

    byte controllerState = -1;
    byte lat_Temp_f = -1;
    float outside_Temp_f = FLOAT_MIN_VALUE;
    float outside_Temp2_f = FLOAT_MIN_VALUE;
    TimeElements time;

	bool isValidTemperature(float value);
    float getTemperatureF(byte highByte, byte lowByte);
    void updateZoneInfo(RingBuffer& ringBuffer);
    void updateOutsideHumidityTemp(RingBuffer& ringBuffer);
    void updateOutsideTemp(RingBuffer& ringBuffer);
    void updateControllerState(RingBuffer& ringBuffer);
    void updateZoneSetpoints(RingBuffer& ringBuffer);
    void updateTime(RingBuffer& ringBuffer);
    void updateZone1Info(RingBuffer& ringBuffer);
    void updateDamperPositions(RingBuffer& ringBuffer);

    void addJson(JsonObject& jsonObject, String key, byte value);
    void addJson(JsonObject& jsonObject, String key, float value);

    void setControllerState(byte value);
    void setLatTemperature(byte value);
    void setOutsideTemperature(float value);
    void setOutsideTemperature2(float value);
	void setDayTime(byte day, byte hour, byte minute, byte second);

    enum ControllerStateFlags {
      DeHumidify 				  = 1 << 7,
      Humidify 				    = 1 << 6,
      Fan_G					      = 1 << 5,
      ReversingValve			= 1 << 4,
      AuxHeat2_W2				  = 1 << 3,
      AuxHeat1_W1				  = 1 << 2,
      CompressorStage2_Y2	= 1 << 1,
      CompressorStage1_Y1	= 1 << 0
    };
};

#endif
