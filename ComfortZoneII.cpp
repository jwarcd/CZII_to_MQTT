// Copyright - Jared Gaillard - 2016
// MIT License
//
// Arduino CZII Project

#include "ComfortZoneII.h"

ComfortZoneII::ComfortZoneII(byte numberZones)
{
  NUMBER_ZONES = numberZones;

  for (byte i = 0; i < NUMBER_ZONES; i++) {
    zones[i] = new Zone(i + 1);
  }
}

bool ComfortZoneII::isStatusModified() {
  return statusModified;
}

void ComfortZoneII::clearStatusModified() {
  statusModified = false;
}

bool ComfortZoneII::isZoneModified() {
  for (byte i = 0; i < NUMBER_ZONES; i++) {
    if (zones[i]->isModified()) {
      return true;
    }
  }
  return false;
}

void ComfortZoneII::clearZoneModified() {
  for (byte i = 0; i < NUMBER_ZONES; i++) {
    zones[i]->setModified(false);
  }
}

Zone* ComfortZoneII::getZone(byte zoneIndex) {
  return zones[zoneIndex];
}

void ComfortZoneII::setControllerState(byte value) {
  if (controllerState != value)
    statusModified = true;

  controllerState = value;
}

void ComfortZoneII::setLatTemperature(byte value) {
  if (!isValidTemperature(value))
	  return;
  
  if (lat_Temp_f != value)
    statusModified = true;

  lat_Temp_f = value;
}

void ComfortZoneII::setOutsideTemperature(float value) {
if (!isValidTemperature(value))
	  return;
  
  if (outside_Temp_f != value)
    statusModified = true;

  outside_Temp_f = value;
}

void ComfortZoneII::setOutsideTemperature2(float value) {
  if (!isValidTemperature(value))
	  return;
	
  if (outside_Temp2_f != value)
    statusModified = true;

  outside_Temp2_f = value;
}

bool ComfortZoneII::isValidTemperature(float value) {
  return value < 200.0 && value > -50.0;
}

void ComfortZoneII::setDayTime(byte day, byte hour, byte minute, byte second){ 
  if( day != time.Wday || hour != time.Hour || minute != time.Minute || second != time.Second){
	statusModified = true;
  }
  
  time.Wday = day;
  time.Hour = hour;
  time.Minute = minute;
  time.Second = second;
}

//
//	Update cached data
//
bool ComfortZoneII::update(RingBuffer& ringBuffer) {

  short bufferLength = ringBuffer.length();

  // see if the buffer has at least the minimum size for a frame
  if (bufferLength < MIN_MESSAGE_SIZE ) {
    //debug_println("ringBuffer: bufferLength < MIN_MESSAGE_SIZE");
    return false;
  }

  byte table, row = 0;
  byte dataLength = ringBuffer.peek(DATA_LENGTH_POS);

  if (dataLength >= 3) {
    table = ringBuffer.peek(DATA_START_POS + 1);
    row = ringBuffer.peek(DATA_START_POS + 2);
  }

  byte function = ringBuffer.peek(FUNCTION_POS);

  if (function == RESPONSE_FUNCTION) {
    switch (table) {
      case 1:
        if (row == 6 && dataLength == 16) {
          updateZone1Info(ringBuffer);
        }
        else if (row == 16 && dataLength == 19) {
          updateZoneSetpoints(ringBuffer);
        }
        else if (row == 18 && dataLength == 7) {
          updateTime(ringBuffer);
        }
        break;
      case 2:
        if (row == 3 && dataLength == 13) {
          updateZoneInfo(ringBuffer);
        }
        break;
      case 9:
        if (row == 3 && dataLength == 10) {
          updateOutsideTemp(ringBuffer);
        }
        else if (row == 5 && dataLength == 4) {
          //updateControllerState(ringBuffer);
        }
        break;
      default:
        break;
    }
    info_println();
  }
  else if (function == WRITE_FUNCTION) {
    switch (table) {
      case 2:
        if (row == 1 && dataLength == 13) {
          updateOutsideHumidityTemp(ringBuffer);
        }
        break;
      case 9:
        if (row == 4 && dataLength == 11) {
          updateDamperPositions(ringBuffer);
        }
        else if (row == 5 && dataLength == 4) {
          updateControllerState(ringBuffer);
        }
      default:
        break;
    }
  }
}

//
//   FRAME: 9.0  1.0  11  0.0.12  0.9.4.15.15.0.0.0.0.0.0.                 136.181
//
void ComfortZoneII::updateDamperPositions(RingBuffer& ringBuffer) {
  for (byte i = 0; i < NUMBER_ZONES; i++) {
    zones[i]->setDamperPosition(ringBuffer.peek(DATA_START_POS + 3 + i));
  }
}

//
//  FRAME: 8.0  1.0  16  0.0.6   0.1.6.0.0.4.64.60.0.0.0.0.0.0.17.50.      74.114
//
void ComfortZoneII::updateZone1Info(RingBuffer& ringBuffer) {
  zones[0]->setTemperature(getTemperatureF(ringBuffer.peek(DATA_START_POS + 5), ringBuffer.peek(DATA_START_POS + 6)));
  zones[0]->setHumidity(ringBuffer.peek(DATA_START_POS + 7));
}

//
//  FRAME: 99.0  1.0  19  0.0.6   0.1.16. 78.77.76.76.76.76.76.76. 68.67.68.68.68.68.68.68.        177.133
//                                        [     cooling          ] [        heating       ]
//
void ComfortZoneII::updateZoneSetpoints(RingBuffer& ringBuffer) {
  for (byte i = 0; i < NUMBER_ZONES; i++) {
    zones[i]->setCoolSetpoint(ringBuffer.peek(DATA_START_POS + 3 + i));
    zones[i]->setHeatSetpoint(ringBuffer.peek(DATA_START_POS + 11 + i));
  }
}

//
//  FRAME: 8.0  1.0  7   0.0.6   0.1.18.3.22.33.47
//
//
void ComfortZoneII::updateTime(RingBuffer& ringBuffer) {
  byte day = ringBuffer.peek(DATA_START_POS + 3);
  byte hour = ringBuffer.peek(DATA_START_POS + 4);
  byte minute = ringBuffer.peek(DATA_START_POS + 5);
  byte second = ringBuffer.peek(DATA_START_POS + 6);

  day++;
  setDayTime(day, hour, minute, second);
}

//
//  FRAME: 1.0  9.0  10  0.0.6   0.9.3.195.3.136.72.255.0.0.
//
void ComfortZoneII::updateOutsideTemp(RingBuffer& ringBuffer) {
  setOutsideTemperature(getTemperatureF(ringBuffer.peek(DATA_START_POS + 4), ringBuffer.peek(DATA_START_POS + 5)));
  setLatTemperature(ringBuffer.peek(DATA_START_POS + 6));
}

//
//  FRAME: 9.0  1.0  4   0.0.12  0.9.5.128.
//
void ComfortZoneII::updateControllerState(RingBuffer& ringBuffer) {
  setControllerState(ringBuffer.peek(DATA_START_POS + 3));
}

//
//  FRAME: 2.0  1.0  13  0.0.12  0.2.1.0.57.3.145.3.0.0.0.2.0.
//
void ComfortZoneII::updateOutsideHumidityTemp(RingBuffer& ringBuffer) {
  setOutsideTemperature2(getTemperatureF(ringBuffer.peek(DATA_START_POS + 5), ringBuffer.peek(DATA_START_POS + 6)));
  zones[0]->setHumidity(ringBuffer.peek(DATA_START_POS + 4));
}

//
//  FRAME: 1.0  2.0  13  0.0.6   0.2.3.1.0.0.0.4.160.74.67.77.0.
//  FRAME: 1.0  2.0  13  0.0.6   0.2.3.0.0.0.0.4.122.71.66.78.0.
//
void ComfortZoneII::updateZoneInfo(RingBuffer& ringBuffer) {
  byte zoneIndex = ringBuffer.peek(2) - 1;
  if (zoneIndex == 0 || zoneIndex >= NUMBER_ZONES)
    return;

  zones[zoneIndex]->setTemperature(getTemperatureF(ringBuffer.peek(DATA_START_POS + 7), ringBuffer.peek(DATA_START_POS + 8)));
  zones[zoneIndex]->setHeatSetpoint(ringBuffer.peek(DATA_START_POS + 10));
  zones[zoneIndex]->setCoolSetpoint(ringBuffer.peek(DATA_START_POS + 11));
}

//
//	Convert the two's compliment temperature into a float (deg F.)
//
float ComfortZoneII::getTemperatureF(byte highByte, byte lowByte) {
  return word(highByte, lowByte) / 16.0;
}

//
//   To Zone JSON String
//
String ComfortZoneII::toZoneJson() {
  StaticJsonBuffer <400> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  for (byte i = 0; i < NUMBER_ZONES; i++) {
    JsonObject& zoneData = root.createNestedObject("z" + String(i + 1));
    addJson(zoneData, "cool", zones[i]->getCoolSetpoint());
    addJson(zoneData, "heat", zones[i]->getHeatSetpoint());
    addJson(zoneData, "temp", zones[i]->getTemperature());
    addJson(zoneData, "hum", zones[i]->getHumidity());
    addJson(zoneData, "damp", zones[i]->getDamperPosition());
  }

  String output;
  root.printTo(output);
  return output;
}

//
//   To Status JSON String
//
String ComfortZoneII::toStatusJson() {
  StaticJsonBuffer <300> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  
  char timeString[10];
  sprintf(timeString, "%02d:%02d:%02d", time.Hour, time.Minute, time.Second);
  root["time"] = timeString;
  root["day"] = dayStr(time.Wday);
  
  addJson(root, "lat", lat_Temp_f);
  addJson(root, "out", outside_Temp_f);
  addJson(root, "out2", outside_Temp2_f);

  if (controllerState != (byte) - 1) {
    JsonObject& state = root.createNestedObject("state");
    state["dehum"] = (int)(bool)(controllerState & DeHumidify);
    state["hum"] = (int)(bool)(controllerState & Humidify);
    state["fan"] = (int)(bool)(controllerState & Fan_G);
    state["rev"] = (int)(bool)(controllerState & ReversingValve);
    state["aux2"] = (int)(bool)(controllerState & AuxHeat2_W2);
    state["aux1"] = (int)(bool)(controllerState & AuxHeat1_W1);
    state["comp2"] = (int)(bool)(controllerState & CompressorStage2_Y2);
    state["comp1"] = (int)(bool)(controllerState & CompressorStage1_Y1);
  }

  String output;
  root.printTo(output);
  return output;
}

//
//   Add to json if value is not the default value
//
void ComfortZoneII::addJson(JsonObject& jsonObject, String key, byte value) {
  if ( value == (byte) - 1)
    return;
  jsonObject[key] = value;
}

//
//    Add to json if value is not the default value
//
void ComfortZoneII::addJson(JsonObject& jsonObject, String key, float value) {
  if ( value > FLOAT_MIN_VALUE) {
    jsonObject[key] = value;
  }
}





