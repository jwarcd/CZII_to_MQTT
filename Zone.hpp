// Copyright - Jared Gaillard - 2016
// MIT License
//
// Arduino CZII Project

#pragma once

#include "Util.h"

//
// Storage class for Zone data
//
class Zone {

  public:
    Zone(byte  number) {
      number = number;
    }

    void setCoolSetpoint(byte value) {
      if (coolSetpoint != value)
        modified = true;
      coolSetpoint = value;
    }

    byte getCoolSetpoint() {
      return coolSetpoint;
    }

    void setHeatSetpoint(byte value) {
      if (heatSetpoint != value)
        modified = true;
      heatSetpoint = value;
    }

    byte getHeatSetpoint() {
      return heatSetpoint;
    }

    void setTemperature(float value) {
      if (temperature != value)
        modified = true;
      temperature = value;
    }

    float getTemperature() {
      return temperature;
    }

    void setHumidity(byte value) {
      if (humidity != value)
        modified = true;
      humidity = value;
    }

    byte getHumidity() {
      return humidity;
    }

    void setDamperPosition(byte value) {
      if (damperPosition != value)
        modified = true;
      damperPosition = value;
    }

    byte getDamperPosition() {
      return damperPosition;
    }

    bool isModified() {
      return modified;
    }

    bool setModified(bool value) {
      modified = value;
    }

  private:
    byte zoneNumber = -1;
    bool modified = false;

    byte coolSetpoint = -1;
    byte heatSetpoint = -1;
    float temperature = FLOAT_MIN_VALUE;
    byte humidity = -1;
    byte damperPosition = -1;
};
