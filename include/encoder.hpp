#pragma once
#include <AS5600.h>
#include <Wire.h>

class EncoderManager {
   public:
    bool begin();
    float getAngleDegrees();

   private:
    AS5600 encoder;
    bool if_initialized = false;
};