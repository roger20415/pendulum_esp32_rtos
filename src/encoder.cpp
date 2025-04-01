#include <AS5600.h>
#include "encoder.hpp"
#include "params.hpp"

bool EncoderManager::begin() {
    Wire.begin(ENCODER_SDA, ENCODER_SCL);
    if_initialized = encoder.begin();
    return if_initialized;
}

float EncoderManager::getAngleDegrees() {
    if (!if_initialized) {
        return -1.5;
    }
    uint16_t raw = encoder.readAngle();
    return raw * AS5600_RAW_TO_DEGREES;
}
