#pragma once
#include <stdint.h>

/*
 * https://circuits4you.com
 * ESP32 LED Blink Example
 * Board ESP23 DEVKIT V1
 *
 * ON Board LED GPIO 2
 * Ref: https://circuits4you.com/2018/02/02/esp32-led-blink-example/
 */
#define ROS_DOMAIN_ID 1
#define NODE_NAME "micro_ros_platformio_node"
#define NAMESPACE ""
#define ESP32_LED 2
#define ENCODER_SDA 21
#define ENCODER_SCL 22
#define UPDATE_SERVO_DELAY 1.0
#define UPDATE_ENCODER_DELAY 1.0
const float SERVO_MOVEMENT_STEP = 1.0;
const size_t NUM_ALL_SERVOS = 1;
const size_t NUM_OBS = 1;
const uint8_t servoMinAngles[] = {0};
const uint8_t servoMaxAngles[] = {180};
const uint8_t servoInitAngles[] = {0};
const size_t SERVO_OFFSET = 0;

static_assert(sizeof(servoMinAngles) == NUM_ALL_SERVOS * sizeof(uint8_t));
static_assert(sizeof(servoMaxAngles) == NUM_ALL_SERVOS * sizeof(uint8_t));
static_assert(sizeof(servoInitAngles) == NUM_ALL_SERVOS * sizeof(uint8_t));

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};
