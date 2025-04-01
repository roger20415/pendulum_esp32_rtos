#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <servoDriver.hpp>
#include <esp32_led.hpp>
#include <params.hpp>
#include "encoder.hpp"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_allocator_t allocator;
rcl_init_options_t init_options;
rclc_support_t support;
rcl_node_t node;

// subscriber
rcl_subscription_t action_sub;
trajectory_msgs__msg__JointTrajectoryPoint action_msg_sub;
rclc_executor_t action_executor_sub;

// publisher
rcl_publisher_t obs_pub;
std_msgs__msg__Float32MultiArray obs_msg_pub;
rclc_executor_t obs_executor_pub;
rcl_timer_t obs_timer;

// Global variables shared between the microROS task and the servo control task
double joint_positions[NUM_ALL_SERVOS] = {0.0};
float obs_data[NUM_OBS] = {1.0};

void encoderTaskFunction(void *parameter);

#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            return false;              \
        }                              \
    }
#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
        }                              \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do {                                   \
        static volatile int64_t init = -1; \
        if (init == -1) {                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS) {    \
            X;                             \   
            init = uxr_millis();           \
        }                                  \
    } while (0)

states state;

void action_subscription_callback(const void *msgin) {
    const trajectory_msgs__msg__JointTrajectoryPoint *msg = (const trajectory_msgs__msg__JointTrajectoryPoint *)msgin;
    for (size_t i = 0; i < msg->positions.size; ++i) {
        joint_positions[i + SERVO_OFFSET] = degrees(msg->positions.data[i]);
    }
}

void obs_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        for (size_t i = 0; i < obs_msg_pub.data.capacity; i++) {
            obs_msg_pub.data.data[i] = obs_data[i];
        }
        RCSOFTCHECK(rcl_publish(&obs_pub, &obs_msg_pub, NULL));
    }
}

/**
 * Functions create_entities and destroy_entities can take several seconds.
 * In order to reduce this rebuild the library with
 * - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
 * - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3
 */
bool create_entities() {
    // Initialize micro-ROS allocator
    allocator = rcl_get_default_allocator();

    // Initialize and modify options (Set DOMAIN ID)
    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
    
    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, NAMESPACE, &support));

    // create action subscriber
    RCCHECK(rclc_subscription_init_default(
        &action_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
        "/inverted_pendulum_target_joint_angles"));

    action_msg_sub.positions.capacity = NUM_ALL_SERVOS;
    action_msg_sub.positions.size = NUM_ALL_SERVOS;
    action_msg_sub.positions.data = (double *)calloc(action_msg_sub.positions.capacity, sizeof(double));

    // create action subscriber executor
    RCCHECK(rclc_executor_init(&action_executor_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&action_executor_sub, &action_sub, &action_msg_sub, &action_subscription_callback, ON_NEW_DATA));

    // create obs publisher
    RCCHECK(rclc_publisher_init_default(
        &obs_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/state_topic"));

    // create obs timer, this timer sets the period for publishing obs data.
    const unsigned int timer_timeout = 50;
    RCCHECK(rclc_timer_init_default(
        &obs_timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),  // Timer period in nanoseconds
        obs_timer_callback));

    obs_msg_pub.data.capacity = NUM_OBS;
    obs_msg_pub.data.size = NUM_OBS;
    obs_msg_pub.data.data = (float *)calloc(obs_msg_pub.data.capacity, sizeof(float));

    // create obs publisher executor
    RCCHECK(rclc_executor_init(&obs_executor_pub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&obs_executor_pub, &obs_timer));

    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // action subscriber
    RCSOFTCHECK(rcl_subscription_fini(&action_sub, &node));
    RCSOFTCHECK(rclc_executor_fini(&action_executor_sub));

    free(action_msg_sub.positions.data);
    action_msg_sub.positions.data = NULL;

    // obs publisher
    RCSOFTCHECK(rcl_publisher_fini(&obs_pub, &node));
    RCSOFTCHECK(rcl_timer_fini(&obs_timer));
    RCSOFTCHECK(rclc_executor_fini(&obs_executor_pub));

    free(obs_msg_pub.data.data);
    obs_msg_pub.data.data = NULL;

    // common
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
    RCSOFTCHECK(rcl_init_options_fini(&init_options));
}

void microROSTaskFunction(void *parameter) {
    // Use inifiite loop to keep the task running like the void loop() function in Arduino framework.
    while (true) {
        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
                break;
            case AGENT_AVAILABLE:
                state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT) {
                    destroy_entities();
                };
                break;
            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&action_executor_sub, RCL_MS_TO_NS(100));
                    rclc_executor_spin_some(&obs_executor_pub, RCL_MS_TO_NS(100));
                }
                break;
            case AGENT_DISCONNECTED:
                destroy_entities();
                state = WAITING_AGENT;
                break;
            default:
                break;
        }
    }
}

void ServoControlTaskFunction(void *parameter) {
    ServoManager servoManager(uint8_t(NUM_ALL_SERVOS), servoMinAngles, servoMaxAngles, servoInitAngles);

    while (true) {
        for (size_t i = 0; i < NUM_ALL_SERVOS; ++i) {
            servoManager.setServoTargetAngle(i, uint8_t(joint_positions[i]));
        }
        servoManager.moveServo();

        // Wait for some time before the next iteration
        vTaskDelay(UPDATE_SERVO_DELAY / portTICK_PERIOD_MS);
    }
}

void encoderTaskFunction(void *parameter) {
    EncoderManager encoderManager;
    encoderManager.begin();
    while (true) {
        obs_data[0] = encoderManager.getAngleDegrees();
        vTaskDelay(UPDATE_ENCODER_DELAY / portTICK_PERIOD_MS);
    }
}

void setup() {
    // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
    delay(100);

    state = WAITING_AGENT;

    // Initialize joint_positions with the initial angles
    for (size_t i = 0; i < NUM_ALL_SERVOS; ++i) {
        joint_positions[i] = double(servoInitAngles[i]);
    }

    xTaskCreate(
        microROSTaskFunction,      // Task function
        "Micro ROS Task",          // Task name
        8192,                      // Stack size (in bytes)
        NULL,                      // Task parameters
        configMAX_PRIORITIES - 1,  // Task priority
        NULL                       // Task handle
    );
    delay(100);

    xTaskCreate(
        ServoControlTaskFunction,  // Task function
        "Servo Control Task",      // Task name
        4096,                    // Stack size (in bytes)
        NULL,                    // Task parameters
        2,                       // Task priority
        NULL                     // Task handle
    );
    delay(100);

    xTaskCreate(
        led_task,    // Task function
        "LED Task",  // Task name
        1024,        // Stack size (in bytes)
        &state,      // Task parameters
        0,           // Task priority
        NULL         // Task handle
    );
    delay(100);

    xTaskCreate(
        encoderTaskFunction,
        "Encoder Task",
        4096,
        NULL,
        1,
        NULL
    );
    delay(100);
}

void loop() {
    // We use xTaskCreate and thus we don't need to put anything here.
}