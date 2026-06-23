#include "microros_task.h"
#include "config.h"
#include "led.h"
#include "status_codes.h"

#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/bool.h>

enum class UrosState : uint8_t {
    WAIT_AGENT,
    SESSION_ACTIVE,
    ERROR
};

static UrosState          s_uros_state = UrosState::WAIT_AGENT;
static rclc_executor_t    s_executor;
static rclc_support_t     s_support;
static rcl_node_t         s_node;

// Publishers
static rcl_publisher_t              s_pub_imu;
static rcl_publisher_t              s_pub_mag;
static rcl_publisher_t              s_pub_imu_status;
static rcl_publisher_t              s_pub_mag_status;
static sensor_msgs__msg__Imu          s_msg_imu = {};
static sensor_msgs__msg__MagneticField s_msg_mag = {};
static std_msgs__msg__Bool            s_msg_imu_status = {};
static std_msgs__msg__Bool            s_msg_mag_status = {};

// Subscriptions
static rcl_subscription_t          s_sub_calib;
static std_msgs__msg__Bool         s_msg_calib = {};

static uint32_t s_last_sync_ms = 0;

// Stable frame IDs
static const char* s_imu_frame = "imu_link";

// ── /calibrate_mag callback ─────────────────────────────────────
static void calibrateMagCallback(const void* msg) {
    const auto* req = static_cast<const std_msgs__msg__Bool*>(msg);
    if (req->data) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_mag_cal.trigger_requested = true;
        xSemaphoreGive(g_state_mutex);
    }
}

// ── Message helpers ─────────────────────────────────────────────
static void fillImuMsg(uint32_t now_ms) {
    int64_t epoch_ns = rmw_uros_epoch_nanos();
    s_msg_imu.header.stamp.sec      = static_cast<int32_t>(epoch_ns / 1000000000L);
    s_msg_imu.header.stamp.nanosec = static_cast<uint32_t>(epoch_ns % 1000000000L);

    s_msg_imu.header.frame_id.data = const_cast<char*>(s_imu_frame);
    s_msg_imu.header.frame_id.size = 8;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    const ImuData imu = g_imu_data;
    xSemaphoreGive(g_state_mutex);

    // Linear acceleration (m/s²)
    s_msg_imu.linear_acceleration.x = imu.accel_x;
    s_msg_imu.linear_acceleration.y = imu.accel_y;
    s_msg_imu.linear_acceleration.z = imu.accel_z;

    // Angular velocity (rad/s, bias-corrected)
    s_msg_imu.angular_velocity.x = imu.gyro_x;
    s_msg_imu.angular_velocity.y = imu.gyro_y;
    s_msg_imu.angular_velocity.z = imu.gyro_z;

    // No orientation estimate — let imu_filter_madgwick compute it
    // orientation_covariance[0] = -1.0 tells ROS "I have no orientation"
    s_msg_imu.orientation.x = 0.0f;
    s_msg_imu.orientation.y = 0.0f;
    s_msg_imu.orientation.z = 0.0f;
    s_msg_imu.orientation.w = 1.0f;
    s_msg_imu.orientation_covariance[0] = -1.0f;

    // Covariances (diagonal only, spec-recommended starting values)
    s_msg_imu.angular_velocity_covariance[0]  = 0.01f;
    s_msg_imu.angular_velocity_covariance[4]  = 0.01f;
    s_msg_imu.angular_velocity_covariance[8]  = 0.01f;
    s_msg_imu.linear_acceleration_covariance[0] = 0.1f;
    s_msg_imu.linear_acceleration_covariance[4] = 0.1f;
    s_msg_imu.linear_acceleration_covariance[8] = 0.1f;
}

static void fillMagMsg() {
    int64_t epoch_ns = rmw_uros_epoch_nanos();
    s_msg_mag.header.stamp.sec      = static_cast<int32_t>(epoch_ns / 1000000000L);
    s_msg_mag.header.stamp.nanosec = static_cast<uint32_t>(epoch_ns % 1000000000L);

    s_msg_mag.header.frame_id.data = const_cast<char*>(s_imu_frame);
    s_msg_mag.header.frame_id.size = 8;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    s_msg_mag.magnetic_field.x = g_mag_data.mag_x * UT_TO_TESLA;   // µT → T
    s_msg_mag.magnetic_field.y = g_mag_data.mag_y * UT_TO_TESLA;
    s_msg_mag.magnetic_field.z = g_mag_data.mag_z * UT_TO_TESLA;
    const bool mag_valid = g_mag_data.valid;
    xSemaphoreGive(g_state_mutex);

    if (!mag_valid) return;

    s_msg_mag.magnetic_field_covariance[0] = 1.0e-7f;
    s_msg_mag.magnetic_field_covariance[4] = 1.0e-7f;
    s_msg_mag.magnetic_field_covariance[8] = 1.0e-7f;
}

// ── Session management ──────────────────────────────────────────
static bool syncRosClock(uint32_t timeout_ms) {
    if (rmw_uros_sync_session(timeout_ms) != RMW_RET_OK) return false;
    s_last_sync_ms = millis();
    return true;
}

static bool createSession() {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    if (rclc_support_init(&s_support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&s_node, "burgerbot_sensors", "", &s_support) != RCL_RET_OK) return false;
    (void)syncRosClock(1000);

    // Publishers
    if (rclc_publisher_init_default(
            &s_pub_imu, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            TOPIC_IMU_RAW) != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &s_pub_mag, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
            TOPIC_IMU_MAG) != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &s_pub_imu_status, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            TOPIC_IMU_STATUS) != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &s_pub_mag_status, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            TOPIC_MAG_STATUS) != RCL_RET_OK) return false;

    // Subscriptions
    if (rclc_subscription_init_default(
            &s_sub_calib, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            TOPIC_CALIB_MAG) != RCL_RET_OK) return false;

    // Executor (1 subscription handler)
    s_executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&s_executor, &s_support.context, 1, &allocator) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(
            &s_executor, &s_sub_calib, &s_msg_calib,
            &calibrateMagCallback, ON_NEW_DATA) != RCL_RET_OK) return false;

    return true;
}

static void destroySession() {
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&s_support.context);
    rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    g_agent_connected = false;
    s_uros_state = UrosState::WAIT_AGENT;

    (void)rclc_executor_fini(&s_executor);
    (void)rcl_subscription_fini(&s_sub_calib, &s_node);
    (void)rcl_publisher_fini(&s_pub_imu, &s_node);
    (void)rcl_publisher_fini(&s_pub_mag, &s_node);
    (void)rcl_publisher_fini(&s_pub_imu_status, &s_node);
    (void)rcl_publisher_fini(&s_pub_mag_status, &s_node);
    (void)rcl_node_fini(&s_node);
    (void)rclc_support_fini(&s_support);

    setStatusLed(LedCode::WAITING_AGENT);
}

// ═════════════════════════════════════════════════════════════════
// microrosTask — Core 1, priority 3
// ═════════════════════════════════════════════════════════════════
void microrosTask(void* pvParams) {
    (void)pvParams;
    vTaskDelay(pdMS_TO_TICKS(2000));

    set_microros_transports();

    while (true) {
        switch (s_uros_state) {
            case UrosState::WAIT_AGENT:
            case UrosState::ERROR:
                setStatusLed(LedCode::WAITING_AGENT);

                vTaskDelay(pdMS_TO_TICKS(500));
                if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                    if (createSession()) {
                        s_uros_state = UrosState::SESSION_ACTIVE;
                        g_agent_connected = true;
                        setStatusLed(LedCode::AGENT_CONNECTED);
                    }
                }
                break;

            case UrosState::SESSION_ACTIVE: {
                uint32_t now = millis();

                // Clock sync heartbeat
                if (now - s_last_sync_ms >= ROS_TIME_SYNC_PERIOD_MS) {
                    if (!syncRosClock(100)) {
                        destroySession();
                        break;
                    }
                }

                // Spin (10ms window)
                rcl_ret_t ret = rclc_executor_spin_some(&s_executor, RCL_MS_TO_NS(10));
                if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
                    destroySession();
                    break;
                }

                // Publish IMU at every tick (100 Hz)
                fillImuMsg(now);

                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                s_msg_imu_status.data = g_imu_data.valid;
                s_msg_mag_status.data = g_mag_data.valid;
                xSemaphoreGive(g_state_mutex);

                if (rcl_publish(&s_pub_imu, &s_msg_imu, NULL) != RCL_RET_OK) {
                    destroySession();
                    break;
                }
                (void)rcl_publish(&s_pub_imu_status, &s_msg_imu_status, NULL);

                // Publish mag every 2nd tick (50 Hz)
                static uint32_t s_mag_pub_div = 0;
                s_mag_pub_div++;
                if (s_mag_pub_div >= MAG_PUB_DIVIDER) {
                    s_mag_pub_div = 0;
                    fillMagMsg();
                    if (rcl_publish(&s_pub_mag, &s_msg_mag, NULL) != RCL_RET_OK) {
                        destroySession();
                        break;
                    }
                    (void)rcl_publish(&s_pub_mag_status, &s_msg_mag_status, NULL);
                }

                vTaskDelay(pdMS_TO_TICKS(1));
                break;
            }

            default:
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }
    }
}
