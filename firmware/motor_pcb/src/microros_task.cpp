#include "microros_task.h"
#include "motors.h"
#include "led.h"
#include "status_codes.h"

#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/float32_multi_array.h>

constexpr uint32_t ROS_TIME_SYNC_PERIOD_MS = 5000;

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
static rcl_publisher_t                  s_pub_odom;
static rcl_publisher_t                  s_pub_speeds;
static nav_msgs__msg__Odometry          s_msg_odom = {};
static std_msgs__msg__Float32MultiArray s_msg_speeds = {};
static float                            s_speed_data[2];
static uint32_t                         s_last_sync_ms = 0;

// Subscriptions
static rcl_subscription_t        s_sub_cmd_vel;
static geometry_msgs__msg__Twist s_msg_cmd_vel = {};

// Stable addresses for micro-ROS message strings
static const char* s_odom_frame  = "odom";
static const char* s_child_frame = "base_footprint";

static void cmdVelCallback(const void* msg) {
    const auto* twist = static_cast<const geometry_msgs__msg__Twist*>(msg);
    float linear  = twist->linear.x;
    float angular = twist->angular.z;

    if (fabsf(linear)  < ZERO_CMD_MPS_EPS)   linear  = 0.0f;
    if (fabsf(angular) < ZERO_CMD_RADPS_EPS) angular = 0.0f;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    setTargetsFromCmdVel(linear, angular);
    g_last_cmd_ms = millis();
    xSemaphoreGive(g_state_mutex);
}

static void fillOdomMsg(const OdomState& odom) {
    int64_t now_ns = rmw_uros_epoch_nanos();
    s_msg_odom.header.stamp.sec      = static_cast<int32_t>(now_ns / 1000000000L);
    s_msg_odom.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000L);

    s_msg_odom.header.frame_id.data    = const_cast<char*>(s_odom_frame);
    s_msg_odom.header.frame_id.size    = 4;
    s_msg_odom.child_frame_id.data     = const_cast<char*>(s_child_frame);
    s_msg_odom.child_frame_id.size     = 13;

    s_msg_odom.pose.pose.position.x = odom.x_m;
    s_msg_odom.pose.pose.position.y = odom.y_m;

    float half = odom.theta_rad * 0.5f;
    s_msg_odom.pose.pose.orientation.z = sinf(half);
    s_msg_odom.pose.pose.orientation.w = cosf(half);

    s_msg_odom.twist.twist.linear.x  = odom.linear_mps;
    s_msg_odom.twist.twist.angular.z = odom.angular_radps;
}

static void fillSpeedsMsg(float rpm_left, float rpm_right) {
    s_msg_speeds.data.data       = s_speed_data;
    s_msg_speeds.data.data[0]    = rpm_left;
    s_msg_speeds.data.data[1]    = rpm_right;
    s_msg_speeds.data.size       = 2;
    s_msg_speeds.data.capacity   = 2;
}

static bool syncRosClock(uint32_t timeout_ms) {
    if (rmw_uros_sync_session(timeout_ms) != RMW_RET_OK) return false;
    s_last_sync_ms = millis();
    return true;
}

static bool createSession() {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    if (rclc_support_init(&s_support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&s_node, "burgerbot_base", "", &s_support) != RCL_RET_OK) return false;
    (void)syncRosClock(1000);

    if (rclc_publisher_init_default(
            &s_pub_odom, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            TOPIC_WHEEL_ODOM) != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &s_pub_speeds, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            TOPIC_WHEEL_SPEEDS) != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(
            &s_sub_cmd_vel, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            TOPIC_CMD_VEL) != RCL_RET_OK) return false;

    s_executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&s_executor, &s_support.context, 1, &allocator) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(
            &s_executor, &s_sub_cmd_vel, &s_msg_cmd_vel,
            &cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK) return false;

    return true;
}

static void destroySession() {
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&s_support.context);
    rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    g_agent_connected = false;
    s_uros_state = UrosState::WAIT_AGENT;

    (void)rclc_executor_fini(&s_executor);
    (void)rcl_subscription_fini(&s_sub_cmd_vel, &s_node);
    (void)rcl_publisher_fini(&s_pub_odom, &s_node);
    (void)rcl_publisher_fini(&s_pub_speeds, &s_node);
    (void)rcl_node_fini(&s_node);
    (void)rclc_support_fini(&s_support);

    setStatusLed(LedCode::WAITING_AGENT);
}

void microrosTask(void* pvParams) {
    (void)pvParams;
    vTaskDelay(pdMS_TO_TICKS(2000));

    // One-time transport init (matches official reconnection example pattern)
    set_microros_transports();

    uint32_t last_pub = 0;

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

                // syncRosClock sends TIMESTAMP heartbeat on the existing session
                // and waits for agent reply — proves bidirectional comm without
                // opening a separate transport (ping_agent closes shared Serial
                // via a temp session, breaking the main session's transport).
                if (now - s_last_sync_ms >= ROS_TIME_SYNC_PERIOD_MS) {
                    if (!syncRosClock(100)) {
                        destroySession();
                        break;
                    }
                }

                // Short spin window (10ms) so we process I/O often and don't
                // starve the lower-priority control loop.
                rcl_ret_t ret = rclc_executor_spin_some(&s_executor, RCL_MS_TO_NS(10));
                if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
                    destroySession();
                    break;
                }

                // Publish telemetry
                if (now - last_pub >= TELEMETRY_PERIOD_MS) {
                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    OdomState odom_copy = s_telemetry_odom;
                    float m1 = s_telemetry_m1_rpm;
                    float m2 = s_telemetry_m2_rpm;
                    xSemaphoreGive(g_state_mutex);

                    fillOdomMsg(odom_copy);
                    fillSpeedsMsg(m1, m2);

                    if (rcl_publish(&s_pub_odom, &s_msg_odom, NULL) != RCL_RET_OK) {
                        destroySession();
                        break;
                    }
                    if (rcl_publish(&s_pub_speeds, &s_msg_speeds, NULL) != RCL_RET_OK) {
                        destroySession();
                        break;
                    }
                    last_pub = now;
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
