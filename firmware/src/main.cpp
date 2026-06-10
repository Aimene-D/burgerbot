#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "imu.h"
#include "odometry.h"
#include "pid.h"

#ifndef USE_MICROROS
#define USE_MICROROS 0
#endif

#if USE_MICROROS
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <tf2_msgs/msg/tf_message.h>
#endif

// ─── PID instances ────────────────────────────────────────
PidController g_m1_pid(M1_KP, M1_KI, M1_KD);
PidController g_m2_pid(M2_KP, M2_KI, M2_KD);

// ─── Agent state ──────────────────────────────────────────
enum class AgentState { Waiting, Connected };
static AgentState g_agent_state = AgentState::Waiting;

#if USE_MICROROS
// ─── micro-ROS entities ───────────────────────────────────
static rcl_allocator_t g_allocator;
static rclc_support_t  g_support;
static rcl_node_t      g_node;
static rcl_subscription_t g_cmd_sub, g_rpm_cmd_sub, g_pid_config_sub, g_calibrate_mag_sub;
static rcl_publisher_t    g_wheel_pub, g_rpm_feedback_pub, g_imu_pub, g_mag_pub;
static rcl_publisher_t    g_odom_pub, g_imu_status_pub, g_mag_status_pub, g_tf_pub;
static rclc_executor_t g_executor;

static geometry_msgs__msg__Twist             g_cmd_msg;
static std_msgs__msg__Float32MultiArray      g_rpm_cmd_msg;
static float g_rpm_cmd_data[2]  = {0.0f, 0.0f};
static std_msgs__msg__Float32MultiArray      g_pid_config_msg;
static float g_pid_config_data[3] = {0.0f, 0.0f, 0.0f};
static std_msgs__msg__Bool                   g_calibrate_mag_msg;
static std_msgs__msg__Float32MultiArray      g_wheel_msg;
static float g_wheel_data[4]    = {0.0f, 0.0f, 0.0f, 0.0f};
static std_msgs__msg__Float32MultiArray      g_rpm_feedback_msg;
static float g_rpm_feedback_data[2] = {0.0f, 0.0f};
static sensor_msgs__msg__Imu                 g_imu_msg;
static sensor_msgs__msg__MagneticField       g_mag_msg;
static nav_msgs__msg__Odometry               g_odom_msg;
static std_msgs__msg__Bool                   g_imu_status_msg;
static std_msgs__msg__Bool                   g_mag_status_msg;
static tf2_msgs__msg__TFMessage              g_tf_msg;
static geometry_msgs__msg__TransformStamped  g_tf_transforms[1];
static bool     g_ros_time_synced       = false;
static uint32_t g_last_ros_time_sync_ms = 0;
static bool     g_ros_entities_created  = false;

inline void setRosString(rosidl_runtime_c__String& s, const char* v) {
  s.data = const_cast<char*>(v); s.size = strlen(v); s.capacity = s.size + 1;
}

bool syncRosClock(uint32_t timeout_ms) {
  if (rmw_uros_sync_session(timeout_ms) != RMW_RET_OK) { g_ros_time_synced = false; return false; }
  g_ros_time_synced = true;
  g_last_ros_time_sync_ms = millis();
  return true;
}

inline void setRosTimeFromMillis(builtin_interfaces__msg__Time& stamp, uint32_t now_ms) {
  uint64_t time_ms = now_ms;
  if (g_ros_time_synced) {
    const int64_t epoch_ms = rmw_uros_epoch_millis();
    if (epoch_ms > 0) {
      // Translate the local-millis argument into ROS-epoch millis by adding
      // the offset between the current ROS epoch and the current local millis.
      const uint32_t now_local_ms = millis();
      const int64_t  offset_ms    = epoch_ms - (int64_t)now_local_ms;
      time_ms = (uint64_t)((int64_t)now_ms + offset_ms);
    }
  }
  stamp.sec    = static_cast<int32_t>(time_ms / 1000ULL);
  stamp.nanosec = static_cast<uint32_t>(time_ms % 1000ULL) * 1000000UL;
}

inline void setYawQuaternion(geometry_msgs__msg__Quaternion& q, float yaw_rad) {
  const float half = 0.5f * yaw_rad;
  q.x = 0.0; q.y = 0.0; q.z = sinf(half); q.w = cosf(half);
}

void initStandardRosMessages() {
  memset(&g_imu_msg, 0, sizeof(g_imu_msg));
  setRosString(g_imu_msg.header.frame_id, "imu_link");
  // Increased from 0.001 to let EKF blend odom yaw when robot is stopped
  // Matched to wheel odometry yaw covariance (0.2) so EKF blends both equally
  g_imu_msg.orientation_covariance[0] = 0.2;
  g_imu_msg.orientation_covariance[4] = 0.2;
  g_imu_msg.orientation_covariance[8] = 0.2;
  // Raised from 0.02 to match wheel odometry twist angular z covariance (0.2),
  // preventing gyro residual bias from causing yaw drift at standstill.
  g_imu_msg.angular_velocity_covariance[0]  = 0.2;
  g_imu_msg.angular_velocity_covariance[4]  = 0.2;
  g_imu_msg.angular_velocity_covariance[8]  = 0.2;
  g_imu_msg.linear_acceleration_covariance[0] = 0.2;
  g_imu_msg.linear_acceleration_covariance[4] = 0.2;
  g_imu_msg.linear_acceleration_covariance[8] = 0.2;

  memset(&g_mag_msg, 0, sizeof(g_mag_msg));
  setRosString(g_mag_msg.header.frame_id, "imu_link");
  g_mag_msg.magnetic_field_covariance[0] = 2.5e-12;
  g_mag_msg.magnetic_field_covariance[4] = 2.5e-12;
  g_mag_msg.magnetic_field_covariance[8] = 2.5e-12;

  memset(&g_odom_msg, 0, sizeof(g_odom_msg));
  setRosString(g_odom_msg.header.frame_id, "odom");
  setRosString(g_odom_msg.child_frame_id,  "base_link");
  g_odom_msg.pose.covariance[0]  = 0.02;
  g_odom_msg.pose.covariance[7]  = 0.02;
  g_odom_msg.pose.covariance[14] = 99999.0;
  g_odom_msg.pose.covariance[21] = 99999.0;
  g_odom_msg.pose.covariance[28] = 99999.0;
  g_odom_msg.pose.covariance[35] = 0.1;
  g_odom_msg.twist.covariance[0]  = 0.05;
  g_odom_msg.twist.covariance[7]  = 0.05;
  g_odom_msg.twist.covariance[14] = 99999.0;
  g_odom_msg.twist.covariance[21] = 99999.0;
  g_odom_msg.twist.covariance[28] = 99999.0;
  g_odom_msg.twist.covariance[35] = 0.2;

  memset(&g_tf_msg, 0, sizeof(g_tf_msg));
  memset(g_tf_transforms, 0, sizeof(g_tf_transforms));
  g_tf_msg.transforms.data     = g_tf_transforms;
  g_tf_msg.transforms.size     = 1;
  g_tf_msg.transforms.capacity = 1;
  setRosString(g_tf_transforms[0].header.frame_id, "odom");
  setRosString(g_tf_transforms[0].child_frame_id,  "base_link");

}

void cmdVelCallback(const void* msg_in) {
  if (g_mag_calibration.active) return;
  const auto* msg = static_cast<const geometry_msgs__msg__Twist*>(msg_in);
  setTargetsFromCmdVel(msg->linear.x, msg->angular.z);
  g_last_cmd_ms = millis();
}

void rpmCmdCallback(const void* msg_in) {
  if (g_mag_calibration.active) return;
  const auto* msg = static_cast<const std_msgs__msg__Float32MultiArray*>(msg_in);
  if (msg->data.size >= 2) {
    g_m1_target_rpm = msg->data.data[0];
    g_m2_target_rpm = msg->data.data[1];
    g_last_cmd_ms   = millis();
  }
}

void pidConfigCallback(const void* msg_in) {
  const auto* msg = static_cast<const std_msgs__msg__Float32MultiArray*>(msg_in);
  if (msg->data.size < 3) return;
  const float kp = msg->data.data[0];
  const float ki = msg->data.data[1];
  const float kd = msg->data.data[2];
  g_m1_pid.SetGains(kp, ki, kd);
  g_m2_pid.SetGains(kp, ki, kd);
  g_m1_pid.Reset();
  g_m2_pid.Reset();
  savePidGainsToNvs(kp, ki, kd);
}

void calibrateMagCallback(const void* msg_in) {
  const auto* msg = static_cast<const std_msgs__msg__Bool*>(msg_in);
  if (msg->data) { startMagCalibration(millis()); return; }
  finishMagCalibration(true);
}

bool createRosEntities() {
  g_allocator = rcl_get_default_allocator();
  if (rclc_support_init(&g_support, 0, nullptr, &g_allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&g_node, "esp32_motor_controller", "", &g_support) != RCL_RET_OK) return false;
  (void)syncRosClock(1000);

  if (rclc_subscription_init_default(&g_cmd_sub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&g_rpm_cmd_sub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/motor_rpm_cmd") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&g_pid_config_sub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/pid_config") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&g_calibrate_mag_sub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/calibrate_mag") != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(&g_wheel_pub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/wheel_speeds") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&g_rpm_feedback_pub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/motor_rpm_feedback") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&g_imu_pub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/raw") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&g_mag_pub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "/imu/mag") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&g_odom_pub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/wheel_odom") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&g_imu_status_pub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/imu/status/valid") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&g_mag_status_pub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/mag/status/valid") != RCL_RET_OK) return false;
  if (PUBLISH_RAW_ODOM_TF) {
    if (rclc_publisher_init_default(&g_tf_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "/tf") != RCL_RET_OK) return false;
  }

  g_wheel_msg.data.data     = g_wheel_data;
  g_wheel_msg.data.size     = 4;
  g_wheel_msg.data.capacity = 4;
  g_rpm_cmd_msg.data.data     = g_rpm_cmd_data;
  g_rpm_cmd_msg.data.size     = 0;
  g_rpm_cmd_msg.data.capacity = 2;
  g_pid_config_msg.data.data     = g_pid_config_data;
  g_pid_config_msg.data.size     = 0;
  g_pid_config_msg.data.capacity = 3;
  g_rpm_feedback_msg.data.data     = g_rpm_feedback_data;
  g_rpm_feedback_msg.data.size     = 2;
  g_rpm_feedback_msg.data.capacity = 2;

  initStandardRosMessages();

  if (rclc_executor_init(&g_executor, &g_support.context, 4, &g_allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&g_executor, &g_cmd_sub,          &g_cmd_msg,          &cmdVelCallback,      ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&g_executor, &g_rpm_cmd_sub,      &g_rpm_cmd_msg,      &rpmCmdCallback,      ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&g_executor, &g_pid_config_sub,   &g_pid_config_msg,   &pidConfigCallback,   ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&g_executor, &g_calibrate_mag_sub,&g_calibrate_mag_msg,&calibrateMagCallback,ON_NEW_DATA) != RCL_RET_OK) return false;

  g_ros_entities_created = true;
  return true;
}

void destroyRosEntities() {
  if (!g_ros_entities_created) return;
  (void)rcl_subscription_fini(&g_calibrate_mag_sub, &g_node);
  (void)rcl_subscription_fini(&g_pid_config_sub,    &g_node);
  (void)rcl_subscription_fini(&g_rpm_cmd_sub,       &g_node);
  (void)rcl_subscription_fini(&g_cmd_sub,           &g_node);
  if (PUBLISH_RAW_ODOM_TF) (void)rcl_publisher_fini(&g_tf_pub, &g_node);
  (void)rcl_publisher_fini(&g_mag_status_pub,  &g_node);
  (void)rcl_publisher_fini(&g_imu_status_pub,  &g_node);
  (void)rcl_publisher_fini(&g_odom_pub,        &g_node);
  (void)rcl_publisher_fini(&g_mag_pub,         &g_node);
  (void)rcl_publisher_fini(&g_imu_pub,         &g_node);
  (void)rcl_publisher_fini(&g_rpm_feedback_pub,&g_node);
  (void)rcl_publisher_fini(&g_wheel_pub,       &g_node);
  (void)rcl_node_fini(&g_node);
  (void)rclc_executor_fini(&g_executor);
  (void)rclc_support_fini(&g_support);
  g_ros_time_synced      = false;
  g_ros_entities_created = false;
}


void publishWheelSpeeds() {
  if (!g_ros_entities_created) return;
  const uint32_t now_ms = millis();

  g_wheel_data[0] = g_m1_target_rpm;
  g_wheel_data[1] = g_m2_target_rpm;
  g_wheel_data[2] = g_m1_measured_rpm;
  g_wheel_data[3] = g_m2_measured_rpm;
  (void)rcl_publish(&g_wheel_pub, &g_wheel_msg, nullptr);

  g_rpm_feedback_data[0] = g_m1_measured_rpm;
  g_rpm_feedback_data[1] = g_m2_measured_rpm;
  (void)rcl_publish(&g_rpm_feedback_pub, &g_rpm_feedback_msg, nullptr);

  setRosTimeFromMillis(g_imu_msg.header.stamp, now_ms);
  g_imu_msg.linear_acceleration.x = g_imu_data.accel_x;
  g_imu_msg.linear_acceleration.y = g_imu_data.accel_y;
  g_imu_msg.linear_acceleration.z = g_imu_data.accel_z;
  g_imu_msg.angular_velocity.x    = g_imu_data.gyro_x;
  g_imu_msg.angular_velocity.y    = g_imu_data.gyro_y;
  g_imu_msg.angular_velocity.z    = g_imu_data.gyro_z;
  const float roll  = atan2f(g_imu_data.accel_y, g_imu_data.accel_z);
  const float pitch = atan2f(-g_imu_data.accel_x,
                   sqrtf(g_imu_data.accel_y * g_imu_data.accel_y +
                         g_imu_data.accel_z * g_imu_data.accel_z));
  const float yaw   = g_orientation.yaw_rad;
  const float cr = cosf(roll  * 0.5f);
  const float sr = sinf(roll  * 0.5f);
  const float cp = cosf(pitch * 0.5f);
  const float sp = sinf(pitch * 0.5f);
  const float cy = cosf(yaw   * 0.5f);
  const float sy = sinf(yaw   * 0.5f);
  g_imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
  g_imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
  g_imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
  g_imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
  (void)rcl_publish(&g_imu_pub, &g_imu_msg, nullptr);

  setRosTimeFromMillis(g_mag_msg.header.stamp, now_ms);
  g_mag_msg.magnetic_field.x = g_compass_data.mag_x * UT_TO_TESLA;
  g_mag_msg.magnetic_field.y = g_compass_data.mag_y * UT_TO_TESLA;
  g_mag_msg.magnetic_field.z = g_compass_data.mag_z * UT_TO_TESLA;
  (void)rcl_publish(&g_mag_pub, &g_mag_msg, nullptr);

  setRosTimeFromMillis(g_odom_msg.header.stamp, now_ms);
  g_odom_msg.pose.pose.position.x = g_odom.x_m;
  g_odom_msg.pose.pose.position.y = g_odom.y_m;
  g_odom_msg.pose.pose.position.z = 0.0;
  setYawQuaternion(g_odom_msg.pose.pose.orientation, g_odom.theta_rad);
  g_odom_msg.twist.twist.linear.x  = g_odom.linear_mps;
  g_odom_msg.twist.twist.linear.y  = 0.0;
  g_odom_msg.twist.twist.angular.z = g_odom.angular_radps;
  (void)rcl_publish(&g_odom_pub, &g_odom_msg, nullptr);

  g_imu_status_msg.data = g_imu_data_valid;
  g_mag_status_msg.data = g_compass_data_valid;
  (void)rcl_publish(&g_imu_status_pub, &g_imu_status_msg, nullptr);
  (void)rcl_publish(&g_mag_status_pub, &g_mag_status_msg, nullptr);

  if (PUBLISH_RAW_ODOM_TF) {
    setRosTimeFromMillis(g_tf_transforms[0].header.stamp, now_ms);
    g_tf_transforms[0].transform.translation.x = g_odom.x_m;
    g_tf_transforms[0].transform.translation.y = g_odom.y_m;
    g_tf_transforms[0].transform.translation.z = 0.0;
    setYawQuaternion(g_tf_transforms[0].transform.rotation, g_odom.theta_rad);
    (void)rcl_publish(&g_tf_pub, &g_tf_msg, nullptr);
  }
}
#endif

// ─── setup & loop ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1500);
  initMotorPinsAndPwm();
  initEncoders();
  initI2cAndSensors();
  loadPidGainsFromNvsAndApply();
  loadMagOffsetsFromNvs();
  g_last_cmd_ms = millis();
#if USE_MICROROS
  set_microros_transports();
#endif
}

void loop() {
  static uint32_t last_control_ms   = millis();
  static uint32_t last_telemetry_ms = millis();
  static uint32_t last_agent_check_ms = 0;
  const uint32_t now = millis();

  readImuData(now);
  readCompassData(now);
  updateYawEstimate(now);
  updateMagCalibrationMode(now);
  logSensorData(now);

  if ((now - last_agent_check_ms) >= 500) {
    last_agent_check_ms = now;
#if USE_MICROROS
    const bool agent_ok = (rmw_uros_ping_agent(50, 1) == RMW_RET_OK);
    if (g_agent_state == AgentState::Waiting && agent_ok) {
      if (createRosEntities()) g_agent_state = AgentState::Connected;
    } else if (g_agent_state == AgentState::Connected && !agent_ok) {
      destroyRosEntities();
      g_agent_state   = AgentState::Waiting;
      g_m1_target_rpm = 0.0f;
      g_m2_target_rpm = 0.0f;
      g_m1_pid.Reset();
      g_m2_pid.Reset();
      stopMotors();
    }
#endif
  }

  if ((now - last_control_ms) >= CONTROL_PERIOD_MS) {
    const uint32_t dt_ms = now - last_control_ms;
    last_control_ms = now;
    runControlStep(dt_ms);
  }

  if (g_agent_state == AgentState::Connected) {
#if USE_MICROROS
    if ((now - g_last_ros_time_sync_ms) >= ROS_TIME_SYNC_PERIOD_MS)
      (void)syncRosClock(100);
    rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(10));

    if ((now - last_telemetry_ms) >= TELEMETRY_PERIOD_MS) {
      last_telemetry_ms = now;
      publishWheelSpeeds();
    }
#endif
  }
}
