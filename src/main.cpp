#include <Arduino.h>
#include <math.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_LSM6DS3.h>

#ifndef USE_MICROROS
#define USE_MICROROS 0
#endif

#ifndef M1_MOTOR_DIR_INVERT
#define M1_MOTOR_DIR_INVERT 0
#endif

#ifndef M2_MOTOR_DIR_INVERT
#define M2_MOTOR_DIR_INVERT 1
#endif

#if USE_MICROROS
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#endif

namespace {

constexpr int M1_RPWM_PIN = 17;
constexpr int M1_LPWM_PIN = 16;
constexpr int M1_REN_PIN = 15;
constexpr int M1_LEN_PIN = 7;

constexpr int M2_RPWM_PIN = 47;
constexpr int M2_LPWM_PIN = 21;
constexpr int M2_REN_PIN = 2;
constexpr int M2_LEN_PIN = 1;

constexpr int M1_ENC_B_PIN = 5;   // C2
constexpr int M1_ENC_A_PIN = 6;   // C1
constexpr int M2_ENC_B_PIN = 38;  // C2
constexpr int M2_ENC_A_PIN = 37;  // C1

constexpr int I2C_SDA_PIN = 8;
constexpr int I2C_SCL_PIN = 9;
constexpr uint32_t I2C_FREQ_HZ = 400000;
constexpr uint8_t LSM6DS3_I2C_ADDR_PRIMARY = 0x6A;
constexpr uint8_t LSM6DS3_I2C_ADDR_ALT = 0x6B;
constexpr uint8_t HMC5883L_I2C_ADDR = 0x1E;
constexpr uint8_t QMC5883_I2C_ADDR = 0x0D;
constexpr uint8_t QMC_REG_X_LSB = 0x00;
constexpr uint8_t QMC_REG_STATUS = 0x06;
constexpr uint8_t QMC_REG_CONTROL_1 = 0x09;
constexpr uint8_t QMC_REG_CONTROL_2 = 0x0A;
constexpr uint8_t QMC_REG_SET_RESET = 0x0B;
constexpr uint8_t QMC_STATUS_DATA_READY = 0x01;
constexpr float QMC_LSB_TO_UT = 0.1f;
constexpr uint32_t IMU_READ_PERIOD_MS = 50;
constexpr uint32_t COMPASS_READ_PERIOD_MS = 50;
constexpr uint32_t SENSOR_DEBUG_PERIOD_MS = 1000;
constexpr float COMPASS_DECLINATION_DEG = 0.0f;
constexpr float HMC_OFFSET_X_UT = 0.0f;
constexpr float HMC_OFFSET_Y_UT = 0.0f;
constexpr float HMC_OFFSET_Z_UT = 0.0f;
constexpr bool ENABLE_TILT_COMPENSATION = false;
constexpr float COMPASS_STALE_EPS_UT = 0.02f;
constexpr uint32_t COMPASS_STALE_WARN_MS = 2000;

constexpr int PWM_FREQ_HZ = 20000;
constexpr int PWM_RES_BITS = 8;
constexpr int PWM_MAX = (1 << PWM_RES_BITS) - 1;
constexpr int PWM_M1_R_CH = 0;
constexpr int PWM_M1_L_CH = 1;
constexpr int PWM_M2_R_CH = 2;
constexpr int PWM_M2_L_CH = 3;

constexpr float ENCODER_PPR = 11.0f;
constexpr float GEAR_RATIO = 18.8f;
// Using channel A rising edges only: 1 count per encoder pulse period.
constexpr float ENCODER_EDGE_MULTIPLIER = 1.0f;
constexpr float COUNTS_PER_OUTPUT_REV = ENCODER_PPR * GEAR_RATIO * ENCODER_EDGE_MULTIPLIER;

constexpr float WHEEL_RADIUS_M = 0.065f;
constexpr float WHEEL_BASE_M = 0.24f;

constexpr uint32_t CONTROL_PERIOD_MS = 10;
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;
constexpr uint32_t CMD_TIMEOUT_MS = 300;
constexpr uint32_t RPM_ESTIMATION_WINDOW_MS = 50;

constexpr float M1_KP = 1.00f;
constexpr float M1_KI = 0.50f;
constexpr float M1_KD = 0.01f;
constexpr float M2_KP = 1.00f;
constexpr float M2_KI = 0.50f;
constexpr float M2_KD = 0.01f;

constexpr float SPEED_LPF_ALPHA = 0.30f;
constexpr int PWM_DEADBAND = 8;
constexpr float ZERO_CMD_RPM_EPS = 0.8f;
constexpr float ZERO_CMD_MPS_EPS = 0.01f;
constexpr float ZERO_CMD_RADPS_EPS = 0.05f;
constexpr float RPM_NOISE_EPS = 0.05f;
constexpr float MAX_PLAUSIBLE_RPM = 700.0f;

constexpr const char* PID_PREFS_NAMESPACE = "pid";
constexpr const char* PID_PREF_KEY_KP = "kp";
constexpr const char* PID_PREF_KEY_KI = "ki";
constexpr const char* PID_PREF_KEY_KD = "kd";

constexpr bool M1_MOTOR_DIR_INVERTED = (M1_MOTOR_DIR_INVERT != 0);
constexpr bool M2_MOTOR_DIR_INVERTED = (M2_MOTOR_DIR_INVERT != 0);

constexpr bool M1_ENCODER_INVERT = true;
constexpr bool M2_ENCODER_INVERT = false;

volatile int32_t g_m1_encoder_count = 0;
volatile int32_t g_m2_encoder_count = 0;

float g_m1_target_rpm = 0.0f;
float g_m2_target_rpm = 0.0f;
float g_m1_measured_rpm = 0.0f;
float g_m2_measured_rpm = 0.0f;

uint32_t g_last_cmd_ms = 0;

Adafruit_LSM6DS3 g_lsm6ds3;
Adafruit_HMC5883_Unified g_hmc5883(5883);

uint8_t g_lsm6ds3_i2c_addr = LSM6DS3_I2C_ADDR_PRIMARY;
bool g_imu_present = false;
bool g_compass_present = false;
bool g_imu_data_valid = false;
bool g_compass_data_valid = false;
bool g_compass_new_sample = false;
uint32_t g_compass_stale_ms = 0;
bool g_compass_using_qmc = false;

struct ImuData {
  float accel_x = 0.0f;
  float accel_y = 0.0f;
  float accel_z = 0.0f;
  float gyro_x = 0.0f;
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;
  uint32_t last_read_ms = 0;
};

struct CompassData {
  float mag_x = 0.0f;
  float mag_y = 0.0f;
  float mag_z = 0.0f;
  float heading_deg = 0.0f;
  uint32_t last_read_ms = 0;
};

ImuData g_imu_data;
CompassData g_compass_data;

#if USE_MICROROS
rcl_allocator_t g_allocator;
rclc_support_t g_support;
rcl_node_t g_node;
rcl_subscription_t g_cmd_sub;
rcl_subscription_t g_rpm_cmd_sub;
rcl_subscription_t g_pid_config_sub;
rcl_publisher_t g_wheel_pub;
rcl_publisher_t g_rpm_feedback_pub;
rcl_publisher_t g_imu_pub;
rcl_publisher_t g_compass_pub;
rclc_executor_t g_executor;

geometry_msgs__msg__Twist g_cmd_msg;
std_msgs__msg__Float32MultiArray g_rpm_cmd_msg;
float g_rpm_cmd_data[2] = {0.0f, 0.0f};
std_msgs__msg__Float32MultiArray g_pid_config_msg;
float g_pid_config_data[3] = {0.0f, 0.0f, 0.0f};
std_msgs__msg__Float32MultiArray g_wheel_msg;
float g_wheel_data[4] = {0.0f, 0.0f, 0.0f, 0.0f};
std_msgs__msg__Float32MultiArray g_rpm_feedback_msg;
float g_rpm_feedback_data[2] = {0.0f, 0.0f};
std_msgs__msg__Float32MultiArray g_imu_msg;
float g_imu_ros_data[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
std_msgs__msg__Float32MultiArray g_compass_msg;
float g_compass_ros_data[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

#endif

bool g_ros_entities_created = false;

enum class AgentState {
  Waiting,
  Connected,
};

AgentState g_agent_state = AgentState::Waiting;

Preferences g_pid_preferences;

class PidController {
 public:
  PidController(float kp, float ki, float kd)
      : kp_(kp), ki_(ki), kd_(kd) {}

  void SetGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void GetGains(float& kp, float& ki, float& kd) const {
    kp = kp_;
    ki = ki_;
    kd = kd_;
  }

  void Reset() {
    integral_ = 0.0f;
    prev_measurement_ = 0.0f;
    first_run_ = true;
  }

  float Update(float setpoint, float measurement, float dt_s) {
    if (dt_s <= 0.0f) {
      return 0.0f;
    }

    const float error = setpoint - measurement;
    integral_ += error * dt_s;
    integral_ = constrain(integral_, -300.0f, 300.0f);

    float d_measurement = 0.0f;
    if (!first_run_) {
      d_measurement = (measurement - prev_measurement_) / dt_s;
    }

    prev_measurement_ = measurement;
    first_run_ = false;

    float output = (kp_ * error) + (ki_ * integral_) - (kd_ * d_measurement);
    output = constrain(output, -static_cast<float>(PWM_MAX), static_cast<float>(PWM_MAX));
    return output;
  }

 private:
  float kp_;
  float ki_;
  float kd_;
  float integral_ = 0.0f;
  float prev_measurement_ = 0.0f;
  bool first_run_ = true;
};

PidController g_m1_pid(M1_KP, M1_KI, M1_KD);
PidController g_m2_pid(M2_KP, M2_KI, M2_KD);

void savePidGainsToNvs(float kp, float ki, float kd) {
  if (!g_pid_preferences.begin(PID_PREFS_NAMESPACE, false)) {
    return;
  }

  g_pid_preferences.putFloat(PID_PREF_KEY_KP, kp);
  g_pid_preferences.putFloat(PID_PREF_KEY_KI, ki);
  g_pid_preferences.putFloat(PID_PREF_KEY_KD, kd);
  g_pid_preferences.end();
}

void loadPidGainsFromNvsAndApply() {
  if (!g_pid_preferences.begin(PID_PREFS_NAMESPACE, true)) {
    return;
  }

  const bool has_kp = g_pid_preferences.isKey(PID_PREF_KEY_KP);
  const bool has_ki = g_pid_preferences.isKey(PID_PREF_KEY_KI);
  const bool has_kd = g_pid_preferences.isKey(PID_PREF_KEY_KD);

  if (has_kp && has_ki && has_kd) {
    const float kp = g_pid_preferences.getFloat(PID_PREF_KEY_KP, M1_KP);
    const float ki = g_pid_preferences.getFloat(PID_PREF_KEY_KI, M1_KI);
    const float kd = g_pid_preferences.getFloat(PID_PREF_KEY_KD, M1_KD);
    g_m1_pid.SetGains(kp, ki, kd);
    g_m2_pid.SetGains(kp, ki, kd);
    g_m1_pid.Reset();
    g_m2_pid.Reset();
  }

  g_pid_preferences.end();
}

inline void stopMotors() {
  ledcWrite(PWM_M1_R_CH, 0);
  ledcWrite(PWM_M1_L_CH, 0);
  ledcWrite(PWM_M2_R_CH, 0);
  ledcWrite(PWM_M2_L_CH, 0);
}

void applyMotorCommand(int motor_index, int pwm_signed) {
  pwm_signed = constrain(pwm_signed, -PWM_MAX, PWM_MAX);
  if (abs(pwm_signed) < PWM_DEADBAND) {
    pwm_signed = 0;
  }

  if (motor_index == 1) {
    const int cmd = M1_MOTOR_DIR_INVERTED ? -pwm_signed : pwm_signed;
    const bool forward = cmd >= 0;
    const int duty = abs(cmd);
    ledcWrite(PWM_M1_R_CH, forward ? duty : 0);
    ledcWrite(PWM_M1_L_CH, forward ? 0 : duty);
  } else {
    const int cmd = M2_MOTOR_DIR_INVERTED ? -pwm_signed : pwm_signed;
    const bool forward = cmd >= 0;
    const int duty = abs(cmd);
    ledcWrite(PWM_M2_R_CH, forward ? duty : 0);
    ledcWrite(PWM_M2_L_CH, forward ? 0 : duty);
  }
}

void setTargetsFromCmdVel(float linear_x_mps, float angular_z_radps) {
  if (fabsf(linear_x_mps) < ZERO_CMD_MPS_EPS) {
    linear_x_mps = 0.0f;
  }
  if (fabsf(angular_z_radps) < ZERO_CMD_RADPS_EPS) {
    angular_z_radps = 0.0f;
  }

  const float v_left = linear_x_mps - (angular_z_radps * WHEEL_BASE_M * 0.5f);
  const float v_right = linear_x_mps + (angular_z_radps * WHEEL_BASE_M * 0.5f);

  const float rpm_left = (v_left / (2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_M)) * 60.0f;
  const float rpm_right = (v_right / (2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_M)) * 60.0f;

  g_m1_target_rpm = rpm_left;
  g_m2_target_rpm = rpm_right;
}

void IRAM_ATTR m1EncoderISR() {
  int8_t delta = (digitalRead(M1_ENC_A_PIN) == digitalRead(M1_ENC_B_PIN)) ? 1 : -1;
  if (M1_ENCODER_INVERT) {
    delta = -delta;
  }
  g_m1_encoder_count += delta;
}

void IRAM_ATTR m2EncoderISR() {
  int8_t delta = (digitalRead(M2_ENC_A_PIN) == digitalRead(M2_ENC_B_PIN)) ? 1 : -1;
  if (M2_ENCODER_INVERT) {
    delta = -delta;
  }
  g_m2_encoder_count += delta;
}

#if USE_MICROROS
void cmdVelCallback(const void* msg_in) {
  const auto* msg = static_cast<const geometry_msgs__msg__Twist*>(msg_in);
  setTargetsFromCmdVel(msg->linear.x, msg->angular.z);
  g_last_cmd_ms = millis();
}

void rpmCmdCallback(const void* msg_in) {
  const auto* msg = static_cast<const std_msgs__msg__Float32MultiArray*>(msg_in);
  if (msg->data.size >= 2) {
    g_m1_target_rpm = msg->data.data[0];
    g_m2_target_rpm = msg->data.data[1];
    g_last_cmd_ms = millis();
  }
}

void pidConfigCallback(const void* msg_in) {
  const auto* msg = static_cast<const std_msgs__msg__Float32MultiArray*>(msg_in);
  if (msg->data.size < 3) {
    return;
  }

  const float kp = msg->data.data[0];
  const float ki = msg->data.data[1];
  const float kd = msg->data.data[2];

  g_m1_pid.SetGains(kp, ki, kd);
  g_m2_pid.SetGains(kp, ki, kd);
  g_m1_pid.Reset();
  g_m2_pid.Reset();
  savePidGainsToNvs(kp, ki, kd);
}

bool createRosEntities() {
  g_allocator = rcl_get_default_allocator();

  if (rclc_support_init(&g_support, 0, nullptr, &g_allocator) != RCL_RET_OK) {
    return false;
  }

  if (rclc_node_init_default(&g_node, "esp32_motor_controller", "", &g_support) != RCL_RET_OK) {
    return false;
  }

  if (rclc_subscription_init_default(
          &g_cmd_sub,
          &g_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "/cmd_vel") != RCL_RET_OK) {
    return false;
  }

  if (rclc_subscription_init_default(
          &g_rpm_cmd_sub,
          &g_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/motor_rpm_cmd") != RCL_RET_OK) {
    return false;
  }

  if (rclc_subscription_init_default(
          &g_pid_config_sub,
          &g_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/pid_config") != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &g_wheel_pub,
          &g_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/wheel_speeds") != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &g_rpm_feedback_pub,
          &g_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/motor_rpm_feedback") != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &g_imu_pub,
          &g_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/imu_raw") != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
          &g_compass_pub,
          &g_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/compass_heading") != RCL_RET_OK) {
    return false;
  }

  g_wheel_msg.data.data = g_wheel_data;
  g_wheel_msg.data.size = 4;
  g_wheel_msg.data.capacity = 4;

  // Pre-allocate incoming array buffers for subscriptions.
  g_rpm_cmd_msg.data.data = g_rpm_cmd_data;
  g_rpm_cmd_msg.data.size = 0;
  g_rpm_cmd_msg.data.capacity = 2;

  g_pid_config_msg.data.data = g_pid_config_data;
  g_pid_config_msg.data.size = 0;
  g_pid_config_msg.data.capacity = 3;

  g_rpm_feedback_msg.data.data = g_rpm_feedback_data;
  g_rpm_feedback_msg.data.size = 2;
  g_rpm_feedback_msg.data.capacity = 2;

  g_imu_msg.data.data = g_imu_ros_data;
  g_imu_msg.data.size = 7;
  g_imu_msg.data.capacity = 7;

  g_compass_msg.data.data = g_compass_ros_data;
  g_compass_msg.data.size = 5;
  g_compass_msg.data.capacity = 5;

  if (rclc_executor_init(&g_executor, &g_support.context, 3, &g_allocator) != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_add_subscription(
          &g_executor,
          &g_cmd_sub,
          &g_cmd_msg,
          &cmdVelCallback,
          ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_add_subscription(
          &g_executor,
          &g_rpm_cmd_sub,
          &g_rpm_cmd_msg,
          &rpmCmdCallback,
          ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_add_subscription(
          &g_executor,
          &g_pid_config_sub,
          &g_pid_config_msg,
          &pidConfigCallback,
          ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  g_ros_entities_created = true;
  return true;
}

void destroyRosEntities() {
  if (!g_ros_entities_created) {
    return;
  }

  (void)rcl_subscription_fini(&g_pid_config_sub, &g_node);
  (void)rcl_subscription_fini(&g_rpm_cmd_sub, &g_node);
  (void)rcl_subscription_fini(&g_cmd_sub, &g_node);
  (void)rcl_publisher_fini(&g_compass_pub, &g_node);
  (void)rcl_publisher_fini(&g_imu_pub, &g_node);
  (void)rcl_publisher_fini(&g_rpm_feedback_pub, &g_node);
  (void)rcl_publisher_fini(&g_wheel_pub, &g_node);
  (void)rcl_node_fini(&g_node);
  (void)rclc_executor_fini(&g_executor);
  (void)rclc_support_fini(&g_support);

  g_ros_entities_created = false;
}

void publishWheelSpeeds() {
  if (!g_ros_entities_created) {
    return;
  }

  g_wheel_data[0] = g_m1_target_rpm;
  g_wheel_data[1] = g_m2_target_rpm;
  g_wheel_data[2] = g_m1_measured_rpm;
  g_wheel_data[3] = g_m2_measured_rpm;

  (void)rcl_publish(&g_wheel_pub, &g_wheel_msg, nullptr);

  g_rpm_feedback_data[0] = g_m1_measured_rpm;
  g_rpm_feedback_data[1] = g_m2_measured_rpm;
  (void)rcl_publish(&g_rpm_feedback_pub, &g_rpm_feedback_msg, nullptr);

  g_imu_ros_data[0] = g_imu_data.accel_x;
  g_imu_ros_data[1] = g_imu_data.accel_y;
  g_imu_ros_data[2] = g_imu_data.accel_z;
  g_imu_ros_data[3] = g_imu_data.gyro_x;
  g_imu_ros_data[4] = g_imu_data.gyro_y;
  g_imu_ros_data[5] = g_imu_data.gyro_z;
  g_imu_ros_data[6] = g_imu_data_valid ? 1.0f : 0.0f;
  (void)rcl_publish(&g_imu_pub, &g_imu_msg, nullptr);

  if (g_compass_new_sample || !g_compass_data_valid) {
    g_compass_ros_data[0] = g_compass_data.mag_x;
    g_compass_ros_data[1] = g_compass_data.mag_y;
    g_compass_ros_data[2] = g_compass_data.mag_z;
    g_compass_ros_data[3] = g_compass_data.heading_deg;
    g_compass_ros_data[4] = g_compass_data_valid ? 1.0f : 0.0f;
    (void)rcl_publish(&g_compass_pub, &g_compass_msg, nullptr);
    g_compass_new_sample = false;
  }
}

#else

bool createRosEntities() {
  return false;
}

void destroyRosEntities() {
}

void publishWheelSpeeds() {
}

#endif

void initMotorPinsAndPwm() {
  pinMode(M1_REN_PIN, OUTPUT);
  pinMode(M1_LEN_PIN, OUTPUT);
  pinMode(M2_REN_PIN, OUTPUT);
  pinMode(M2_LEN_PIN, OUTPUT);

  digitalWrite(M1_REN_PIN, HIGH);
  digitalWrite(M1_LEN_PIN, HIGH);
  digitalWrite(M2_REN_PIN, HIGH);
  digitalWrite(M2_LEN_PIN, HIGH);

  ledcSetup(PWM_M1_R_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(PWM_M1_L_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(PWM_M2_R_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(PWM_M2_L_CH, PWM_FREQ_HZ, PWM_RES_BITS);

  ledcAttachPin(M1_RPWM_PIN, PWM_M1_R_CH);
  ledcAttachPin(M1_LPWM_PIN, PWM_M1_L_CH);
  ledcAttachPin(M2_RPWM_PIN, PWM_M2_R_CH);
  ledcAttachPin(M2_LPWM_PIN, PWM_M2_L_CH);

  stopMotors();
}

void initEncoders() {
  pinMode(M1_ENC_A_PIN, INPUT_PULLUP);
  pinMode(M1_ENC_B_PIN, INPUT_PULLUP);
  pinMode(M2_ENC_A_PIN, INPUT_PULLUP);
  pinMode(M2_ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_ENC_A_PIN), m1EncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A_PIN), m2EncoderISR, RISING);
}

bool probeI2cAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool writeI2cRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readI2cRegisters(uint8_t address, uint8_t start_reg, uint8_t* out_data, size_t len) {
  Wire.beginTransmission(address);
  Wire.write(start_reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t read_count = Wire.requestFrom(static_cast<int>(address), static_cast<int>(len), 1);
  if (read_count != len) {
    return false;
  }

  for (size_t i = 0; i < len; ++i) {
    out_data[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool initQmc5883() {
  const bool ctrl2_ok = writeI2cRegister(QMC5883_I2C_ADDR, QMC_REG_CONTROL_2, 0x00);
  const bool reset_ok = writeI2cRegister(QMC5883_I2C_ADDR, QMC_REG_SET_RESET, 0x01);
  // OSR=512, RNG=8G, ODR=50Hz, MODE=continuous.
  const bool ctrl1_ok = writeI2cRegister(QMC5883_I2C_ADDR, QMC_REG_CONTROL_1, 0x15);
  return ctrl1_ok && ctrl2_ok && reset_ok;
}

bool readQmc5883MagneticField(float& mag_x_ut, float& mag_y_ut, float& mag_z_ut) {
  uint8_t status = 0;
  if (!readI2cRegisters(QMC5883_I2C_ADDR, QMC_REG_STATUS, &status, 1)) {
    return false;
  }
  if ((status & QMC_STATUS_DATA_READY) == 0) {
    return false;
  }

  uint8_t raw[6] = {0};
  if (!readI2cRegisters(QMC5883_I2C_ADDR, QMC_REG_X_LSB, raw, sizeof(raw))) {
    return false;
  }

  const int16_t raw_x = static_cast<int16_t>((static_cast<uint16_t>(raw[1]) << 8) | raw[0]);
  const int16_t raw_y = static_cast<int16_t>((static_cast<uint16_t>(raw[3]) << 8) | raw[2]);
  const int16_t raw_z = static_cast<int16_t>((static_cast<uint16_t>(raw[5]) << 8) | raw[4]);

  mag_x_ut = static_cast<float>(raw_x) * QMC_LSB_TO_UT;
  mag_y_ut = static_cast<float>(raw_y) * QMC_LSB_TO_UT;
  mag_z_ut = static_cast<float>(raw_z) * QMC_LSB_TO_UT;
  return true;
}

void initI2cAndSensors() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  delay(80);

  const bool imu_primary_ok = probeI2cAddress(LSM6DS3_I2C_ADDR_PRIMARY);
  const bool imu_alt_ok = probeI2cAddress(LSM6DS3_I2C_ADDR_ALT);
  const bool compass_addr_ok = probeI2cAddress(HMC5883L_I2C_ADDR);
  const bool qmc_addr_ok = probeI2cAddress(QMC5883_I2C_ADDR);

  if (imu_primary_ok) {
    g_lsm6ds3_i2c_addr = LSM6DS3_I2C_ADDR_PRIMARY;
  } else if (imu_alt_ok) {
    g_lsm6ds3_i2c_addr = LSM6DS3_I2C_ADDR_ALT;
  }

  g_imu_present = g_lsm6ds3.begin_I2C(g_lsm6ds3_i2c_addr, &Wire);
  g_compass_present = g_hmc5883.begin();
  g_compass_using_qmc = false;
  if (!g_compass_present && qmc_addr_ok) {
    g_compass_present = initQmc5883();
    g_compass_using_qmc = g_compass_present;
  }

  Serial.printf("I2C SDA=%d SCL=%d Freq=%luHz\n", I2C_SDA_PIN, I2C_SCL_PIN, static_cast<unsigned long>(I2C_FREQ_HZ));
  Serial.printf("I2C probe: LSM6DS3(0x6A)=%s LSM6DS3(0x6B)=%s HMC5883L(0x1E)=%s\n",
                imu_primary_ok ? "OK" : "MISS",
                imu_alt_ok ? "OK" : "MISS",
                compass_addr_ok ? "OK" : "MISS");
  if (!compass_addr_ok && qmc_addr_ok) {
    Serial.println("WARNING: device found at 0x0D (QMC5883). HMC5883 library may return fixed values.");
  }
  Serial.printf("Sensor init: IMU=%s addr=0x%02X COMPASS=%s (%s)\n",
                g_imu_present ? "OK" : "FAILED",
                g_lsm6ds3_i2c_addr,
                g_compass_present ? "OK" : "FAILED",
                g_compass_using_qmc ? "QMC5883" : "HMC5883L");
}

float computeHeadingDegrees(float mag_x, float mag_y, float mag_z) {
  float heading_rad = 0.0f;

  if (ENABLE_TILT_COMPENSATION && g_imu_data_valid) {
    const float roll = atan2f(g_imu_data.accel_y, g_imu_data.accel_z);
    const float pitch = atan2f(-g_imu_data.accel_x,
                               sqrtf((g_imu_data.accel_y * g_imu_data.accel_y) +
                                     (g_imu_data.accel_z * g_imu_data.accel_z)));
    const float xh = (mag_x * cosf(pitch)) + (mag_z * sinf(pitch));
    const float yh =
        (mag_x * sinf(roll) * sinf(pitch)) + (mag_y * cosf(roll)) - (mag_z * sinf(roll) * cosf(pitch));
    heading_rad = atan2f(yh, xh);
  } else {
    heading_rad = atan2f(mag_y, mag_x);
  }

  heading_rad += COMPASS_DECLINATION_DEG * static_cast<float>(M_PI) / 180.0f;

  while (heading_rad < 0.0f) {
    heading_rad += 2.0f * static_cast<float>(M_PI);
  }
  while (heading_rad >= 2.0f * static_cast<float>(M_PI)) {
    heading_rad -= 2.0f * static_cast<float>(M_PI);
  }

  return heading_rad * 180.0f / static_cast<float>(M_PI);
}

void readImuData(uint32_t now_ms) {
  if (!g_imu_present || ((now_ms - g_imu_data.last_read_ms) < IMU_READ_PERIOD_MS)) {
    return;
  }

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  if (g_lsm6ds3.getEvent(&accel, &gyro, &temp)) {
    g_imu_data.accel_x = accel.acceleration.x;
    g_imu_data.accel_y = accel.acceleration.y;
    g_imu_data.accel_z = accel.acceleration.z;
    g_imu_data.gyro_x = gyro.gyro.x;
    g_imu_data.gyro_y = gyro.gyro.y;
    g_imu_data.gyro_z = gyro.gyro.z;
    g_imu_data.last_read_ms = now_ms;
    g_imu_data_valid = true;
  } else {
    g_imu_data_valid = false;
  }
}

void readCompassData(uint32_t now_ms) {
  if (!g_compass_present || ((now_ms - g_compass_data.last_read_ms) < COMPASS_READ_PERIOD_MS)) {
    return;
  }

  const float prev_x = g_compass_data.mag_x;
  const float prev_y = g_compass_data.mag_y;
  const float prev_z = g_compass_data.mag_z;
  const uint32_t prev_read_ms = g_compass_data.last_read_ms;

  float mag_x = 0.0f;
  float mag_y = 0.0f;
  float mag_z = 0.0f;

  if (g_compass_using_qmc) {
    if (!readQmc5883MagneticField(mag_x, mag_y, mag_z)) {
      return;
    }
  } else {
    sensors_event_t event;
    g_hmc5883.getEvent(&event);
    mag_x = event.magnetic.x;
    mag_y = event.magnetic.y;
    mag_z = event.magnetic.z;
  }

  mag_x -= HMC_OFFSET_X_UT;
  mag_y -= HMC_OFFSET_Y_UT;
  mag_z -= HMC_OFFSET_Z_UT;

  if (isnan(mag_x) || isnan(mag_y) || isnan(mag_z) || isinf(mag_x) || isinf(mag_y) || isinf(mag_z)) {
    g_compass_data_valid = false;
    return;
  }

  g_compass_data.mag_x = mag_x;
  g_compass_data.mag_y = mag_y;
  g_compass_data.mag_z = mag_z;
  g_compass_data.heading_deg = computeHeadingDegrees(mag_x, mag_y, mag_z);
  g_compass_data.last_read_ms = now_ms;
  g_compass_data_valid = true;
  g_compass_new_sample = true;

  const float dx = fabsf(mag_x - prev_x);
  const float dy = fabsf(mag_y - prev_y);
  const float dz = fabsf(mag_z - prev_z);
  if ((dx < COMPASS_STALE_EPS_UT) && (dy < COMPASS_STALE_EPS_UT) && (dz < COMPASS_STALE_EPS_UT)) {
    if (prev_read_ms > 0) {
      g_compass_stale_ms += (now_ms - prev_read_ms);
    }
  } else {
    g_compass_stale_ms = 0;
  }
}

void logSensorData(uint32_t now_ms) {
  static uint32_t last_sensor_log_ms = 0;
  if ((now_ms - last_sensor_log_ms) < SENSOR_DEBUG_PERIOD_MS) {
    return;
  }
  last_sensor_log_ms = now_ms;

  Serial.printf("IMU[%s] A[%.2f %.2f %.2f] G[%.3f %.3f %.3f] | MAG[%s] M[%.2f %.2f %.2f] HDG=%.1f\n",
                g_imu_data_valid ? "OK" : "NA",
                g_imu_data.accel_x,
                g_imu_data.accel_y,
                g_imu_data.accel_z,
                g_imu_data.gyro_x,
                g_imu_data.gyro_y,
                g_imu_data.gyro_z,
                g_compass_data_valid ? "OK" : "NA",
                g_compass_data.mag_x,
                g_compass_data.mag_y,
                g_compass_data.mag_z,
                g_compass_data.heading_deg);

  if (g_compass_stale_ms >= COMPASS_STALE_WARN_MS) {
    Serial.printf("WARNING: compass unchanged for %lums; rotate robot or verify module/address.\n",
                  static_cast<unsigned long>(g_compass_stale_ms));
  }
}

void runControlStep(uint32_t dt_ms) {
  static int32_t prev_m1_count = 0;
  static int32_t prev_m2_count = 0;
  static int32_t accum_m1_delta = 0;
  static int32_t accum_m2_delta = 0;
  static uint32_t accum_dt_ms = 0;

  noInterrupts();
  const int32_t m1_count = g_m1_encoder_count;
  const int32_t m2_count = g_m2_encoder_count;
  interrupts();

  int32_t m1_delta = m1_count - prev_m1_count;
  int32_t m2_delta = m2_count - prev_m2_count;
  const float dt_s = static_cast<float>(dt_ms) / 1000.0f;

  // Reject impossible pulse bursts using a dynamic bound based on dt and max RPM.
  const float max_counts_f =
      ((MAX_PLAUSIBLE_RPM / 60.0f) * COUNTS_PER_OUTPUT_REV * dt_s * 1.5f) + 1.0f;
  const int32_t max_delta_counts = static_cast<int32_t>(max_counts_f);

  if (abs(m1_delta) > max_delta_counts) {
    m1_delta = 0;
  }
  if (abs(m2_delta) > max_delta_counts) {
    m2_delta = 0;
  }

  prev_m1_count = m1_count;
  prev_m2_count = m2_count;

  accum_m1_delta += m1_delta;
  accum_m2_delta += m2_delta;
  accum_dt_ms += dt_ms;

  if (accum_dt_ms >= RPM_ESTIMATION_WINDOW_MS) {
    const float window_dt_s = static_cast<float>(accum_dt_ms) / 1000.0f;
    const float rpm_factor = (60.0f / COUNTS_PER_OUTPUT_REV) / window_dt_s;

    float m1_rpm_raw = static_cast<float>(accum_m1_delta) * rpm_factor;
    float m2_rpm_raw = static_cast<float>(accum_m2_delta) * rpm_factor;

    // Reject encoder spikes/noise that exceed physical motor limits.
    if (fabsf(m1_rpm_raw) > MAX_PLAUSIBLE_RPM) {
      m1_rpm_raw = 0.0f;
    }
    if (fabsf(m2_rpm_raw) > MAX_PLAUSIBLE_RPM) {
      m2_rpm_raw = 0.0f;
    }

    g_m1_measured_rpm = (SPEED_LPF_ALPHA * m1_rpm_raw) + ((1.0f - SPEED_LPF_ALPHA) * g_m1_measured_rpm);
    g_m2_measured_rpm = (SPEED_LPF_ALPHA * m2_rpm_raw) + ((1.0f - SPEED_LPF_ALPHA) * g_m2_measured_rpm);

    accum_m1_delta = 0;
    accum_m2_delta = 0;
    accum_dt_ms = 0;
  }

  if (fabsf(g_m1_measured_rpm) < RPM_NOISE_EPS) {
    g_m1_measured_rpm = 0.0f;
  }
  if (fabsf(g_m2_measured_rpm) < RPM_NOISE_EPS) {
    g_m2_measured_rpm = 0.0f;
  }

  if ((millis() - g_last_cmd_ms) > CMD_TIMEOUT_MS) {
    g_m1_target_rpm = 0.0f;
    g_m2_target_rpm = 0.0f;
  }

  const bool stop_requested =
      (fabsf(g_m1_target_rpm) <= ZERO_CMD_RPM_EPS) &&
      (fabsf(g_m2_target_rpm) <= ZERO_CMD_RPM_EPS);

  if (stop_requested) {
    g_m1_target_rpm = 0.0f;
    g_m2_target_rpm = 0.0f;
    g_m1_pid.Reset();
    g_m2_pid.Reset();
    stopMotors();
    return;
  }

  const int m1_pwm = static_cast<int>(g_m1_pid.Update(g_m1_target_rpm, g_m1_measured_rpm, dt_s));
  const int m2_pwm = static_cast<int>(g_m2_pid.Update(g_m2_target_rpm, g_m2_measured_rpm, dt_s));

  applyMotorCommand(1, m1_pwm);
  applyMotorCommand(2, m2_pwm);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1500);

  initMotorPinsAndPwm();
  initEncoders();
  initI2cAndSensors();
  loadPidGainsFromNvsAndApply();

  g_last_cmd_ms = millis();

#if USE_MICROROS
  set_microros_transports();
#endif
}

void loop() {
  static uint32_t last_control_ms = millis();
  static uint32_t last_telemetry_ms = millis();
  static uint32_t last_agent_check_ms = 0;

  const uint32_t now = millis();

  readImuData(now);
  readCompassData(now);
  logSensorData(now);

  if ((now - last_agent_check_ms) >= 500) {
    last_agent_check_ms = now;

#if USE_MICROROS
    const bool agent_ok = (rmw_uros_ping_agent(50, 1) == RMW_RET_OK);

    if (g_agent_state == AgentState::Waiting && agent_ok) {
      if (createRosEntities()) {
        g_agent_state = AgentState::Connected;
      }
    } else if (g_agent_state == AgentState::Connected && !agent_ok) {
      destroyRosEntities();
      g_agent_state = AgentState::Waiting;
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
    rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(2));

    if ((now - last_telemetry_ms) >= TELEMETRY_PERIOD_MS) {
      last_telemetry_ms = now;
      publishWheelSpeeds();
    }
#endif
  }
}