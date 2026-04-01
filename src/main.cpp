#include <Arduino.h>
#include <math.h>

#ifndef USE_MICROROS
#define USE_MICROROS 0
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

constexpr int M2_RPWM_PIN = 20;
constexpr int M2_LPWM_PIN = 21;
constexpr int M2_REN_PIN = 2;
constexpr int M2_LEN_PIN = 1;

constexpr int M1_ENC_B_PIN = 5;   // C2
constexpr int M1_ENC_A_PIN = 6;   // C1
constexpr int M2_ENC_B_PIN = 38;  // C2
constexpr int M2_ENC_A_PIN = 37;  // C1

constexpr int PWM_FREQ_HZ = 20000;
constexpr int PWM_RES_BITS = 8;
constexpr int PWM_MAX = (1 << PWM_RES_BITS) - 1;
constexpr int PWM_M1_R_CH = 0;
constexpr int PWM_M1_L_CH = 1;
constexpr int PWM_M2_R_CH = 2;
constexpr int PWM_M2_L_CH = 3;

constexpr float ENCODER_PPR = 11.0f;
constexpr float GEAR_RATIO = 18.8f;
constexpr float ENCODER_EDGE_MULTIPLIER = 2.0f;
constexpr float COUNTS_PER_OUTPUT_REV = ENCODER_PPR * GEAR_RATIO * ENCODER_EDGE_MULTIPLIER;

constexpr float WHEEL_RADIUS_M = 0.05f;
constexpr float WHEEL_BASE_M = 0.20f;

constexpr uint32_t CONTROL_PERIOD_MS = 10;
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;
constexpr uint32_t CMD_TIMEOUT_MS = 300;

constexpr float M1_KP = 1.00f;
constexpr float M1_KI = 0.50f;
constexpr float M1_KD = 0.01f;
constexpr float M2_KP = 1.00f;
constexpr float M2_KI = 0.50f;
constexpr float M2_KD = 0.01f;

constexpr float SPEED_LPF_ALPHA = 0.30f;
constexpr int PWM_DEADBAND = 8;

constexpr bool M1_ENCODER_INVERT = false;
constexpr bool M2_ENCODER_INVERT = false;

volatile int32_t g_m1_encoder_count = 0;
volatile int32_t g_m2_encoder_count = 0;

float g_m1_target_rpm = 0.0f;
float g_m2_target_rpm = 0.0f;
float g_m1_measured_rpm = 0.0f;
float g_m2_measured_rpm = 0.0f;

uint32_t g_last_cmd_ms = 0;

#if USE_MICROROS
rcl_allocator_t g_allocator;
rclc_support_t g_support;
rcl_node_t g_node;
rcl_subscription_t g_cmd_sub;
rcl_publisher_t g_wheel_pub;
rclc_executor_t g_executor;

geometry_msgs__msg__Twist g_cmd_msg;
std_msgs__msg__Float32MultiArray g_wheel_msg;
float g_wheel_data[4] = {0.0f, 0.0f, 0.0f, 0.0f};

#endif

bool g_ros_entities_created = false;

enum class AgentState {
  Waiting,
  Connected,
};

AgentState g_agent_state = AgentState::Waiting;

class PidController {
 public:
  PidController(float kp, float ki, float kd)
      : kp_(kp), ki_(ki), kd_(kd) {}

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

  const bool forward = pwm_signed >= 0;
  const int duty = abs(pwm_signed);

  if (motor_index == 1) {
    ledcWrite(PWM_M1_R_CH, forward ? duty : 0);
    ledcWrite(PWM_M1_L_CH, forward ? 0 : duty);
  } else {
    ledcWrite(PWM_M2_R_CH, forward ? duty : 0);
    ledcWrite(PWM_M2_L_CH, forward ? 0 : duty);
  }
}

void setTargetsFromCmdVel(float linear_x_mps, float angular_z_radps) {
  const float v_left = linear_x_mps - (angular_z_radps * WHEEL_BASE_M * 0.5f);
  const float v_right = linear_x_mps + (angular_z_radps * WHEEL_BASE_M * 0.5f);

  const float rpm_left = (v_left / (2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_M)) * 60.0f;
  const float rpm_right = (v_right / (2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_M)) * 60.0f;

  g_m1_target_rpm = rpm_left;
  g_m2_target_rpm = rpm_right;
}

void IRAM_ATTR m1EncoderISR() {
  const bool a = digitalRead(M1_ENC_A_PIN);
  const bool b = digitalRead(M1_ENC_B_PIN);
  int dir = (a == b) ? 1 : -1;
  if (M1_ENCODER_INVERT) {
    dir = -dir;
  }
  g_m1_encoder_count += dir;
}

void IRAM_ATTR m2EncoderISR() {
  const bool a = digitalRead(M2_ENC_A_PIN);
  const bool b = digitalRead(M2_ENC_B_PIN);
  int dir = (a == b) ? 1 : -1;
  if (M2_ENCODER_INVERT) {
    dir = -dir;
  }
  g_m2_encoder_count += dir;
}

#if USE_MICROROS
void cmdVelCallback(const void* msg_in) {
  const auto* msg = static_cast<const geometry_msgs__msg__Twist*>(msg_in);
  setTargetsFromCmdVel(msg->linear.x, msg->angular.z);
  g_last_cmd_ms = millis();
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

  if (rclc_publisher_init_default(
          &g_wheel_pub,
          &g_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/wheel_speeds") != RCL_RET_OK) {
    return false;
  }

  g_wheel_msg.data.data = g_wheel_data;
  g_wheel_msg.data.size = 4;
  g_wheel_msg.data.capacity = 4;

  if (rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator) != RCL_RET_OK) {
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

  g_ros_entities_created = true;
  return true;
}

void destroyRosEntities() {
  if (!g_ros_entities_created) {
    return;
  }

  (void)rcl_subscription_fini(&g_cmd_sub, &g_node);
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

  attachInterrupt(digitalPinToInterrupt(M1_ENC_A_PIN), m1EncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A_PIN), m2EncoderISR, CHANGE);
}

void runControlStep(uint32_t dt_ms) {
  static int32_t prev_m1_count = 0;
  static int32_t prev_m2_count = 0;

  noInterrupts();
  const int32_t m1_count = g_m1_encoder_count;
  const int32_t m2_count = g_m2_encoder_count;
  interrupts();

  const int32_t m1_delta = m1_count - prev_m1_count;
  const int32_t m2_delta = m2_count - prev_m2_count;

  prev_m1_count = m1_count;
  prev_m2_count = m2_count;

  const float dt_s = static_cast<float>(dt_ms) / 1000.0f;
  const float rpm_factor = (60.0f / COUNTS_PER_OUTPUT_REV) / dt_s;

  const float m1_rpm_raw = static_cast<float>(m1_delta) * rpm_factor;
  const float m2_rpm_raw = static_cast<float>(m2_delta) * rpm_factor;

  g_m1_measured_rpm = (SPEED_LPF_ALPHA * m1_rpm_raw) + ((1.0f - SPEED_LPF_ALPHA) * g_m1_measured_rpm);
  g_m2_measured_rpm = (SPEED_LPF_ALPHA * m2_rpm_raw) + ((1.0f - SPEED_LPF_ALPHA) * g_m2_measured_rpm);

  if ((millis() - g_last_cmd_ms) > CMD_TIMEOUT_MS) {
    g_m1_target_rpm = 0.0f;
    g_m2_target_rpm = 0.0f;
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