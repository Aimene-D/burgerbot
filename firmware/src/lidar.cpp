#include "lidar.h"
#include "config.h"
#include <LDS_LDS02RR.h>
#include <math.h>

#define LIDAR_PWM_FREQ    1000     // 10 kHz is too fast for IRLZ44N from a GPIO; drop to 1 kHz
#define LIDAR_PWM_BITS    11
#define LIDAR_PWM_CHANNEL 4

static LDS_LDS02RR lidar;
static HardwareSerial LidarSerial(2);

// ── Real double buffer ──────────────────────────────────────────────
static float    g_ranges_buf[2][360];
static float    g_intensities_buf[2][360];
static uint32_t g_scan_start_ms[2]    = {0, 0};  // time when this buffer's rotation began
static uint32_t g_scan_duration_ms[2] = {0, 0};  // measured rotation period
static volatile uint8_t g_write_idx   = 0;       // buffer the lidar task is currently filling
static volatile uint8_t g_read_idx    = 0;       // last completed buffer ready for consumer
static volatile bool    g_scan_ready  = false;

int lidar_serial_read_callback() {
  return LidarSerial.read();
}

size_t lidar_serial_write_callback(const uint8_t* buffer, size_t length) {
  return LidarSerial.write(buffer, length);
}

void lidar_scan_point_callback(float angle_deg, float distance_mm,
                                float quality, bool scan_completed) {
  int idx = (360 - ((int)(angle_deg + 0.5f)) + 180) % 360;
  if (idx < 0) idx += 360;

  const float dist_m = distance_mm / 1000.0f;
  const uint8_t w = g_write_idx;

  // Use INFINITY for invalid / no-return so RViz won't plot a point at the lidar origin.
  const bool invalid = (distance_mm == 0) || (dist_m < 0.12f) || (dist_m > 3.5f);
  g_ranges_buf[w][idx]      = invalid ? INFINITY : dist_m;
  g_intensities_buf[w][idx] = invalid ? 0.0f : quality;

  if (scan_completed) {
    // Stamp duration on the buffer we just finished filling.
    const uint32_t now_ms = millis();
    g_scan_duration_ms[w] = now_ms - g_scan_start_ms[w];

    // Flip buffers: publish what we just filled, start filling the other one.
    g_read_idx = w;
    const uint8_t next_w = w ^ 1u;
    g_write_idx = next_w;

    // Reset the next buffer to INFINITY so stale points from two rotations ago
    // can't leak through if a bin fails to update this rotation.
    for (int i = 0; i < 360; ++i) {
      g_ranges_buf[next_w][i]      = INFINITY;
      g_intensities_buf[next_w][i] = 0.0f;
    }
    g_scan_start_ms[next_w] = now_ms;

    g_scan_ready = true;
  }
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
  int pin = LIDAR_MOTOR_PIN;

  if (value <= (float)LDS::DIR_INPUT) {
    if (value == (float)LDS::DIR_OUTPUT_PWM) {
      ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_BITS);
      ledcAttachPin(pin, LIDAR_PWM_CHANNEL);
    } else {
      pinMode(pin, (value == (float)LDS::DIR_INPUT) ? INPUT : OUTPUT);
    }
    return;
  }

  if (value < (float)LDS::VALUE_PWM) {
    digitalWrite(pin, (value == (float)LDS::VALUE_HIGH) ? HIGH : LOW);
  } else {
    int pwm_value = ((1 << LIDAR_PWM_BITS) - 1) * value;
    ledcWrite(LIDAR_PWM_CHANNEL, pwm_value);
  }
}

void lidar_info_callback(LDS::info_t code, String info) {}
void lidar_error_callback(LDS::result_t code, String info) {}
void lidar_packet_callback(uint8_t* packet, uint16_t length, bool scan_completed) {}

static void lidarTask(void* pvParameters) {
  lidar.setScanPointCallback(lidar_scan_point_callback);
  lidar.setSerialReadCallback(lidar_serial_read_callback);
  lidar.setSerialWriteCallback(lidar_serial_write_callback);
  lidar.setMotorPinCallback(lidar_motor_pin_callback);
  lidar.setInfoCallback(lidar_info_callback);
  lidar.setErrorCallback(lidar_error_callback);
  lidar.setPacketCallback(lidar_packet_callback);

  LidarSerial.begin(lidar.getSerialBaudRate(), SERIAL_8N1, LIDAR_RX_PIN, -1);
  lidar.init();
  lidar.start();

  for (;;) {
    lidar.loop();
    vTaskDelay(1);
  }
}

void initLidar() {
  // Start both buffers full of INFINITY so the very first published frame
  // has well-defined "no return" bins instead of zeros.
  for (int b = 0; b < 2; ++b) {
    for (int i = 0; i < 360; ++i) {
      g_ranges_buf[b][i]      = INFINITY;
      g_intensities_buf[b][i] = 0.0f;
    }
  }
  g_scan_start_ms[0] = millis();
  g_scan_start_ms[1] = millis();

  xTaskCreatePinnedToCore(lidarTask, "lidar_task", 8192, nullptr, 1, nullptr, 1);
}

bool lidarScanReady() {
  return g_scan_ready;
}

// Returns the millis() timestamp at which the published rotation BEGAN,
// and the measured duration of that rotation in milliseconds. Both are
// needed to fill LaserScan.header.stamp / scan_time / time_increment correctly.
void lidarGetScan(float* ranges, float* intensities,
                  uint32_t* scan_start_ms, uint32_t* scan_duration_ms) {
  const uint8_t r = g_read_idx;
  memcpy(ranges,      g_ranges_buf[r],      360 * sizeof(float));
  memcpy(intensities, g_intensities_buf[r], 360 * sizeof(float));
  if (scan_start_ms)    *scan_start_ms    = g_scan_start_ms[r];
  if (scan_duration_ms) *scan_duration_ms = g_scan_duration_ms[r];
  g_scan_ready = false;
}
