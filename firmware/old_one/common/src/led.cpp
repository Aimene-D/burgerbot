#include "led.h"
#include <Adafruit_NeoPixel.h>
#include "config.h"

static Adafruit_NeoPixel s_led(STATUS_LED_COUNT, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);
static LedCode s_current_code = LedCode::BOOTING;

static const LedPattern s_patterns[] = {
    {  0,   0, 64,   0},   // BOOTING       — blue solid
    { 64,  64,  0, 500},   // WAITING_AGENT — yellow blink 500ms
    {  0,  64,  0,   0},   // AGENT_CONNECTED — green solid
    {  0,  64, 64,   0},   // CONTROL_ACTIVE — cyan solid
    {  0,  32,  0,2000},   // CMD_TIMEOUT   — green pulse 2s
    { 64,   0,  0, 100},   // ERROR         — red blink 100ms
    { 64,   0,  0,   0},   // STOP          — red solid
    { 32,   0, 32,1000},   // MAG_CALIB     — purple pulse 1s
};
static constexpr int NUM_PATTERNS = sizeof(s_patterns) / sizeof(s_patterns[0]);

const LedPattern& getPatternForCode(LedCode code) {
    int idx = static_cast<int>(code);
    if (idx < 0 || idx >= NUM_PATTERNS) idx = 0;
    return s_patterns[idx];
}

void initStatusLed() {
    s_led.begin();
    s_led.setBrightness(64);   // moderate brightness
    s_led.show();
}

void setStatusLed(LedCode code) {
    s_current_code = code;
}

LedCode getStatusLed() {
    return s_current_code;
}

void ledTask(void* param) {
    (void)param;
    TickType_t last_wake = xTaskGetTickCount();
    bool led_on = true;
    uint32_t last_toggle_ms = 0;

    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));  // 20 Hz refresh

        LedCode code = s_current_code;
        const LedPattern& pat = getPatternForCode(code);
        uint32_t now = millis();

        if (pat.blink_ms == 0) {
            // Solid colour
            s_led.setPixelColor(0, s_led.Color(pat.r, pat.g, pat.b));
        } else {
            // Blink: toggle at pat.blink_ms interval
            if ((now - last_toggle_ms) >= pat.blink_ms) {
                last_toggle_ms = now;
                led_on = !led_on;
            }
            if (led_on) {
                s_led.setPixelColor(0, s_led.Color(pat.r, pat.g, pat.b));
            } else {
                s_led.setPixelColor(0, s_led.Color(0, 0, 0));
            }
        }
        s_led.show();
    }
}
