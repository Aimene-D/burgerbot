#pragma once

// WS2812B status codes shared by all BurgerBot PCBs.
// Each code maps to a colour + blink pattern rendered by ledTask().

enum class LedCode {
    BOOTING,           // Blue solid         — power-on, peripherals initializing
    WAITING_AGENT,     // Yellow blink 500ms — micro-ROS agent not connected
    AGENT_CONNECTED,   // Green solid        — agent online, no motor cmd
    CONTROL_ACTIVE,    // Cyan solid         — motors receiving commands
    CMD_TIMEOUT,       // Green pulse 2s     — agent connected, motors idle >300ms
    ERROR,             // Red blink 100ms    — hardware fault detected
    STOP,              // Red solid          — emergency stop / brake active
    MAG_CALIB          // Purple pulse 1s    — mag calibration in progress (sensor PCB only)
};
