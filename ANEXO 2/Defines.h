#ifndef DEFINES_H_
#define DEFINES_H_

#include <string>
#include <cstdint>

// Definir tipos
using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;

// Usar uint32_t en lugar de u32
struct Defines {
    static const uint32_t LOG_PORT = 6667;
    static const uint32_t SYNSCAN_PORT = 11880;
    static const uint32_t PULSE_GUIDE_PORT = 5002;
    static const uint32_t PULSE_DISCOVERY_PORT = 5003;
    static const uint32_t DISCOVERY_PT = 0x36;
    static const uint32_t TIMER_FREQ = 50000; 
};

// Configuración WiFi para ESP-IDF
#if 0
#define WIFI_STATION
#define WIFI_STATION_SSID "WIFI_SSID"
#define WIFI_STATION_PASS "here_the_password"
#endif

#define OVERCLOCK
#ifdef OVERCLOCK
#define CPU_FREQ eCF_160MHz
#else
#define CPU_FREQ eCF_80MHz
#endif

#define INFINITE 0xFFFFFFFF

#define MIN(a, b) ((a < b)?a:b)

enum class EAxis {
    AXIS_RA = 1, AXIS_DEC = 2, AXIS_BOTH = 3, AXIS_NONE = 0
};

enum class ESlewType {
    GOTO, TRACKING, NONE
};

enum class ESpeed {
    FAST, SLOW, NONE
};

enum class EDirection {
    CCW, CW, NONE
};

#endif /* DEFINES_H_ */