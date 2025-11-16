#pragma once
#include "esp_log.h"
#include <cstdarg>

namespace Logger {

    // Configura el tamaño máximo del buffer para logs
    constexpr size_t LOG_BUFFER_SIZE = 256;
    constexpr const char* DEFAULT_TAG = "SYNSCAN";

    // Inicializa el logger (opcional, para configurar nivel global)
    void init(esp_log_level_t level = ESP_LOG_INFO);

    // --- Funciones sin tag ---
    void notice(const char* fmt, ...);
    void verbose(const char* fmt, ...);
    void info(const char* fmt, ...);
    void warning(const char* fmt, ...);
    void error(const char* fmt, ...);

    // --- Funciones con tag personalizado ---
    void notice(const char* tag, const char* fmt, ...);
    void verbose(const char* tag, const char* fmt, ...);
    void info(const char* tag, const char* fmt, ...);
    void warning(const char* tag, const char* fmt, ...);
    void error(const char* tag, const char* fmt, ...);

} // namespace Logger
