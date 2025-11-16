
#include "Logger.h"
#include <cstdio>
#include <cstring>

namespace Logger {

    // --- Función interna para escritura segura ---
    static void log_write_safe(esp_log_level_t level, const char* tag, const char* fmt, va_list args) {
        char buffer[LOG_BUFFER_SIZE];
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        buffer[sizeof(buffer) - 1] = '\0';
        esp_log_write(level, tag, "%s\n", buffer);
    }

    void init(esp_log_level_t level) {
        esp_log_level_set("*", level);
        esp_log_level_set(DEFAULT_TAG, level);
    }

    // --- Versiones sin tag ---
    void notice(const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        log_write_safe(ESP_LOG_INFO, DEFAULT_TAG, fmt, ap);
        va_end(ap);
    }

    void verbose(const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        log_write_safe(ESP_LOG_VERBOSE, DEFAULT_TAG, fmt, ap);
        va_end(ap);
    }

    void info(const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        log_write_safe(ESP_LOG_INFO, DEFAULT_TAG, fmt, ap);
        va_end(ap);
    }

    void warning(const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        log_write_safe(ESP_LOG_WARN, DEFAULT_TAG, fmt, ap);
        va_end(ap);
    }

    void error(const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        log_write_safe(ESP_LOG_ERROR, DEFAULT_TAG, fmt, ap);
        va_end(ap);
    }

    // --- Versiones con tag ---
    void notice(const char* tag, const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        log_write_safe(ESP_LOG_INFO, tag, fmt, ap);
        va_end(ap);
    }

    void verbose(const char* tag, const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        log_write_safe(ESP_LOG_VERBOSE, tag, fmt, ap);
        va_end(ap);
    }

    void info(const char* tag, const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        log_write_safe(ESP_LOG_INFO, tag, fmt, ap);
        va_end(ap);
    }

    void warning(const char* tag, const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        log_write_safe(ESP_LOG_WARN, tag, fmt, ap);
        va_end(ap);
    }

    void error(const char* tag, const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        log_write_safe(ESP_LOG_ERROR, tag, fmt, ap);
        va_end(ap);
    }

} // namespace Logger
