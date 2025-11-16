#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "Logger.h"
#include "MainApp.h"
#include "SerialComm.h"

static const char* TAG = "SynScan";

// Cambiar a puntero para controlar mejor el orden de inicialización
SerialComm* serialComm = nullptr;
MainApp* app = nullptr;

void uartEventTask(void* parameter) {
    uart_event_t event;
    
    while (true) {
        // Esperar eventos UART
        if (serialComm != nullptr && 
            xQueueReceive(serialComm->getUartQueue(), &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    // Hay datos disponibles, procesarlos
                    serialComm->checkForData();
                    break;
                    
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    // Manejar overflow
                    uart_flush_input(UART_NUM_1);
                    if (serialComm != nullptr) {
                        xQueueReset(serialComm->getUartQueue());
                    }
                    break;
                    
                case UART_BREAK:
                case UART_PARITY_ERR:
                case UART_FRAME_ERR:
                    // Manejar errores
                    ESP_LOGE(TAG, "UART error: %d", event.type);
                    break;
                    
                default:
                    ESP_LOGD(TAG, "UART event: %d", event.type);
                    break;
            }
        }
    }
}

/**
 * Will be called when system initialization was completed
 */
void ready() {
    ESP_LOGI(TAG, "System ready - initializing SynScan");
    
    // Crear SerialComm PRIMERO
    serialComm = new SerialComm();
    if (serialComm == nullptr) {
        ESP_LOGE(TAG, "Failed to create SerialComm!");
        return;
    }
    
    // Verificar que la cola se creó correctamente
    if (serialComm->getUartQueue() == nullptr) {
        ESP_LOGE(TAG, "UART queue not created!");
        return;
    }
    
    // Crear task para eventos UART
    xTaskCreate(uartEventTask, "uart_event_task", 4096, NULL, 2, NULL);
    
    // Crear MainApp después de SerialComm
    app = new MainApp();
    if (app == nullptr) {
        ESP_LOGE(TAG, "Failed to create MainApp!");
        return;
    }
    
    ESP_LOGI(TAG, "SynScan initialization completed");
}

/**
 * Main initialization function for ESP-IDF
 */
extern "C" void app_main(void) {
    // Set log level
    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_LOGI(TAG, "System initialized, starting SynScan...");

    // Call ready function after short delay
    vTaskDelay(pdMS_TO_TICKS(1000));
    ready();

    // Main loop
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Puedes aumentar este delay
    }
}