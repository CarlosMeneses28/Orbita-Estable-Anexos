#include "SerialComm.h"
#include "Logger.h"
#include "Commands.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <cstring>
#include "driver/gpio.h"

#define BUF_SIZE 256

static const char* TAG = "SerialComm";

SerialComm::SerialComm() {
    // Configurar UART (usando UART_NUM_1 como en el código original)
    mUartNum = UART_NUM_1;
    
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(mUartNum, BUF_SIZE * 2, 0, 10, &mUartQueue, 0));

    // Configurar parámetros UART
    ESP_ERROR_CHECK(uart_param_config(mUartNum, &uart_config));

    // Configurar pines UART para ESP32-C6
    // UART1 - GPIO13 16 (TX), GPIO12 17 (RX) - ajusta según tu hardware
    ESP_ERROR_CHECK(uart_set_pin(mUartNum, 13, 12, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Inicializar buffer
    mSize = 0;
    memset(mData, 0, sizeof(mData));
    
    ESP_LOGI(TAG, "SerialComm initialized on UART1, 9600 baud");
}

SerialComm::~SerialComm() {
    // Limpiar driver UART al destruir
    uart_driver_delete(mUartNum);
}

void SerialComm::checkForData() {
    uint8_t data[128];
    int length = 0;
    
    length = uart_read_bytes(mUartNum, data, sizeof(data), 0);
    
    if (length > 0) {
        //ESP_LOGI(TAG, "Received %d bytes from UART", length);
        
        for (int i = 0; i < length; i++) {
            char arrivedChar = data[i];
            
            // Procesar cada carácter
            if (mSize < MAX_MSG_SIZE) {
                if (arrivedChar == '\r') {
                    // Fin de mensaje - procesar comando completo
                    mData[mSize] = '\0';
                    ESP_LOGI(TAG, "SERIAL <= [%d]: %s", mSize, mData);
                    
                    // Parsear comando
                    Command* cmd = CommandFactory::parseData(mData, mSize);
                    
                    if (cmd != nullptr) {
                        
                        if (mListener != nullptr) {
                            mListener->onCommandReceived(this, cmd);
                        } else {
                            ESP_LOGW(TAG, "No listener set for SerialComm");
                            delete cmd;
                        }
                    } else {
                        ESP_LOGW(TAG, "Failed to parse command: %s", mData);
                    }
                    
                    // Resetear buffer
                    mSize = 0;  // LÍNEA 77
                    memset(mData, 0, sizeof(mData));
                    //ESP_LOGI(TAG, "Buffer reset completed");
                } else {
                    // Agregar carácter al buffer
                    if (arrivedChar >= 32 || arrivedChar == '\r') {
                        mData[mSize++] = arrivedChar;
                    }
                }
            } else {
                // Buffer overflow
                ESP_LOGW(TAG, "Serial buffer overflow, resetting");
                mSize = 0;
                memset(mData, 0, sizeof(mData));
            }
        }
    }
}

bool SerialComm::sendReply(const Reply* reply) {
    if (!reply) {
        ESP_LOGE(TAG, "SerialComm::sendReply: null reply");
        return false;
    }
    
    std::string str = reply->toString();

    if (str == "=205100\r") {
            str = "=030106\r";
        }

    ESP_LOGI(TAG, "SERIAL => %s", str.c_str());
    
    // Enviar datos por UART
    int bytes_written = uart_write_bytes(mUartNum, str.c_str(), str.length());
    
    if (bytes_written != static_cast<int>(str.length())) {
        ESP_LOGE(TAG, "SerialComm::sendReply: Send error, wrote %d/%zu bytes", 
                     bytes_written, str.length());
        return false;
    }
    
    // Esperar a que se transmitan todos los datos (timeout de 1 segundo)
    if (uart_wait_tx_done(mUartNum, 1000 / portTICK_PERIOD_MS) != ESP_OK) {
        ESP_LOGW(TAG, "SerialComm::sendReply: TX timeout");
        // No retornamos false aquí porque los datos pueden haberse enviado parcialmente
    }
    
    return true;
}

void SerialComm::processRxData() {
    ESP_LOGI(TAG, "ENTRO A CHECKFORDATA");
    // Este método ahora está integrado en checkForData()
    // Se mantiene la declaración por compatibilidad con el header
    // Puede usarse para procesamiento adicional si es necesario
}