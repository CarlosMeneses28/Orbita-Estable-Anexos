#ifndef APP_SERIALCOMM_H_
#define APP_SERIALCOMM_H_

#include "Comm.h"
#include "driver/uart.h"

class SerialComm: public Comm {
    static const int MAX_MSG_SIZE = 10;
    
    // Reemplazar HardwareSerial con UART de ESP-IDF
    uart_port_t mUartNum;
    QueueHandle_t mUartQueue;
    int mSize = 0;
    char mData[MAX_MSG_SIZE + 1];

    // Reemplazar callback de Stream
    void processRxData();

public:
    SerialComm();
    virtual ~SerialComm();

    bool sendReply(const Reply* reply);
    
    // Método para procesar datos recibidos (llamar desde app_main)
    void checkForData();

    QueueHandle_t getUartQueue() { return mUartQueue; }
};

#endif /* APP_SERIALCOMM_H_ */