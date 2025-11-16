#include "MainApp.h"
#include "Logger.h"
#include "TickTimer.h"
#include "SerialComm.h"
#include "Motor.h"
#include "Commands.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char* TAG = "MainApp";

extern SerialComm* serialComm;

MainApp::MainApp() {

    ESP_LOGI(TAG, "Initializing objects...");
    
    // Configurar listener para comunicación serial
    if (serialComm != nullptr) {
        serialComm->setListener(this);
        ESP_LOGI(TAG, "SerialComm listener set");
    } else {
        ESP_LOGE(TAG, "SerialComm is null - cannot set listener!");
    }

    // Inicializar motores
    mRaMotor = new Motor(EAxis::AXIS_RA);
    mDecMotor = new Motor(EAxis::AXIS_DEC);

    // Configurar timer de 1 segundo
    esp_timer_create_args_t timer_args = {};
    timer_args.callback = &MainApp::oneSecTimerCallback;
    timer_args.arg = this;
    timer_args.dispatch_method = ESP_TIMER_TASK;
    timer_args.name = "mainapp_1sec";
    
    if (esp_timer_create(&timer_args, &mOneSecTimer) != ESP_OK) {
        ESP_LOGI(TAG, "Failed to create 1-second timer");
    }

    ESP_LOGI(TAG, "Setting callbacks...");
    
    // Agregar motores como listeners del timer principal
    AppTimer.addListener(mRaMotor);
    AppTimer.addListener(mDecMotor);

    // Configurar intervalo del timer principal
    float interval_ticks = (float)TickTimer::TICK_FREQ / (float)Defines::TIMER_FREQ;
    AppTimer.setIntervalTicks((uint32_t)interval_ticks);

    ESP_LOGI(TAG, "Start timers...");
    
    // Iniciar timers
    AppTimer.start(true);
    
    if (mOneSecTimer) {
        if (esp_timer_start_periodic(mOneSecTimer, 1000000) != ESP_OK) {
            ESP_LOGI(TAG, "Failed to start 1-second timer");
        }
    }

    ESP_LOGI(TAG, "Init done.");
}

MainApp::~MainApp() {
    // Detener timers
    AppTimer.stop();
    
    if (mOneSecTimer) {
        esp_timer_stop(mOneSecTimer);
        esp_timer_delete(mOneSecTimer);
        mOneSecTimer = NULL;
    }
    
    // Liberar memoria
    delete mRaMotor;
    mRaMotor = NULL;
    
    delete mDecMotor;
    mDecMotor = NULL;
}

void MainApp::oneSecTick() {
    // Mostrar información de los motores cada segundo
    if (mRaMotor) mRaMotor->printInfo();
    if (mDecMotor) mDecMotor->printInfo();
}

void MainApp::oneSecTimerCallback(void* arg) {
    MainApp* app = static_cast<MainApp*>(arg);
    if (app) {
        app->oneSecTick();
    }
}

void MainApp::onCommandReceived(Comm* comm, Command* cmd) {
    if (cmd != nullptr) {
        Reply* reply = nullptr;
        
        // Procesar comando según el eje
        switch (cmd->getAxis()) {
            case EAxis::AXIS_RA: {
                if (mRaMotor) {
                    reply = mRaMotor->processCommand(cmd);
                }
                break;
            }
                        
            case EAxis::AXIS_DEC: {
                if (mDecMotor) {
                    reply = mDecMotor->processCommand(cmd);
                }
                break;
            }
                        
            case EAxis::AXIS_BOTH: {
                ESP_LOGI(TAG, "onCommandReceived: Command %d for both axis", (int)cmd->getCmd());
                if (mRaMotor) {
                    reply = mRaMotor->processCommand(cmd);
                }
                if (mDecMotor) {
                    Reply* reply2 = mDecMotor->processCommand(cmd);
                    delete reply2; // Liberar la segunda respuesta inmediatamente
                }
                break;
            }
                        
            case EAxis::AXIS_NONE: {
                ESP_LOGW(TAG, "onCommandReceived: Command with no axis specified");
                break;
            }

            default: {
                ESP_LOGW(TAG, "onCommandReceived: Unknown axis");
                break;
            }
        }
        
        // Enviar respuesta si existe
        if (reply != nullptr && comm != nullptr) {
            comm->sendReply(reply);
            delete reply; // Liberar la respuesta después de enviarla
        }
        
        // Liberar el comando
        delete cmd;
    }
}