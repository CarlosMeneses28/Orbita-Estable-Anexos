#ifndef MAINAPP_H_
#define MAINAPP_H_

#include <cstdint>
#include "esp_timer.h"
#include "Comm.h"  // Necesario para ICommListener

// Declaraciones forward
class Command;
class SerialComm;
class Motor;

class MainApp: public ICommListener {
private:
    Motor* mRaMotor = NULL;
    Motor* mDecMotor = NULL;
    
    esp_timer_handle_t mOneSecTimer = NULL;

public:
    MainApp();
    virtual ~MainApp();

    void onCommandReceived(Comm* comm, Command* cmd) override;
    void oneSecTick();
    
private:
    static void oneSecTimerCallback(void* arg);  
};


#endif /* MAINAPP_H_ */