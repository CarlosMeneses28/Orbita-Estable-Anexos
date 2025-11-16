#ifndef _TICK_TIMER_H_
#define _TICK_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "esp_attr.h"

#ifdef __cplusplus
}
#endif

#include <vector>
#include <functional>
#include "Defines.h"

// Reemplazar Vector de Sming con std::vector
using namespace std;

class ITickListener {
public:
    virtual ~ITickListener(){}

    virtual void IRAM_ATTR onTick() = 0;
};

class TickTimer
{
private:
    uint32_t mInternal = 0;
    uint32_t mTicks = 0;
    vector<ITickListener*> mListener;
    bool mRepeating = false;
    bool mStarted = false;
    timer_group_t mGroup = TIMER_GROUP_0;
    timer_idx_t mTimer = TIMER_0;

    static portMUX_TYPE mux;

public:
    static const uint32_t TICK_FREQ = 5000000;

    TickTimer();
    virtual ~TickTimer();

    void IRAM_ATTR addListener(ITickListener* listener);
    void IRAM_ATTR removeListener(ITickListener* listener);

    bool IRAM_ATTR start(bool repeating = true);
    bool IRAM_ATTR startOnce() { return start(false); }
    bool IRAM_ATTR stop();
    bool IRAM_ATTR restart();
    bool isStarted();

    uint32_t getIntervalTicks() const;
    bool IRAM_ATTR setIntervalTicks(uint32_t ticks);

    void IRAM_ATTR onTick();
    
private:
    static bool IRAM_ATTR timerCallback(void* arg);
};

extern TickTimer AppTimer;

#endif