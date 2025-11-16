#include "TickTimer.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "driver/timer.h"

static const char* TAG = "TickTimer";

TickTimer AppTimer;

// Callback estático para el timer de ESP-IDF
static bool IRAM_ATTR timer_callback(void* arg) {
    TickTimer* timer = (TickTimer*)arg;
    if (timer) {
        timer->onTick();
    }
    return true; // Retornar true para timer periódico
}

TickTimer::TickTimer() {
    mGroup = TIMER_GROUP_0;
    mTimer = TIMER_0;
    mTicks = 0;
    mInternal = 0;
    mRepeating = false;
    mStarted = false;
}

TickTimer::~TickTimer() {
    stop();
}

void IRAM_ATTR TickTimer::onTick() {
    // Iterar sobre una copia para evitar problemas si se modifican los listeners durante la callback
    std::vector<ITickListener*> listenersCopy = mListener;
    
    for (ITickListener* listener : listenersCopy) {
        if (listener) {
            listener->onTick();
        }
    }
}

bool IRAM_ATTR TickTimer::start(bool repeating) {
    if (mInternal == 0) {
        ESP_LOGE(TAG, "Cannot start timer with interval 0");
        return false;
    }

    mRepeating = repeating;
    
    // Configurar el timer
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = repeating ? TIMER_AUTORELOAD_EN : TIMER_AUTORELOAD_DIS,
        .divider = 16, // 80 MHz / 16 = 5 MHz (igual que el código original)
    };
    
    timer_init(mGroup, mTimer, &config);
    
    // Configurar la alarma
    timer_set_counter_value(mGroup, mTimer, 0);
    timer_set_alarm_value(mGroup, mTimer, mInternal);
    
    // Habilitar interrupciones
    timer_enable_intr(mGroup, mTimer);
    timer_isr_callback_add(mGroup, mTimer, timer_callback, this, 0);
    
    // Iniciar el timer
    timer_start(mGroup, mTimer);
    
    mStarted = true;
    ESP_LOGD(TAG, "Timer started with interval %lu ticks, repeating: %d", mInternal, repeating);
    
    return true;
}

bool IRAM_ATTR TickTimer::stop() {
    if (!mStarted) {
        return true;
    }
    
    timer_pause(mGroup, mTimer);
    timer_disable_intr(mGroup, mTimer);
    timer_isr_callback_remove(mGroup, mTimer);
    
    mStarted = false;
    ESP_LOGD(TAG, "Timer stopped");
    
    return true;
}

bool IRAM_ATTR TickTimer::restart() {
    stop();
    return start(mRepeating);
}

bool TickTimer::isStarted() {
    return mStarted;
}

uint32_t TickTimer::getIntervalTicks() const {
    return mInternal;
}

bool IRAM_ATTR TickTimer::setIntervalTicks(uint32_t ticks) {
    if (ticks == 0) {
        ESP_LOGE(TAG, "Interval cannot be 0");
        return false;
    }
    
    mInternal = ticks;
    
    if (mStarted) {
        // Actualizar el valor de alarma si el timer está corriendo
        timer_set_alarm_value(mGroup, mTimer, mInternal);
    }
    
    ESP_LOGD(TAG, "Timer interval set to %lu ticks", ticks);
    return true;
}

void IRAM_ATTR TickTimer::addListener(ITickListener* listener) {
    if (!listener) return;
    
    // Usar portENTER_CRITICAL para sección crítica (equivalente a ETS_INTR_LOCK)
    portENTER_CRITICAL(&mux);
    
    // Verificar si ya existe
    bool exists = false;
    for (auto existing : mListener) {
        if (existing == listener) {
            exists = true;
            break;
        }
    }
    
    if (!exists) {
        mListener.push_back(listener);
    }
    
    portEXIT_CRITICAL(&mux);
}

void IRAM_ATTR TickTimer::removeListener(ITickListener* listener) {
    if (!listener) return;
    
    portENTER_CRITICAL(&mux);
    
    // Buscar y eliminar el listener
    for (auto it = mListener.begin(); it != mListener.end(); ++it) {
        if (*it == listener) {
            mListener.erase(it);
            break;
        }
    }
    
    portEXIT_CRITICAL(&mux);
}

// Necesitamos declarar el mutex estático
portMUX_TYPE TickTimer::mux = portMUX_INITIALIZER_UNLOCKED;