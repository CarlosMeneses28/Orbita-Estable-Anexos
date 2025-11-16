#ifndef MOTOR_H_
#define MOTOR_H_

#include "Defines.h"
#include "Commands.h"
#include "TickTimer.h"
#include "esp_timer.h"
#include "driver/gpio.h"

class Motor: public ITickListener {
public:
    static const u8 VERSION_MAJOR = 2;
    static const u8 VERSION_MINOR = 0;
    static const u8 VERSION_MICRO = 5;
    static const u8 VERSION_PATCH = 1;

    static const u32 MOUNT_RATIO = 1;
    static const u32 GEAR_RATIO_DEN = 1;
    static const u32 HIGH_SPEED_RATIO = 1;

    // Variables calculadas en el constructor
    u32 MICRO_STEPS = 0;
    float GEAR_RATIO_NUM = 0.0f;
    float STEP_DEGREE = 0.0f;
    float GEAR_RATIO = 0.0f;
    float DEGREE_PER_STEP = 0.0f;
    u32 STEPS_PER_WORM_REV = 0;
    u32 STEPS_PER_RA_REV = 0;
    float SIDERAL_SPEED_ARCSEC = 15.04108444f;
    float ARCSEC_TO_DEGREE = 0.000277778f;
    float SIDERAL_SPEED_DEGREE = SIDERAL_SPEED_ARCSEC * ARCSEC_TO_DEGREE;
    float SIDERAL_PERIOD_FREQ = 0.0f;
    float SIDERAL_STEP_COUNT = 0.0f;

private:
    float mGearRatioNum = 0.0;
    float mStepDegree = 0.0;

    float pos = 0.0f;
    int mPecPeriod = 0;
    int mPosition = 0x800000;
    int mTargetPosition = INFINITE;
    int mOrigPosition = INFINITE;

    int mBreakCount = 0;
    int mBreakPosition = 3500;
    int mMicroStepm = 0;

    int mSideralStepPeriod = 0;
    int mStepPeriod = 0;
    int mFastStepMult = 500;
    int mStepCount = 0;
    bool mStepDown = false;
    int mCurrentStepPeriod = 0;
    int mCurrentStepFreq = 0;

    bool mToStop = false;
    bool mMoving = false;
    bool mMicroStep = false;

    ESpeed mSpeed = ESpeed::SLOW;
    ESlewType mType = ESlewType::NONE;
    EDirection mDir = EDirection::NONE;
    u32 mAutoGuideMode = 1000;

    gpio_num_t mDirPin = GPIO_NUM_NC;
    gpio_num_t mStepPin = GPIO_NUM_NC;
    gpio_num_t mMode[3] = {GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC};
    gpio_num_t mAGPlus = GPIO_NUM_NC;
    gpio_num_t mAGMinus = GPIO_NUM_NC;

    esp_timer_handle_t m100msecTimer = NULL;

    int mMaxCount = 0;
    int mMinCount = 0;
    int mCommStatusWD = 0;
    int mCommStatusTimeout = 20;
    int mPulseTotalTicks = 0;
    int mPulseCurrentTicks = 0;
    EDirection mPulseDir = EDirection::NONE;
    int mPulseStepFreq = 0;
    bool mIsPulseDone = false;
    EAxis mAxis;

    int updateCurrentStepPeriod(u32 target_step);
    void setStepPeriod(u32 period);
    void setMicroSteps(bool active);
    void setDir(EDirection dir);
    void stop();
    int degreeToPosition(float degree);
    void on100msecTick();
    static void timerCallback(void* arg);

    friend void IRAM_ATTR st4_isr_handler(void* arg);

public:
    Motor(EAxis axis);
    virtual ~Motor();

    void IRAM_ATTR onTick() override;
    Reply* processCommand(const Command* cmd);
    void printInfo() const;
};

#endif /* MOTOR_H_ */
