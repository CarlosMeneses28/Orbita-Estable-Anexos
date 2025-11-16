#include "Motor.h"
#include "Logger.h"
#include "Commands.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include <cstdio>
u32 mult = 0;
// 
static const char* TAG = "Motor";

void IRAM_ATTR st4_isr_handler(void* arg);

// Macros para conversión de steps a frecuencia
#define STEP_TO_FREQ(x) ((x) != 0 ? (Defines::TIMER_FREQ / (x)) : INFINITE)
#define FREQ_TO_STEP(x) ((x) != 0 ? (Defines::TIMER_FREQ / (x)) : INFINITE) 

// Variable para el mutex (necesita definición)
portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;

Motor::Motor(EAxis axis) {
    ESP_LOGI(TAG, "Motor[%d]: Initializing %s motor...", (int)axis,
            axis == EAxis::AXIS_RA ? "RA" : "DEC");
    mAxis = axis;
    
    // Configurar pines GPIO según el eje - usar GPIO_NUM_X directamente
    if (mAxis == EAxis::AXIS_RA) {
		mDirPin = GPIO_NUM_10;
        mStepPin = GPIO_NUM_11; 
		mAGPlus = GPIO_NUM_4;
		mAGMinus = GPIO_NUM_5;

        mGearRatioNum = 100.0;     
        mStepDegree = (360.0/8190.0); 
		mMicroStepm = 1;
		pos = -90.0f;

    } else if (mAxis == EAxis::AXIS_DEC) {
        mDirPin = GPIO_NUM_22;
        mStepPin = GPIO_NUM_21;
		mAGPlus = GPIO_NUM_20;
		mAGMinus = GPIO_NUM_2;

		mGearRatioNum = 50.0;   
        mStepDegree = (360.0/8190.0);
		mMicroStepm = 1; 

		pos = 0.0f;
    }

	// Calcular variables derivadas
	GEAR_RATIO_NUM = mGearRatioNum;
	STEP_DEGREE = mStepDegree;
	MICRO_STEPS = mMicroStepm;
    GEAR_RATIO = ((float)GEAR_RATIO_NUM / (float)GEAR_RATIO_DEN);
    DEGREE_PER_STEP = (float)mStepDegree / (GEAR_RATIO * (float)MOUNT_RATIO * (float)MICRO_STEPS);

    STEPS_PER_WORM_REV = (u32)((360.0f / (float)mStepDegree) * (float)MICRO_STEPS * GEAR_RATIO);
    STEPS_PER_RA_REV = STEPS_PER_WORM_REV * MOUNT_RATIO;

    SIDERAL_PERIOD_FREQ = ((float)MICRO_STEPS * SIDERAL_SPEED_DEGREE) /
                          ((float)mStepDegree / (GEAR_RATIO * (float)MOUNT_RATIO));
    SIDERAL_STEP_COUNT = (float)Defines::TIMER_FREQ / SIDERAL_PERIOD_FREQ;

    mSideralStepPeriod = (int)(SIDERAL_STEP_COUNT);
    if (mSideralStepPeriod <= 0) mSideralStepPeriod = 1;
    mStepPeriod = mSideralStepPeriod;
    mCurrentStepPeriod = mSideralStepPeriod;
    mCurrentStepFreq = (mCurrentStepPeriod != 0) ? (Defines::TIMER_FREQ / mCurrentStepPeriod) : 0;

	mMaxCount = degreeToPosition(pos +90.0f);
    mMinCount = degreeToPosition(pos -90.0f);
    mPosition = degreeToPosition(pos);
	
    // Log de información de configuración
	ESP_LOGI(TAG, "MICRO_STEPS = %lu", mMicroStepm);
    ESP_LOGI(TAG, "MOUNT_RATIO = %lu", MOUNT_RATIO);
    ESP_LOGI(TAG, "GEAR_RATIO_NUM = %.2f", GEAR_RATIO_NUM);
    ESP_LOGI(TAG, "GEAR_RATIO_DEN = %lu", GEAR_RATIO_DEN);
    ESP_LOGI(TAG, "GEAR_RATIO = %.2f", GEAR_RATIO);
    ESP_LOGI(TAG, "HIGH_SPEED_RATIO = %lu", HIGH_SPEED_RATIO);
    ESP_LOGI(TAG, "STEP_DEGREE = %.5f", STEP_DEGREE);
    ESP_LOGI(TAG, "STEPS_PER_WORM_REV = %lu", STEPS_PER_WORM_REV);
    ESP_LOGI(TAG, "STEPS_PER_RA_REV = %lu", STEPS_PER_RA_REV);
    ESP_LOGI(TAG, "SIDERAL_STEP_COUNT = %.2f", SIDERAL_STEP_COUNT);
    ESP_LOGI(TAG, "SIDERAL_PERIOD_FREQ = %.2f", SIDERAL_PERIOD_FREQ);
    ESP_LOGI(TAG, "TIMER_FREQ = %d", Defines::TIMER_FREQ); 

    // Configurar GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config_t step_dir_conf = io_conf;
    step_dir_conf.pin_bit_mask = ((uint64_t)1ULL << (uint64_t)mDirPin) |
                                 ((uint64_t)1ULL << (uint64_t)mStepPin);
    gpio_config(&step_dir_conf);

    gpio_config_t mode_conf = io_conf;
    mode_conf.pin_bit_mask = ((uint64_t)1ULL << (uint64_t)mMode[0]) |
                             ((uint64_t)1ULL << (uint64_t)mMode[1]) |
							 ((uint64_t)1ULL << (uint64_t)mMode[2]);
    gpio_config(&mode_conf);

	// ===================================================
	// 🔹 NUEVO: Configurar pines de autoguiado (entradas)
	// ===================================================
	gpio_config_t ag_conf = {};
	ag_conf.intr_type = GPIO_INTR_NEGEDGE;     // interrupción por flanco de bajada (pulso ST4 activo)
	ag_conf.mode = GPIO_MODE_INPUT;            // modo entrada
	ag_conf.pull_up_en = GPIO_PULLUP_ENABLE;   // pull-up interno, porque el ST4 cierra a GND
	ag_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	ag_conf.pin_bit_mask = ((uint64_t)1ULL << (uint64_t)mAGPlus) |
						   ((uint64_t)1ULL << (uint64_t)mAGMinus);
	gpio_config(&ag_conf);

	// Instalar servicio ISR (solo una vez en todo el programa)
	gpio_install_isr_service(0);

	// Asignar handlers a cada pin
	gpio_isr_handler_add(mAGPlus,  st4_isr_handler, this);
	gpio_isr_handler_add(mAGMinus, st4_isr_handler, this);

    // Estado inicial
    setDir(EDirection::CW);
    gpio_set_level(mStepPin, 0);
    mMicroStep = true;

    // Iniciar timer 100 ms
    const esp_timer_create_args_t timer_args = {
        .callback = &Motor::timerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "motor_100ms_timer"
    };
    esp_timer_create(&timer_args, &m100msecTimer);
    esp_timer_start_periodic(m100msecTimer, 100000);
}

Motor::~Motor() {
    if (m100msecTimer) {
        esp_timer_stop(m100msecTimer);
        esp_timer_delete(m100msecTimer);
    }
    stop();
}

void IRAM_ATTR st4_isr_handler(void* arg)
{
    Motor* motor = static_cast<Motor*>(arg);
    uint32_t gpio_num = gpio_get_level(motor->mAGPlus) ? motor->mAGPlus : motor->mAGMinus;

    if (gpio_num == motor->mAGPlus) {
        ESP_EARLY_LOGI("ST4", "Pulso AG+ detectado");
        // Aquí podrías aumentar temporalmente la velocidad hacia CW
    } 
    else if (gpio_num == motor->mAGMinus) {
        ESP_EARLY_LOGI("ST4", "Pulso AG- detectado");
        // Aquí podrías mover en sentido contrario
    }
}

void Motor::setMicroSteps(bool active) {
    // Siempre activo para ESP32-C6 (simplificado)
    if (HIGH_SPEED_RATIO == 1) {
        active = true; // misma velocidad alta y baja
    }
    if (mMicroStep != active) {
    }
}

int Motor::updateCurrentStepPeriod(u32 target_step) {

    int ret = target_step;

    if (mCurrentStepPeriod != target_step) {
        int current_step_freq = mCurrentStepFreq;
        int target_step_freq = (target_step != 0) ? (Defines::TIMER_FREQ / target_step) : INFINITE;
        int diff_freq = abs(current_step_freq - target_step_freq);

        
        // Solo aplicar aceleración si la diferencia es mayor que la frecuencia sideral
        if (diff_freq > SIDERAL_PERIOD_FREQ) {
            int block_freq = diff_freq / 2;
            if (block_freq < SIDERAL_PERIOD_FREQ) {
                block_freq = SIDERAL_PERIOD_FREQ;
            }

            if (current_step_freq < target_step_freq) {
                current_step_freq += block_freq;
                if (current_step_freq > target_step_freq) {
                    current_step_freq = target_step_freq;
                }
            } else if (current_step_freq > target_step_freq) {
                current_step_freq -= block_freq;
                if (current_step_freq < target_step_freq) {
                    current_step_freq = target_step_freq;
                }
            }
            ret = (current_step_freq != 0) ? (Defines::TIMER_FREQ / current_step_freq) : INFINITE;
            
            // Debido a la división entera, forzar aumento/disminución
            if (ret == mCurrentStepPeriod) {
                if (target_step < ret) {
                    ret--;
                } else {
                    ret++;
                }
            }
        }
    }
    if (ret < 4) {
        ret = 4; // mínimo 4 periodos por step
    }
    return ret;
}

void Motor::setStepPeriod(u32 period) {

	//ESP_LOGW(TAG, "4");

    u32 speed = (period != 0) ? (mSideralStepPeriod / period) : INFINITE;
    if (!mMicroStep) {
        speed *= HIGH_SPEED_RATIO;
    }

    // Limitar velocidad a mFastStepMult
    /*if (speed > mFastStepMult) {
        period = mSideralStepPeriod / mFastStepMult;
        if (!mMicroStep) {
            period *= HIGH_SPEED_RATIO;
        }
    }*/
    mStepPeriod = period;
}

void Motor::setDir(EDirection dir) {
    if (dir != mDir) {
        ESP_LOGI(TAG, "Motor[%d]: Direction: %s", (int)mAxis,
                dir == EDirection::CW ? "CW" : "CCW");
        mDir = dir;

        if (mDir == EDirection::CW) {
			gpio_set_level(mDirPin, 0);
        } else {
			gpio_set_level(mDirPin, 1);
        }
    }
}

int Motor::degreeToPosition(float degree) {
	ESP_LOGW("", "Entrada:%.2f Salida1: 0x%X (%u) Salida2: 0x%X (%u)", degree, 
	(int)(degree / DEGREE_PER_STEP), (int)(degree / DEGREE_PER_STEP),
	0x800000 + (int)(degree / DEGREE_PER_STEP), 0x800000 + (int)(degree / DEGREE_PER_STEP));
    return (int)(0x800000 + (int)(degree / DEGREE_PER_STEP));
}

void Motor::stop() {
    mMoving = false;
    gpio_set_level(mStepPin, 0);
    mStepCount = 0;
}

void Motor::on100msecTick() {
    if (mType == ESlewType::GOTO) {
        // Parar si el controlador no envía comandos (protección por fallo de comunicación)
        if (mCommStatusWD++ >= mCommStatusTimeout) {
            mToStop = true;
        }

        if (mToStop) {
            // Desacelerar a velocidad sideral y parar
            mCurrentStepPeriod = updateCurrentStepPeriod(mSideralStepPeriod);
            mCurrentStepFreq = (mCurrentStepPeriod != 0) ? (Defines::TIMER_FREQ / mCurrentStepPeriod) : INFINITE;
            if (mCurrentStepPeriod >= mSideralStepPeriod) {
                stop();
            }
        } else {
            if (mTargetPosition == INFINITE) {
                stop();
            }
            // Goto normal
            u32 remain = abs(mTargetPosition - mPosition) / 10;
            u32 elapsed = abs(mPosition - mOrigPosition) / 10;
            mult = MIN(remain, elapsed);
            mult = MIN(mFastStepMult, mult);
            mult = MIN(mSideralStepPeriod, mult);

            if (!mMicroStep) {
                mult /= HIGH_SPEED_RATIO;
            }
			
            if (mult > 1) {
                mCurrentStepPeriod = (mSideralStepPeriod / mult);
            } else {
                mCurrentStepPeriod = ((int)(SIDERAL_STEP_COUNT / 2.0f));
            }
            mCurrentStepFreq = (mCurrentStepPeriod != 0) ? (Defines::TIMER_FREQ / mCurrentStepPeriod) : INFINITE;
        }
    } else {
        // Tracking
        int sideral = mSideralStepPeriod;
        int target_period = mToStop ? sideral : mStepPeriod;
        mCurrentStepPeriod = updateCurrentStepPeriod(target_period);
        mCurrentStepFreq = (mCurrentStepPeriod != 0) ? Defines::TIMER_FREQ / mCurrentStepPeriod : INFINITE;

        if (mCurrentStepPeriod >= mSideralStepPeriod && mToStop) {
            stop();
        }
    }

    if (mIsPulseDone) {
        ESP_LOGI(TAG, "Motor[%d]::on100msecTick: Pulse guide done", (int)mAxis);
        mIsPulseDone = false;
    }
}

void IRAM_ATTR Motor::onTick() {
    bool pulse_moving = false;
    int current_step_period = mCurrentStepPeriod;
    EDirection current_dir = mDir;

    if (mPulseTotalTicks > 0 && ++mPulseCurrentTicks <= mPulseTotalTicks) {
        // Modificar periodo de step actual con pulse guide
        pulse_moving = true;
        if (mMoving) {
            int pulse_step_freq = mPulseStepFreq;
            if (mDir != mPulseDir) {
                pulse_step_freq *= -1;
            }
            int result_step_freq = mCurrentStepFreq + pulse_step_freq;
            if (result_step_freq < 0) {
                // Si la frecuencia es negativa, cambiar dirección
                current_dir = (current_dir == EDirection::CW) ? EDirection::CCW : EDirection::CW;
                result_step_freq = abs(result_step_freq);
            }
            current_step_period = (result_step_freq != 0) ? Defines::TIMER_FREQ / result_step_freq : INFINITE;
        } else {
            // Si el motor no se mueve, usar valores de pulse guide
            current_dir = mPulseDir;
            setDir(current_dir);
            current_step_period = (mPulseStepFreq != 0) ? Defines::TIMER_FREQ / mPulseStepFreq : INFINITE;
        }
        if (mPulseCurrentTicks == mPulseTotalTicks) {
            mIsPulseDone = true;
            mPulseTotalTicks = 0;
        }
    }

    if (mMoving || pulse_moving) {
        mStepCount++;

        u32 p2 = current_step_period / 2;
        /*if (choco==true){
            choco=false;
            ESP_LOGI(TAG, "Motor[%d]::onTick: StepCount=%d, CurrentStepPeriod=%d, P2=%d", (int)mAxis, mStepCount, current_step_period, p2);
        }*/
        
        if (mStepCount >= current_step_period) {
            mStepCount = 0;
            mStepDown = true;
            gpio_set_level(mStepPin, 0);

            u32 amount = mMicroStep ? 1 : HIGH_SPEED_RATIO;
            if (current_dir == EDirection::CCW) {
                amount = -amount;
            }

            mPosition += amount;

            if (mType == ESlewType::GOTO) {
                if (current_dir == EDirection::CW) {
                    if (mPosition >= mTargetPosition) {
                        stop();
                    }
                } else {
                    if (mPosition <= mTargetPosition) {
                        stop();
                    }
                }
            }
        } else if (mStepCount >= p2 && mStepDown) {
            mStepDown = false;
            gpio_set_level(mStepPin, 1);
            
        }
    }
}

void Motor::printInfo() const {
    u32 speed = mSideralStepPeriod / mCurrentStepPeriod;
    if (!mMicroStep) {
        speed *= HIGH_SPEED_RATIO;
    }

	//ESP_LOGW(TAG, "Motor[%d] Remain: %d", (int)mAxis, (mTargetPosition != INFINITE) ? abs(mTargetPosition - mPosition) : 0);
		ESP_LOGI(TAG, 
            "Motor[%d]: remain: %d, elapsed: %d, mult: %u, step period: %d step freq: %d, position: %d, target: %d",
            (int)mAxis, abs(mTargetPosition - mPosition), abs(mPosition - mOrigPosition), mult, mCurrentStepPeriod, mCurrentStepFreq,
            mPosition, mTargetPosition);
    
}

void Motor::timerCallback(void* arg) {
    Motor* motor = static_cast<Motor*>(arg);
    if (motor) {
        motor->on100msecTick();
    }
}

Reply* Motor::processCommand(const Command* cmd) {
    Reply* reply = NULL;
    mCommStatusWD = 0;
    
    switch (cmd->getCmd()) {
        case ECommand::GET_VERSION: {
		VersionReply* version_reply = new VersionReply();
		version_reply->setVersion(VERSION_MAJOR, VERSION_MINOR, VERSION_MICRO,
				VERSION_PATCH);
		reply = version_reply;
		break;
	}
	case ECommand::PULSE_GUIDE: {
		PulseGuideCmd* pulse_cmd = (PulseGuideCmd*)cmd;
		ESP_LOGI(TAG, "Motor[%d]: Pulse guide: dir = %d, msec = %d, rate(x10) = %d",
				mAxis, pulse_cmd->GetDir(), pulse_cmd->GetDuration(), (int)(pulse_cmd->GetRate()*10.0f));
		mPulseDir = pulse_cmd->GetDir();
		mPulseStepFreq = (int)((float)(Defines::TIMER_FREQ/(int)(mSideralStepPeriod))*pulse_cmd->GetRate());
		mPulseCurrentTicks = 0;
		mPulseTotalTicks = (pulse_cmd->GetDuration()*Defines::TIMER_FREQ)/1000;
		break;
	}
	case ECommand::SET_POSITION: {
		ESP_LOGI(TAG, "Motor[%d]: SET_POSITION", mAxis);
		if (!mMoving) {
			SetPositionCommand* pos_cmd = (SetPositionCommand*) cmd;
			mPosition = pos_cmd->GetPosition();
			ESP_LOGI(TAG, "Motor[%d]: position = %d [%X]", mAxis, mPosition,
					mPosition);
			reply = new EmptyReply();
		} else {
			ESP_LOGE(TAG, "Motor[%d]: Motor not stopped", mAxis);
			reply = new ErrorReply(ErrorReply::EErrorCode::Motor_not_Stopped);
		}
		break;
	}
	case ECommand::INIT_DONE: {
		ESP_LOGI(TAG, "Motor[%d]: INIT_DONE", mAxis);
		reply = new EmptyReply();
		break;
	}
	case ECommand::SET_MOTION_MODE: {
		ESP_LOGI(TAG, "Motor[%d]: SET_MOTION_MODE", mAxis);
		if (!mMoving) {
			SetMotionMode* motion_mode = (SetMotionMode*) cmd;
			mSpeed = motion_mode->getSpeed();
			mType = motion_mode->getType();
			setDir(motion_mode->getDir());
			ESP_LOGI(TAG, "Motor[%d]: slew = %s, speed = %s, dir = %s", mAxis,
					mType == ESlewType::GOTO ? "GOTO" : "TRACK",
					mSpeed == ESpeed::FAST ? "FAST" : "SLOW",
					mDir == EDirection::CW ? "CW" : "CCW");
			reply = new EmptyReply();
		} else {
			ESP_LOGE(TAG, "Motor[%d]: Motor not stopped", mAxis);
			reply = new ErrorReply(ErrorReply::EErrorCode::Motor_not_Stopped);
		}
		break;
	}
	case ECommand::SET_GOTO_TARGET: {
		ESP_LOGI(TAG, "Motor[%d]: SET_GOTO_TARGET", mAxis);
		if (!mMoving) {
			SetGotoTarget* target = (SetGotoTarget*) cmd;
			mOrigPosition = mPosition;
			mTargetPosition = target->GetPosition();
			ESP_LOGI(TAG, "Motor[%d]: target = %d [%X]", mTargetPosition,
					mTargetPosition);
			reply = new EmptyReply();
		} else {
			ESP_LOGE(TAG, "Motor[%d]: Motor not stopped", mAxis);
			reply = new ErrorReply(ErrorReply::EErrorCode::Motor_not_Stopped);
		}
		break;
	}
	case ECommand::SET_GOTO_TARGET_INCREMENT: {
		ESP_LOGI(TAG, "Motor[%d]: SET_GOTO_TARGET_INCREMENT", mAxis);
		if (!mMoving) {
			SetGotoTargetIncrement* target = (SetGotoTargetIncrement*) cmd;
			mOrigPosition = mPosition;
			int increment = target->GetIncrement();
			if (mDir == EDirection::CW) {
				mTargetPosition = mPosition + increment;
			} else {
				mTargetPosition = mPosition - increment;
			}
			ESP_LOGI(TAG, 
					"Motor[%d]: position = %d, increment = %d, target = %d",
					mAxis, mPosition, increment, mTargetPosition);
			reply = new EmptyReply();
		} else {
			ESP_LOGE(TAG, "Motor[%d]: Motor not stopped", mAxis);
			reply = new ErrorReply(ErrorReply::EErrorCode::Motor_not_Stopped);
		}
		break;
	}
	case ECommand::SET_BREAK_POINT_INCREMENT: {
		ESP_LOGI(TAG, "Motor[%d]: SET_BREAK_POINT_INCREMENT", mAxis);
		if (!mMoving) {
			SetBreakPointIncrement* target = (SetBreakPointIncrement*) cmd;
			mBreakPosition = target->GetIncrement();
			ESP_LOGI(TAG, "Motor[%d]: break point = %d", mAxis,
					mBreakPosition);
			reply = new EmptyReply();
		} else {
			ESP_LOGE(TAG, "Motor[%d]: Motor not stopped", mAxis);
			reply = new ErrorReply(ErrorReply::EErrorCode::Motor_not_Stopped);
		}
		break;
	}
	case ECommand::GET_GOTO_TARGET: {
		ESP_LOGI(TAG, "Motor[%d]: GET_GOTO_TARGET", mAxis);
		PositionReply* position_reply = new PositionReply();
		position_reply->setData(mTargetPosition, 6);
		reply = position_reply;
		break;
	}
	case ECommand::SET_STEP_PERIOD: {
		ESP_LOGI(TAG, "Motor[%d]: SET_STEP_PERIOD", mAxis);
		SetStepPeriod* step_period = (SetStepPeriod*) cmd;
		//ESP_LOGW(TAG, "3");
		setStepPeriod(step_period->GetPeriod());
		ESP_LOGI(TAG, "Motor[%d]: Step period = %d", mAxis, mStepPeriod);
		reply = new EmptyReply();
		break;
	}
	case ECommand::GET_STEP_PERIOD: {
		ESP_LOGI(TAG, "Motor[%d]: GET_STEP_PERIOD: %d", mAxis, mStepPeriod);
		DataReply* step_reply = new DataReply();
		step_reply->setData(mStepPeriod, 6);
		reply = step_reply;
		break;
	}
	case ECommand::START_MOTION: {
		ESP_LOGI(TAG, "Motor[%d]: START_MOTION!", mAxis);
		switch (mSpeed) {
		case ESpeed::SLOW:
			setMicroSteps(true);
			break;
		case ESpeed::FAST:
			setMicroSteps(false);
			break;
		default: {
			ESP_LOGW(TAG, "Motor[%d]: Speed not set, using slow speed",
					mAxis);
			setMicroSteps(true);
			break;
		}
		}
		if (mType != ESlewType::NONE) {
			mCurrentStepPeriod = mSideralStepPeriod;
			mCurrentStepFreq = Defines::TIMER_FREQ/mCurrentStepPeriod;
			mToStop = false;
			mMoving = true;
			reply = new EmptyReply();
		} else {
			reply = new ErrorReply(ErrorReply::EErrorCode::Not_Initialized);
		}
		break;
	}
	case ECommand::STOP_MOTION: {
		ESP_LOGI(TAG, "Motor[%d]: STOP_MOTION", mAxis);
		mToStop = true;
		reply = new EmptyReply();
		break;
	}
	case ECommand::INSTANT_STOP: {
		ESP_LOGW(TAG, "Motor[%d]: INSTANT_STOP", mAxis);
		stop();
		break;
	}
	case ECommand::SET_SWITCH_ONOFF: {
		ESP_LOGI(TAG, "Motor[%d]: SET_SWITCH_ONOFF", mAxis);
		//Ignore
		reply = new EmptyReply();
		break;
	}
	case ECommand::SET_AUTOGUIDE_SPEED: {
		ESP_LOGI(TAG, "Motor[%d]: SET_AUTOGUIDE_SPEED", mAxis);
		SetAutoGuideSpeed* auto_cmd = (SetAutoGuideSpeed*) cmd;
		mAutoGuideMode = auto_cmd->getSpeed();
		ESP_LOGI(TAG, "Motor[%d]: Autoguide speed: %d/1000", mAxis,
				mAutoGuideMode);
		reply = new EmptyReply();
		break;
	}
	case ECommand::RUN_BOOTLOADER_MODE: {
		ESP_LOGI(TAG, "Motor[%d]: RUN_BOOTLOADER_MODE", mAxis);
		ErrorReply* error_reply = new ErrorReply();
		error_reply->setError(ErrorReply::EErrorCode::Unknown_Command);
		reply = error_reply;
		break;
	}
	case ECommand::SET_POLAR_SCOPE_LED_BRIGHTNESS: {
		ESP_LOGI(TAG, "Motor[%d]: SET_POLAR_SCOPE_LED_BRIGHTNESS", mAxis);
		//Ignore
		reply = new EmptyReply();
		break;
	}
	case ECommand::GET_PEC_PERIOD: {
		ESP_LOGI(TAG, "Motor[%d]: GET_PEC_PERIOD", mAxis);
		DataReply* data_reply = new DataReply();
		data_reply->setData(mPecPeriod, 6);
		reply = data_reply;
		break;
	}
	case ECommand::GET_COUNTS_PER_REVOLUTION: {
		ESP_LOGI(TAG, "Motor[%d]: GET_COUNTS_PER_REVOLUTION", mAxis);
		DataReply* data_reply = new DataReply();
		data_reply->setData(STEPS_PER_RA_REV, 6);
		reply = data_reply;
		break;
	}
	case ECommand::GET_TIMER_FREQ: {
		ESP_LOGI(TAG, "Motor[%d]: GET_TIMER_FREQ", mAxis);
		DataReply* data_reply = new DataReply();
		data_reply->setData(Defines::TIMER_FREQ, 6);
		reply = data_reply;
		break;
	}
	case ECommand::GET_POSITION: {
		PositionReply* position_reply = new PositionReply();
		position_reply->setData(mPosition, 6);
		reply = position_reply;
		break;
	}
	case ECommand::GET_STATUS: {
		StatusReply* status_reply = new StatusReply();
		status_reply->setInitDone(true);
		status_reply->setBlocked(false);
		status_reply->setRunning(mMoving);
		status_reply->setSlewMode(mType);
		status_reply->setSpeedMode(mSpeed);
		status_reply->setDirection(mDir);
		reply = status_reply;
		break;
	}
	case ECommand::GET_HIGH_SPEED_RATIO: {
		ESP_LOGI(TAG, "Motor[%d]: GET_HIGH_SPEED_RATIO", mAxis);
		DataReply* data_reply = new DataReply();
		data_reply->setData(HIGH_SPEED_RATIO, 2);
		reply = data_reply;
		break;
	}
	case ECommand::GET_SIDERAL_PERIOD: {
		ESP_LOGI(TAG, "Motor[%d]: GET_SIDERAL_PERIOD: %d ticks", mAxis,
				mSideralStepPeriod);
		DataReply* data_reply = new DataReply();
		data_reply->setData(mSideralStepPeriod, 6);
		reply = data_reply;
		break;
	}
	case ECommand::GET_AXIS_POSITION: {
		ESP_LOGI(TAG, "Motor[%d]: GET_AXIS_POSITION", mAxis);
		PositionReply* position_reply = new PositionReply();
		position_reply->setData(mPosition, 6);
		reply = position_reply;
		break;
	}
	case ECommand::EXT_SET: {
		ESP_LOGI(TAG, "Motor[%d]: EXT_SET", mAxis);
		ErrorReply* error_reply = new ErrorReply();
		error_reply->setError(ErrorReply::EErrorCode::Unknown_Command);
		reply = error_reply;
		break;
	}
	case ECommand::EXT_GET: {
		ESP_LOGI(TAG, "Motor[%d]: EXT_GET", mAxis);
		GetExtended* ext = (GetExtended*) cmd;
		if (ext->getType() == GetExtended::EType::STATUS_EX) {
			StatusExReply* status_reply = new StatusExReply();
			status_reply->setDualEncSupport(false);
			status_reply->setEQAZModeSupport(false);
			status_reply->setHasPolarLed(false);
			status_reply->setOriginalIdxPosSupport(false);
			status_reply->setPPECSupport(false);
			status_reply->setPecTracking(false);
			status_reply->setPecTraining(false);
			status_reply->setTorqueSelectionSupport(false);
			status_reply->setTwoAxesSeparate(false);
			reply = status_reply;
		} else {
			ErrorReply* error_reply = new ErrorReply();
			error_reply->setError(ErrorReply::EErrorCode::Unknown_Command);
			reply = error_reply;
		}
		break;
	}
        // ... (continuar con el resto de los casos del switch)
        // El resto del switch permanece igual que en el código original
        // Solo necesitas cambiar las llamadas a Serial.printf por Logger::notice
        
        default: {
            ErrorReply* error_reply = new ErrorReply();
            error_reply->setError(ErrorReply::EErrorCode::Unknown_Command);
            reply = error_reply;
            break;
        }
    }

    return reply;
}
