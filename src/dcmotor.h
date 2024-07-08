#pragma once

#include "esp_timer.h"
#include "hal/ledc_types.h"
#include "hal/pcnt_types.h"
#include "soc/gpio_num.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>


class QEncoder {
    std::atomic<int64_t> _accum = 0;
    pcnt_unit_handle_t _unit = nullptr;
    pcnt_channel_handle_t _channel = nullptr;
public:
    QEncoder(gpio_num_t encA, gpio_num_t encB) {
        constexpr auto LOW = std::numeric_limits<int16_t>::min();
        constexpr auto HIGH = std::numeric_limits<int16_t>::max();

        pcnt_unit_config_t unitConfig = {
            .low_limit = LOW,
            .high_limit = HIGH,
            .intr_priority = 0,
            .flags = { 0 }
        };
        auto res = pcnt_new_unit(&unitConfig, &_unit);
        if (res != ESP_OK) {
            throw std::runtime_error("Failed to create PCNT unit");
        }

        pcnt_chan_config_t channelConfig = {
            .edge_gpio_num = encA,
            .level_gpio_num = encB,
            .flags = { }
        };
        channelConfig.flags.invert_edge_input = 0;
        channelConfig.flags.invert_level_input = 0;
        channelConfig.flags.virt_edge_io_level = 0;
        channelConfig.flags.virt_level_io_level = 0;
        channelConfig.flags.io_loop_back = 0;

        res = pcnt_new_channel(_unit, &channelConfig, &_channel);
        if (res != ESP_OK) {
            throw std::runtime_error("Failed to create PCNT channel");
        }

        res = pcnt_channel_set_edge_action(_channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        if (res != ESP_OK) {
            throw std::runtime_error("Failed to configure pulse counter channel edge action");
        }

        res = pcnt_channel_set_level_action(_channel, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
        if (res != ESP_OK) {
            throw std::runtime_error("Failed to configure pulse counter channel level action");
        }

        pcnt_event_callbacks_t callbacks = {
            .on_reach = [](pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) -> bool {
                auto& encoder = *static_cast<QEncoder*>(user_ctx);
                encoder._accum = encoder._accum.load() + edata->watch_point_value;
                return false;
            }
        };
        res = pcnt_unit_add_watch_point(_unit, LOW);
        if (res != ESP_OK) {
            throw std::runtime_error("Failed to add pulse counter watch point");
        }
        res = pcnt_unit_add_watch_point(_unit, HIGH);
        if (res != ESP_OK) {
            throw std::runtime_error("Failed to add pulse counter watch point");
        }

        res = pcnt_unit_register_event_callbacks(_unit, &callbacks, this);
        if (res != ESP_OK) {
            throw std::runtime_error("Failed to register pulse counter event callbacks");
        }

        res = pcnt_unit_enable(_unit);
        if (res != ESP_OK) {
            throw std::runtime_error("Failed to enable pulse counter unit");
        }

        res = pcnt_unit_start(_unit);
        if (res != ESP_OK) {
            throw std::runtime_error("Failed to start pulse counter channel");
        }
    }

    QEncoder(const QEncoder&) = delete;
    QEncoder& operator=(const QEncoder&) = delete;
    QEncoder(QEncoder&& other) = delete;
    QEncoder& operator=(QEncoder&& other) = delete;

    ~QEncoder() {
        if (_channel) {
            pcnt_del_channel(_channel);
        }

        if (_unit) {
            pcnt_unit_disable(_unit);
            pcnt_del_unit(_unit);
        }
    }

    int64_t getPos() {
        int count = 0;
        auto res = pcnt_unit_get_count(_unit, &count);
        if (res != ESP_OK) {
            throw std::runtime_error("Failed to get pulse counter count");
        }

        return _accum.load() + count;
    }
};


class DCMotor {
    QEncoder _enc;
    const int _regP;

    const ledc_channel_t _channelA;
    const ledc_channel_t _channelB;

    std::atomic<int> _maxSpeed;

    std::atomic<int64_t> _actualTargetPos;  // ticks * 1024

    std::atomic<int64_t> _endTargetPos;  // ticks * 1024
    std::atomic<int64_t> _endTargetTime;  // ms
    std::atomic<bool> _handled = false;
    std::atomic<unsigned> _endPosTolerance = 1 << 10;

    std::function<void()> _onTarget;

    enum Mode {
        INFINITE,
        TIME,
        POSITION,
        FREE,
        BREAK
    };
    Mode mode = FREE;

    void setOutput(int power) {
        if (power < 0) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelA, -power);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelB, 0);
        }
        else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelA, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelB, power);
        }
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelA);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelB);
    }

    void callHandler() {
        _handled = true;
        if (_onTarget) {
            _onTarget();
        }
    }

public:
    DCMotor(gpio_num_t motA, gpio_num_t motB, gpio_num_t encA, gpio_num_t encB, int regP, ledc_timer_t timer, ledc_channel_t channelA, ledc_channel_t channelB):
        _enc(encA, encB), _regP(regP), _channelA(channelA), _channelB(channelB), _maxSpeed(0), _actualTargetPos(0)
    {
        ledc_channel_config_t confA = {
            .gpio_num = motA,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = channelA,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = timer,
            .duty = 0,
            .hpoint = 0,
            .flags = { 0 }
        };
        ledc_channel_config(&confA);

        ledc_channel_config_t confB = {
            .gpio_num = motB,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = channelB,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = timer,
            .duty = 0,
            .hpoint = 0,
            .flags = { 0 }
        };
        ledc_channel_config(&confB);
    }

    ~DCMotor() {
        ledc_stop(LEDC_LOW_SPEED_MODE, _channelA, 0);
        ledc_stop(LEDC_LOW_SPEED_MODE, _channelB, 0);
    }

    DCMotor(const DCMotor&) = delete;
    DCMotor& operator=(const DCMotor&) = delete;
    DCMotor(DCMotor&&) = delete;
    DCMotor& operator=(DCMotor&&) = delete;

    void tick() {
        if (mode == BREAK || mode == FREE) {
            return;
        }
        if (mode == TIME && esp_timer_get_time() > _endTargetTime) {
            stop(true);
            callHandler();
            return;
        }

        int64_t pos = _enc.getPos() << 10;
        if (mode == TIME || mode == INFINITE) {
            _actualTargetPos += _maxSpeed;
        }
        else if (mode == POSITION) {
            int absSpeed = std::abs(_maxSpeed);
            int actualSpeed = std::clamp((_endTargetPos - pos) >> 8, static_cast<int64_t>(-absSpeed), static_cast<int64_t>(absSpeed));
            _actualTargetPos += actualSpeed;
            if (!_handled && std::abs(_endTargetPos - pos) < _endPosTolerance) {
                callHandler();
            }
        }

        int64_t error = (_actualTargetPos - pos);
        error = error * _regP >> 10;
        int power = std::clamp(error, static_cast<int64_t>(-1024), static_cast<int64_t>(1024));

        setOutput(power);
    }

    // ticks/second
    void setSpeed(int speed) {
        _maxSpeed = speed;
    }

    void setEndPosTolerance(unsigned tolerance) {
        _endPosTolerance = tolerance << 10;
    }

    void moveInfinite() {
        mode = INFINITE;
        _actualTargetPos = _enc.getPos() << 10;
        _handled = false;
    }

    void moveTime(int64_t time) {
        mode = TIME;
        _actualTargetPos = _enc.getPos() << 10;
        _endTargetTime = esp_timer_get_time() + time * 1000;
        _handled = false;
    }

    void moveDistance(int64_t delta) {
        mode = POSITION;
        _actualTargetPos = _enc.getPos() << 10;
        _endTargetPos = _actualTargetPos + (delta << 10);
        _handled = false;
    }

    void stop(bool brake) {
        _endTargetPos = _enc.getPos() << 10;
        _actualTargetPos = _endTargetPos.load();

        mode = brake ? BREAK : FREE;
        if (brake) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelA, 1024);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelB, 1024);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelA);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelB);
        }
        else {
            setOutput(0);
        }

        if (!_handled) {
            callHandler();
        }
    }

    // called in interrupt context, must not call onTarget from callback
    void onTarget(std::function<void()> callback) {
        _onTarget = callback;
    }

    esp_timer_handle_t _tmr;

    void startTicker() {
        esp_timer_create_args_t tmrArgs = {
            .callback = [](void* arg) {
                auto& motor = *static_cast<DCMotor*>(arg);
                motor.tick();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_ISR,
            .name = "DCMotorTick",
            .skip_unhandled_events = true
        };

        esp_timer_create(&tmrArgs, &_tmr);
        esp_timer_start_periodic(_tmr, 1024);
    }

    void stopTicker() {
        esp_timer_stop(_tmr);
        esp_timer_delete(_tmr);
    }

    int64_t getPosition() {
        return _enc.getPos();
    }
};
