#pragma once

#include "esp_timer.h"
#include "hal/ledc_types.h"
#include "hal/pcnt_types.h"
#include "soc/gpio_num.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <type_traits>


// XXX: use C23's ckd_mul after moving to newer GCC
template<typename T>
bool ckd_mul(T* res, auto a, auto b) {
    return __builtin_mul_overflow(a, b, res);
}


static inline constexpr int TMR_FREQ_EXP = CONFIG_DCMOTOR_CONTROL_FREQ_EXP;
static inline constexpr uint32_t TMR_FREQ = 1 << TMR_FREQ_EXP;
static inline constexpr uint32_t TMR_PERIOD_US = 1000000 / TMR_FREQ;

static inline constexpr int REG_P_EXP = CONFIG_DCMOTOR_KP_EXP + TMR_FREQ_EXP;
static inline constexpr int REG_I_EXP = CONFIG_DCMOTOR_KI_EXP + 2 * TMR_FREQ_EXP;
static inline constexpr int REG_D_EXP = CONFIG_DCMOTOR_KD_EXP;
static inline constexpr int REG_V_EXP = CONFIG_DCMOTOR_KV_EXP + TMR_FREQ_EXP;
static inline constexpr int REG_A_EXP = CONFIG_DCMOTOR_KA_EXP;

static inline constexpr int ERR_FILTER_EXP = CONFIG_DCMOTOR_FILTER_EXP;
static inline constexpr int ERR_FILTER_COEFF = CONFIG_DCMOTOR_FILTER_COEFF;
static inline constexpr int ERR_FILTER_COEFF_INV = (1 << ERR_FILTER_EXP) - ERR_FILTER_COEFF;


class QEncoder {
    std::atomic<int64_t> _accum = 0;
    pcnt_unit_handle_t _unit = nullptr;
    pcnt_channel_handle_t _channel = nullptr;
    gpio_num_t _encA;
    gpio_num_t _encB;
public:
    QEncoder(gpio_num_t encA, gpio_num_t encB): _encA(encA), _encB(encB) {
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

        bool l1 = gpio_get_level(_encA);
        bool l2 = gpio_get_level(_encB);

        if (res != ESP_OK) {
            throw std::runtime_error("Failed to get pulse counter count");
        }

        return (_accum.load() + count) * 2 + (l1 == l2 ? 1 : 0);
    }
};


struct RegParams {
    int kp;
    int ki;
    int kd;
    int kv;
    int ka;
    int kc; // applied when speed is non-zero
    unsigned maxIOut;  // 0 for no limit
    unsigned unwindFactor = 1;
};


template<typename T, typename U, typename Res = decltype(std::declval<T>() * std::declval<U>())>
static inline auto mulclamp(T a, U b, Res min = std::numeric_limits<Res>::min(), Res max = std::numeric_limits<Res>::max()) {
    if (a == 0 || b == 0) {
        return static_cast<Res>(0);
    }

    Res res;
    if (ckd_mul(&res, a, b)) {
        return std::signbit(a) != std::signbit(b) ? min : max;
    }
    if (res < min) {
        return min;
    }
    if (res > max) {
        return max;
    }

    return res;
}

static inline auto mulclampsymexp(auto a, auto b, auto max, auto exp) {
    using Res = decltype(a * b);
    static_assert(std::is_signed_v<Res>);

    return mulclamp(a, b, -static_cast<Res>(max) << exp, static_cast<Res>(max) << exp) >> exp;
}

class DCMotor {
    enum Mode {
        INFINITE,
        TIME,
        POSITION,
        FREE,
        BREAK,
        RAW
    };

    QEncoder _enc;
    RegParams _reg;
    const ledc_channel_t _channelA;
    const ledc_channel_t _channelB;

    std::atomic<int> _maxSpeed = 0 << TMR_FREQ_EXP;  // ticks/second * TMR_FREQ
    std::atomic<int> _ramp = 0;  // ticks/second^2; 0 for instant speed change
    std::atomic<int> _power = 0;  // used for raw power mode
    std::atomic<unsigned> _endPosTolerance = 1;

    struct MotorState {
        Mode mode = FREE;

        int vwSpeed = 0;  // ticks/second * TMR_FREQ; ramp-adjusted speed of virtual wheel
        int64_t vwPos = 0;  // virtual wheel POSITION

        uint64_t endTargetPos = 0;  // ticks * TMR_FREQ
        uint64_t endTargetTime = 0;  // us

        bool handled = false;  // whether the target has been reached and the handler called

        int64_t errorSum = 0;
        int lastError = 0;
        int lastDerivative = 0;
    };
    std::atomic<std::shared_ptr<std::function<void()>>> _onTarget = nullptr;

    std::atomic<bool> _reset = false;
    std::atomic<std::shared_ptr<MotorState>> _state = std::make_shared<MotorState>();

    using Reporter = std::function<void(uint64_t time, int64_t pos, int64_t vwTarget, int vwSpeed, int error, int power, int pOut, int iOut, int dOut, int vOut, int aOut)>;
    std::atomic<std::shared_ptr<Reporter>> _reporter = nullptr;

    void setOutput(int power) {
        if (power < 0) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelA, 1023);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelB, 1023 + power);
        }
        else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelA, 1023 - power);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelB, 1023);
        }
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelA);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelB);
    }

    void applyStop(bool brake) {
        if (brake) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelA, 1023);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelB, 1023);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelA);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelB);
        }
        else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelA, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelB, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelA);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelB);
        }

        if (!_state.load()->handled) {
            callHandler();
        }
    }

    void callHandler() {
        _state.load()->handled = true;
        auto ot = _onTarget.load();
        if (ot) {
            (*ot)();
        }
    }

    void tick() {
        auto stPtr = _state.load();
        auto& st = *stPtr;

        if (_reset.exchange(false)) {
            st.errorSum = 0;
            st.lastError = 0;
            st.lastDerivative = 0;
            if (st.mode == BREAK || st.mode == FREE) {
                applyStop(st.mode == BREAK);
            }
            else if (st.mode == RAW) {
                setOutput(_power);
            }
        }
        if (st.mode == BREAK || st.mode == FREE || st.mode == RAW) {
            auto reporter = _reporter.load();
            if (reporter) {
                (*reporter)(esp_timer_get_time(), _enc.getPos() << TMR_FREQ_EXP, 0, 0, 0, _power, 0, 0, 0, 0, 0);
            }
            return;
        }

        int64_t pos = _enc.getPos();
        int dSpeed = 0;
        if (st.mode == INFINITE) {
            int tgtSpeed = _maxSpeed.load();  // avoid atomic operations later
            int ramp = _ramp.load();
            if (ramp == 0) {
                dSpeed = tgtSpeed - st.vwSpeed;
            }
            else if (st.vwSpeed < tgtSpeed) {
                dSpeed = std::min(static_cast<int>(+ramp), tgtSpeed - st.vwSpeed);
            }
            else if (st.vwSpeed > tgtSpeed) {
                dSpeed = std::max(static_cast<int>(-ramp), tgtSpeed - st.vwSpeed);
            }
            st.vwSpeed += dSpeed;
            st.vwPos += st.vwSpeed >> TMR_FREQ_EXP;
        }
        else if (st.mode == TIME) {
            uint64_t time = esp_timer_get_time();
            if (time > st.endTargetTime) {
                stop(true);
                callHandler();
                return;
            }

            int ramp = _ramp.load();  // avoid atomic operations later
            if (ramp == 0) {
                dSpeed = _maxSpeed - st.vwSpeed;
            }
            else if (((st.endTargetTime - time) * ramp) > (std::abs(st.vwSpeed) * TMR_PERIOD_US)) {
                int tgtSpeed = _maxSpeed;
                if (st.vwSpeed < tgtSpeed) {
                    dSpeed = std::min(static_cast<int>(+ramp), tgtSpeed - st.vwSpeed);
                }
                else if (st.vwSpeed > tgtSpeed) {
                    dSpeed = std::max(static_cast<int>(-ramp), tgtSpeed - st.vwSpeed);
                }
            }
            else {
                if (std::abs(st.vwSpeed) < 2 * ramp) {
                    dSpeed = -st.vwSpeed;
                }
                else if (st.vwSpeed < 0) {
                    dSpeed = +ramp;
                }
                else {
                    dSpeed = -ramp;
                }
            }
            st.vwSpeed += dSpeed;
            st.vwPos += st.vwSpeed >> TMR_FREQ_EXP;
        }
        else if (st.mode == POSITION) {
            int64_t endDist = st.endTargetPos - st.vwPos;
            int tgtSpeed = std::abs(_maxSpeed.load());
            if (endDist < 0) {
                endDist = -endDist;
                tgtSpeed = -tgtSpeed;
            }
            int ramp = _ramp.load();  // avoid atomic operations later
            int vwSpeedNorm = std::abs(st.vwSpeed) + ramp;

            if (ramp == 0) {  // infinite
                if (std::abs(endDist) <= (std::abs(st.vwSpeed) >> TMR_FREQ_EXP)) {
                    dSpeed = -st.vwSpeed;
                }
                else {
                    dSpeed = tgtSpeed - st.vwSpeed;
                }
            }
            else if ((endDist * (2 * ramp)) <= ((static_cast<int64_t>(vwSpeedNorm) * vwSpeedNorm) >> TMR_FREQ_EXP)) {  // s = v^2 / 2a
                // ramp-down
                if (std::abs(st.vwSpeed) < 2 * ramp || (st.vwSpeed >> TMR_FREQ_EXP) == 0) {
                    // snap to zero when speed is close
                    dSpeed = -st.vwSpeed;
                }
                else if (st.vwSpeed < 0) {
                    dSpeed = +ramp;
                }
                else {
                    dSpeed = -ramp;
                }
            }
            else {
                // ramp-up
                if (st.vwSpeed < tgtSpeed) {
                    dSpeed = std::min(static_cast<int>(ramp), tgtSpeed - st.vwSpeed);
                }
                else if (st.vwSpeed > tgtSpeed) {
                    dSpeed = std::max(static_cast<int>(-ramp), tgtSpeed - st.vwSpeed);
                }
            }

            st.vwSpeed += dSpeed;

            if (st.vwSpeed == 0) {
                st.vwPos = st.endTargetPos;
            }
            else {
                // should never overshoot target because of the above condition
                st.vwPos += st.vwSpeed >> TMR_FREQ_EXP;
            }

            if (!st.handled && st.vwSpeed == 0 && std::abs(static_cast<int64_t>((st.endTargetPos >> TMR_FREQ_EXP) - pos)) < _endPosTolerance) {
                callHandler();
            }
        }

        int64_t error64 = st.vwPos - (pos << TMR_FREQ_EXP);
        int error = std::clamp(error64, static_cast<int64_t>(std::numeric_limits<int>::min() / 2), static_cast<int64_t>(std::numeric_limits<int>::max() / 2));

        // DERIVATIVE
        int oldLastError = st.lastError;

        st.lastError = (static_cast<int64_t>(oldLastError) * ERR_FILTER_COEFF + static_cast<int64_t>(error) * ERR_FILTER_COEFF_INV) >> ERR_FILTER_EXP;
        int dErr = st.lastError - oldLastError;

        st.lastDerivative = (static_cast<int64_t>(st.lastDerivative) * ERR_FILTER_COEFF + ERR_FILTER_COEFF_INV * dErr) >> ERR_FILTER_EXP;

        // INTEGRAL
        int64_t maxErrorSum = (_reg.ki != 0 && _reg.maxIOut != 0) ?
            (static_cast<int64_t>(_reg.maxIOut) << REG_I_EXP) / std::abs(_reg.ki)
          : std::numeric_limits<decltype(st.errorSum)>::max() >> 1;

        if (std::signbit(st.lastError) != std::signbit(st.errorSum) && st.errorSum != 0) {
            // prevent integral windup
            int step = error * (_reg.unwindFactor + 1);
            if (std::abs(st.errorSum) < std::abs(step)) {
                st.errorSum = 0;
            }
            else {
                st.errorSum += step;
            }
        }
        else {
            st.errorSum = std::clamp(st.errorSum + error, -maxErrorSum, maxErrorSum);
        }

        int pout = mulclampsymexp(static_cast<int>(st.lastError), static_cast<int>(_reg.kp), 1023, REG_P_EXP);
        int iout = mulclampsymexp(st.errorSum, _reg.ki, 1023, REG_I_EXP);
        int dout = mulclampsymexp(st.lastDerivative, _reg.kd, 1023, REG_D_EXP);
        int vout = mulclampsymexp(st.vwSpeed, _reg.kv, 1023, REG_V_EXP);
        int cout = st.vwSpeed != 0 ? (st.vwSpeed > 0 ? _reg.kc : -_reg.kc) : 0;
        int aout = mulclampsymexp(dSpeed, _reg.ka, 1023, REG_A_EXP);

        int out = pout + iout + dout + vout + cout + aout;
        int power = std::clamp(out, -1023, 1023);

        setOutput(power);

        auto reporter = _reporter.load();
        if (reporter) {
            (*reporter)(
                esp_timer_get_time(), pos << TMR_FREQ_EXP, st.vwPos, st.vwSpeed, error, power,
                pout, iout, dout, vout, aout
            );
        }
    }
public:
    DCMotor(gpio_num_t motA, gpio_num_t motB, gpio_num_t encA, gpio_num_t encB, RegParams regParams, ledc_timer_t timer, ledc_channel_t channelA, ledc_channel_t channelB):
        _enc(encA, encB), _reg(regParams), _channelA(channelA), _channelB(channelB)
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

    // ticks/second
    void setSpeed(int speed) {
        _maxSpeed = speed << TMR_FREQ_EXP;
    }

    void setRamp(int ramp) {
        _ramp = std::abs(ramp);
    }

    void setEndPosTolerance(unsigned tolerance) {
        _endPosTolerance = tolerance;
    }

    void moveInfinite() {
        auto st = std::make_shared<MotorState>();
        st->mode = INFINITE;
        st->vwPos = _enc.getPos() << TMR_FREQ_EXP;

        _state.store(st);
    }

    void moveTime(int64_t time) {
        auto st = std::make_shared<MotorState>();
        st->mode = TIME;
        st->vwPos = _enc.getPos() << TMR_FREQ_EXP;
        st->endTargetTime = esp_timer_get_time() + time * 1000;

        _state.store(st);
    }

    void moveDistance(int64_t delta) {
        auto st = std::make_shared<MotorState>();
        st->mode = POSITION;
        st->vwPos = _enc.getPos() << TMR_FREQ_EXP;
        st->endTargetPos = st->vwPos + (delta << TMR_FREQ_EXP);

        _state.store(st);
    }

    void setRawPower(int power) {
        auto st = std::make_shared<MotorState>();
        st->mode = RAW;
        _power = power;

        _state.store(st);
        _reset = true;
    }

    void stop(bool brake) {
        auto st = std::make_shared<MotorState>();
        st->mode = brake ? BREAK : FREE;

        _state.store(st);
        _reset = true;
    }

    void onTarget(std::function<void()> callback) {
        if (!callback) {
            _onTarget.store(nullptr);
            return;
        }
        _onTarget = std::make_shared<std::function<void()>>(callback);
    }

    esp_timer_handle_t _tmr;

    void startTicker() {
        esp_timer_create_args_t tmrArgs = {
            .callback = [](void* arg) {
                auto& motor = *static_cast<DCMotor*>(arg);
                motor.tick();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "DCMotorTick",
            .skip_unhandled_events = true
        };

        esp_timer_create(&tmrArgs, &_tmr);
        esp_timer_start_periodic(_tmr, TMR_PERIOD_US);
    }

    void stopTicker() {
        esp_timer_stop(_tmr);
        esp_timer_delete(_tmr);
    }

    int64_t getPosition() {
        return _enc.getPos();
    }

    // can only be used when not in regulated mode
    void setRegulator(RegParams params) {
        _reg = params;
    }

    void setReporter(Reporter reporter) {
        if (!reporter) {
            _reporter.store(nullptr);
            return;
        }
        _reporter = std::make_shared<Reporter>(reporter);
    }

    bool isReporting() {
        return _reporter.load() != nullptr;
    }
};
