#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/ledc_types.h"
#include "soc/gpio_num.h"
#include "esp_console.h"
#include "esp_system.h"

#include "dcmotor.h"

#include <memory>
#include <cstring>
#include <tuple>
#include <deque>


using DCMotorPtr = std::shared_ptr<DCMotor>;


using Report = std::tuple<uint64_t, int64_t, int64_t, int, int, int, int, int, int, int, int>;

std::deque<Report> reportsLeft;
std::deque<Report> reportsRight;

int everyNth = 1;

struct Reporter {
    std::deque<Report>* reports;
    int _counter;
    int _everyNth;

    Reporter(std::deque<Report>& reports, int everyNth) : reports(&reports), _counter(everyNth), _everyNth(everyNth) {
        assert(_everyNth > 0);
    }

    void operator()(uint64_t time, int64_t pos, int64_t vwTarget, int vwSpeed, int error, int power, int pOut, int iOut, int dOut, int vOut, int aOut) {
        if ((--_counter) > 0) {
            return;
        }
        _counter = _everyNth;

        try {
            reports->push_back({time, pos, vwTarget, vwSpeed, error, power, pOut, iOut, dOut, vOut, aOut});
        }
        catch (const std::exception& e) {
            reports->clear();
            ESP_LOGE("test", "Reporter error, clearing reports: %s", e.what());
        }
    }
};


int main() {
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 4000,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };
    ledc_timer_config(&timer_conf);

    struct Motors {
        DCMotorPtr left;
        DCMotorPtr right;
    };

    RegParams params = {
        .kp = 20,
        .ki = 0,
        .kd = 0,
        .kv = 0,
        .ka = 0,
        .kc = 0,
        .maxIOut = 0,
        .unwindFactor = 2
    };

    Motors* motors = new Motors{
        std::make_shared<DCMotor>(
            GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_39, GPIO_NUM_40,
            params, LEDC_TIMER_0, LEDC_CHANNEL_0, LEDC_CHANNEL_1
        ),
        std::make_shared<DCMotor>(
            GPIO_NUM_45, GPIO_NUM_13, GPIO_NUM_42, GPIO_NUM_41,
            params, LEDC_TIMER_0, LEDC_CHANNEL_2, LEDC_CHANNEL_3
        )
    };
    motors->left->startTicker();
    motors->right->startTicker();

    motors->left->onTarget([]() {
        printf("Left motor target reached\n");
    });
    motors->right->onTarget([]() {
        printf("Right motor target reached\n");
    });

    esp_console_repl_t* repl;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "dcmotor> ";

#if CONFIG_ESP_CONSOLE_UART_DEFAULT
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usb_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usb_config, &repl_config, &repl));
#endif

    esp_console_cmd_t move_t_cmd = {
        command: "move_t",
        help: "Move a motor by time",
        hint: "<motor> <time>",
        func: nullptr,
        func_w_context: [](void* ctx, int argc, char** argv) {
            if (argc < 3) {
                return ESP_ERR_INVALID_ARG;
            }

            auto& motors = *static_cast<Motors*>(ctx);
            DCMotorPtr motor;
            if (argv[1][0] == 'l') {
                motor = motors.left;
            }
            else if (argv[1][0] == 'r') {
                motor = motors.right;
            }
            else {
                printf("Invalid motor %s\n", argv[1]);
                return ESP_ERR_INVALID_ARG;
            }

            int time;
            if (sscanf(argv[2], "%d", &time) != 1) {
                printf("Invalid time %s\n", argv[2]);
                return ESP_ERR_INVALID_ARG;
            }

            printf("Moving motor %s for %d ms\n", argv[1], time);

            motor->moveTime(time);

            return ESP_OK;
        },
        context: motors
    };

    esp_console_cmd_t move_d_cmd = {
        command: "move_d",
        help: "Move a motor by distance",
        hint: "<motor> <distance>",
        func: nullptr,
        func_w_context: [](void* ctx, int argc, char** argv) {
            if (argc < 3) {
                return ESP_ERR_INVALID_ARG;
            }

            auto& motors = *static_cast<Motors*>(ctx);
            DCMotorPtr motor;
            if (argv[1][0] == 'l') {
                motor = motors.left;
            }
            else if (argv[1][0] == 'r') {
                motor = motors.right;
            }
            else {
                printf("Invalid motor %s\n", argv[1]);
                return ESP_ERR_INVALID_ARG;
            }

            int distance;
            if (sscanf(argv[2], "%d", &distance) != 1) {
                printf("Invalid distance %s\n", argv[2]);
                return ESP_ERR_INVALID_ARG;
            }

            printf("Moving motor %s by %d ticks\n", argv[1], distance);

            motor->moveDistance(distance);

            return ESP_OK;
        },
        context: motors
    };

    esp_console_cmd_t move_i_cmd = {
        command: "move_i",
        help: "Move a motor infinitely",
        hint: "<motor>",
        func: nullptr,
        func_w_context: [](void* ctx, int argc, char** argv) {
            if (argc < 2) {
                return ESP_ERR_INVALID_ARG;
            }

            auto& motors = *static_cast<Motors*>(ctx);
            DCMotorPtr motor;
            if (argv[1][0] == 'l') {
                motor = motors.left;
            }
            else if (argv[1][0] == 'r') {
                motor = motors.right;
            }
            else {
                printf("Invalid motor %s\n", argv[1]);
                return ESP_ERR_INVALID_ARG;
            }

            printf("Moving motor %s infinitely\n", argv[1]);

            motor->moveInfinite();

            return ESP_OK;
        },
        context: motors
    };

    esp_console_cmd_t speed_cmd = {
        command: "speed",
        help: "Set the speed of a motor",
        hint: "<motor> <speed>",
        func: nullptr,
        func_w_context: [](void* ctx, int argc, char** argv) {
            if (argc < 3) {
                return ESP_ERR_INVALID_ARG;
            }

            auto& motors = *static_cast<Motors*>(ctx);
            DCMotorPtr motor;
            if (argv[1][0] == 'l') {
                motor = motors.left;
            }
            else if (argv[1][0] == 'r') {
                motor = motors.right;
            }
            else {
                printf("Invalid motor %s\n", argv[1]);
                return ESP_ERR_INVALID_ARG;
            }

            int speed;
            if (sscanf(argv[2], "%d", &speed) != 1) {
                printf("Invalid speed %s\n", argv[2]);
                return ESP_ERR_INVALID_ARG;
            }

            printf("Setting motor %s speed to %d\n", argv[1], speed);

            motor->setSpeed(speed);

            return ESP_OK;
        },
        context: motors
    };

    esp_console_cmd_t ramp_cmd = {
        command: "ramp",
        help: "Set the ramp of a motor",
        hint: "<motor> <ramp>",
        func: nullptr,
        func_w_context: [](void* ctx, int argc, char** argv) {
            if (argc < 3) {
                return ESP_ERR_INVALID_ARG;
            }

            auto& motors = *static_cast<Motors*>(ctx);
            DCMotorPtr motor;
            if (argv[1][0] == 'l') {
                motor = motors.left;
            }
            else if (argv[1][0] == 'r') {
                motor = motors.right;
            }
            else {
                printf("Invalid motor %s\n", argv[1]);
                return ESP_ERR_INVALID_ARG;
            }

            int ramp;
            if (sscanf(argv[2], "%d", &ramp) != 1) {
                printf("Invalid ramp %s\n", argv[2]);
                return ESP_ERR_INVALID_ARG;
            }

            printf("Setting motor %s ramp to %d\n", argv[1], ramp);

            motor->setRamp(ramp);

            return ESP_OK;
        },
        context: motors
    };

    esp_console_cmd_t stop_cmd = {
        command: "stop",
        help: "Stop a motor",
        hint: "<motor> <mode>",
        func: nullptr,
        func_w_context: [](void* ctx, int argc, char** argv) {
            if (argc < 3) {
                return ESP_ERR_INVALID_ARG;
            }

            auto& motors = *static_cast<Motors*>(ctx);
            DCMotorPtr motor;
            if (argv[1][0] == 'l') {
                motor = motors.left;
            }
            else if (argv[1][0] == 'r') {
                motor = motors.right;
            }
            else {
                printf("Invalid motor %s\n", argv[1]);
                return ESP_ERR_INVALID_ARG;
            }

            bool brake = false;
            if (argv[2][0] == 'b') {
                brake = true;
            }
            else if (argv[2][0] != 'f') {
                printf("Invalid mode %s\n", argv[2]);
                return ESP_ERR_INVALID_ARG;
            }

            printf("Stopping motor %s with %s\n", argv[1], brake ? "brake" : "free");

            motor->stop(brake);

            return ESP_OK;
        },
        context: motors
    };

    esp_console_cmd_t pos_cmd = {
        command: "pos",
        help: "Get the position of a motor",
        hint: "<motor>",
        func: nullptr,
        func_w_context: [](void* ctx, int argc, char** argv) {
            if (argc < 2) {
                return ESP_ERR_INVALID_ARG;
            }

            auto& motors = *static_cast<Motors*>(ctx);
            DCMotorPtr motor;
            if (argv[1][0] == 'l') {
                motor = motors.left;
            }
            else if (argv[1][0] == 'r') {
                motor = motors.right;
            }
            else {
                printf("Invalid motor %s\n", argv[1]);
                return ESP_ERR_INVALID_ARG;
            }

            printf("Motor %s position: %lld\n", argv[1], motor->getPosition());

            return ESP_OK;
        },
        context: motors
    };

    esp_console_cmd_t reg_cmd = {
        command: "reg",
        help: "Set the regulator parameters of a motor",
        hint: "<motor> <kp> <posI> <posD> <kv> <ka> <kc> <maxIOut>",
        func: nullptr,
        func_w_context: [](void* ctx, int argc, char** argv) {
            if (argc < 5) {
                return ESP_ERR_INVALID_ARG;
            }

            auto& motors = *static_cast<Motors*>(ctx);
            DCMotorPtr motor;
            if (argv[1][0] == 'l') {
                motor = motors.left;
            }
            else if (argv[1][0] == 'r') {
                motor = motors.right;
            }
            else {
                printf("Invalid motor %s\n", argv[1]);
                return ESP_ERR_INVALID_ARG;
            }

            RegParams params = {};
            if (sscanf(argv[2], "%d", &params.kp) != 1) {
                printf("Invalid kp %s\n", argv[2]);
                return ESP_ERR_INVALID_ARG;
            }
            if (sscanf(argv[3], "%d", &params.ki) != 1) {
                printf("Invalid posI %s\n", argv[3]);
                return ESP_ERR_INVALID_ARG;
            }
            if (sscanf(argv[4], "%d", &params.kd) != 1) {
                printf("Invalid posD %s\n", argv[4]);
                return ESP_ERR_INVALID_ARG;
            }
            if (sscanf(argv[5], "%d", &params.kv) != 1) {
                printf("Invalid kv %s\n", argv[5]);
                return ESP_ERR_INVALID_ARG;
            }
            if (argc > 6 && sscanf(argv[6], "%d", &params.ka) != 1) {
                printf("Invalid ka %s\n", argv[6]);
                return ESP_ERR_INVALID_ARG;
            }
            if (argc > 7 && sscanf(argv[7], "%d", &params.kc) != 1) {
                printf("Invalid kc %s\n", argv[7]);
                return ESP_ERR_INVALID_ARG;
            }
            if (argc > 8 && sscanf(argv[8], "%d", &params.maxIOut) != 1) {
                printf("Invalid maxIOut %s\n", argv[8]);
                return ESP_ERR_INVALID_ARG;
            }
            if (argc > 9 && sscanf(argv[9], "%d", &params.unwindFactor) != 1) {
                printf("Invalid unwindFactor %s\n", argv[9]);
                return ESP_ERR_INVALID_ARG;
            }

            printf("Setting motor %s parameters to p=%d i=%d d=%d v=%d a=%d c=%d imax=%d, unw=%d\n",
                argv[1], params.kp, params.ki, params.kd, params.kv, params.ka, params.kc, params.maxIOut, params.unwindFactor
            );

            motor->setRegulator(params);

            return ESP_OK;
        },
        context: motors
    };

    esp_console_cmd_t freq_cmd = {
        command: "freq",
        help: "Set the frequency of the motor PWM",
        hint: "<freq>",
        func: nullptr,
        func_w_context: [](void* ctx, int argc, char** argv) {
            if (argc < 2) {
                return ESP_ERR_INVALID_ARG;
            }

            unsigned int freq;
            if (sscanf(argv[1], "%u", &freq) != 1) {
                printf("Invalid freq %s\n", argv[1]);
                return ESP_ERR_INVALID_ARG;
            }

            printf("Setting motor PWM frequency to %d\n", freq);

            ledc_timer_config_t timer_conf = {
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .duty_resolution = LEDC_TIMER_10_BIT,
                .timer_num = LEDC_TIMER_0,
                .freq_hz = static_cast<uint32_t>(freq),
                .clk_cfg = LEDC_AUTO_CLK,
                .deconfigure = false
            };
            ledc_timer_config(&timer_conf);

            return ESP_OK;
        },
        context: nullptr
    };

    esp_console_cmd_t report_cmd = {
        command: "report",
        help: "Control reporting",
        hint: "<motor> <enable/disable/clear/print>",
        func: nullptr,
        func_w_context: [](void* ctx, int argc, char** argv) {
            if (argc < 3) {
                return ESP_ERR_INVALID_ARG;
            }

            auto& motors = *static_cast<Motors*>(ctx);
            DCMotorPtr motor;
            if (argv[1][0] == 'l') {
                motor = motors.left;
            }
            else if (argv[1][0] == 'r') {
                motor = motors.right;
            }
            else {
                printf("Invalid motor %s\n", argv[1]);
                return ESP_ERR_INVALID_ARG;
            }

            if (argv[2][0] == 'e') {
                motor->setReporter(motor == motors.left ?
                    Reporter(reportsLeft, everyNth) :
                    Reporter(reportsRight, everyNth)
                );
            }
            else if (argv[2][0] == 'd') {
                motor->setReporter(nullptr);
            }
            else if (argv[2][0] == 'c') {
                if (motor == motors.left) {
                    reportsLeft.clear();
                }
                else {
                    reportsRight.clear();
                }
            }
            else if (argv[2][0] == 'p') {
                auto& reports = motor == motors.left ? reportsLeft : reportsRight;
                for (auto& report : reports) {
                    printf("%llu,%lld,%lld,%d,%d,%d,%d,%d,%d,%d,%d\n",
                        std::get<0>(report), std::get<1>(report), std::get<2>(report),
                        std::get<3>(report), std::get<4>(report), std::get<5>(report),
                        std::get<6>(report), std::get<7>(report), std::get<8>(report),
                        std::get<9>(report), std::get<10>(report)
                    );
                }
            }
            else {
                printf("Invalid command %s\n", argv[2]);
                return ESP_ERR_INVALID_ARG;
            }

            return ESP_OK;
        },
        context: motors
    };

    esp_console_cmd_t everynth_cmd = {
        command: "everynth",
        help: "Report only every nth data point",
        hint: "<n>",
        func: nullptr,
        func_w_context: [](void* ctx, int argc, char** argv) {
            if (argc < 2) {
                return ESP_ERR_INVALID_ARG;
            }

            int n;
            if (sscanf(argv[1], "%d", &n) != 1) {
                printf("Invalid n %s\n", argv[1]);
                return ESP_ERR_INVALID_ARG;
            }
            if (n < 1) {
                printf("Invalid n %d\n", n);
                return ESP_ERR_INVALID_ARG;
            }

            everyNth = n;

            printf("Reporting only every %d data points\n", n);

            return ESP_OK;
        },
        context: nullptr
    };

    esp_console_cmd_t raw_cmd = {
        command: "raw",
        help: "Set the raw power of a motor",
        hint: "<motor> <power>",
        func: nullptr,
        func_w_context: [](void* ctx, int argc, char** argv) {
            if (argc < 3) {
                return ESP_ERR_INVALID_ARG;
            }

            auto& motors = *static_cast<Motors*>(ctx);
            DCMotorPtr motor;
            if (argv[1][0] == 'l') {
                motor = motors.left;
            }
            else if (argv[1][0] == 'r') {
                motor = motors.right;
            }
            else {
                printf("Invalid motor %s\n", argv[1]);
                return ESP_ERR_INVALID_ARG;
            }

            int power;
            if (sscanf(argv[2], "%d", &power) != 1) {
                printf("Invalid power %s\n", argv[2]);
                return ESP_ERR_INVALID_ARG;
            }

            printf("Setting motor %s raw power to %d\n", argv[1], power);

            motor->setRawPower(power);

            return ESP_OK;
        },
        context: motors
    };

    esp_console_cmd_register(&move_t_cmd);
    esp_console_cmd_register(&move_d_cmd);
    esp_console_cmd_register(&move_i_cmd);
    esp_console_cmd_register(&speed_cmd);
    esp_console_cmd_register(&ramp_cmd);
    esp_console_cmd_register(&stop_cmd);
    esp_console_cmd_register(&pos_cmd);
    esp_console_cmd_register(&reg_cmd);
    esp_console_cmd_register(&freq_cmd);
    esp_console_cmd_register(&report_cmd);
    esp_console_cmd_register(&everynth_cmd);
    esp_console_cmd_register(&raw_cmd);


    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}


extern "C" void app_main(void) {
    main();

    vTaskDelete(NULL);
}
