/*
モーターを前進させて動くことを確認するコード
*/

#include <ros/ros.h>
//#include <pigpio.h>
#include <pigpiod_if2.h>
#include <cstdint>
#include <algorithm>

// ==================== 実験・調整用パラメータ ====================
namespace Tuning {
    // --- 1. モーター個体差のPWM調整係数 ---
    constexpr float MOTOR_FL_MULT = 1.0f;
    constexpr float MOTOR_FR_MULT = 1.0f;
    constexpr float MOTOR_RL_MULT = 1.0f;
    constexpr float MOTOR_RR_MULT = 1.0f;

    // --- 2. モーター動作パラメータ ---
    constexpr int MOTOR_SPEED = 150;      // 前進時のベース速度(duty)
    constexpr int MAX_DUTY = 255;         // デューティー比の最大値
    constexpr int PWM_FREQ = 3000;        // PWM周波数
}
// ==============================================================

struct MotorConfig {
    int pwm_freq = Tuning::PWM_FREQ;  // PWM周波数

    // モーターのピン配置
    uint8_t MA_1 = 22;
    uint8_t MA_2 = 10;
    uint8_t MB_1 = 18;
    uint8_t MB_2 = 12;
    uint8_t MC_1 = 19;
    uint8_t MC_2 = 13;
    uint8_t MD_1 = 20;
    uint8_t MD_2 = 21;
};

class MotorSpinner {
public:
    static const int num_motor = 4;

    // モーターごとのデューティー比（各モーター2ピン分）
    int motor_duties[num_motor * 2] = {0, 0, 0, 0, 0, 0, 0, 0};

    MotorSpinner(const MotorConfig& config, int pi_handle)
        : cfg(config), pi(pi_handle) {}

    void start() {
        set_PWM_frequency(pi, cfg.MA_1, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MA_2, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MB_1, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MB_2, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MC_1, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MC_2, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MD_1, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MD_2, cfg.pwm_freq);
        set_PWM_frequency(pi, 27, cfg.pwm_freq);
    }

    // 前進方向のmotor_dutiesを計算する（出力はしない）
    void DutyUpdate(int speed) {
        float motor_mults[4] = {
            Tuning::MOTOR_FL_MULT,
            Tuning::MOTOR_FR_MULT,
            Tuning::MOTOR_RL_MULT,
            Tuning::MOTOR_RR_MULT
        };

        auto calc_duty = [](int base_speed, float mult) {
            return static_cast<int>(std::min(static_cast<float>(base_speed) * mult, static_cast<float>(Tuning::MAX_DUTY)));
        };

        motor_duties[0] = 0;     motor_duties[1] = calc_duty(speed, motor_mults[0]);  // MA (FL)
        motor_duties[2] = 0;     motor_duties[3] = calc_duty(speed, motor_mults[1]);  // MB (FR)
        motor_duties[4] = 0;     motor_duties[5] = calc_duty(speed, motor_mults[2]);  // MC (RL)
        motor_duties[6] = 0;     motor_duties[7] = calc_duty(speed, motor_mults[3]);  // MD (RR)
    }

    void PWMshootor() {
        set_PWM_dutycycle(pi, cfg.MA_1, motor_duties[0]);
        set_PWM_dutycycle(pi, cfg.MA_2, motor_duties[1]);
        set_PWM_dutycycle(pi, cfg.MB_1, motor_duties[2]);
        set_PWM_dutycycle(pi, cfg.MB_2, motor_duties[3]);
        set_PWM_dutycycle(pi, cfg.MC_1, motor_duties[4]);
        set_PWM_dutycycle(pi, cfg.MC_2, motor_duties[5]);
        set_PWM_dutycycle(pi, cfg.MD_1, motor_duties[6]);
        set_PWM_dutycycle(pi, cfg.MD_2, motor_duties[7]);
        set_PWM_dutycycle(pi, 27, 100);
    }

    void stop() {
        for (int i = 0; i < num_motor * 2; i++) motor_duties[i] = 0;
        PWMshootor();
    }

private:
    MotorConfig cfg;
    int pi;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "servo");
    ros::NodeHandle n;

    int pi = pigpio_start(NULL, NULL);

    MotorConfig motor_cfg;
    MotorSpinner motors(motor_cfg, pi);
    motors.start();

    motors.DutyUpdate(Tuning::MOTOR_SPEED);

    ros::Rate rate(110);
    while (ros::ok()) {
        motors.PWMshootor();

        ros::spinOnce();
        rate.sleep();
    }

    motors.stop();
    return 0;
}
