#include <ros/ros.h>
//#include <pigpio.h>
#include <pigpiod_if2.h>
#include <cstdint>


struct MotorConfig {
    int max_duty = 255;   // デューティー比の最大値
    int pwm_freq = 3000;  // PWM周波数

    // モーターのピン配置
    uint8_t MA_1 = 4;
    uint8_t MA_2 = 10;
    uint8_t MB_1 = 12;
    uint8_t MB_2 = 18;
    uint8_t MC_1 = 13;
    uint8_t MC_2 = 19;
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

    // モーターとの接続など（PWM周波数の設定）
    void start() {
        set_PWM_frequency(pi, cfg.MA_1, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MA_2, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MB_1, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MB_2, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MC_1, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MC_2, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MD_1, cfg.pwm_freq);
        set_PWM_frequency(pi, cfg.MD_2, cfg.pwm_freq);
    }

    // 前進方向のmotor_dutiesを計算する（出力はしない）
    void DutyUpdate(int speed) {
        motor_duties[0] = 0;     motor_duties[1] = speed;  // MA_1, MA_2
        motor_duties[2] = 0;     motor_duties[3] = speed;  // MB_1, MB_2
        motor_duties[4] = 0;     motor_duties[5] = speed;  // MC_1, MC_2
        motor_duties[6] = 0;     motor_duties[7] = speed;  // MD_1, MD_2
    }

    // motor_dutiesの内容を実際にPWM出力する
    void PWMshootor() {
        set_PWM_dutycycle(pi, cfg.MA_1, motor_duties[0]);
        set_PWM_dutycycle(pi, cfg.MA_2, motor_duties[1]);
        set_PWM_dutycycle(pi, cfg.MB_1, motor_duties[2]);
        set_PWM_dutycycle(pi, cfg.MB_2, motor_duties[3]);
        set_PWM_dutycycle(pi, cfg.MC_1, motor_duties[4]);
        set_PWM_dutycycle(pi, cfg.MC_2, motor_duties[5]);
        set_PWM_dutycycle(pi, cfg.MD_1, motor_duties[6]);
        set_PWM_dutycycle(pi, cfg.MD_2, motor_duties[7]);
    }

    // モーターとの接続解除など
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

    motors.DutyUpdate(100);

    ros::Rate rate(50);
    while (ros::ok()) {
        motors.PWMshootor();

        ros::spinOnce();
        rate.sleep();
    }

    motors.stop();
    return 0;
}