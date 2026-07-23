"""
目標の方角へ移動するためのduty比を計算するコード
姿勢を常にボールへ向けながら移動
"""

#include <ros/ros.h>
//#include <pigpio.h>
#include <pigpiod_if2.h>
#include <cstdint>
#include <cmath>
#include <algorithm>

#include "std_msgs/Float32.h"  // 画像ラズパイからの目的方角。メッセージ型は仮（詳細未定）

float purpose_theta_g = 0.0f;  // ロボットを進ませる方向
float ball_dir_theta = 0.052f;

void directionCallback(const std_msgs::Float32::ConstPtr& msg)
{
    purpose_theta_g = msg->data;
}

struct MotorConfig {
    int max_duty = 255;         // デューティー比の最大値
    int pwm_freq = 3000;        // PWM周波数
    int translate_speed = 150;  // 並進方向の速度(duty)。元のmotor_speedに相当
    float heading_kp = 80.0f;   // ボール方向へ姿勢を向けるための比例ゲイン

    // モーターのピン配置
    // 割り当て: MA=左前(FL), MB=右前(FR), MC=左後(RL), MD=右後(RR)
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

    // move_theta: ロボットを進ませたい方向 [rad]（正面基準、反時計回りが正）
    // ball_theta: ボールへの方角 [rad]（この方向へ姿勢を向け続ける）
    void DutyUpdate(float move_theta, float ball_theta) {
        // 並進成分（move_thetaの方向へ移動。姿勢の向きとは無関係）
        float vx = std::cos(move_theta);
        float vy = std::sin(move_theta);

        // 回転だけ行いたいので並進成分は 0
        vx = 0.0f;
        vy = 0.0f;

        // 回転成分（ball_thetaを0に近づける比例制御 = 常にボールを向く）
        float w = cfg.heading_kp * ball_theta;

        // メカナム標準式（並進と回転を合成）
        float fl = vx - vy - w;
        float fr = vx + vy + w;
        float rl = vx + vy - w;
        float rr = vx - vy + w;

        float wheel_ratios[4] = {fl, fr, rl, rr};

        float max_ratio = 0.0f;
        for (float ratio : wheel_ratios) {
            max_ratio = std::max(max_ratio, std::abs(ratio));
        }
        if (max_ratio < 1e-6f) {
            max_ratio = 1.0f;
        }

        for (int i = 0; i < 4; ++i) {
            float normalized = wheel_ratios[i] / max_ratio;
            int duty = static_cast<int>(std::min(
                std::abs(normalized) * cfg.translate_speed,
                static_cast<float>(cfg.max_duty)));

            if (normalized >= 0) {
                motor_duties[i * 2] = duty;
                motor_duties[i * 2 + 1] = 0;
            } else {
                motor_duties[i * 2] = 0;
                motor_duties[i * 2 + 1] = duty;
            }
        }
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

    ros::Subscriber dir_sub = n.subscribe<std_msgs::Float32>(
        "/purpose_direction", 10, directionCallback);

    ros::Rate rate(50);
    while (ros::ok()) {
        // 現状は進行方向を決める別ソース(LiDAR等)がまだ無いため、
        // 暫定的にボール方向をそのまま進行方向としても使っている。
        // 経路計画を追加する際はmove_thetaだけ別の値に差し替えればよい。
        motors.DutyUpdate(purpose_theta_g, purpose_theta_g);
        motors.PWMshootor();

        ros::spinOnce();
        rate.sleep();
    }

    motors.stop();
    return 0;
}