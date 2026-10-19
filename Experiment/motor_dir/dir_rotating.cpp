/*
目標の方角へ移動するためのduty比を計算するコード
姿勢を常にボールへ向けながら移動
注：モーターの回転テストの意味合いが強い

UART受信ノード(別ノード)がpublishするボール情報(theta, distance, is_detected)を
subscribeしてモーターを制御する。UARTのバイト列解析はこのノードでは行わない。
*/

#include <ros/ros.h>
//#include <pigpio.h>
#include <pigpiod_if2.h>
#include <cstdint>
#include <cmath>
#include <algorithm>

#include "std_msgs/Float32MultiArray.h"  // 暫定。UART受信ノードのメッセージ形式が確定したら差し替える

// ==================== 実験・調整用パラメータ ====================
namespace Tuning {
    // --- 1. モーター個体差のPWM調整係数 ---
    // (FL:左前, FR:右前, RL:左後, RR:右後)
    // 基準を1.0fとし、回転が遅いモーターの数値を上げる（または速いものを下げる）
    constexpr float MOTOR_FL_MULT = 1.0f;
    constexpr float MOTOR_FR_MULT = 1.0f;
    constexpr float MOTOR_RL_MULT = 1.0f;
    constexpr float MOTOR_RR_MULT = 1.0f;

    // --- 2. モーターの基本動作パラメータ ---
    constexpr int TRANSLATE_SPEED = 150;        // 並進ベース速度(duty)
    constexpr float HEADING_KP = 80.0f;         // ボール方向へ姿勢を向けるための比例ゲイン
    constexpr float STOP_DISTANCE = 0.3f;       // [m] これ以下では並進せず、姿勢合わせのみ行う
    constexpr float FULL_SPEED_DISTANCE = 1.5f; // [m] これ以上で並進速度が最大になる
    constexpr double TIMEOUT_S = 0.5;           // [s] 通信ロスト判定閾値
}
// ==============================================================

// ==================== ボール情報の受信 ====================
float purpose_theta_g = 0.0f;     // ボールへの方角 [rad]
float purpose_distance_g = 0.0f;  // ボールまでの距離 [m]（遠いほどスピード出す）
bool purpose_detected_g = false;  // ボール検出状態
bool has_received_g = false;      // 一度でも受信したか
ros::Time last_update_g;          // 最後に正常なデータを受信した時刻

void ballTargetCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 3) {
        ROS_WARN("ball_target: unexpected size %zu", msg->data.size());
        return;
    }

    float theta = msg->data[0];
    float distance = msg->data[1];
    bool detected = msg->data[2] > 0.5f;

    if (!std::isfinite(theta) || !std::isfinite(distance) || distance < 0.0f) {
        ROS_WARN("ball_target: invalid value (theta=%f, distance=%f)", theta, distance);
        return;
    }

    purpose_theta_g = theta;
    purpose_distance_g = distance;
    purpose_detected_g = detected;
    has_received_g = true;
    last_update_g = ros::Time::now();
}

struct MotorConfig {
    int max_duty = 255;
    int pwm_freq = 3000;
    
    // Tuningから初期値を取得
    int translate_speed = Tuning::TRANSLATE_SPEED;
    float heading_kp = Tuning::HEADING_KP;
    float stop_distance = Tuning::STOP_DISTANCE;
    float full_speed_distance = Tuning::FULL_SPEED_DISTANCE;
    double timeout_s = Tuning::TIMEOUT_S;

    // ピン配置
    uint8_t MA_1 = 22;
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
    }

    float CalcTranslateScale(float distance) const {
        float span = cfg.full_speed_distance - cfg.stop_distance;
        if (span < 1e-6f) {
            return distance > cfg.stop_distance ? 1.0f : 0.0f;
        }
        float ratio = (distance - cfg.stop_distance) / span;
        return std::min(std::max(ratio, 0.0f), 1.0f);
    }

    void DutyUpdate(float move_theta, float ball_theta, float distance) {
        float scale = CalcTranslateScale(distance);
        float vx = scale * std::cos(move_theta);
        float vy = scale * std::sin(move_theta);

        float w = cfg.heading_kp * ball_theta;

        float fl = vx - vy - w;
        float fr = vx + vy + w;
        float rl = vx + vy - w;
        float rr = vx - vy + w;

        float wheel_ratios[4] = {fl, fr, rl, rr};

        float motor_mults[4] = {
            Tuning::MOTOR_FL_MULT,
            Tuning::MOTOR_FR_MULT,
            Tuning::MOTOR_RL_MULT,
            Tuning::MOTOR_RR_MULT
        };

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
                std::abs(normalized) * cfg.translate_speed * motor_mults[i],
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

    void stop() {
        for (int i = 0; i < num_motor * 2; i++) motor_duties[i] = 0;
        PWMshootor();
    }

    double timeout() const { return cfg.timeout_s; }

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

    ros::Subscriber ball_sub = n.subscribe<std_msgs::Float32MultiArray>(
        "/ball_target", 10, ballTargetCallback);

    ros::Rate rate(50);
    while (ros::ok()) {
        bool stale = !has_received_g ||
                     (ros::Time::now() - last_update_g).toSec() > motors.timeout();

        if (stale) {
            motors.stop();
        } else if (!purpose_detected_g) {
            motors.stop();
        } else {
            motors.DutyUpdate(purpose_theta_g, purpose_theta_g, purpose_distance_g);
            motors.PWMshootor();
        }

        ros::spinOnce();
        rate.sleep();
    }

    motors.stop();
    return 0;
}
