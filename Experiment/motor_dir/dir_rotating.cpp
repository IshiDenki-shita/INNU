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

// ==================== ボール情報の受信 ====================
// 通信設計書(2026/10/09)に基づく値。座標系は車体中心基準、正面=0[rad]、反時計回りが正。
//   theta_rad  : ボール相対方位 [rad]  (-pi < theta <= pi)
//   distance_m : ボール水平距離 [m]    (0以上)
//   is_detected: ボール検出状態
// 暫定メッセージ: Float32MultiArray の data = {theta_rad, distance_m, detected(0 or 1)}

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

    // 不正値は破棄（検証の主担当はUART受信ノード。ここは最後の防壁）
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
    int max_duty = 255;         // デューティー比の最大値
    int pwm_freq = 3000;        // PWM周波数
    int translate_speed = 150;  // 並進方向の速度(duty)。元のmotor_speedに相当
    float heading_kp = 80.0f;   // ボール方向へ姿勢を向けるための比例ゲイン

    // 距離に応じた並進速度の調整（値は実験で調整する）
    float stop_distance = 0.3f;        // [m] これ以下では並進せず、姿勢合わせのみ行う
    float full_speed_distance = 1.5f;  // [m] これ以上で並進速度が最大になる

    // 通信ロスト判定
    double timeout_s = 0.5;  // これを超えて未更新ならモーターを停止する

    // モーターのピン配置
    // 割り当て: MA=左前(FL), MB=右前(FR), MC=左後(RL), MD=右後(RR)
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

    // ボールまでの距離[m]から並進成分の係数(0.0〜1.0)を求める。遠いほど大きい。
    float CalcTranslateScale(float distance) const {
        float span = cfg.full_speed_distance - cfg.stop_distance;
        if (span < 1e-6f) {
            return distance > cfg.stop_distance ? 1.0f : 0.0f;
        }
        float ratio = (distance - cfg.stop_distance) / span;
        return std::min(std::max(ratio, 0.0f), 1.0f);
    }

    // move_theta: ロボットを進ませたい方向 [rad]（正面基準、反時計回りが正）
    // ball_theta: ボールへの方角 [rad]（この方向へ姿勢を向け続ける）
    // distance  : ボールまでの距離 [m]（並進速度の調整に使う）
    void DutyUpdate(float move_theta, float ball_theta, float distance) {
        // 並進成分（move_thetaの方向へ移動。姿勢の向きとは無関係）
        // 距離が近いほど小さくなり、stop_distance以下では0（回転のみ）
        float scale = CalcTranslateScale(distance);
        float vx = scale * std::cos(move_theta);
        float vy = scale * std::sin(move_theta);

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

    // 通信ロスト判定用の閾値を参照するために公開
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
            // 通信ロスト（または未受信）：安全のため停止
            motors.stop();
        } else if (!purpose_detected_g) {
            // ボール未検出：探索動作は未定義のため、現状は停止
            motors.stop();
        } else {
            // 現状は進行方向を決める別ソース(LiDAR等)がまだ無いため、
            // 暫定的にボール方向をそのまま進行方向としても使っている。
            // 経路計画を追加する際はmove_thetaだけ別の値に差し替えればよい。
            motors.DutyUpdate(purpose_theta_g, purpose_theta_g, purpose_distance_g);
            motors.PWMshootor();
        }

        ros::spinOnce();
        rate.sleep();
    }

    motors.stop();
    return 0;
}