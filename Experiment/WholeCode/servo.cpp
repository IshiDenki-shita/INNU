#include <ros/ros.h>
#include <pigpiod_if2.h>
#include <sensor_msgs/LaserScan.h>
#include <innu_uart/BallTarget.h>

#include <cmath>
#include <algorithm>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <limits>
#include <cstdint>

// ==================== 実験・調整用パラメータ ====================
namespace Tuning {
    // --- 1. モーター個体差のPWM調整係数 ---
    constexpr float MOTOR_FL_MULT = 1.0f;
    constexpr float MOTOR_FR_MULT = 1.0f;
    constexpr float MOTOR_RL_MULT = 1.0f;
    constexpr float MOTOR_RR_MULT = 1.0f;

    // --- 2. モーターの基本動作パラメータ ---
    constexpr int TRANSLATE_SPEED = 100;        // 並進ベース速度(duty)
    constexpr float HEADING_KP = 80.0f;         // ボール方向へ姿勢を向けるための比例ゲイン
    constexpr float STOP_DISTANCE = 0.3f;       // [m] これ以下では並進せず姿勢合わせのみ
    constexpr float FULL_SPEED_DISTANCE = 1.5f;   // [m] これ以上で並進速度最大

    // --- 3. LiDAR 経路計算・BANの調整パラメータ ---
    constexpr float ROBOT_RADIUS_M = 0.20f;     // ロボットの半径 [m]
    constexpr float SAFETY_MARGIN_M = 0.10f;    // 障害物回避の追加安全マージン [m]
    constexpr float MAX_EVAL_DIST = 5.0f;       // 評価対象とする最大障害物距離 [m]
    constexpr float MIN_VALID_DIST = 0.10f;     // [m] これより近い点は自身の車体やノイズとみなしてBAN（無視）する
}
// ==============================================================

namespace {
    constexpr float kPi = 3.14159265358979323846f;
    float normalizeAngle(float angle) {
        angle = std::fmod(angle + kPi, 2.0f * kPi);
        if (angle <= 0.0f) angle += 2.0f * kPi;
        return angle - kPi;
    }
    float angleDifference(float a, float b) {
        return std::abs(normalizeAngle(a - b));
    }
}

// ==================== ボール情報の受信 ====================
float purpose_theta_g = 0.0f;     // ボールへの相対方位 [rad]
float purpose_distance_g = 0.0f;  // ボールまでの距離 [m]
bool purpose_detected_g = false;  // ボール検出状態
bool has_received_g = false;      // 通信受信履歴
ros::Time last_update_g;          // 最終受信時刻

void ballTargetCallback(const innu_uart::BallTarget::ConstPtr& msg)
{
    float theta = msg->theta_rad;
    float distance = msg->distance_m;
    bool detected = msg->detected;

    if (!std::isfinite(theta) || !std::isfinite(distance) || distance < 0.0f) return;

    purpose_theta_g = theta;
    purpose_distance_g = distance;
    purpose_detected_g = detected;
    has_received_g = true;
    last_update_g = ros::Time::now();
}

// ==================== LiDAR情報の受信 ====================
sensor_msgs::LaserScan latest_scan_g;
bool scan_received_g = false;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    latest_scan_g = *msg;
    scan_received_g = true;
}

// ==================== モーター制御設定 & クラス ====================
struct MotorConfig {
    int max_duty = 255;
    int pwm_freq = 3000;
    
    int translate_speed = Tuning::TRANSLATE_SPEED;
    float heading_kp = Tuning::HEADING_KP;
    float stop_distance = Tuning::STOP_DISTANCE;
    float full_speed_distance = Tuning::FULL_SPEED_DISTANCE;

    double timeout_s = 0.5;

    // ピン配置 (MA=FL, MB=FR, MC=RL, MD=RR)
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

// ==================== LiDAR経路計算モジュール ====================
class VHFRouter {
public:
    struct Config {
        float robot_radius_m = Tuning::ROBOT_RADIUS_M;
        float safety_margin_m = Tuning::SAFETY_MARGIN_M;
        float max_eval_dist = Tuning::MAX_EVAL_DIST;
        float lidar_yaw_offset_rad = 0.0f;
    };

    struct Obstacle {
        float angle;      // 障害物の方位 [rad]
        float distance;   // 障害物までの距離 [m]
        float blocked_half_width; // 障害物が占有・ブロックする角度の半幅 [rad]
    };

    VHFRouter(const Config& config) : cfg(config) {}

    bool calculateBestDirection(const sensor_msgs::LaserScan& scan, float target_theta, float& out_move_theta) {
        if (scan.ranges.empty()) return false;

        const float safety_radius = cfg.robot_radius_m + cfg.safety_margin_m;
        std::vector<Obstacle> obstacles;

        // 1. 有効な障害物点群をすべて抽出（近距離ノイズBANを適用）
        for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
            float distance = scan.ranges[i];
            float obstacle_angle = scan.angle_min + i * scan.angle_increment;

            // 無効値および MIN_VALID_DIST 以下の近距離ノイズをBANして排除
            if (!std::isfinite(distance) || 
                distance < scan.range_min || 
                distance < Tuning::MIN_VALID_DIST || 
                distance > scan.range_max || 
                distance > cfg.max_eval_dist) {
                continue;
            }

            float ratio = std::min(1.0f, safety_radius / distance);
            float blocked_half_width = std::asin(ratio);

            obstacles.push_back({obstacle_angle, distance, blocked_half_width});
        }

        float target_theta_lidar = normalizeAngle(target_theta - cfg.lidar_yaw_offset_rad);

        bool found = false;
        float best_cost = std::numeric_limits<float>::infinity();
        float best_lidar_theta = 0.0f;

        // 2. LiDARの全ビームの角度（連続的な候補角度）に対して直接通過可否とコストを評価
        for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
            float candidate_lidar = scan.angle_min + i * scan.angle_increment;

            // 候補方向が障害物ブロック領域内にあるか判定
            bool is_blocked = false;
            float min_obs_dist = std::numeric_limits<float>::infinity();

            for (const auto& obs : obstacles) {
                if (angleDifference(candidate_lidar, obs.angle) <= obs.blocked_half_width) {
                    is_blocked = true;
                    break; // ロボットの車体幅にかかる位置にあれば進行不可
                }
                float diff = angleDifference(candidate_lidar, obs.angle);
                if (diff < (kPi / 4.0f)) { // 近傍の障害物とのクリアランスを評価
                    min_obs_dist = std::min(min_obs_dist, obs.distance);
                }
            }

            if (is_blocked) continue;

            // コスト計算：目標角度からのズレ + 障害物の近さに対するペナルティ
            float angle_cost = angleDifference(candidate_lidar, target_theta_lidar);
            float clearance_penalty = 0.0f;
            if (min_obs_dist < cfg.max_eval_dist) {
                clearance_penalty = (cfg.max_eval_dist - min_obs_dist) * 0.1f;
            }

            float total_cost = angle_cost + clearance_penalty;

            if (total_cost < best_cost) {
                found = true;
                best_cost = total_cost;
                best_lidar_theta = candidate_lidar;
            }
        }

        if (!found) {
            return false;
        } else {
            out_move_theta = normalizeAngle(best_lidar_theta + cfg.lidar_yaw_offset_rad);
            return true;
        }
    }

private:
    Config cfg;
};

// ==================== メイン関数 ====================
int main(int argc, char** argv) {
    ros::init(argc, argv, "servo");
    ros::NodeHandle n;

    int pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        ROS_ERROR("pigpio start failed");
        return 1;
    }

    MotorConfig motor_cfg;
    MotorSpinner motors(motor_cfg, pi);
    motors.start();

    VHFRouter::Config vhf_cfg;
    VHFRouter router(vhf_cfg);

    ros::Subscriber ball_sub = n.subscribe<innu_uart::BallTarget>("/ball_target", 10, ballTargetCallback);
    ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);

    ros::Rate rate(50);

    while (ros::ok()) {
        bool stale = !has_received_g || (ros::Time::now() - last_update_g).toSec() > motors.timeout();

        if (stale) {
            ROS_INFO_THROTTLE(1.0, "通信ロストまたは未受信：安全停止");
            motors.stop();
        } else if (!purpose_detected_g) {
            ROS_INFO_THROTTLE(1.0, "ボール未検出：停止");
            motors.stop();
        } else if (!scan_received_g) {
            ROS_INFO_THROTTLE(1.0, "LiDAR未受信：停止");
            motors.stop();
        } else {
            float move_theta = 0.0f;
            if (router.calculateBestDirection(latest_scan_g, purpose_theta_g, move_theta)) {
                motors.DutyUpdate(move_theta, purpose_theta_g, purpose_distance_g);
                motors.PWMshootor();
            } else {
                ROS_WARN_THROTTLE(1.0, "全方向障害物ブロック：停止");
                motors.stop();
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    motors.stop();
    pigpio_stop(pi);
    return 0;
}
