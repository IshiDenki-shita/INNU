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
    int max_duty = 255;         // デューティー比の最大値
    int pwm_freq = 3000;        // PWM周波数
    int translate_speed = 150;  // 並進方向の速度(duty)
    float heading_kp = 80.0f;   // ボール方向へ姿勢を向けるための比例ゲイン

    float stop_distance = 0.3f;        // [m] これ以下では並進せず姿勢合わせのみ
    float full_speed_distance = 1.5f;  // [m] これ以上で並進速度最大

    double timeout_s = 0.5;  // 通信ロスト判定閾値 [s]

    // モーターのピン配置 (MA=FL, MB=FR, MC=RL, MD=RR)
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

    // move_theta: 障害物回避を考慮した進行目標方向 [rad]
    // ball_theta: ロボットの正面を向けるボール方向 [rad]
    // distance  : ボールまでの距離 [m]
    void DutyUpdate(float move_theta, float ball_theta, float distance) {
        float scale = CalcTranslateScale(distance);
        float vx = scale * std::cos(move_theta);
        float vy = scale * std::sin(move_theta);

        // 姿勢制御（常にボール方向を向く）
        float w = cfg.heading_kp * ball_theta;

        // メカナム制御式
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

// ==================== VHF経路計算モジュール ====================
class VHFRouter {
public:
    struct Config {
        float robot_radius_m = 0.20f;
        float safety_margin_m = 0.10f;
        float max_eval_dist = 5.0f;
        float lidar_yaw_offset_rad = 0.0f;
        int sector_count = 10;
    };

    VHFRouter(const Config& config) : cfg(config) {}

    bool calculateBestDirection(const sensor_msgs::LaserScan& scan, float target_theta, float& out_move_theta) {
        if (scan.ranges.empty()) return false;

        const float sector_rad = (2.0f * kPi) / cfg.sector_count;
        const float safety_radius = cfg.robot_radius_m + cfg.safety_margin_m;
        std::vector<float> min_dist(cfg.sector_count, std::numeric_limits<float>::infinity());

        for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
            float distance = scan.ranges[i];
            float obstacle_angle = scan.angle_min + i * scan.angle_increment;

            if (!std::isfinite(distance) || distance < scan.range_min || distance > scan.range_max || distance <= 0.0f) {
                continue;
            }

            float ratio = std::min(1.0f, safety_radius / distance);
            float blocked_half_width = std::asin(ratio);

            for (int sector = 0; sector < cfg.sector_count; ++sector) {
                float candidate_lidar = normalizeAngle(-kPi + sector * sector_rad);
                if (angleDifference(candidate_lidar, obstacle_angle) <= blocked_half_width + (sector_rad * 0.5f)) {
                    if (distance < min_dist[sector]) {
                        min_dist[sector] = distance;
                    }
                }
            }
        }

        std::vector<int> scores(cfg.sector_count, 10);
        for (int i = 0; i < cfg.sector_count; ++i) {
            if (min_dist[i] <= safety_radius) {
                scores[i] = 1;
            } else if (std::isinf(min_dist[i]) || min_dist[i] >= cfg.max_eval_dist) {
                scores[i] = 10;
            } else {
                float ratio = (min_dist[i] - safety_radius) / (cfg.max_eval_dist - safety_radius);
                scores[i] = 1 + static_cast<int>(ratio * 9.0f);
                scores[i] = std::min(std::max(scores[i], 1), 10);
            }
        }

        float target_theta_lidar = normalizeAngle(target_theta - cfg.lidar_yaw_offset_rad);
        bool found = false;
        float best_cost = std::numeric_limits<float>::infinity();
        int best_sector = -1;
        float best_lidar_theta = 0.0f;

        for (int sector = 0; sector < cfg.sector_count; ++sector) {
            if (scores[sector] <= 1) continue; 

            float candidate_lidar = normalizeAngle(-kPi + sector * sector_rad);
            
            float angle_cost = angleDifference(candidate_lidar, target_theta_lidar);
            float clearance_penalty = (10 - scores[sector]) * 0.2f; 
            float total_cost = angle_cost + clearance_penalty;

            if (total_cost < best_cost) {
                found = true;
                best_cost = total_cost;
                best_sector = sector;
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

    // pigpio デーモン接続初期化
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

    // サブスクライバの登録
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
            // VHF経路計算の呼び出し
            if (router.calculateBestDirection(latest_scan_g, purpose_theta_g, move_theta)) {
                // 回避角度 move_theta、ボール方位 purpose_theta_g、ボール距離 purpose_distance_g をもとにデューティ比更新
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
