// VHFp2_test.cpp
// 元の lidar_motion_node.cpp をベースにしたテスト版。
// 空間を10方向に分割し、1〜10のスケールで開き具合を評価。
// 結果を1回の検出あたり1ラインでターミナルに出力します。

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <innu_msgs/BallTarget.h>
#include <pigpiod_if2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace {

constexpr double kPi = 3.14159265358979323846;

// 角度を -π < angle <= π に正規化する。
double normalizeAngle(double angle) {
    angle = std::fmod(angle + kPi, 2.0 * kPi);
    if (angle <= 0.0) {
        angle += 2.0 * kPi;
    }
    return angle - kPi;
}

double angleDifference(double a, double b) {
    return std::abs(normalizeAngle(a - b));
}

bool isValidTheta(double theta) {
    return std::isfinite(theta) && theta > -kPi && theta <= kPi;
}

bool isValidDistance(double distance) {
    return std::isfinite(distance) && distance >= 0.0;
}

}  // namespace

struct RoutingConfig {
    double robot_radius_m = 0.20;
    double safety_margin_m = 0.10;
    // テスト用に10方向（36度刻み）に分割
    int sector_count = 10;
    double max_eval_dist = 5.0; // 完全に進行可能と見なす最大距離

    // LiDARの正面と車体正面のずれ。実機で校正する。
    double lidar_yaw_offset_rad = 0.0;

    // 受信停止・LiDAR停止を検出する猶予時間。
    double ball_timeout_sec = 0.50;
    double scan_timeout_sec = 0.50;
};

struct MotorConfig {
    int pwm_frequency = 3000;
    int max_duty = 255;
    int translate_duty = 150;
    double heading_kp = 80.0;
    double max_turn_component = 1.0;

    // 実機配線と一致することを要確認。
    std::array<unsigned, 8> pins = {
        22, 10,  // 前左
        12, 18,  // 前右
        13, 19,  // 後左
        20, 21   // 後右
    };
};

class LidarMotionNode {
public:
    LidarMotionNode(ros::NodeHandle& nh, int pigpio_handle)
        : nh_(nh), pi_(pigpio_handle) {
        loadParameters();

        scan_sub_ = nh_.subscribe(
            "/scan", 10, &LidarMotionNode::scanCallback, this);

        ball_sub_ = nh_.subscribe(
            "/ball_target", 10, &LidarMotionNode::ballCallback, this);

        // 50 Hzで経路計算とPWM出力を行う。
        control_timer_ = nh_.createTimer(
            ros::Duration(0.02),
            &LidarMotionNode::controlTimerCallback,
            this);
    }

    bool startMotor() {
        if (pi_ < 0) {
            ROS_ERROR("pigpioへの接続に失敗しました。");
            return false;
        }

        for (unsigned pin : motor_cfg_.pins) {
            set_PWM_frequency(pi_, pin, motor_cfg_.pwm_frequency);
            set_PWM_dutycycle(pi_, pin, 0);
        }
        return true;
    }

    void stopMotor() {
        if (pi_ >= 0) {
            for (unsigned pin : motor_cfg_.pins) {
                set_PWM_dutycycle(pi_, pin, 0);
            }
        }
    }

private:
    struct BallState {
        bool received = false;
        bool valid = false;
        bool detected = false;
        double theta_rad = 0.0;
        double distance_m = 0.0;
        ros::WallTime received_at;
    };

    void loadParameters() {
        nh_.param("robot_radius_m", routing_cfg_.robot_radius_m, routing_cfg_.robot_radius_m);
        nh_.param("safety_margin_m", routing_cfg_.safety_margin_m, routing_cfg_.safety_margin_m);
        nh_.param("lidar_yaw_offset_rad", routing_cfg_.lidar_yaw_offset_rad, routing_cfg_.lidar_yaw_offset_rad);
        nh_.param("ball_timeout_sec", routing_cfg_.ball_timeout_sec, routing_cfg_.ball_timeout_sec);
        nh_.param("scan_timeout_sec", routing_cfg_.scan_timeout_sec, routing_cfg_.scan_timeout_sec);
        nh_.param("translate_duty", motor_cfg_.translate_duty, motor_cfg_.translate_duty);
        nh_.param("heading_kp", motor_cfg_.heading_kp, motor_cfg_.heading_kp);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_scan_ = *msg;
        scan_received_ = true;
        scan_received_at_ = ros::WallTime::now();
    }

    void ballCallback(const innu_msgs::BallTarget::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        ball_.received = true;
        ball_.received_at = ros::WallTime::now();
        ball_.detected = msg->detected;

        if (!msg->detected) {
            ball_.valid = true;
            ball_.theta_rad = 0.0;
            ball_.distance_m = 0.0;
            return;
        }

        if (!isValidTheta(msg->theta_rad) || !isValidDistance(msg->distance_m)) {
            ball_.valid = false;
            return;
        }

        ball_.valid = true;
        ball_.theta_rad = msg->theta_rad;
        ball_.distance_m = msg->distance_m;
    }

    void controlTimerCallback(const ros::TimerEvent&) {
        sensor_msgs::LaserScan scan;
        BallState ball;
        bool scan_received = false;
        ros::WallTime scan_received_at;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            scan = latest_scan_;
            ball = ball_;
            scan_received = scan_received_;
            scan_received_at = scan_received_at_;
        }

        const ros::WallTime now = ros::WallTime::now();

        if (!ball.received || !ball.valid || !ball.detected ||
            (now - ball.received_at).toSec() >= routing_cfg_.ball_timeout_sec) {
            stopMotor();
            return;
        }

        if (!scan_received || (now - scan_received_at).toSec() >= routing_cfg_.scan_timeout_sec) {
            stopMotor();
            return;
        }

        double move_theta_body = 0.0;
        if (!calculateSafeDirection(scan, ball.theta_rad, move_theta_body)) {
            stopMotor();
            return;
        }

        driveMecanum(move_theta_body, ball.theta_rad);
    }

    bool calculateSafeDirection(
        const sensor_msgs::LaserScan& scan,
        double ball_theta_body,
        double& move_theta_body) const {
        
        if (scan.ranges.empty() || !std::isfinite(scan.angle_increment) || scan.angle_increment == 0.0) {
            return false;
        }

        const int sector_count = routing_cfg_.sector_count;
        const double sector_rad = (2.0 * kPi) / sector_count;
        
        // 各セクターの最小障害物距離（初期値は無限大）
        std::vector<double> min_dist(sector_count, std::numeric_limits<double>::infinity());
        
        const double target_theta_lidar = normalizeAngle(ball_theta_body - routing_cfg_.lidar_yaw_offset_rad);
        const double safety_radius = routing_cfg_.robot_radius_m + routing_cfg_.safety_margin_m;

        bool has_valid_measurement = false;

        // 点群から各セクターの最小距離を求める
        for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
            const double distance = scan.ranges[i];
            const double obstacle_angle = scan.angle_min + i * scan.angle_increment;

            if (!std::isfinite(distance) || distance < scan.range_min || distance > scan.range_max || distance <= 0.0) {
                continue;
            }

            has_valid_measurement = true;

            // ロボットの外接円に対して衝突し得る角度範囲を算出
            const double ratio = std::min(1.0, safety_radius / distance);
            const double blocked_half_width = std::asin(ratio);

            for (int sector = 0; sector < sector_count; ++sector) {
                const double candidate_lidar = normalizeAngle(-kPi + sector * sector_rad);
                
                // 障害物がセクターに被る場合、最小距離を更新
                if (angleDifference(candidate_lidar, obstacle_angle) <= blocked_half_width + (sector_rad * 0.5)) {
                    if (distance < min_dist[sector]) {
                        min_dist[sector] = distance;
                    }
                }
            }
        }

        if (!has_valid_measurement) return false;

        // 距離から開き具合（1〜10）へスコア化
        std::vector<int> scores(sector_count, 10);
        for (int i = 0; i < sector_count; ++i) {
            if (min_dist[i] <= safety_radius) {
                scores[i] = 1; // 進行不可
            } else if (std::isinf(min_dist[i]) || min_dist[i] >= routing_cfg_.max_eval_dist) {
                scores[i] = 10; // 完全に進行可能
            } else {
                double ratio = (min_dist[i] - safety_radius) / (routing_cfg_.max_eval_dist - safety_radius);
                scores[i] = 1 + static_cast<int>(ratio * 9.0);
                scores[i] = std::clamp(scores[i], 1, 10);
            }
        }

        bool found = false;
        double best_cost = std::numeric_limits<double>::infinity();
        int best_sector = -1;
        double best_lidar_theta = 0.0;

        // ボール方向に最も近い、安全な進行方向を選ぶ
        for (int sector = 0; sector < sector_count; ++sector) {
            if (scores[sector] <= 1) continue; // 障害物がある方向は除外

            const double candidate_lidar = normalizeAngle(-kPi + sector * sector_rad);
            const double cost = angleDifference(candidate_lidar, target_theta_lidar);

            if (cost < best_cost) {
                found = true;
                best_cost = cost;
                best_sector = sector;
                best_lidar_theta = candidate_lidar;
            }
        }

        // ターミナルへ1行で出力
        std::ostringstream output;
        output << "セクタースコア(1-10): [ ";
        for (int s : scores) {
            output << std::setw(2) << s << " ";
        }
        output << "] ";

        if (!found) {
            output << "| 最適方向: NONE (全方向ブロック)";
            std::cout << output.str() << std::endl;
            return false;
        }

        move_theta_body = normalizeAngle(best_lidar_theta + routing_cfg_.lidar_yaw_offset_rad);
        
        output << "| 最適方向: " 
               << std::setw(4) << static_cast<int>(best_lidar_theta * 180.0 / kPi) 
               << " 度 (スコア: " << scores[best_sector] << ")";
        std::cout << output.str() << std::endl;

        return true;
    }

    void driveMecanum(double move_theta, double ball_theta) {
        // [元のdriveMecanum処理は変更なし]
        const double vx = std::cos(move_theta);
        const double vy = std::sin(move_theta);

        const double w = std::clamp(
            motor_cfg_.heading_kp * ball_theta,
            -motor_cfg_.max_turn_component,
            motor_cfg_.max_turn_component);

        std::array<double, 4> wheel = {
            vx - vy - w,  // front-left
            vx + vy + w,  // front-right
            vx + vy - w,  // rear-left
            vx - vy + w   // rear-right
        };

        double max_abs = 1.0;
        for (double value : wheel) {
            max_abs = std::max(max_abs, std::abs(value));
        }

        for (int i = 0; i < 4; ++i) {
            const double normalized = wheel[i] / max_abs;
            const int duty = static_cast<int>(std::lround(
                std::min(
                    std::abs(normalized) * motor_cfg_.translate_duty,
                    static_cast<double>(motor_cfg_.max_duty))));

            if (pi_ < 0) continue; // テスト環境でpigpioが無くても落ちないように保護

            const unsigned forward_pin = motor_cfg_.pins[i * 2];
            const unsigned reverse_pin = motor_cfg_.pins[i * 2 + 1];

            if (normalized >= 0.0) {
                set_PWM_dutycycle(pi_, forward_pin, duty);
                set_PWM_dutycycle(pi_, reverse_pin, 0);
            } else {
                set_PWM_dutycycle(pi_, forward_pin, 0);
                set_PWM_dutycycle(pi_, reverse_pin, duty);
            }
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Subscriber ball_sub_;
    ros::Timer control_timer_;

    int pi_;
    RoutingConfig routing_cfg_;
    MotorConfig motor_cfg_;

    std::mutex mutex_;
    sensor_msgs::LaserScan latest_scan_;
    bool scan_received_ = false;
    ros::WallTime scan_received_at_;
    BallState ball_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_motion_node_test");
    ros::NodeHandle nh("~");

    // テスト時にpigpioが動作しない環境でもシミュレートできるようにする
    int pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) {
        ROS_WARN("pigpioへの接続に失敗しました。ハードウェア出力はスキップしロジックのみ実行します。");
    }

    LidarMotionNode node(nh, pi);

    if (pi >= 0 && !node.startMotor()) {
        return 1;
    }

    ros::spin();

    node.stopMotor();
    if (pi >= 0) {
        pigpio_stop(pi);
    }
    return 0;
}