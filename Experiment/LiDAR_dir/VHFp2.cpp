// lidar_motion_node.cpp
// LiDAR経路計算とメカナムPWM出力を同じROSノードで行う。
// UARTバイト列の解析は行わず、uart_receiver_nodeがpublishする
// /ball_target だけを受け取る。

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
    double sector_rad = 5.0 * kPi / 180.0;

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
        4, 10,   // 前左
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
        for (unsigned pin : motor_cfg_.pins) {
            set_PWM_dutycycle(pi_, pin, 0);
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
        nh_.param("robot_radius_m",
                  routing_cfg_.robot_radius_m,
                  routing_cfg_.robot_radius_m);
        nh_.param("safety_margin_m",
                  routing_cfg_.safety_margin_m,
                  routing_cfg_.safety_margin_m);
        nh_.param("sector_deg", sector_deg_, 5.0);
        routing_cfg_.sector_rad = sector_deg_ * kPi / 180.0;

        nh_.param("lidar_yaw_offset_rad",
                  routing_cfg_.lidar_yaw_offset_rad,
                  routing_cfg_.lidar_yaw_offset_rad);
        nh_.param("ball_timeout_sec",
                  routing_cfg_.ball_timeout_sec,
                  routing_cfg_.ball_timeout_sec);
        nh_.param("scan_timeout_sec",
                  routing_cfg_.scan_timeout_sec,
                  routing_cfg_.scan_timeout_sec);

        nh_.param("translate_duty",
                  motor_cfg_.translate_duty,
                  motor_cfg_.translate_duty);
        nh_.param("heading_kp",
                  motor_cfg_.heading_kp,
                  motor_cfg_.heading_kp);
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

        // detected=false は通信としては正常だが、走行は停止する。
        if (!msg->detected) {
            ball_.valid = true;
            ball_.theta_rad = 0.0;
            ball_.distance_m = 0.0;
            return;
        }

        // UART受信ノードでも検証するが、制御ノード側でも防御的に検証する。
        if (!isValidTheta(msg->theta_rad) ||
            !isValidDistance(msg->distance_m)) {
            ROS_WARN_THROTTLE(
                1.0, "不正なボール情報を受信したため停止します。");
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

        // 通信断、未検出、不正データでは必ず停止する。
        if (!ball.received ||
            !ball.valid ||
            !ball.detected ||
            (now - ball.received_at).toSec() >= routing_cfg_.ball_timeout_sec) {
            ROS_WARN_THROTTLE(1.0, "ボール情報が無効またはタイムアウト。停止します。");
            stopMotor();
            return;
        }

        // LiDAR未受信・更新停止でも必ず停止する。
        if (!scan_received ||
            (now - scan_received_at).toSec() >= routing_cfg_.scan_timeout_sec) {
            ROS_WARN_THROTTLE(1.0, "LiDARデータが無効またはタイムアウト。停止します。");
            stopMotor();
            return;
        }

        double move_theta_body = 0.0;
        if (!calculateSafeDirection(scan, ball.theta_rad, move_theta_body)) {
            ROS_WARN_THROTTLE(1.0, "安全な進行方向がないため停止します。");
            stopMotor();
            return;
        }

        // ball_theta と move_theta は別の値として扱う。
        driveMecanum(move_theta_body, ball.theta_rad);
    }

    bool calculateSafeDirection(
        const sensor_msgs::LaserScan& scan,
        double ball_theta_body,
        double& move_theta_body) const {
        if (scan.ranges.empty() ||
            !std::isfinite(scan.angle_increment) ||
            scan.angle_increment == 0.0) {
            return false;
        }

        const int sector_count =
            static_cast<int>(std::ceil((2.0 * kPi) / routing_cfg_.sector_rad));

        std::vector<bool> safe(sector_count, true);
        std::vector<bool> covered(sector_count, false);

        // 画像側のball_thetaは車体座標系。
        // LiDAR座標系へ変換して、候補方向との比較に用いる。
        const double target_theta_lidar = normalizeAngle(
            ball_theta_body - routing_cfg_.lidar_yaw_offset_rad);

        for (int sector = 0; sector < sector_count; ++sector) {
            const double candidate_lidar =
                normalizeAngle(-kPi + sector * routing_cfg_.sector_rad);

            // LiDARの実際の測定角度範囲外は安全と見なさない。
            for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
                const double scan_angle =
                    scan.angle_min + i * scan.angle_increment;

                if (angleDifference(scan_angle, candidate_lidar) <=
                    routing_cfg_.sector_rad * 0.5) {
                    covered[sector] = true;
                    break;
                }
            }
        }

        bool has_valid_measurement = false;

        for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
            const double distance = scan.ranges[i];
            const double obstacle_angle =
                scan.angle_min + i * scan.angle_increment;

            // inf は「測定上限まで障害物なし」として扱う。
            if (std::isinf(distance) && distance > 0.0) {
                has_valid_measurement = true;
                continue;
            }

            // NaN、0、範囲外の値は障害物計算に使用しない。
            if (!std::isfinite(distance) ||
                distance < scan.range_min ||
                distance > scan.range_max ||
                distance <= 0.0) {
                continue;
            }

            has_valid_measurement = true;

            const double safety_radius =
                routing_cfg_.robot_radius_m + routing_cfg_.safety_margin_m;

            // ロボットの外接円に対して衝突し得る角度範囲を算出する。
            const double ratio = std::min(1.0, safety_radius / distance);
            const double blocked_half_width = std::asin(ratio);

            for (int sector = 0; sector < sector_count; ++sector) {
                const double candidate_lidar =
                    normalizeAngle(-kPi + sector * routing_cfg_.sector_rad);

                if (angleDifference(candidate_lidar, obstacle_angle) <=
                    blocked_half_width) {
                    safe[sector] = false;
                }
            }
        }

        if (!has_valid_measurement) {
            return false;
        }

        bool found = false;
        double best_cost = std::numeric_limits<double>::infinity();
        double best_lidar_theta = 0.0;

        for (int sector = 0; sector < sector_count; ++sector) {
            if (!covered[sector] || !safe[sector]) {
                continue;
            }

            const double candidate_lidar =
                normalizeAngle(-kPi + sector * routing_cfg_.sector_rad);

            // 目標（ボール）に最も近い安全な進行方向を選ぶ。
            const double cost =
                angleDifference(candidate_lidar, target_theta_lidar);

            if (cost < best_cost) {
                found = true;
                best_cost = cost;
                best_lidar_theta = candidate_lidar;
            }
        }

        if (!found) {
            return false;
        }

        // LiDAR座標系から車体座標系へ戻す。
        move_theta_body = normalizeAngle(
            best_lidar_theta + routing_cfg_.lidar_yaw_offset_rad);
        return true;
    }

    void driveMecanum(double move_theta, double ball_theta) {
        const double vx = std::cos(move_theta);
        const double vy = std::sin(move_theta);

        // ボールの方向へ車体を向ける回転成分。
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
    double sector_deg_ = 5.0;

    std::mutex mutex_;
    sensor_msgs::LaserScan latest_scan_;
    bool scan_received_ = false;
    ros::WallTime scan_received_at_;
    BallState ball_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_motion_node");
    ros::NodeHandle nh("~");

    const int pi = pigpio_start(nullptr, nullptr);
    LidarMotionNode node(nh, pi);

    if (!node.startMotor()) {
        return 1;
    }

    ros::spin();

    node.stopMotor();
    if (pi >= 0) {
        pigpio_stop(pi);
    }
    return 0;
}