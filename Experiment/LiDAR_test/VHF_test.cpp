// VHF_test.cpp
// 目的：目的の方向に近づくかつより開放的な方角を選べるかテストする
// 動作：
// - LiDARと目標方向(ball_target)をsubscribe。
// - VHFロジックで安全かつ目標に近く、より開放的な方向を算出。
// - 選んだ方向を表示（1回の検出あたり1ライン出力）。
// - 他のノード（dir_rotating.cpp等）と結合できるようPWM出力を廃止し純粋な角度計算モジュールとして構築。

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <innu_msgs/BallTarget.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace {
    constexpr double kPi = 3.14159265358979323846;

    double normalizeAngle(double angle) {
        angle = std::fmod(angle + kPi, 2.0 * kPi);
        if (angle <= 0.0) angle += 2.0 * kPi;
        return angle - kPi;
    }

    double angleDifference(double a, double b) {
        return std::abs(normalizeAngle(a - b));
    }
}

class VHFTestNode {
public:
    VHFTestNode(ros::NodeHandle& nh) : nh_(nh) {
        // パラメータ
        nh_.param("robot_radius_m", robot_radius_m_, 0.20);
        nh_.param("safety_margin_m", safety_margin_m_, 0.10);
        nh_.param("max_eval_dist", max_eval_dist_, 5.0);
        nh_.param("lidar_yaw_offset_rad", lidar_yaw_offset_rad_, 0.0);
        sector_count_ = 10;

        scan_sub_ = nh_.subscribe("/scan", 10, &VHFTestNode::scanCallback, this);
        ball_sub_ = nh_.subscribe("/ball_target", 10, &VHFTestNode::ballCallback, this);
        
        // メインループ(50Hz相当)
        timer_ = nh_.createTimer(ros::Duration(0.02), &VHFTestNode::processCallback, this);
    }

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_scan_ = *msg;
        scan_received_ = true;
    }

    void ballCallback(const innu_msgs::BallTarget::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        ball_received_ = msg->detected;
        if (msg->detected) {
            ball_theta_rad_ = msg->theta_rad;
        }
    }

    void processCallback(const ros::TimerEvent&) {
        sensor_msgs::LaserScan scan;
        bool has_scan = false;
        double target_theta = 0.0;
        bool has_target = false;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            scan = latest_scan_;
            has_scan = scan_received_;
            target_theta = ball_theta_rad_;
            has_target = ball_received_;
        }

        if (!has_scan || scan.ranges.empty()) {
            return;
        }

        const double sector_rad = (2.0 * kPi) / sector_count_;
        const double safety_radius = robot_radius_m_ + safety_margin_m_;
        std::vector<double> min_dist(sector_count_, std::numeric_limits<double>::infinity());

        for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
            const double distance = scan.ranges[i];
            const double obstacle_angle = scan.angle_min + i * scan.angle_increment;

            if (!std::isfinite(distance) || distance < scan.range_min || distance > scan.range_max || distance <= 0.0) {
                continue;
            }

            const double ratio = std::min(1.0, safety_radius / distance);
            const double blocked_half_width = std::asin(ratio);

            for (int sector = 0; sector < sector_count_; ++sector) {
                const double candidate_lidar = normalizeAngle(-kPi + sector * sector_rad);
                if (angleDifference(candidate_lidar, obstacle_angle) <= blocked_half_width + (sector_rad * 0.5)) {
                    if (distance < min_dist[sector]) {
                        min_dist[sector] = distance;
                    }
                }
            }
        }

        std::vector<int> scores(sector_count_, 10);
        for (int i = 0; i < sector_count_; ++i) {
            if (min_dist[i] <= safety_radius) {
                scores[i] = 1;
            } else if (std::isinf(min_dist[i]) || min_dist[i] >= max_eval_dist_) {
                scores[i] = 10;
            } else {
                double ratio = (min_dist[i] - safety_radius) / (max_eval_dist_ - safety_radius);
                scores[i] = 1 + static_cast<int>(ratio * 9.0);
                scores[i] = std::clamp(scores[i], 1, 10);
            }
        }

        // ターゲット未検出時は正面(0.0)を仮の目標とする
        const double target_theta_lidar = normalizeAngle((has_target ? target_theta : 0.0) - lidar_yaw_offset_rad_);

        bool found = false;
        double best_cost = std::numeric_limits<double>::infinity();
        int best_sector = -1;
        double best_lidar_theta = 0.0;

        for (int sector = 0; sector < sector_count_; ++sector) {
            if (scores[sector] <= 1) continue; 

            const double candidate_lidar = normalizeAngle(-kPi + sector * sector_rad);
            
            // VHFのロジック: 目標方向への近さ(コスト) ＋ 開き具合によるペナルティ
            // スコアが低い(障害物が近い)ほどペナルティを重くする
            const double angle_cost = angleDifference(candidate_lidar, target_theta_lidar);
            const double clearance_penalty = (10 - scores[sector]) * 0.2; // 調整用ウェイト

            const double total_cost = angle_cost + clearance_penalty;

            if (total_cost < best_cost) {
                found = true;
                best_cost = total_cost;
                best_sector = sector;
                best_lidar_theta = candidate_lidar;
            }
        }

        std::ostringstream output;
        output << "目標方向: " << std::setw(4) << static_cast<int>((has_target ? target_theta : 0.0) * 180.0 / kPi) << "度 | "
               << "スコア: [ ";
        for (int s : scores) {
            output << std::setw(2) << s << " ";
        }
        output << "] | ";

        if (!found) {
            output << "最適方向: NONE (全方向ブロック)";
        } else {
            const double move_theta_body = normalizeAngle(best_lidar_theta + lidar_yaw_offset_rad_);
            output << "最適方向: " << std::setw(4) << static_cast<int>(move_theta_body * 180.0 / kPi)
                   << "度 (スコア: " << scores[best_sector] << ")";
        }
        
        std::cout << output.str() << std::endl;
    }

    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Subscriber ball_sub_;
    ros::Timer timer_;

    std::mutex mutex_;
    sensor_msgs::LaserScan latest_scan_;
    bool scan_received_ = false;
    double ball_theta_rad_ = 0.0;
    bool ball_received_ = false;

    double robot_radius_m_;
    double safety_margin_m_;
    double max_eval_dist_;
    double lidar_yaw_offset_rad_;
    int sector_count_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vhf_test_node");
    ros::NodeHandle nh("~");
    VHFTestNode node(nh);
    ros::spin();
    return 0;
}
