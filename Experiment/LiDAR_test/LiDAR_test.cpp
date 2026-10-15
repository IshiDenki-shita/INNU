// LiDAR_test.cpp
// 目的：LiDARのトピックを正しくsubscribeできるか確認する
// 動作：
// - 空間を10方向に分割し、1〜10のスケールで開き具合を評価。
// - 検出した距離の最小値と最大値も表示。
// - 結果を1回の検出あたり1ラインでターミナルに出力します。

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <algorithm>
#include <cmath>
#include <limits>
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

class LidarTestNode {
public:
    LidarTestNode(ros::NodeHandle& nh) : nh_(nh) {
        // パラメータ
        nh_.param("robot_radius_m", robot_radius_m_, 0.20);
        nh_.param("safety_margin_m", safety_margin_m_, 0.10);
        nh_.param("max_eval_dist", max_eval_dist_, 5.0);
        sector_count_ = 10;

        scan_sub_ = nh_.subscribe("/scan", 10, &LidarTestNode::scanCallback, this);
    }

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        if (scan->ranges.empty() || !std::isfinite(scan->angle_increment) || scan->angle_increment == 0.0) {
            return;
        }

        const double sector_rad = (2.0 * kPi) / sector_count_;
        std::vector<double> min_dist(sector_count_, std::numeric_limits<double>::infinity());
        
        double overall_min = std::numeric_limits<double>::infinity();
        double overall_max = 0.0;
        const double safety_radius = robot_radius_m_ + safety_margin_m_;
        bool has_valid_measurement = false;

        for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
            const double distance = scan->ranges[i];
            const double obstacle_angle = scan->angle_min + i * scan->angle_increment;

            if (!std::isfinite(distance) || distance < scan->range_min || distance > scan->range_max || distance <= 0.0) {
                continue;
            }

            has_valid_measurement = true;

            if (distance < overall_min) overall_min = distance;
            if (distance > overall_max && distance <= max_eval_dist_) overall_max = distance;

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

        if (!has_valid_measurement) {
            std::cout << "有効なLiDARデータがありません。" << std::endl;
            return;
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

        std::ostringstream output;
        output << "距離(Min: " << std::fixed << std::setprecision(2) << overall_min << "m, "
               << "Max: " << overall_max << "m) | "
               << "スコア: [ ";
        for (int s : scores) {
            output << std::setw(2) << s << " ";
        }
        output << "]";
        std::cout << output.str() << std::endl;
    }

    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;

    double robot_radius_m_;
    double safety_margin_m_;
    double max_eval_dist_;
    int sector_count_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_test_node");
    ros::NodeHandle nh("~");
    LidarTestNode node(nh);
    ros::spin();
    return 0;
}
