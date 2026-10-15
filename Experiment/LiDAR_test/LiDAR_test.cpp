// LiDAR_test.cpp
// 目的：LiDARのトピックを正しくsubscribeできるかテストする（ステップ2用）
// 動作：
// - dir_rotating.cpp や VHF_test.cpp と共通のメインループ構造を採用し、差し替えテストを容易にしています。
// - 空間を10方向に分割し、1〜10のスケールで開き具合（障害物とのマージン）を評価。
// - 検出した距離の最小値(overall_min)と最大値(overall_max)を表示。
// - 結果を1回の検出あたり1ラインでターミナルに出力します（PWM出力は行いません）。

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>

#include <cmath>
#include <algorithm>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <limits>


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

// ==================== ボール情報の受信 (dir_rotating.cpp / VHF_test.cpp と共通構造) ====================
bool has_ball_received_g = false;
ros::Time last_ball_update_g;

void ballTargetCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 3) return;
    has_ball_received_g = true;
    last_ball_update_g = ros::Time::now();
}

// ==================== LiDAR情報の受信 ====================
sensor_msgs::LaserScan latest_scan_g;
bool scan_received_g = false;
ros::Time last_scan_update_g;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    latest_scan_g = *msg;
    scan_received_g = true;
    last_scan_update_g = ros::Time::now();
}

// ==================== LiDAR評価クラス ====================
class LidarEvaluator {
public:
    struct Config {
        float robot_radius_m = 0.20f;
        float safety_margin_m = 0.10f;
        float max_eval_dist = 5.0f;
        int sector_count = 10;
    };

    LidarEvaluator(const Config& config) : cfg(config) {}

    void evaluateAndPrint(const sensor_msgs::LaserScan& scan) {
        if (scan.ranges.empty()) {
            std::cout << "LiDAR: データが空です" << std::endl;
            return;
        }

        const float sector_rad = (2.0f * kPi) / cfg.sector_count;
        const float safety_radius = cfg.robot_radius_m + cfg.safety_margin_m;
        
        std::vector<float> min_dist(cfg.sector_count, std::numeric_limits<float>::infinity());
        float overall_min = std::numeric_limits<float>::infinity();
        float overall_max = 0.0f;
        bool has_valid_measurement = false;

        for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
            float distance = scan.ranges[i];
            float obstacle_angle = scan.angle_min + i * scan.angle_increment;

            if (!std::isfinite(distance) || distance < scan.range_min || distance > scan.range_max || distance <= 0.0f) {
                continue;
            }

            has_valid_measurement = true;
            if (distance < overall_min) overall_min = distance;
            if (distance > overall_max && distance <= cfg.max_eval_dist) overall_max = distance;

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

        if (!has_valid_measurement) {
            std::cout << "LiDAR: 有効な測定値がありません" << std::endl;
            return;
        }

        // 距離を1〜10のスコア（開き具合）に変換
        std::vector<int> scores(cfg.sector_count, 10);
        for (int i = 0; i < cfg.sector_count; ++i) {
            if (min_dist[i] <= safety_radius) {
                scores[i] = 1; // 進行不可
            } else if (std::isinf(min_dist[i]) || min_dist[i] >= cfg.max_eval_dist) {
                scores[i] = 10; // 開放的
            } else {
                float ratio = (min_dist[i] - safety_radius) / (cfg.max_eval_dist - safety_radius);
                scores[i] = 1 + static_cast<int>(ratio * 9.0f);
                scores[i] = std::min(std::max(scores[i], 1), 10);

            }
        }

        // ターミナルへ1行出力
        std::ostringstream output;
        output << "距離 (Min: " << std::fixed << std::setprecision(2) << overall_min << "m, "
               << "Max: " << overall_max << "m) | "
               << "10方向スコア: [ ";
        for (int s : scores) {
            output << std::setw(2) << s << " ";
        }
        output << "]";
        std::cout << output.str() << std::endl;
    }

private:
    Config cfg;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_test_node");
    ros::NodeHandle n;

    LidarEvaluator::Config eval_cfg;
    LidarEvaluator evaluator(eval_cfg);

    // dir_rotating.cpp / VHF_test.cpp と共通のサブスクライバ設定
    ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);
    ros::Subscriber ball_sub = n.subscribe<std_msgs::Float32MultiArray>("/ball_target", 10, ballTargetCallback);

    ros::Rate rate(50);
    double scan_timeout_s = 0.5;

    while (ros::ok()) {
        bool scan_stale = !scan_received_g || (ros::Time::now() - last_scan_update_g).toSec() > scan_timeout_s;

        if (scan_stale) {
            ROS_INFO_THROTTLE(1.0, "LiDARデータ未受信またはタイムアウト");
        } else {
            // LiDARデータを評価して表示
            evaluator.evaluateAndPrint(latest_scan_g);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
