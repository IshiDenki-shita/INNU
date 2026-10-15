// VHF_test.cpp
// 目的：目的の方向に近づくかつより開放的な方角を選べるかテストする（ステップ3用）
// 動作：
// - 今後の dir_rotating.cpp との結合（ステップ4）を見据え、
//   メッセージ型やメインループの構造を dir_rotating.cpp と完全に一致させています。
// - PWM出力は行わず、計算された最適な進行方向をターミナルに表示します。

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

// ==================== ボール情報の受信 (dir_rotating.cppと完全共通) ====================
float purpose_theta_g = 0.0f;
float purpose_distance_g = 0.0f;
bool purpose_detected_g = false;
bool has_received_g = false;
ros::Time last_update_g;

void ballTargetCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 3) return;
    float theta = msg->data[0];
    float distance = msg->data[1];
    bool detected = msg->data[2] > 0.5f;

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

// ==================== VHF経路計算モジュール ====================
// 結合時、このクラスはそのまま dir_rotating.cpp へコピーできます
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

    // 障害物を考慮して進むべき最適な方向(out_move_theta)を算出する
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
                scores[i] = 1; // 進行不可
            } else if (std::isinf(min_dist[i]) || min_dist[i] >= cfg.max_eval_dist) {
                scores[i] = 10; // 完全に安全
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
            
            // コスト計算：角度のズレ + 開き具合によるペナルティ
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

        std::ostringstream output;
        output << "目標方向: " << std::setw(4) << static_cast<int>(target_theta * 180.0f / kPi) << "度 | "
               << "スコア: [ ";
        for (int s : scores) {
            output << std::setw(2) << s << " ";
        }
        output << "] | ";

        if (!found) {
            output << "最適方向: NONE (全方向ブロック)";
            std::cout << output.str() << std::endl;
            return false;
        } else {
            out_move_theta = normalizeAngle(best_lidar_theta + cfg.lidar_yaw_offset_rad);
            output << "最適方向: " << std::setw(4) << static_cast<int>(out_move_theta * 180.0f / kPi)
                   << "度 (スコア: " << scores[best_sector] << ")";
            std::cout << output.str() << std::endl;
            return true;
        }
    }

private:
    Config cfg;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vhf_test_node");
    ros::NodeHandle n;

    VHFRouter::Config vhf_cfg;
    VHFRouter router(vhf_cfg);

    // dir_rotating.cpp に合わせたサブスクライバ構成
    ros::Subscriber ball_sub = n.subscribe<std_msgs::Float32MultiArray>("/ball_target", 10, ballTargetCallback);
    ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);

    ros::Rate rate(50);
    double timeout_s = 0.5;

    while (ros::ok()) {
        bool stale = !has_received_g || (ros::Time::now() - last_update_g).toSec() > timeout_s;

        if (stale) {
            ROS_INFO_THROTTLE(1.0, "通信ロストまたは未受信");
        } else if (!purpose_detected_g) {
            ROS_INFO_THROTTLE(1.0, "ボール未検出");
        } else if (!scan_received_g) {
            ROS_INFO_THROTTLE(1.0, "LiDAR未受信");
        } else {
            float move_theta = 0.0f;
            if (router.calculateBestDirection(latest_scan_g, purpose_theta_g, move_theta)) {
                // 【合体時のヒント】
                // モーター出力時はここに以下のコードを追記します：
                // motors.DutyUpdate(move_theta, purpose_theta_g, purpose_distance_g);
                // motors.PWMshootor();
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
