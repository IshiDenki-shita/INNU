
#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <algorithm>

#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"  // 画像ラズパイからの目的方角。メッセージ型は仮（詳細未定）


constexpr int MAX_DATA_SIZE = 8000;  // ROSのLiDAR点群バッファの長さ

int purpose_idx = 0;           // 画像ラズパイから来たボールへの方角（インデックス）


void directionCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // 詳しい実装は未定。ここでは受信したthetaをインデックスに変換して保持するのみ
    float purpose_theta = msg->data;
    purpose_idx = static_cast<int>(
        static_cast<float>(MAX_DATA_SIZE) * (purpose_theta / M_PI) / 2.0f);
}

// ---- 経路計算の設定（MotorConfigに相当。実験で決める係数のみ持つ） ----
struct LidarRoutingConfig
{
    float RADIUS_OF_MYSELF = 1.0f;      // ロボット自身の半径
    float THRESH_DANGER_DIST = 2.0f;    // 危険な壁の近さ
    float alpha = 2.0f;                 // 方角ごとのヒューリスティックの係数（実験で決定）
    float beta = 3.0f;
};

class LidarRouting
{
public:
    explicit LidarRouting(const LidarRoutingConfig& config)
        : cfg_(config),
          real_data_(MAX_DATA_SIZE, 0.0f),
          temp_data_(MAX_DATA_SIZE, 0.0f)
    {
    }

    // LiDARからの受信割り込み（メンバ関数コールバック）
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        int n = std::min(static_cast<int>(scan->ranges.size()), MAX_DATA_SIZE);
        for (int i = 0; i < n; ++i)
        {
            real_data_[i] = scan->ranges[i];
        }
    }

    // 壁が近い点からどの範囲に衝突危険性があるか計算
    int calc_invalid_width(float dist) const
    {
        // dist <= RADIUS_OF_MYSELF だとasinの定義域外になるためクランプ
        float ratio = std::clamp(cfg_.RADIUS_OF_MYSELF / dist, -1.0f, 1.0f);
        float invalid_theta = std::asin(ratio);
        float invalid_width = MAX_DATA_SIZE * invalid_theta / static_cast<float>(M_PI) / 2.0f;
        return static_cast<int>(invalid_width);
    }

    // 方角ごとの経路としての妥当さを計算（壁から遠いほど高スコア）
    float calc_direction_score(float dist, int idx) const
    {
        int temp_idx = purpose_idx;

        int raw_delta = std::abs(temp_idx - idx);
        int delta_idx = std::min(raw_delta, MAX_DATA_SIZE - raw_delta);
        delta_idx = std::max(delta_idx, 1);  // 0除算防止（目的方角そのものは最小距離1として扱う）

        return std::exp(dist * cfg_.alpha) / (static_cast<float>(delta_idx) * cfg_.beta);
    }

    // 進行方向として最も妥当な方角を計算
    float calc_route()
    {
        temp_data_ = real_data_;  // データを取り出してから計算開始

        std::vector<int> valid_data(MAX_DATA_SIZE, 1);

        // 危険な（壁が近い）方向とその周辺の角度を排除
        for (int i = 0; i < MAX_DATA_SIZE; ++i)
        {
            if (temp_data_[i] < cfg_.THRESH_DANGER_DIST)
            {
                int invalid_width = calc_invalid_width(temp_data_[i]);
                int start = std::max(i - invalid_width, 0);
                int finish = std::min(i + invalid_width, MAX_DATA_SIZE);

                for (int j = start; j < finish; ++j)
                {
                    valid_data[j] = 0;
                }
            }
        }

        // 壁が遠くて目的に近い方向を選択
        float best_score = -1.0f;
        int best_dir = 0;

        for (int i = 0; i < MAX_DATA_SIZE; ++i)
        {
            if (!valid_data[i])
            {
                continue;  // 危険と判定された方向はスキップ
            }

            float score = calc_direction_score(temp_data_[i], i);

            if (score > best_score)
            {
                best_score = score;
                best_dir = i;
            }
        }

        // θに変換して完了
        return static_cast<float>(M_PI) * static_cast<float>(best_dir) / static_cast<float>(MAX_DATA_SIZE);
    }

private:
    LidarRoutingConfig cfg_;
    std::vector<float> real_data_;
    std::vector<float> temp_data_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_routing");
    ros::NodeHandle n;

    LidarRoutingConfig cfg;
    LidarRouting routing(cfg);

    ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>(
        "/scan", 10, &LidarRouting::scanCallback, &routing);
    ros::Subscriber dir_sub = n.subscribe<std_msgs::Float32>(
        "/purpose_direction", 10, directionCallback);

    ros::Rate rate(10);

    while (ros::ok())
    {
        float direction = routing.calc_route();
        ROS_INFO("best_direction = %f", direction);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}