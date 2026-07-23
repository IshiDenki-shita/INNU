/*
UARTがC++のコードに組み込まれるイメージのコード
*/

#include <ros/ros.h>
//#include <pigpio.h>
#include <pigpiod_if2.h>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>

struct MotorConfig {
    int max_duty = 255;         // デューティー比の最大値
    int pwm_freq = 3000;        // PWM周波数
    int translate_speed = 150;  // 並進方向の速度(duty)。元のmotor_speedに相当
    float heading_kp = 80.0f;   // ボール方向へ姿勢を向けるための比例ゲイン

    // モーターのピン配置
    // 割り当て: MA=左前(FL), MB=右前(FR), MC=左後(RL), MD=右後(RR)
    uint8_t MA_1 = 4;
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

    // move_theta: ロボットを進ませたい方向 [rad]（正面基準、反時計回りが正）
    // ball_theta: ボールへの方角 [rad]（この方向へ姿勢を向け続ける）
    void DutyUpdate(float move_theta, float ball_theta) {
        // 並進成分（move_thetaの方向へ移動。姿勢の向きとは無関係）
        float vx = std::cos(move_theta);
        float vy = std::sin(move_theta);

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

private:
    MotorConfig cfg;
    int pi;
};

// ==================== UART受信 ====================
// 画像ラズパイからのフレーム構成:
//   [0xAA(開始バイト)][angle: float32][distance: float32][detected: uint8][checksum: uint8]
//   checksumはangle+distance+detected(9byte)のXOR。
// 受信は別スレッドで行い、パース結果をatomicに書き込むだけに留める(参照スワップのみ)。
// メインループは一定間隔でgetLatest()を呼び、重い処理はしない。

struct UARTConfig {
    std::string port = "/dev/serial0";  // 画像ラズパイからのUART受信ポート
    speed_t baudrate = B9600;
    int timeout_ms = 500;  // これを超えて未更新ならロスト扱い
};

class UARTReceiver {
public:
    struct BallTarget {
        float angle = 0.0f;
        float distance = 0.0f;
        bool detected = false;
    };

    UARTReceiver(const UARTConfig& config) : cfg(config) {}

    // シリアルポートを開き、受信スレッドを起動する
    bool start() {
        fd = ::open(cfg.port.c_str(), O_RDONLY | O_NOCTTY);
        if (fd < 0) {
            ROS_ERROR("UART port open failed: %s (errno=%d)", cfg.port.c_str(), errno);
            return false;
        }

        termios tty{};
        if (tcgetattr(fd, &tty) != 0) {
            ROS_ERROR("tcgetattr failed (errno=%d)", errno);
            ::close(fd);
            fd = -1;
            return false;
        }

        cfsetispeed(&tty, cfg.baudrate);
        cfsetospeed(&tty, cfg.baudrate);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_iflag = 0;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            ROS_ERROR("tcsetattr failed (errno=%d)", errno);
            ::close(fd);
            fd = -1;
            return false;
        }

        running = true;
        worker = std::thread(&UARTReceiver::readLoop, this);
        return true;
    }

    // 最新値を取得する。staleにタイムアウト/未接続かどうかを返す。
    BallTarget getLatest(bool& stale) const {
        BallTarget t;
        t.angle = latest_angle.load(std::memory_order_relaxed);
        t.distance = latest_distance.load(std::memory_order_relaxed);
        t.detected = latest_detected.load(std::memory_order_relaxed);

        int64_t last_ms = last_update_ms.load(std::memory_order_relaxed);
        stale = (fd < 0) || (nowMs() - last_ms > cfg.timeout_ms);

        return t;
    }

    bool isConnected() const { return fd >= 0; }

    void stop() {
        running = false;
        if (worker.joinable()) worker.join();
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }

private:
    static constexpr uint8_t START_BYTE = 0xAA;
    static constexpr size_t PAYLOAD_SIZE = 9;              // angle(4) + distance(4) + detected(1)
    static constexpr size_t TAIL_SIZE = PAYLOAD_SIZE + 1;  // ペイロード + checksum(1)

    void readLoop() {
        uint8_t byte;
        uint8_t tail[TAIL_SIZE];

        while (running) {
            ssize_t n = ::read(fd, &byte, 1);
            if (n <= 0) {
                handleReadError();
                return;
            }
            if (byte != START_BYTE) continue;  // 開始バイトが見つかるまで読み捨てる

            size_t received = 0;
            while (received < TAIL_SIZE && running) {
                ssize_t r = ::read(fd, tail + received, TAIL_SIZE - received);
                if (r <= 0) {
                    handleReadError();
                    return;
                }
                received += static_cast<size_t>(r);
            }
            if (received < TAIL_SIZE) continue;

            uint8_t checksum = tail[PAYLOAD_SIZE];
            if (calcChecksum(tail, PAYLOAD_SIZE) != checksum) {
                continue;  // チェックサム不一致：フレームを破棄して再同期を待つ
            }

            float angle, distance;
            std::memcpy(&angle, tail, 4);
            std::memcpy(&distance, tail + 4, 4);
            bool detected = tail[8] != 0;

            latest_angle.store(angle, std::memory_order_relaxed);
            latest_distance.store(distance, std::memory_order_relaxed);
            latest_detected.store(detected, std::memory_order_relaxed);
            last_update_ms.store(nowMs(), std::memory_order_relaxed);
        }
    }

    void handleReadError() {
        // シリアルポート切断時は再接続せず、エラーを出して受信を停止する
        ROS_ERROR("UART read error (errno=%d). Stopping receiver.", errno);
        running = false;
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }

    static uint8_t calcChecksum(const uint8_t* data, size_t len) {
        uint8_t checksum = 0;
        for (size_t i = 0; i < len; ++i) checksum ^= data[i];
        return checksum;
    }

    static int64_t nowMs() {
        using namespace std::chrono;
        return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    }

    UARTConfig cfg;
    int fd = -1;
    std::atomic<bool> running{false};
    std::thread worker;

    std::atomic<float> latest_angle{0.0f};
    std::atomic<float> latest_distance{0.0f};
    std::atomic<bool> latest_detected{false};
    std::atomic<int64_t> last_update_ms{0};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "servo");
    ros::NodeHandle n;

    int pi = pigpio_start(NULL, NULL);

    MotorConfig motor_cfg;
    MotorSpinner motors(motor_cfg, pi);
    motors.start();

    UARTConfig uart_cfg;
    UARTReceiver uart(uart_cfg);
    if (!uart.start()) {
        ROS_ERROR("UART receiver failed to start. Aborting.");
        return 1;
    }

    ros::Rate rate(50);
    while (ros::ok()) {
        bool stale = false;
        UARTReceiver::BallTarget target = uart.getLatest(stale);

        if (stale || !uart.isConnected()) {
            // 通信ロスト、またはシリアル切断：安全のためduty比を0にする
            motors.stop();
        } else if (!target.detected) {
            // ボール未検出時の探索動作は未定。現状は安全側に倒して停止する。
            motors.stop();
        } else {
            // 現状は進行方向を決める別ソース(LiDAR等)がまだ無いため、
            // 暫定的にボール方向をそのまま進行方向としても使っている。
            // 経路計画を追加する際はmove_thetaだけ別の値に差し替えればよい。
            // target.distanceは現状未使用(将来、接近速度の調整等に使う想定)。
            motors.DutyUpdate(target.angle, target.angle);
            motors.PWMshootor();
        }

        ros::spinOnce();
        rate.sleep();
    }

    uart.stop();
    motors.stop();
    return 0;
}