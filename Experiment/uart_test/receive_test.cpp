/*
UART受信テスト用の最小構成コード。
dir_rotating.cppからROS/pigpio/モーター制御を取り除き、
UARTReceiverが正しくフレームをデコードできているかを
ターミナル上でリアルタイムに確認するためだけのプログラム。

ビルド:
  g++ -std=c++17 -O2 -pthread -o uart_receive_test uart_receive_test.cpp

実行:
  ./uart_receive_test                # デフォルトポート(/dev/serial0)を使用
  ./uart_receive_test /dev/ttyUSB0   # ポートを指定する場合
  Ctrl+Cで終了。
*/

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <csignal>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>

std::atomic<bool> g_keep_running{true};

void handleSigint(int) {
    g_keep_running = false;
}

// ==================== UART受信 ====================
// 画像ラズパイからのフレーム構成:
//   [0xAA(開始バイト)][angle: float32][distance: float32][detected: uint8][checksum: uint8]
//   checksumはangle+distance+detected(9byte)のXOR。
// 受信は別スレッドで行い、パース結果をatomicに書き込むだけに留める(参照スワップのみ)。

struct UARTConfig {
    std::string port = "/dev/serial0";
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

    bool start() {
        fd = ::open(cfg.port.c_str(), O_RDONLY | O_NOCTTY);
        if (fd < 0) {
            std::fprintf(stderr, "UART port open failed: %s (errno=%d)\n",
                         cfg.port.c_str(), errno);
            return false;
        }

        termios tty{};
        if (tcgetattr(fd, &tty) != 0) {
            std::fprintf(stderr, "tcgetattr failed (errno=%d)\n", errno);
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
        // VMIN=0, VTIME=2(0.2秒): データが無くてもタイムアウトで返るようにする。
        // こうしないと、通信が来ない間readLoopがread()でブロックし続け、
        // stop()のjoin()が終わらなくなる(シャットダウンが固まる)。
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 2;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            std::fprintf(stderr, "tcsetattr failed (errno=%d)\n", errno);
            ::close(fd);
            fd = -1;
            return false;
        }

        running = true;
        worker = std::thread(&UARTReceiver::readLoop, this);
        return true;
    }

    BallTarget getLatest(bool& stale) const {
        BallTarget t;
        t.angle = latest_angle.load(std::memory_order_relaxed);
        t.distance = latest_distance.load(std::memory_order_relaxed);
        t.detected = latest_detected.load(std::memory_order_relaxed);

        int64_t last_ms = last_update_ms.load(std::memory_order_relaxed);
        stale = (fd < 0) || (nowMs() - last_ms > cfg.timeout_ms);

        return t;
    }

    // 検証用の統計情報(正常フレーム数・チェックサム不一致数)
    void getStats(uint64_t& ok, uint64_t& checksum_err) const {
        ok = frame_ok_count.load(std::memory_order_relaxed);
        checksum_err = checksum_error_count.load(std::memory_order_relaxed);
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
    static constexpr size_t PAYLOAD_SIZE = 9;
    static constexpr size_t TAIL_SIZE = PAYLOAD_SIZE + 1;

    void readLoop() {
        uint8_t byte;
        uint8_t tail[TAIL_SIZE];

        while (running) {
            ssize_t n = ::read(fd, &byte, 1);
            if (n == 0) continue;       // タイムアウト(データなし)。runningを見て継続
            if (n < 0) {                // 実エラー(切断等)
                handleReadError();
                return;
            }
            if (byte != START_BYTE) continue;

            size_t received = 0;
            while (received < TAIL_SIZE && running) {
                ssize_t r = ::read(fd, tail + received, TAIL_SIZE - received);
                if (r == 0) continue;   // タイムアウト、再試行
                if (r < 0) {
                    handleReadError();
                    return;
                }
                received += static_cast<size_t>(r);
            }
            if (received < TAIL_SIZE) continue;  // running=falseで打ち切られた場合

            uint8_t checksum = tail[PAYLOAD_SIZE];
            if (calcChecksum(tail, PAYLOAD_SIZE) != checksum) {
                checksum_error_count.fetch_add(1, std::memory_order_relaxed);
                continue;  // チェックサム不一致：フレームを破棄して再同期
            }

            float angle, distance;
            std::memcpy(&angle, tail, 4);
            std::memcpy(&distance, tail + 4, 4);
            bool detected = tail[8] != 0;

            latest_angle.store(angle, std::memory_order_relaxed);
            latest_distance.store(distance, std::memory_order_relaxed);
            latest_detected.store(detected, std::memory_order_relaxed);
            last_update_ms.store(nowMs(), std::memory_order_relaxed);
            frame_ok_count.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void handleReadError() {
        std::fprintf(stderr, "UART read error (errno=%d). Stopping receiver.\n", errno);
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

    std::atomic<uint64_t> frame_ok_count{0};
    std::atomic<uint64_t> checksum_error_count{0};
};

// ==================== main ====================

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSigint);

    UARTConfig cfg;
    if (argc >= 2) {
        cfg.port = argv[1];
    }

    std::printf("UART受信テスト開始: port=%s baudrate=9600\n", cfg.port.c_str());
    std::printf("Ctrl+Cで終了します。\n\n");

    UARTReceiver uart(cfg);
    if (!uart.start()) {
        std::fprintf(stderr, "UART開始に失敗しました。\n");
        return 1;
    }

    while (g_keep_running) {
        bool stale = false;
        UARTReceiver::BallTarget t = uart.getLatest(stale);

        uint64_t ok = 0, err = 0;
        uart.getStats(ok, err);

        std::printf(
            "\rangle=%+7.2f distance=%+7.2f detected=%d stale=%d ok_frames=%llu checksum_err=%llu   ",
            t.angle, t.distance, t.detected ? 1 : 0, stale ? 1 : 0,
            static_cast<unsigned long long>(ok), static_cast<unsigned long long>(err));
        std::fflush(stdout);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::printf("\n終了処理中...\n");
    uart.stop();

    uint64_t ok = 0, err = 0;
    uart.getStats(ok, err);
    std::printf("受信結果: 正常フレーム=%llu件, チェックサム不一致=%llu件\n",
                static_cast<unsigned long long>(ok), static_cast<unsigned long long>(err));

    return 0;
}