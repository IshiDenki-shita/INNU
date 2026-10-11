/*
UART受信テスト用の最小構成コード。
通信設計書に従い、固定長15 byteのUARTフレームを検証して表示する。

ビルド:
  g++ -std=c++17 -O2 -pthread -o uart_receive_test receive_test.cpp

実行:
  ./uart_receive_test                # デフォルトポート(/dev/serial0)
  ./uart_receive_test /dev/ttyUSB0   # ポート指定
  Ctrl+Cで終了。
*/

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#include <termios.h>
#include <unistd.h>

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "UART protocol requires IEEE-754 float32");

std::atomic<bool> g_keep_running{true};

void handleSigint(int) {
    g_keep_running = false;
}

struct UARTConfig {
    std::string port = "/dev/serial0";
    speed_t baudrate = B9600;
    int timeout_ms = 500;
};

class UARTReceiver {
public:
    struct BallTarget {
        float theta_rad = 0.0f;
        float distance_m = 0.0f;
        bool detected = false;
        uint8_t sequence = 0;
    };

    struct Stats {
        uint64_t valid_frames = 0;
        uint64_t crc_errors = 0;
        uint64_t rejected_frames = 0;
        uint64_t sequence_gaps = 0;
    };

    explicit UARTReceiver(const UARTConfig& config) : cfg_(config) {}

    bool start() {
        if (crc16(reinterpret_cast<const uint8_t*>("123456789"), 9) != 0x29B1) {
            std::fprintf(stderr, "CRC-16/CCITT-FALSE self-check failed\n");
            return false;
        }

        const int opened_fd = ::open(cfg_.port.c_str(), O_RDONLY | O_NOCTTY);
        if (opened_fd < 0) {
            std::fprintf(stderr, "UART port open failed: %s (errno=%d)\n",
                         cfg_.port.c_str(), errno);
            return false;
        }

        termios tty{};
        if (tcgetattr(opened_fd, &tty) != 0) {
            std::fprintf(stderr, "tcgetattr failed (errno=%d)\n", errno);
            ::close(opened_fd);
            return false;
        }

        cfmakeraw(&tty);
        cfsetispeed(&tty, cfg_.baudrate);
        cfsetospeed(&tty, cfg_.baudrate);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        tty.c_cflag |= CS8;
#ifdef CRTSCTS
        tty.c_cflag &= ~CRTSCTS;
#endif
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 2;

        if (tcsetattr(opened_fd, TCSANOW, &tty) != 0) {
            std::fprintf(stderr, "tcsetattr failed (errno=%d)\n", errno);
            ::close(opened_fd);
            return false;
        }

        fd_.store(opened_fd, std::memory_order_release);
        running_.store(true, std::memory_order_release);
        worker_ = std::thread(&UARTReceiver::readLoop, this);
        return true;
    }

    BallTarget getLatest(bool& stale) const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stale = !running_.load(std::memory_order_acquire) ||
                fd_.load(std::memory_order_acquire) < 0 ||
                !has_valid_frame_ ||
                (nowMs() - last_update_ms_) >= cfg_.timeout_ms;
        return latest_;
    }

    Stats getStats() const {
        return {
            valid_frame_count_.load(std::memory_order_relaxed),
            crc_error_count_.load(std::memory_order_relaxed),
            rejected_frame_count_.load(std::memory_order_relaxed),
            sequence_gap_count_.load(std::memory_order_relaxed)
        };
    }

    bool isConnected() const {
        return fd_.load(std::memory_order_acquire) >= 0 &&
               running_.load(std::memory_order_acquire);
    }

    void stop() {
        running_.store(false, std::memory_order_release);
        if (worker_.joinable()) {
            worker_.join();
        }

        const int old_fd = fd_.exchange(-1, std::memory_order_acq_rel);
        if (old_fd >= 0) {
            ::close(old_fd);
        }
    }

private:
    static constexpr uint8_t SOF0 = 0xAA;
    static constexpr uint8_t SOF1 = 0x55;
    static constexpr uint8_t VERSION = 0x01;
    static constexpr size_t FRAME_SIZE = 15;
    static constexpr size_t CRC_INPUT_OFFSET = 2;
    static constexpr size_t CRC_INPUT_SIZE = 11;

    void readLoop() {
        uint8_t byte = 0;
        while (running_.load(std::memory_order_acquire)) {
            const int descriptor = fd_.load(std::memory_order_acquire);
            if (descriptor < 0) {
                break;
            }

            const ssize_t count = ::read(descriptor, &byte, 1);
            if (count == 0) {
                continue;
            }
            if (count < 0) {
                if (errno == EINTR) {
                    continue;
                }
                handleReadError();
                return;
            }
            consumeByte(byte);
        }
    }

    void consumeByte(uint8_t byte) {
        if (frame_size_ == 0) {
            if (byte == SOF0) {
                frame_buffer_[frame_size_++] = byte;
            }
            return;
        }

        if (frame_size_ == 1) {
            if (byte == SOF1) {
                frame_buffer_[frame_size_++] = byte;
            } else if (byte == SOF0) {
                frame_buffer_[0] = byte;
            } else {
                frame_size_ = 0;
            }
            return;
        }

        frame_buffer_[frame_size_++] = byte;
        if (frame_size_ == FRAME_SIZE) {
            const bool valid = processFrame();
            if (valid) {
                frame_size_ = 0;
            } else {
                resynchronize();
            }
        }
    }

    bool processFrame() {
        if (frame_buffer_[0] != SOF0 || frame_buffer_[1] != SOF1) {
            rejected_frame_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        if (frame_buffer_[2] != VERSION) {
            std::fprintf(stderr, "Unsupported UART version: 0x%02X\n",
                         frame_buffer_[2]);
            rejected_frame_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        const uint16_t received_crc =
            static_cast<uint16_t>(frame_buffer_[13]) |
            (static_cast<uint16_t>(frame_buffer_[14]) << 8);
        const uint16_t calculated_crc =
            crc16(frame_buffer_.data() + CRC_INPUT_OFFSET, CRC_INPUT_SIZE);
        if (received_crc != calculated_crc) {
            crc_error_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        const uint8_t flags = frame_buffer_[12];
        if ((flags & 0xFE) != 0) {
            std::fprintf(stderr, "UART frame has reserved flag bits set: 0x%02X\n",
                         flags);
            rejected_frame_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        const float theta_rad = decodeFloat32LE(frame_buffer_.data() + 4);
        const float distance_m = decodeFloat32LE(frame_buffer_.data() + 8);
        const bool detected = (flags & 0x01) != 0;
        const float pi_f32 = static_cast<float>(3.14159265358979323846);

        if (!std::isfinite(theta_rad) || theta_rad <= -pi_f32 ||
            theta_rad > pi_f32 || !std::isfinite(distance_m) ||
            distance_m < 0.0f ||
            (!detected && (theta_rad != 0.0f || distance_m != 0.0f))) {
            std::fprintf(stderr,
                         "Invalid UART values: theta=%g distance=%g detected=%d\n",
                         static_cast<double>(theta_rad),
                         static_cast<double>(distance_m), detected ? 1 : 0);
            rejected_frame_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        const uint8_t sequence = frame_buffer_[3];
        if (has_sequence_) {
            const uint8_t expected = static_cast<uint8_t>(last_sequence_ + 1);
            if (sequence != expected) {
                std::fprintf(stderr,
                             "UART sequence gap: expected=%u received=%u\n",
                             static_cast<unsigned>(expected),
                             static_cast<unsigned>(sequence));
                sequence_gap_count_.fetch_add(1, std::memory_order_relaxed);
            }
        }
        last_sequence_ = sequence;
        has_sequence_ = true;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_ = {theta_rad, distance_m, detected, sequence};
            last_update_ms_ = nowMs();
            has_valid_frame_ = true;
        }
        valid_frame_count_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    void resynchronize() {
        for (size_t i = 1; i + 1 < FRAME_SIZE; ++i) {
            if (frame_buffer_[i] == SOF0 && frame_buffer_[i + 1] == SOF1) {
                const size_t remaining = FRAME_SIZE - i;
                std::memmove(frame_buffer_.data(), frame_buffer_.data() + i,
                             remaining);
                frame_size_ = remaining;
                return;
            }
        }

        if (frame_buffer_[FRAME_SIZE - 1] == SOF0) {
            frame_buffer_[0] = SOF0;
            frame_size_ = 1;
        } else {
            frame_size_ = 0;
        }
    }

    void handleReadError() {
        const int error = errno;
        std::fprintf(stderr, "UART read error: %s\n", std::strerror(error));
        running_.store(false, std::memory_order_release);
    }

    static float decodeFloat32LE(const uint8_t* bytes) {
        const uint32_t bits =
            static_cast<uint32_t>(bytes[0]) |
            (static_cast<uint32_t>(bytes[1]) << 8) |
            (static_cast<uint32_t>(bytes[2]) << 16) |
            (static_cast<uint32_t>(bytes[3]) << 24);
        float value = 0.0f;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    static uint16_t crc16(const uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; ++i) {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (int bit = 0; bit < 8; ++bit) {
                crc = (crc & 0x8000)
                          ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                          : static_cast<uint16_t>(crc << 1);
            }
        }
        return crc;
    }

    static int64_t nowMs() {
        using namespace std::chrono;
        return duration_cast<milliseconds>(
                   steady_clock::now().time_since_epoch())
            .count();
    }

    UARTConfig cfg_;
    std::atomic<int> fd_{-1};
    std::atomic<bool> running_{false};
    std::thread worker_;

    mutable std::mutex state_mutex_;
    BallTarget latest_;
    int64_t last_update_ms_ = 0;
    bool has_valid_frame_ = false;

    std::array<uint8_t, FRAME_SIZE> frame_buffer_{};
    size_t frame_size_ = 0;
    bool has_sequence_ = false;
    uint8_t last_sequence_ = 0;

    std::atomic<uint64_t> valid_frame_count_{0};
    std::atomic<uint64_t> crc_error_count_{0};
    std::atomic<uint64_t> rejected_frame_count_{0};
    std::atomic<uint64_t> sequence_gap_count_{0};
};

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSigint);

    UARTConfig cfg;
    if (argc >= 2) {
        cfg.port = argv[1];
    }

    std::printf("UART受信テスト開始: port=%s baudrate=9600 8N1 timeout=%dms\n",
                cfg.port.c_str(), cfg.timeout_ms);
    std::printf("Ctrl+Cで終了します。\n\n");

    UARTReceiver uart(cfg);
    if (!uart.start()) {
        std::fprintf(stderr, "UART開始に失敗しました。\n");
        return 1;
    }

    while (g_keep_running) {
        bool stale = false;
        const UARTReceiver::BallTarget target = uart.getLatest(stale);
        const UARTReceiver::Stats stats = uart.getStats();

        std::printf(
            "\rseq=%3u theta=%+8.4f rad distance=%7.3f m detected=%d stale=%d "
            "valid=%llu crc_err=%llu rejected=%llu seq_gaps=%llu   ",
            static_cast<unsigned>(target.sequence),
            static_cast<double>(target.theta_rad),
            static_cast<double>(target.distance_m), target.detected ? 1 : 0,
            stale ? 1 : 0,
            static_cast<unsigned long long>(stats.valid_frames),
            static_cast<unsigned long long>(stats.crc_errors),
            static_cast<unsigned long long>(stats.rejected_frames),
            static_cast<unsigned long long>(stats.sequence_gaps));
        std::fflush(stdout);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::printf("\n終了処理中...\n");
    uart.stop();

    const UARTReceiver::Stats stats = uart.getStats();
    std::printf(
        "受信結果: 正常フレーム=%llu件, CRCエラー=%llu件, "
        "破棄フレーム=%llu件, sequence欠落=%llu件\n",
        static_cast<unsigned long long>(stats.valid_frames),
        static_cast<unsigned long long>(stats.crc_errors),
        static_cast<unsigned long long>(stats.rejected_frames),
        static_cast<unsigned long long>(stats.sequence_gaps));

    return 0;
}
