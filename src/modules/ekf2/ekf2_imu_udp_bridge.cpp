#include "ekf2_imu_udp_bridge.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <px4_platform_common/log.h>

#include <matrix/math.hpp>

namespace ekf2::udp
{
namespace
{
constexpr size_t kPacketSize = 55;
constexpr size_t kMaxQueueDepth = 512;
constexpr unsigned kMaxDrainPerCycle = 256;
constexpr hrt_abstime kLogInterval = 5_s;
constexpr hrt_abstime kWarningInterval = 1_s;

struct __attribute__((packed)) ImuPacket {
    uint64_t timestamp;
    uint64_t timestamp_sample;
    uint32_t accel_device_id;
    uint32_t gyro_device_id;
    float delta_angle[3];
    float delta_velocity[3];
    uint16_t delta_angle_dt;
    uint16_t delta_velocity_dt;
    uint8_t delta_velocity_clipping;
    uint8_t accel_calibration_count;
    uint8_t gyro_calibration_count;
};

void close_socket(int &fd)
{
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}

void prepare_socket_common(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    int reuse = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        PX4_WARN("[IMU UDP] setsockopt(SO_REUSEADDR) failed: %s", strerror(errno));
    }

    int buf = 16 * 1024 * 1024;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &buf, sizeof(buf)) < 0) {
        PX4_WARN("[IMU UDP] setsockopt(SO_RCVBUF) failed: %s", strerror(errno));
    }
    if (setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &buf, sizeof(buf)) < 0) {
        PX4_WARN("[IMU UDP] setsockopt(SO_SNDBUF) failed: %s", strerror(errno));
    }
}

bool ensure_tx_socket(ImuUdpBridgeState &state, hrt_abstime timestamp)
{
    if (state.tx_socket >= 0) {
        return true;
    }

    state.tx_socket = socket(AF_INET, SOCK_DGRAM, 0);

    if (state.tx_socket < 0) {
        PX4_ERR("[IMU UDP TX] socket() failed: %s", strerror(errno));
        return false;
    }

    prepare_socket_common(state.tx_socket);

    memset(&state.tx_addr, 0, sizeof(state.tx_addr));
    state.tx_addr.sin_family = AF_INET;
    state.tx_addr.sin_port = htons(14567);
    state.tx_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    state.tx_count = 0;
    state.tx_last_log = timestamp;
    PX4_INFO("[IMU UDP TX] Initialized: sending EKF2 IMU data to 127.0.0.1:14567");
    return true;
}

bool ensure_rx_socket(ImuUdpBridgeState &state, hrt_abstime timestamp)
{
    if (state.rx_socket >= 0) {
        return true;
    }

    state.rx_socket = socket(AF_INET, SOCK_DGRAM, 0);

    if (state.rx_socket < 0) {
        PX4_ERR("[IMU UDP RX] socket() failed: %s", strerror(errno));
        return false;
    }

    prepare_socket_common(state.rx_socket);

    memset(&state.rx_addr, 0, sizeof(state.rx_addr));
    state.rx_addr.sin_family = AF_INET;
    state.rx_addr.sin_port = htons(14568);
    state.rx_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(state.rx_socket, reinterpret_cast<sockaddr *>(&state.rx_addr), sizeof(state.rx_addr)) < 0) {
        PX4_ERR("[IMU UDP RX] bind() failed on port 14568: %s", strerror(errno));
        close_socket(state.rx_socket);
        return false;
    }

    state.rx_count = 0;
    state.rx_last_log = timestamp;
    state.rx_last_warning_log = 0;
    state.last_rx_timestamp_sample = 0;

    PX4_INFO("[IMU UDP RX] Initialized: listening for AI IMU feedback on 0.0.0.0:14568");
    return true;
}

uint8_t clipping_mask(const imuSample &imu)
{
    uint8_t mask = 0;
    mask |= (imu.delta_vel_clipping[0] ? 0x01 : 0);
    mask |= (imu.delta_vel_clipping[1] ? 0x02 : 0);
    mask |= (imu.delta_vel_clipping[2] ? 0x04 : 0);
    return mask;
}

imuSample packet_to_sample(const ImuPacket &packet)
{
    imuSample sample{};
    sample.time_us = packet.timestamp_sample;
    sample.delta_ang_dt = packet.delta_angle_dt * 1e-6f;
    sample.delta_ang = matrix::Vector3f{packet.delta_angle[0], packet.delta_angle[1], packet.delta_angle[2]};
    sample.delta_vel_dt = packet.delta_velocity_dt * 1e-6f;
    sample.delta_vel = matrix::Vector3f{packet.delta_velocity[0], packet.delta_velocity[1], packet.delta_velocity[2]};
    sample.delta_vel_clipping[0] = packet.delta_velocity_clipping & 0x01;
    sample.delta_vel_clipping[1] = (packet.delta_velocity_clipping >> 1) & 0x01;
    sample.delta_vel_clipping[2] = (packet.delta_velocity_clipping >> 2) & 0x01;
    return sample;
}

ImuPacket sample_to_packet(const imuSample &imu, uint32_t accel_device_id, uint32_t gyro_device_id,
                           uint8_t accel_calibration_count, uint8_t gyro_calibration_count)
{
    ImuPacket packet{};
    packet.timestamp = imu.time_us;
    packet.timestamp_sample = imu.time_us;
    packet.accel_device_id = accel_device_id;
    packet.gyro_device_id = gyro_device_id;

    memcpy(packet.delta_angle, imu.delta_ang.data(), sizeof(packet.delta_angle));
    memcpy(packet.delta_velocity, imu.delta_vel.data(), sizeof(packet.delta_velocity));

    packet.delta_angle_dt = static_cast<uint16_t>(imu.delta_ang_dt * 1e6f);
    packet.delta_velocity_dt = static_cast<uint16_t>(imu.delta_vel_dt * 1e6f);
    packet.delta_velocity_clipping = clipping_mask(imu);
    packet.accel_calibration_count = accel_calibration_count;
    packet.gyro_calibration_count = gyro_calibration_count;
    return packet;
}

void flush_tx_queue(ImuUdpBridgeState &state, hrt_abstime timestamp)
{
    while (!state.tx_queue.empty() && state.tx_socket >= 0) {
        const auto &buffer = state.tx_queue.front();
        ssize_t sent = sendto(state.tx_socket, buffer.data(), buffer.size(), 0,
                              reinterpret_cast<const sockaddr *>(&state.tx_addr), sizeof(state.tx_addr));

        if (sent == static_cast<ssize_t>(buffer.size())) {
            state.tx_queue.pop_front();
            ++state.tx_count;

            if (state.tx_count == 1) {
                PX4_INFO("[IMU UDP TX] First packet sent at %.1f ms", static_cast<double>(timestamp) / 1000.0);
            } else if ((timestamp - state.tx_last_log) > kLogInterval) {
                PX4_INFO("[IMU UDP TX] Sent %u packets | queue depth=%zu", state.tx_count,
                         static_cast<size_t>(state.tx_queue.size()));
                state.tx_last_log = timestamp;
            }

        } else if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            break;
        } else {
            if ((timestamp - state.tx_last_error_log) > kWarningInterval) {
                PX4_WARN("[IMU UDP TX] sendto() failed: %s", strerror(errno));
                state.tx_last_error_log = timestamp;
            }
            break;
        }
    }
}

void drain_rx_socket(ImuUdpBridgeState &state, hrt_abstime timestamp)
{
    if (state.rx_socket < 0) {
        return;
    }

    ImuPacket packet{};
    sockaddr_in src{};
    socklen_t src_len = sizeof(src);
    unsigned drained = 0;

    while (drained < kMaxDrainPerCycle) {
        ssize_t n = recvfrom(state.rx_socket, &packet, sizeof(packet), 0,
                             reinterpret_cast<sockaddr *>(&src), &src_len);

        if (n == static_cast<ssize_t>(sizeof(packet))) {
            if (state.rx_queue.size() >= kMaxQueueDepth) {
                state.rx_queue.pop_front();
                if ((timestamp - state.rx_last_warning_log) > kWarningInterval) {
                    PX4_WARN("[IMU UDP RX] Queue full, dropping oldest packet");
                    state.rx_last_warning_log = timestamp;
                }
            }

            state.rx_queue.emplace_back();
            memcpy(state.rx_queue.back().data(), &packet, sizeof(packet));
            ++state.rx_count;

            if (state.rx_count == 1) {
                PX4_INFO("[IMU UDP RX] First packet received | ts=%llu us", (unsigned long long)packet.timestamp_sample);
            } else if ((timestamp - state.rx_last_log) > kLogInterval) {
                PX4_INFO("[IMU UDP RX] Received %u packets | queue depth=%zu", state.rx_count,
                         static_cast<size_t>(state.rx_queue.size()));
                state.rx_last_log = timestamp;
            }

            if (state.last_rx_timestamp_sample > 0 && packet.timestamp_sample <= state.last_rx_timestamp_sample) {
                if ((timestamp - state.rx_last_warning_log) > kWarningInterval) {
                    PX4_WARN("[IMU UDP RX] Non-monotonic timestamp received (%llu <= %llu)",
                             (unsigned long long)packet.timestamp_sample,
                             (unsigned long long)state.last_rx_timestamp_sample);
                    state.rx_last_warning_log = timestamp;
                }
            }

            state.last_rx_timestamp_sample = packet.timestamp_sample;
            ++drained;

        } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            break;
        } else if (n < 0) {
            if ((timestamp - state.rx_last_warning_log) > kWarningInterval) {
                PX4_WARN("[IMU UDP RX] recvfrom() failed: %s", strerror(errno));
                state.rx_last_warning_log = timestamp;
            }
            break;
        } else if (n == 0) {
            break;
        } else {
            if ((timestamp - state.rx_last_warning_log) > kWarningInterval) {
                PX4_WARN("[IMU UDP RX] Unexpected packet size %zd", n);
                state.rx_last_warning_log = timestamp;
            }
        }
    }
}

} // namespace

ImuUdpBridgeState::~ImuUdpBridgeState()
{
    reset();
}

void ImuUdpBridgeState::reset()
{
    close_socket(tx_socket);
    close_socket(rx_socket);
    tx_queue.clear();
    rx_queue.clear();
    last_ai_sample_valid = false;
    tx_count = 0;
    rx_count = 0;
    tx_last_log = 0;
    rx_last_log = 0;
    tx_last_error_log = 0;
    rx_last_warning_log = 0;
    last_rx_timestamp_sample = 0;
}

bool PublishImuSample(ImuUdpBridgeState &state, bool ai_mode_active, const imuSample &imu,
                      hrt_abstime timestamp, uint32_t accel_device_id, uint32_t gyro_device_id,
                      uint8_t accel_calibration_count, uint8_t gyro_calibration_count)
{
    if (!ai_mode_active) {
        // Keep raw data subscription alive but close UDP resources when not needed.
        state.reset();
        return false;
    }

    if (!ensure_tx_socket(state, timestamp)) {
        return false;
    }

    ImuPacket packet = sample_to_packet(imu, accel_device_id, gyro_device_id,
                                        accel_calibration_count, gyro_calibration_count);

    std::array<uint8_t, kPacketSize> buffer{};
    memcpy(buffer.data(), &packet, sizeof(packet));

    if (state.tx_queue.size() >= kMaxQueueDepth) {
        state.tx_queue.pop_front();
        if ((timestamp - state.tx_last_error_log) > kWarningInterval) {
            PX4_WARN("[IMU UDP TX] Queue full, dropping oldest packet");
            state.tx_last_error_log = timestamp;
        }
    }

    state.tx_queue.push_back(buffer);
    flush_tx_queue(state, timestamp);
    return true;
}

bool ConsumeAiSample(ImuUdpBridgeState &state, bool ai_mode_active, imuSample &imu_out,
                     hrt_abstime timestamp)
{
    if (!ai_mode_active) {
        state.reset();
        return false;
    }

    if (!ensure_rx_socket(state, timestamp)) {
        return false;
    }

    drain_rx_socket(state, timestamp);

    if (!state.rx_queue.empty()) {
        ImuPacket packet{};
        memcpy(&packet, state.rx_queue.front().data(), sizeof(packet));
        state.rx_queue.pop_front();

        imuSample sample = packet_to_sample(packet);
        state.last_ai_sample = sample;
        state.last_ai_sample_valid = true;
        imu_out = sample;
        return true;
    }

    if (state.last_ai_sample_valid) {
        if ((timestamp - state.rx_last_warning_log) > kWarningInterval) {
            PX4_WARN("[IMU UDP RX] No new packets, re-using last AI sample");
            state.rx_last_warning_log = timestamp;
        }

        imu_out = state.last_ai_sample;
        return true;
    }

    if ((timestamp - state.rx_last_warning_log) > kWarningInterval) {
        PX4_WARN("[IMU UDP RX] Awaiting first AI packet");
        state.rx_last_warning_log = timestamp;
    }

    return false;
}

} // namespace ekf2::udp

