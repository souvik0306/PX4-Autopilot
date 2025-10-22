#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <netinet/in.h>

#include <drivers/drv_hrt.h>

#include "EKF/common.h"

namespace ekf2::udp
{

struct ImuUdpBridgeState {
    int tx_socket{-1};
    sockaddr_in tx_addr{};
    uint32_t tx_count{0};
    hrt_abstime tx_last_log{0};
    hrt_abstime tx_last_error_log{0};

    int rx_socket{-1};
    sockaddr_in rx_addr{};
    uint32_t rx_count{0};
    hrt_abstime rx_last_log{0};
    hrt_abstime rx_last_warning_log{0};

    std::deque<std::array<uint8_t, 55>> tx_queue;
    std::deque<std::array<uint8_t, 55>> rx_queue;

    imuSample last_ai_sample{};
    bool last_ai_sample_valid{false};
    uint64_t last_rx_timestamp_sample{0};

    ~ImuUdpBridgeState();

    void reset();
};

bool PublishImuSample(ImuUdpBridgeState &state, bool ai_mode_active, const imuSample &imu,
                      hrt_abstime timestamp, uint32_t accel_device_id, uint32_t gyro_device_id,
                      uint8_t accel_calibration_count, uint8_t gyro_calibration_count);

bool ConsumeAiSample(ImuUdpBridgeState &state, bool ai_mode_active, imuSample &imu_out,
                     hrt_abstime timestamp);

} // namespace ekf2::udp

