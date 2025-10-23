/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file EKF2_UdpPublisher.hpp
 * High-performance UDP publisher for IMU samples to AI preprocessor
 *
 * Features:
 * - 250Hz IMU sample streaming with <5ms typical latency
 * - Fixed-size ring buffer (64 samples, ~256ms buffer depth)
 * - Sequence tracking for packet loss detection
 * - Efficient binary protocol with minimal overhead
 * - Detailed statistics logging
 */

#ifndef EKF2_UDP_PUBLISHER_HPP
#define EKF2_UDP_PUBLISHER_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <matrix/math.hpp>

using matrix::Vector3f;

// Compact binary protocol for efficient transmission
#pragma pack(push, 1)
struct ImuUdpPacket {
	uint64_t timestamp_us;      // 8 bytes - sample timestamp
	uint32_t sequence;          // 4 bytes - packet sequence number
	float gyro_x;               // 4 bytes - instantaneous gyro rad/s
	float gyro_y;               // 4 bytes
	float gyro_z;               // 4 bytes
	float accel_x;              // 4 bytes - instantaneous accel m/s²
	float accel_y;              // 4 bytes
	float accel_z;              // 4 bytes
	float delta_ang_dt;         // 4 bytes - integration time for gyro
	float delta_vel_dt;         // 4 bytes - integration time for accel
	uint16_t crc16;             // 2 bytes - integrity check
	// Total: 50 bytes per packet
};
#pragma pack(pop)

class EKF2_UdpPublisher
{
public:
	static constexpr size_t RING_BUFFER_SIZE = 64;  // Power of 2 for efficient modulo
	static constexpr const char *TARGET_IP = "127.0.0.1";
	static constexpr uint16_t TARGET_PORT = 14567;
	static constexpr uint32_t STATS_LOG_INTERVAL_US = 1000000; // 1 second

	EKF2_UdpPublisher();
	~EKF2_UdpPublisher();

	/**
	 * Initialize UDP socket and configure for low-latency transmission
	 * @return true if initialization successful
	 */
	bool init();

	/**
	 * Publish a single IMU sample with minimal latency
	 * Converts delta values to instantaneous rates
	 *
	 * @param timestamp_us Sample timestamp in microseconds
	 * @param delta_ang Angular velocity delta (rad)
	 * @param delta_ang_dt Integration time for angular velocity (s)
	 * @param delta_vel Velocity delta (m/s)
	 * @param delta_vel_dt Integration time for velocity (s)
	 * @return true if sample was successfully queued/sent
	 */
	bool publishSample(uint64_t timestamp_us,
	                   const Vector3f &delta_ang,
	                   float delta_ang_dt,
	                   const Vector3f &delta_vel,
	                   float delta_vel_dt);

	/**
	 * Log comprehensive statistics (call periodically, e.g., every second)
	 */
	void logStatistics();

	/**
	 * Get current statistics
	 */
	struct Statistics {
		uint64_t total_samples;
		uint64_t total_sent;
		uint64_t send_failures;
		uint64_t buffer_overruns;
		uint32_t current_sequence;
		float avg_latency_us;
		float max_latency_us;
	};

	Statistics getStatistics() const { return _stats; }

private:
	/**
	 * Calculate CRC16-CCITT for packet integrity
	 */
	static uint16_t calculateCrc16(const uint8_t *data, size_t len);

	/**
	 * Send packet with error handling
	 * @return true if sent successfully
	 */
	bool sendPacket(const ImuUdpPacket &pkt);

	// Socket management
	int _socket_fd{-1};
	struct sockaddr_in _target_addr{};
	bool _initialized{false};

	// Ring buffer for queue management
	struct BufferSlot {
		ImuUdpPacket packet;
		uint64_t enqueue_time_us;
		bool occupied;
	};
	BufferSlot _ring_buffer[RING_BUFFER_SIZE]{};
	size_t _write_idx{0};
	size_t _read_idx{0};
	size_t _buffer_count{0};

	// Sequence tracking
	uint32_t _sequence_counter{0};

	// Statistics tracking
	Statistics _stats{};
	uint64_t _first_sample_time_us{0};
	uint64_t _last_stats_log_time_us{0};
	uint64_t _stats_interval_sample_count{0};
	float _latency_sum_us{0};

	// Performance monitoring
	uint64_t _last_sample_time_us{0};
	float _min_sample_interval_us{1e6f};
	float _max_sample_interval_us{0};
};

#endif // EKF2_UDP_PUBLISHER_HPP
