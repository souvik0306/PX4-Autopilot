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
 * @file EKF2_TcpPublisher.hpp
 * TCP-based IMU publisher for AI preprocessing pipeline
 *
 * Architecture:
 * - Server-mode TCP socket listening on port 14567
 * - Accepts single AI client connection
 * - Streams IMU samples at 250Hz over persistent TCP connection
 * - Uses shared ImuNetworkPacket format for compatibility
 *
 * Features:
 * - Reliable delivery (TCP guarantees ordering and delivery)
 * - Connection state management (detect client disconnects)
 * - Non-blocking I/O to prevent EKF2 thread blocking
 * - Graceful reconnection support
 */

#ifndef EKF2_TCP_PUBLISHER_HPP
#define EKF2_TCP_PUBLISHER_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <matrix/math.hpp>

// Shared packet structure for network communication
#include "EKF2_ImuPacket.hpp"

using matrix::Vector3f;

class EKF2_TcpPublisher
{
public:
	static constexpr uint16_t LISTEN_PORT = 14567;
	static constexpr uint32_t STATS_LOG_INTERVAL_US = 5000000; // 5 seconds
	static constexpr int MAX_PENDING_CONNECTIONS = 1; // Only accept one AI client

	EKF2_TcpPublisher();
	~EKF2_TcpPublisher();

	/**
	 * Initialize TCP server socket
	 * @return true if initialization successful
	 */
	bool init();

	/**
	 * Publish a single IMU sample via TCP
	 * Non-blocking; drops sample if client buffer full
	 *
	 * @param timestamp_us Sample timestamp in microseconds
	 * @param delta_ang Angular velocity delta (rad)
	 * @param delta_ang_dt Integration time for angular velocity (s)
	 * @param delta_vel Velocity delta (m/s)
	 * @param delta_vel_dt Integration time for velocity (s)
	 * @return true if sample was successfully sent
	 */
	bool publishSample(uint64_t timestamp_us,
	                   const Vector3f &delta_ang,
	                   float delta_ang_dt,
	                   const Vector3f &delta_vel,
	                   float delta_vel_dt);

	/**
	 * Log comprehensive statistics
	 */
	void logStatistics();

	/**
	 * Check if client is connected
	 */
	bool isClientConnected() const { return _client_fd >= 0; }

	/**
	 * Statistics structure
	 */
	struct Statistics {
		uint64_t total_samples;
		uint64_t total_sent;
		uint64_t send_failures;
		uint64_t client_disconnects;
		uint32_t current_sequence;
		float avg_latency_us;
		float max_latency_us;
	};

	Statistics getStatistics() const { return _stats; }

private:
	/**
	 * Accept new client connection (non-blocking)
	 * @return true if new client accepted
	 */
	bool acceptClient();

	/**
	 * Send packet to connected client
	 * @return true if sent successfully
	 */
	bool sendPacket(const ImuNetworkPacket &pkt);

	/**
	 * Close client connection
	 */
	void closeClient();

	/**
	 * Calculate CRC16-CCITT for packet integrity
	 */
	static uint16_t calculateCrc16(const uint8_t *data, size_t len);

	// Server socket management
	int _server_fd{-1};
	int _client_fd{-1};
	struct sockaddr_in _server_addr{};
	struct sockaddr_in _client_addr{};
	bool _initialized{false};

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
	uint64_t _last_client_connect_time_us{0};
};

#endif // EKF2_TCP_PUBLISHER_HPP
