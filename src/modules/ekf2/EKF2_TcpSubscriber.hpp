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
 * @file EKF2_TcpSubscriber.hpp
 * TCP-based AI-processed IMU subscriber
 *
 * Architecture:
 * - Server-mode TCP socket listening on port 14568
 * - Accepts connection from Python AI client
 * - Receives AI-processed IMU samples
 * - Converts back to imu_sample_s format for EKF2
 * - Two-layer buffering: RX ring buffer + AI inference queue
 *
 * Features:
 * - Reliable delivery (TCP guarantees ordering)
 * - CRC validation for data integrity
 * - Thread-safe atomic operations for concurrent access
 * - Non-blocking receiver thread
 */

#ifndef EKF2_TCP_SUBSCRIBER_HPP
#define EKF2_TCP_SUBSCRIBER_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/atomic.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <matrix/math.hpp>

// Shared packet structure for network communication
#include "EKF2_ImuPacket.hpp"

class EKF2_TcpSubscriber
{
public:
	static constexpr uint16_t LISTEN_PORT = 14568;
	static constexpr uint32_t RX_RING_BUFFER_SIZE = 128;
	static constexpr uint32_t AI_QUEUE_SIZE = 256;
	static constexpr uint32_t STATS_LOG_INTERVAL_US = 5000000; // 5 seconds
	static constexpr int MAX_PENDING_CONNECTIONS = 1;

	// AI-processed sample structure (internal queue format)
	struct AiProcessedSample {
		uint64_t timestamp_us;
		uint32_t sequence;
		matrix::Vector3f gyro;      // rad/s
		matrix::Vector3f accel;     // m/s^2
		float delta_ang_dt;         // s
		float delta_vel_dt;         // s
		uint64_t received_time_us;  // Time received by subscriber
	};

	EKF2_TcpSubscriber();
	~EKF2_TcpSubscriber();

	/**
	 * Initialize TCP server and start receiver thread
	 * @return true if initialization successful
	 */
	bool init();

	/**
	 * Get next AI-processed sample from queue (non-blocking)
	 * @param sample Output sample structure
	 * @return true if sample available
	 */
	bool getAiSample(AiProcessedSample &sample);

	/**
	 * Check if samples available in queue
	 */
	bool hasSamples() const;

	/**
	 * Get current queue depth
	 */
	uint32_t getQueueDepth() const;

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
		uint64_t total_received;
		uint64_t crc_failures;
		uint64_t sequence_gaps;
		uint64_t queue_overflows;
		uint64_t client_disconnects;
		uint32_t last_sequence;
		float avg_latency_us;
		float max_latency_us;
		float avg_receive_rate_hz;
	};

	Statistics getStatistics() const { return _stats; }

private:
	/**
	 * Receiver thread entry point
	 */
	static void *receiverThreadEntry(void *arg);

	/**
	 * Receiver loop (runs in dedicated thread)
	 */
	void receiverLoop();

	/**
	 * Accept new client connection (non-blocking)
	 */
	bool acceptClient();

	/**
	 * Receive and process packets from client
	 */
	void receivePackets();

	/**
	 * Close client connection
	 */
	void closeClient();

	/**
	 * Validate packet CRC
	 */
	bool validatePacket(const ImuNetworkPacket &pkt) const;

	/**
	 * Calculate CRC16-CCITT
	 */
	static uint16_t calculateCrc16(const uint8_t *data, size_t len);

	/**
	 * Push sample to RX ring buffer (atomic, lock-free)
	 */
	bool pushToRxBuffer(const AiProcessedSample &sample);

	/**
	 * Pop sample from RX ring buffer (atomic, lock-free)
	 */
	bool popFromRxBuffer(AiProcessedSample &sample);

	/**
	 * Transfer samples from RX buffer to AI queue
	 */
	void transferToAiQueue();

	// TCP server management
	int _server_fd{-1};
	int _client_fd{-1};
	struct sockaddr_in _server_addr{};
	struct sockaddr_in _client_addr{};
	bool _initialized{false};

	// Receiver thread
	pthread_t _receiver_thread{};
	volatile bool _receiver_running{false};

	// Two-layer buffering
	AiProcessedSample _rx_ring_buffer[RX_RING_BUFFER_SIZE];
	px4::atomic<uint32_t> _rx_head{0};
	px4::atomic<uint32_t> _rx_tail{0};

	AiProcessedSample _ai_queue[AI_QUEUE_SIZE];
	px4::atomic<uint32_t> _ai_head{0};
	px4::atomic<uint32_t> _ai_tail{0};

	// TCP receive buffer
	uint8_t _tcp_buffer[4096];
	size_t _tcp_buffer_used{0};

	// Statistics tracking
	Statistics _stats{};
	uint64_t _first_packet_time_us{0};
	uint64_t _last_stats_log_time_us{0};
	uint64_t _stats_interval_packet_count{0};
	float _latency_sum_us{0};
	uint32_t _expected_sequence{0};
	bool _first_packet_received{false};

	// Client info logging
	char _client_ip[INET_ADDRSTRLEN]{};
	uint16_t _client_port{0};
	bool _client_info_logged{false};
};

#endif // EKF2_TCP_SUBSCRIBER_HPP
