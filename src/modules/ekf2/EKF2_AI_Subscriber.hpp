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
 * @file EKF2_AI_Subscriber.hpp
 * Optimized AI subscriber for IMU data reception and validation
 *
 * TX/RX Sanity Check Architecture:
 * - Direct UDP receive → AI queue (single-layer buffering for minimal latency)
 * - High-performance receiver thread with 10µs polling interval
 * - Comprehensive statistics for throughput and ordering validation
 *
 * Features:
 * - UDP receiver on port 14567 at 250Hz
 * - Large OS socket buffer (256KB) for burst handling
 * - CRC16-CCITT validation with minimal logging overhead
 * - Lock-free atomic operations for statistics
 * - Simple sequence gap detection (no complex wraparound logic)
 */

#ifndef EKF2_AI_SUBSCRIBER_HPP
#define EKF2_AI_SUBSCRIBER_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <atomic>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <matrix/math.hpp>

// Include UDP packet structure from publisher
#include "EKF2_UdpPublisher.hpp"

using matrix::Vector3f;

// Extended packet with reception metadata
struct RxImuPacket {
	ImuUdpPacket data;
	uint64_t rx_timestamp_us;   // When packet was received
	float latency_us;           // Processing latency
	bool valid;                 // CRC and validation status
};

class EKF2_AI_Subscriber
{
public:
	// Buffer configuration
	static constexpr size_t RX_RING_BUFFER_SIZE = 128;  // Power of 2, kept for compatibility (not used in TX/RX test)
	static constexpr size_t AI_INFERENCE_QUEUE_SIZE = 256; // Power of 2, ~1 second buffer at 250Hz
	static constexpr uint16_t LISTEN_PORT = 14567;
	static constexpr float MAX_ACCEPTED_LATENCY_US = 50000.0f; // 50ms
	static constexpr uint32_t STATS_LOG_INTERVAL_US = 1000000; // 1 second

	EKF2_AI_Subscriber();
	~EKF2_AI_Subscriber();

	/**
	 * Initialize UDP socket and start receiving thread
	 * @return true if initialization successful
	 */
	bool init();

	/**
	 * Stop receiver and cleanup resources
	 */
	void stop();

	/**
	 * Pop sample from AI inference queue (called by inference thread)
	 * @param sample Output sample if available
	 * @return true if sample was available
	 */
	bool popAiSample(RxImuPacket &sample);

	/**
	 * Log comprehensive statistics
	 */
	void logStatistics();

	/**
	 * Comprehensive statistics structure
	 */
	struct Statistics {
		// UDP reception stats
		uint64_t total_packets_received;
		uint64_t packets_dropped_crc_error;
		uint64_t packets_dropped_latency;
		uint64_t packets_dropped_rx_overflow;

		// RX buffer stats
		uint64_t rx_buffer_overruns;
		uint32_t rx_buffer_current_size;
		uint32_t rx_buffer_max_size;

		// AI queue stats
		uint64_t ai_queue_drops;
		uint32_t ai_queue_current_size;
		uint32_t ai_queue_max_size;

		// Timing stats
		float avg_latency_us;
		float max_latency_us;
		float min_latency_us;
		float avg_rx_rate_hz;

		// Sequence tracking
		uint32_t last_sequence_received;
		uint64_t sequence_gaps;

		// Error tracking
		uint64_t socket_errors;
		uint64_t validation_errors;
	};

	Statistics getStatistics() const;

private:
	/**
	 * UDP receiver thread entry point
	 */
	static void* receiverThreadEntry(void* arg);

	/**
	 * Main receiver loop
	 */
	void receiverLoop();

	/**
	 * Validate received packet
	 */
	bool validatePacket(const ImuUdpPacket &packet, float latency_us);

	/**
	 * Calculate CRC16-CCITT for packet validation
	 */
	static uint16_t calculateCrc16(const uint8_t *data, size_t len);

	/**
	 * Push sample to RX ring buffer (non-blocking, overwrites oldest)
	 */
	void pushToRxBuffer(const RxImuPacket &sample);

	/**
	 * Pop sample from RX buffer to AI queue (internal transfer)
	 */
	bool popFromRxBuffer(RxImuPacket &sample);

	/**
	 * Push sample to AI inference queue (drops oldest if full)
	 */
	void pushToAiQueue(const RxImuPacket &sample);

	// Socket management
	int _socket_fd{-1};
	struct sockaddr_in _listen_addr{};
	bool _initialized{false};
	std::atomic<bool> _should_exit{false};

	// Threading
	pthread_t _receiver_thread{};
	bool _thread_running{false};

	// RX Ring Buffer (Layer 1) - Thread-safe
	RxImuPacket _rx_buffer[RX_RING_BUFFER_SIZE]{};
	std::atomic<size_t> _rx_write_idx{0};
	std::atomic<size_t> _rx_read_idx{0};
	std::atomic<size_t> _rx_buffer_count{0};

	// AI Inference Queue (Layer 2) - Thread-safe
	RxImuPacket _ai_queue[AI_INFERENCE_QUEUE_SIZE]{};
	std::atomic<size_t> _ai_write_idx{0};
	std::atomic<size_t> _ai_read_idx{0};
	std::atomic<size_t> _ai_queue_count{0};

	// Statistics (thread-safe atomics)
	mutable std::atomic<uint64_t> _total_packets_received{0};
	mutable std::atomic<uint64_t> _packets_dropped_crc_error{0};
	mutable std::atomic<uint64_t> _packets_dropped_latency{0};
	mutable std::atomic<uint64_t> _packets_dropped_rx_overflow{0};
	mutable std::atomic<uint64_t> _rx_buffer_overruns{0};
	mutable std::atomic<uint64_t> _ai_queue_drops{0};
	mutable std::atomic<uint64_t> _sequence_gaps{0};
	mutable std::atomic<uint64_t> _socket_errors{0};
	mutable std::atomic<uint64_t> _validation_errors{0};

	// Timing tracking
	uint64_t _first_packet_time_us{0};
	uint64_t _last_stats_log_time_us{0};
	uint64_t _stats_interval_packet_count{0};
	float _latency_sum_us{0};
	float _max_latency_us{0};
	float _min_latency_us{1e6f};
	uint32_t _last_sequence_received{0};
	uint32_t _rx_buffer_max_size{0};
	uint32_t _ai_queue_max_size{0};
};

#endif // EKF2_AI_SUBSCRIBER_HPP
