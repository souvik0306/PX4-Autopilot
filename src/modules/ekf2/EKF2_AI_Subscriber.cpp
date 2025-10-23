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

#include "EKF2_AI_Subscriber.hpp"
#include <px4_platform_common/log.h>
#include <matrix/Vector3.hpp>

EKF2_AI_Subscriber::EKF2_AI_Subscriber()
{
	// Initialize buffers
	memset(_rx_buffer, 0, sizeof(_rx_buffer));
	memset(_ai_queue, 0, sizeof(_ai_queue));
}

EKF2_AI_Subscriber::~EKF2_AI_Subscriber()
{
	stop();
}

bool EKF2_AI_Subscriber::init()
{
	if (_initialized) {
		return true;
	}

	// Create UDP socket
	_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (_socket_fd < 0) {
		PX4_ERR("EKF2_AI_Subscriber: Failed to create socket: %s", strerror(errno));
		return false;
	}

	// Configure for non-blocking operation
	int flags = fcntl(_socket_fd, F_GETFL, 0);
	if (flags == -1) {
		PX4_ERR("EKF2_AI_Subscriber: fcntl F_GETFL failed: %s", strerror(errno));
		close(_socket_fd);
		_socket_fd = -1;
		return false;
	}

	if (fcntl(_socket_fd, F_SETFL, flags | O_NONBLOCK) == -1) {
		PX4_ERR("EKF2_AI_Subscriber: fcntl F_SETFL failed: %s", strerror(errno));
		close(_socket_fd);
		_socket_fd = -1;
		return false;
	}

	// Enable address reuse
	int reuse = 1;
	if (setsockopt(_socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
		PX4_WARN("EKF2_AI_Subscriber: Failed to set SO_REUSEADDR: %s", strerror(errno));
	}

	// Set receive buffer size for high-frequency data
	int rcvbuf = 65536; // 64KB receive buffer
	if (setsockopt(_socket_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
		PX4_WARN("EKF2_AI_Subscriber: Failed to set SO_RCVBUF: %s", strerror(errno));
	}

	// Bind to listen address
	memset(&_listen_addr, 0, sizeof(_listen_addr));
	_listen_addr.sin_family = AF_INET;
	_listen_addr.sin_port = htons(LISTEN_PORT);
	_listen_addr.sin_addr.s_addr = INADDR_ANY;

	if (bind(_socket_fd, (struct sockaddr*)&_listen_addr, sizeof(_listen_addr)) < 0) {
		PX4_ERR("EKF2_AI_Subscriber: Failed to bind socket to port %u: %s", LISTEN_PORT, strerror(errno));
		close(_socket_fd);
		_socket_fd = -1;
		return false;
	}

	PX4_INFO("EKF2_AI_Subscriber: Bound to port %u - Ready to receive AI-processed IMU data", LISTEN_PORT);

	// Reset statistics
	_should_exit = false;
	_total_packets_received = 0;
	_packets_dropped_crc_error = 0;
	_packets_dropped_latency = 0;
	_packets_dropped_rx_overflow = 0;
	_rx_buffer_overruns = 0;
	_ai_queue_drops = 0;
	_sequence_gaps = 0;
	_socket_errors = 0;
	_validation_errors = 0;

	_first_packet_time_us = 0;
	_last_stats_log_time_us = hrt_absolute_time();
	_stats_interval_packet_count = 0;
	_latency_sum_us = 0;
	_max_latency_us = 0;
	_min_latency_us = 1e6f;
	_last_sequence_received = 0;
	_rx_buffer_max_size = 0;
	_ai_queue_max_size = 0;

	// Start receiver thread
	if (pthread_create(&_receiver_thread, nullptr, receiverThreadEntry, this) != 0) {
		PX4_ERR("EKF2_AI_Subscriber: Failed to create receiver thread");
		close(_socket_fd);
		_socket_fd = -1;
		return false;
	}

	_thread_running = true;
	_initialized = true;

	PX4_INFO("EKF2_AI_Subscriber: Initialized successfully - Listening on port %u", LISTEN_PORT);
	return true;
}

void EKF2_AI_Subscriber::stop()
{
	if (!_initialized) {
		return;
	}

	_should_exit = true;

	if (_thread_running) {
		// Wait for thread to finish
		pthread_join(_receiver_thread, nullptr);
		_thread_running = false;
	}

	if (_socket_fd >= 0) {
		close(_socket_fd);
		_socket_fd = -1;
	}

	_initialized = false;
	PX4_INFO("EKF2_AI_Subscriber: Stopped");
}

void* EKF2_AI_Subscriber::receiverThreadEntry(void* arg)
{
	EKF2_AI_Subscriber* subscriber = static_cast<EKF2_AI_Subscriber*>(arg);
	subscriber->receiverLoop();
	return nullptr;
}

void EKF2_AI_Subscriber::receiverLoop()
{
	ImuUdpPacket rx_packet;
	struct sockaddr_in sender_addr;
	socklen_t sender_len = sizeof(sender_addr);

	PX4_INFO("EKF2_AI_Subscriber: Receiver thread started");

	while (!_should_exit) {
		// Receive packet (non-blocking)
		ssize_t bytes_received = recvfrom(_socket_fd, &rx_packet, sizeof(rx_packet), 0,
		                                  (struct sockaddr*)&sender_addr, &sender_len);

		if (bytes_received < 0) {
			   if (errno == EAGAIN) {
				// No data available, continue
				usleep(100); // 100µs sleep to prevent busy-waiting
				continue;
			} else {
				_socket_errors++;
				PX4_ERR("EKF2_AI_Subscriber: recvfrom failed: %s", strerror(errno));
				usleep(1000); // 1ms sleep on error
				continue;
			}
		}

		if (bytes_received != sizeof(ImuUdpPacket)) {
			_validation_errors++;
			PX4_WARN("EKF2_AI_Subscriber: Invalid packet size: %zd (expected %zu)",
			         bytes_received, sizeof(ImuUdpPacket));
			continue;
		}

		uint64_t rx_time = hrt_absolute_time();
		_total_packets_received++;
		_stats_interval_packet_count++;

		// Track first packet time
		if (_first_packet_time_us == 0) {
			_first_packet_time_us = rx_time;
			// Log with wall-clock time for clarity
			time_t now_sec = time(nullptr);
			struct tm local_tm;
			localtime_r(&now_sec, &local_tm);
			char time_buf[32];
			strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &local_tm);
			PX4_INFO("EKF2_AI_Subscriber: First packet received at %s (wall time), timestamp_us=%" PRIu64 ", t=%.3f s from boot",
			         time_buf, rx_packet.timestamp_us, (double)rx_time/1e6);
		}

		// Calculate latency
		float latency_us = rx_time - rx_packet.timestamp_us;

		// Validate packet
		if (!validatePacket(rx_packet, latency_us)) {
			continue; // Validation failed, packet already counted in error stats
		}

		// Create extended packet with metadata
		RxImuPacket ext_packet;
		ext_packet.data = rx_packet;
		ext_packet.rx_timestamp_us = rx_time;
		ext_packet.latency_us = latency_us;
		ext_packet.valid = true;

		// Update latency statistics
		_latency_sum_us += latency_us;
		_max_latency_us = fmaxf(_max_latency_us, latency_us);
		_min_latency_us = fminf(_min_latency_us, latency_us);

		// Check for sequence gaps (with intelligent handling for wraparound and reset)
		if (_last_sequence_received > 0) {
			uint32_t expected_seq = _last_sequence_received + 1;

			// Handle sequence number wraparound (uint32_t rollover)
			bool is_gap = false;
			if (rx_packet.sequence < _last_sequence_received) {
				// Check if this is a wraparound or a significant backward jump
				uint32_t backward_jump = _last_sequence_received - rx_packet.sequence;
				if (backward_jump > 1000000) {  // Likely wraparound (very large backward jump)
					// Accept as normal wraparound, reset tracking
					expected_seq = rx_packet.sequence;
				} else if (backward_jump > 10) { // Significant backward jump, likely restart
					// Sequence reset detected (e.g., EKF restart), reset tracking
					PX4_INFO("EKF2_AI_Subscriber: Sequence reset detected - Last: %" PRIu32 ", New: %" PRIu32,
					         _last_sequence_received, rx_packet.sequence);
					expected_seq = rx_packet.sequence;
				} else {
					// Small backward jump - this is a real gap
					is_gap = true;
				}
			} else if (rx_packet.sequence != expected_seq) {
				// Forward gap
				is_gap = true;
			}

			if (is_gap) {
				_sequence_gaps++;
				// Only log significant gaps to avoid spam from occasional out-of-order packets
				if ((rx_packet.sequence > expected_seq && (rx_packet.sequence - expected_seq) > 5) ||
				    (rx_packet.sequence < expected_seq && (expected_seq - rx_packet.sequence) > 5)) {
					PX4_WARN("EKF2_AI_Subscriber: Sequence gap detected - Expected: %" PRIu32 ", Got: %" PRIu32 " (gap: %d)",
					         expected_seq, rx_packet.sequence, (int32_t)(rx_packet.sequence - expected_seq));
				}
			}
		}
		_last_sequence_received = rx_packet.sequence;

		// Push to RX buffer (non-blocking, overwrites oldest)
		pushToRxBuffer(ext_packet);

		// Transfer from RX buffer to AI queue if space available
		RxImuPacket transfer_packet;
		if (popFromRxBuffer(transfer_packet)) {
			pushToAiQueue(transfer_packet);
		}
	}

	PX4_INFO("EKF2_AI_Subscriber: Receiver thread stopped");
}

bool EKF2_AI_Subscriber::validatePacket(const ImuUdpPacket &packet, float latency_us)
{
	// Check latency threshold
	if (latency_us > MAX_ACCEPTED_LATENCY_US) {
		_packets_dropped_latency++;
		if (_packets_dropped_latency % 100 == 1) { // Log every 100th drop to avoid spam
			   PX4_WARN("EKF2_AI_Subscriber: Packet dropped due to high latency: %.2f ms (max: %.1f ms)",
						(double)(latency_us/1000.0f), (double)(MAX_ACCEPTED_LATENCY_US/1000.0f));
		}
		return false;
	}

	// Validate CRC
	uint16_t calculated_crc = calculateCrc16(reinterpret_cast<const uint8_t*>(&packet),
	                                         sizeof(ImuUdpPacket) - sizeof(uint16_t));
	if (calculated_crc != packet.crc16) {
		_packets_dropped_crc_error++;
		PX4_WARN("EKF2_AI_Subscriber: CRC mismatch - Calculated: 0x%04X, Received: 0x%04X",
		         calculated_crc, packet.crc16);
		return false;
	}

	// Validate delta times
	if (packet.delta_ang_dt <= 0.0f || packet.delta_vel_dt <= 0.0f) {
		_validation_errors++;
			   PX4_WARN("EKF2_AI_Subscriber: Invalid delta times - gyro_dt: %.6f, accel_dt: %.6f",
						(double)packet.delta_ang_dt, (double)packet.delta_vel_dt);
		return false;
	}

	return true;
}

uint16_t EKF2_AI_Subscriber::calculateCrc16(const uint8_t *data, size_t len)
{
	// CRC16-CCITT (0xFFFF initial value, 0x1021 polynomial)
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < len; i++) {
		crc ^= static_cast<uint16_t>(data[i]) << 8;

		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			} else {
				crc = crc << 1;
			}
		}
	}

	return crc;
}

void EKF2_AI_Subscriber::pushToRxBuffer(const RxImuPacket &sample)
{
	size_t write_idx = _rx_write_idx.load();
	size_t next_write_idx = (write_idx + 1) % RX_RING_BUFFER_SIZE;

	// Check if buffer is full
	if (_rx_buffer_count.load() >= RX_RING_BUFFER_SIZE) {
		_rx_buffer_overruns++;
		// Overwrite oldest sample (advance read index)
		_rx_read_idx = (_rx_read_idx.load() + 1) % RX_RING_BUFFER_SIZE;
	} else {
		_rx_buffer_count++;
	}

	// Store sample
	_rx_buffer[write_idx] = sample;
	_rx_write_idx = next_write_idx;

	// Update max size tracking
	uint32_t current_size = _rx_buffer_count.load();
	if (current_size > _rx_buffer_max_size) {
		_rx_buffer_max_size = current_size;
	}
}

bool EKF2_AI_Subscriber::popFromRxBuffer(RxImuPacket &sample)
{
	if (_rx_buffer_count.load() == 0) {
		return false;
	}

	size_t read_idx = _rx_read_idx.load();
	sample = _rx_buffer[read_idx];

	_rx_read_idx = (read_idx + 1) % RX_RING_BUFFER_SIZE;
	_rx_buffer_count--;

	return true;
}

void EKF2_AI_Subscriber::pushToAiQueue(const RxImuPacket &sample)
{
	size_t write_idx = _ai_write_idx.load();
	size_t next_write_idx = (write_idx + 1) % AI_INFERENCE_QUEUE_SIZE;

	// Check if queue is full
	if (_ai_queue_count.load() >= AI_INFERENCE_QUEUE_SIZE) {
		_ai_queue_drops++;
		// Drop oldest sample (advance read index)
		_ai_read_idx = (_ai_read_idx.load() + 1) % AI_INFERENCE_QUEUE_SIZE;
	} else {
		_ai_queue_count++;
	}

	// Store sample
	_ai_queue[write_idx] = sample;
	_ai_write_idx = next_write_idx;

	// Update max size tracking
	uint32_t current_size = _ai_queue_count.load();
	if (current_size > _ai_queue_max_size) {
		_ai_queue_max_size = current_size;
	}
}

bool EKF2_AI_Subscriber::popAiSample(RxImuPacket &sample)
{
	if (_ai_queue_count.load() == 0) {
		return false;
	}

	size_t read_idx = _ai_read_idx.load();
	sample = _ai_queue[read_idx];

	_ai_read_idx = (read_idx + 1) % AI_INFERENCE_QUEUE_SIZE;
	_ai_queue_count--;

	return true;
}

void EKF2_AI_Subscriber::logStatistics()
{
	uint64_t now = hrt_absolute_time();

	// Log at configured interval
	if (now - _last_stats_log_time_us < STATS_LOG_INTERVAL_US) {
		return;
	}

	float elapsed_s = (now - _last_stats_log_time_us) / 1e6f;
	float rx_rate_hz = 0.0f;
	if (elapsed_s > 0.0f) {
		rx_rate_hz = _stats_interval_packet_count / elapsed_s;
	}

	// Calculate time since first packet
	float time_since_start_s = 0.0f;
	if (_first_packet_time_us > 0) {
		time_since_start_s = (now - _first_packet_time_us) / 1e6f;
	}

	// Calculate average latency
	float avg_latency_us = 0.0f;
	uint64_t total_packets = _total_packets_received.load();
	if (total_packets > 0) {
		avg_latency_us = _latency_sum_us / static_cast<float>(total_packets);
	}

	// Get current buffer sizes
	uint32_t rx_current = _rx_buffer_count.load();
	uint32_t ai_current = _ai_queue_count.load();

	// Calculate success rate
	uint64_t total_drops = _packets_dropped_crc_error.load() + _packets_dropped_latency.load() +
	                       _packets_dropped_rx_overflow.load();
	float success_rate = 100.0f;
	if (total_packets > 0) {
		success_rate = 100.0f * (total_packets - total_drops) / total_packets;
	}

	// Comprehensive logging
	PX4_INFO("=== EKF2_AI_Subscriber Statistics ===");
	PX4_INFO("Time: %.1f s since start | %.2f s since last log",
		  (double)time_since_start_s, (double)elapsed_s);
	PX4_INFO("RX Rate: %.1f Hz (target: 250 Hz) | Success: %.1f%%",
		  (double)rx_rate_hz, (double)success_rate);

	PX4_INFO("Packets: Total %" PRIu64 " | CRC errors %" PRIu64 " | Latency drops %" PRIu64 " | RX overflows %" PRIu64,
	         total_packets, _packets_dropped_crc_error.load(),
	         _packets_dropped_latency.load(), _packets_dropped_rx_overflow.load());

	PX4_INFO("RX Buffer: %u/%u (max: %u) | Overruns: %" PRIu64,
	         rx_current, (uint32_t)RX_RING_BUFFER_SIZE, _rx_buffer_max_size,
	         _rx_buffer_overruns.load());

	PX4_INFO("AI Queue: %u/%u (max: %u) | Drops: %" PRIu64,
	         ai_current, (uint32_t)AI_INFERENCE_QUEUE_SIZE, _ai_queue_max_size,
	         _ai_queue_drops.load());

	PX4_INFO("Latency: avg %.1f µs | min %.1f µs | max %.1f µs (limit: %.0f µs)",
		  (double)avg_latency_us, (double)_min_latency_us, (double)_max_latency_us, (double)MAX_ACCEPTED_LATENCY_US);

	PX4_INFO("Sequence: Last %" PRIu32 " | Gaps: %" PRIu64 " | Socket errors: %" PRIu64,
	         _last_sequence_received, _sequence_gaps.load(), _socket_errors.load());

	// Warnings for anomalies
	if (success_rate < 95.0f) {
		PX4_WARN("EKF2_AI_Subscriber: Low success rate: %.1f%% - Check network/sender", (double)success_rate);
	}

	if (rx_rate_hz < 240.0f || rx_rate_hz > 260.0f) {
		PX4_WARN("EKF2_AI_Subscriber: RX rate outside expected range: %.1f Hz", (double)rx_rate_hz);
	}

	if (_max_latency_us > MAX_ACCEPTED_LATENCY_US * 0.8f) {
		PX4_WARN("EKF2_AI_Subscriber: High max latency: %.1f ms (approaching limit)",
			  (double)(_max_latency_us/1000.0f));
	}

	if (_ai_queue_drops.load() > 0) {
		PX4_WARN("EKF2_AI_Subscriber: AI queue drops detected: %" PRIu64 " - Increase inference rate",
		         _ai_queue_drops.load());
	}

	// Reset interval statistics
	_last_stats_log_time_us = now;
	_stats_interval_packet_count = 0;
	_min_latency_us = 1e6f;
}

EKF2_AI_Subscriber::Statistics EKF2_AI_Subscriber::getStatistics() const
{
	Statistics stats;

	// UDP reception stats
	stats.total_packets_received = _total_packets_received.load();
	stats.packets_dropped_crc_error = _packets_dropped_crc_error.load();
	stats.packets_dropped_latency = _packets_dropped_latency.load();
	stats.packets_dropped_rx_overflow = _packets_dropped_rx_overflow.load();

	// Buffer stats
	stats.rx_buffer_overruns = _rx_buffer_overruns.load();
	stats.rx_buffer_current_size = _rx_buffer_count.load();
	stats.rx_buffer_max_size = _rx_buffer_max_size;

	stats.ai_queue_drops = _ai_queue_drops.load();
	stats.ai_queue_current_size = _ai_queue_count.load();
	stats.ai_queue_max_size = _ai_queue_max_size;

	// Timing stats
	if (_total_packets_received.load() > 0) {
		stats.avg_latency_us = _latency_sum_us / static_cast<float>(_total_packets_received.load());
	} else {
		stats.avg_latency_us = 0.0f;
	}
	stats.max_latency_us = _max_latency_us;
	stats.min_latency_us = _min_latency_us;

	// Calculate current RX rate
	uint64_t now = hrt_absolute_time();
	if (_first_packet_time_us > 0 && now > _first_packet_time_us) {
		float elapsed_s = (now - _first_packet_time_us) / 1e6f;
		stats.avg_rx_rate_hz = _total_packets_received.load() / elapsed_s;
	} else {
		stats.avg_rx_rate_hz = 0.0f;
	}

	// Sequence and error stats
	stats.last_sequence_received = _last_sequence_received;
	stats.sequence_gaps = _sequence_gaps.load();
	stats.socket_errors = _socket_errors.load();
	stats.validation_errors = _validation_errors.load();

	return stats;
}
