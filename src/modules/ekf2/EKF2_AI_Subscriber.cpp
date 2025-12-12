/*WAS MEANT FOR INITIAL UDP TESTING - DELETE IF UNUSED*/



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

	// Create UDP socket for receiving AI-processed IMU data
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

	// Set large receive buffer for high-frequency data (250Hz * 50 bytes = 12.5 KB/s)
	// Use 256KB buffer to handle ~20 seconds of buffering
	int rcvbuf = 4*1024*1024; // 256KB receive buffer
	if (setsockopt(_socket_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
		PX4_WARN("EKF2_AI_Subscriber: Failed to set SO_RCVBUF: %s", strerror(errno));
	}

	// Verify actual buffer size achieved
	int actual_rcvbuf = 0;
	socklen_t optlen = sizeof(actual_rcvbuf);
	if (getsockopt(_socket_fd, SOL_SOCKET, SO_RCVBUF, &actual_rcvbuf, &optlen) == 0) {
		PX4_INFO("EKF2_AI_Subscriber: SO_RCVBUF set to %d bytes (requested: %d)", actual_rcvbuf, rcvbuf);
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
	ImuNetworkPacket rx_packet;
	struct sockaddr_in sender_addr;
	socklen_t sender_len = sizeof(sender_addr);

	PX4_INFO("EKF2_AI_Subscriber: Receiver thread started");

	while (!_should_exit) {
		// Receive packet (non-blocking)
		ssize_t bytes_received = recvfrom(_socket_fd, &rx_packet, sizeof(rx_packet), 0,
		                                  (struct sockaddr*)&sender_addr, &sender_len);

		if (bytes_received < 0) {
			if (errno == EAGAIN) {
				// No data available - use minimal sleep to balance CPU vs latency
				usleep(10); // 10µs sleep (allows ~100k checks/sec)
				continue;
			} else {
				_socket_errors++;
				// Only log socket errors occasionally to avoid spam
				if (_socket_errors.load() % 1000 == 1) {
					PX4_ERR("EKF2_AI_Subscriber: recvfrom failed: %s", strerror(errno));
				}
				usleep(1000); // 1ms sleep on error
				continue;
		}
	}

	if (bytes_received != sizeof(ImuNetworkPacket)) {
		_validation_errors++;
		// Only log size errors occasionally
		if (_validation_errors.load() % 100 == 1) {
			PX4_WARN("EKF2_AI_Subscriber: Invalid packet size: %zd (expected %zu)",
			         bytes_received, sizeof(ImuNetworkPacket));
		}
		continue;
	}		uint64_t rx_time = hrt_absolute_time();
		_total_packets_received++;
		_stats_interval_packet_count++;

		// Track first packet time (one-time log)
		if (_first_packet_time_us == 0) {
			_first_packet_time_us = rx_time;
			char addr_buf[INET_ADDRSTRLEN];
			inet_ntop(AF_INET, &(sender_addr.sin_addr), addr_buf, sizeof(addr_buf));
			PX4_INFO("EKF2_AI_Subscriber: First packet received - seq=%" PRIu32 ", timestamp_us=%" PRIu64 " from %s:%u",
			         rx_packet.sequence, rx_packet.timestamp_us, addr_buf, ntohs(sender_addr.sin_port));
		}

		// Calculate latency
		float latency_us = rx_time - rx_packet.timestamp_us;

		// Fast validation (CRC + latency check)
		if (!validatePacket(rx_packet, latency_us)) {
			continue; // Validation failed, packet already counted in error stats
		}

		// Update latency statistics (lock-free)
		_latency_sum_us += latency_us;
		if (latency_us > _max_latency_us) _max_latency_us = latency_us;
		if (latency_us < _min_latency_us) _min_latency_us = latency_us;

		// Simple sequence gap tracking (no complex wraparound logic for TX/RX test)
		if (_last_sequence_received > 0) {
			uint32_t expected_seq = _last_sequence_received + 1;

			// Simple gap detection
			if (rx_packet.sequence != expected_seq) {
				_sequence_gaps++;
			}
		}
		_last_sequence_received = rx_packet.sequence;

		// Create extended packet with metadata
		RxImuPacket ext_packet;
		ext_packet.data = rx_packet;
		ext_packet.rx_timestamp_us = rx_time;
		ext_packet.latency_us = latency_us;
		ext_packet.valid = true;

		// Direct push to AI queue (skip RX buffer layer for TX/RX sanity check)
		// This reduces latency and simplifies the data path
		pushToAiQueue(ext_packet);
	}

	PX4_INFO("EKF2_AI_Subscriber: Receiver thread stopped");
}

bool EKF2_AI_Subscriber::validatePacket(const ImuNetworkPacket &packet, float latency_us)
{
	// Check latency threshold
	if (latency_us > MAX_ACCEPTED_LATENCY_US) {
		_packets_dropped_latency++;
		// Minimal logging - only log first few and then every 1000th drop
		uint64_t drops = _packets_dropped_latency.load();
		if (drops <= 3 || drops % 1000 == 0) {
			PX4_WARN("EKF2_AI_Subscriber: High latency packet dropped: %.2f ms (limit: %.1f ms, total drops: %" PRIu64 ")",
			         (double)(latency_us/1000.0f), (double)(MAX_ACCEPTED_LATENCY_US/1000.0f), drops);
		}
		return false;
	}

	// Validate CRC
	uint16_t calculated_crc = calculateCrc16(reinterpret_cast<const uint8_t*>(&packet),
	                                         sizeof(ImuNetworkPacket) - sizeof(uint16_t));
	if (calculated_crc != packet.crc16) {
		_packets_dropped_crc_error++;
		// Minimal logging - only log first few CRC errors
		uint64_t crc_errors = _packets_dropped_crc_error.load();
		if (crc_errors <= 3) {
			PX4_WARN("EKF2_AI_Subscriber: CRC mismatch - Calculated: 0x%04X, Received: 0x%04X",
			         calculated_crc, packet.crc16);
		}
		return false;
	}

	// Validate delta times (sanity check for corrupted data)
	if (packet.delta_ang_dt <= 0.0f || packet.delta_vel_dt <= 0.0f ||
	    packet.delta_ang_dt > 0.1f || packet.delta_vel_dt > 0.1f) {
		_validation_errors++;
		// Minimal logging
		uint64_t val_errors = _validation_errors.load();
		if (val_errors <= 3) {
			PX4_WARN("EKF2_AI_Subscriber: Invalid delta times - gyro_dt: %.6f, accel_dt: %.6f",
			         (double)packet.delta_ang_dt, (double)packet.delta_vel_dt);
		}
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
	uint32_t ai_current = _ai_queue_count.load();

	// Calculate success rate
	uint64_t total_drops = _packets_dropped_crc_error.load() + _packets_dropped_latency.load();
	float success_rate = 100.0f;
	if (total_packets > 0) {
		success_rate = 100.0f * (total_packets - total_drops) / total_packets;
	}

	// Streamlined logging for TX/RX sanity check
	PX4_INFO("=== AI_SUB Stats [%.1f s] ===", (double)time_since_start_s);
	PX4_INFO("RX: %.1f Hz | Success: %.2f%% | Total: %" PRIu64,
	         (double)rx_rate_hz, (double)success_rate, total_packets);

	PX4_INFO("Latency: avg %.0f µs | min %.0f µs | max %.0f µs",
	         (double)avg_latency_us, (double)_min_latency_us, (double)_max_latency_us);

	PX4_INFO("AI Queue: %u/%u (max: %u) | Drops: %" PRIu64,
	         ai_current, (uint32_t)AI_INFERENCE_QUEUE_SIZE, _ai_queue_max_size,
	         _ai_queue_drops.load());

	PX4_INFO("Sequence: Last %" PRIu32 " | Gaps: %" PRIu64,
	         _last_sequence_received, _sequence_gaps.load());

	// Error summary (only if errors present)
	uint64_t crc_errors = _packets_dropped_crc_error.load();
	uint64_t latency_drops = _packets_dropped_latency.load();
	uint64_t socket_errs = _socket_errors.load();
	uint64_t val_errs = _validation_errors.load();

	if (crc_errors > 0 || latency_drops > 0 || socket_errs > 0 || val_errs > 0) {
		PX4_INFO("Errors: CRC %" PRIu64 " | Latency %" PRIu64 " | Socket %" PRIu64 " | Validation %" PRIu64,
		         crc_errors, latency_drops, socket_errs, val_errs);
	}

	// Critical warnings
	if (success_rate < 95.0f) {
		PX4_WARN("AI_SUB: Low success rate: %.1f%% - Check network quality", (double)success_rate);
	}

	if (rx_rate_hz < 200.0f && total_packets > 100) {
		PX4_WARN("AI_SUB: Low RX rate: %.1f Hz (expected ~250 Hz)", (double)rx_rate_hz);
	}

	if (_sequence_gaps.load() > (total_packets / 10)) {
		PX4_WARN("AI_SUB: High sequence gap rate: %" PRIu64 " gaps in %" PRIu64 " packets",
		         _sequence_gaps.load(), total_packets);
	}

	// Reset interval statistics
	_last_stats_log_time_us = now;
	_stats_interval_packet_count = 0;
	_min_latency_us = 1e6f;
}

EKF2_AI_Subscriber::Statistics EKF2_AI_Subscriber::getStatistics() const
{
	Statistics stats;

	// Network reception stats
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
