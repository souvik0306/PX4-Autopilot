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

#include "EKF2_UdpPublisher.hpp"

EKF2_UdpPublisher::EKF2_UdpPublisher()
{
	// Initialize all buffer slots
	for (size_t i = 0; i < RING_BUFFER_SIZE; i++) {
		_ring_buffer[i].occupied = false;
	}
}

EKF2_UdpPublisher::~EKF2_UdpPublisher()
{
	if (_socket_fd >= 0) {
		close(_socket_fd);
		_socket_fd = -1;
	}
}

bool EKF2_UdpPublisher::init()
{
	if (_initialized) {
		return true;
	}

	// Create UDP socket
	_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (_socket_fd < 0) {
		PX4_ERR("EKF2_UdpPublisher: Failed to create socket: %s", strerror(errno));
		return false;
	}

	// Configure for non-blocking operation to prevent blocking on send
	int flags = fcntl(_socket_fd, F_GETFL, 0);
	if (flags == -1) {
		PX4_ERR("EKF2_UdpPublisher: fcntl F_GETFL failed: %s", strerror(errno));
		close(_socket_fd);
		_socket_fd = -1;
		return false;
	}

	if (fcntl(_socket_fd, F_SETFL, flags | O_NONBLOCK) == -1) {
		PX4_ERR("EKF2_UdpPublisher: fcntl F_SETFL failed: %s", strerror(errno));
		close(_socket_fd);
		_socket_fd = -1;
		return false;
	}

	// Configure target address
	memset(&_target_addr, 0, sizeof(_target_addr));
	_target_addr.sin_family = AF_INET;
	_target_addr.sin_port = htons(TARGET_PORT);

	if (inet_pton(AF_INET, TARGET_IP, &_target_addr.sin_addr) <= 0) {
		PX4_ERR("EKF2_UdpPublisher: Invalid target IP address: %s", TARGET_IP);
		close(_socket_fd);
		_socket_fd = -1;
		return false;
	}

	// Set socket buffer size for better throughput at 250Hz
	int sndbuf = 65536; // 64KB send buffer
	if (setsockopt(_socket_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
		PX4_WARN("EKF2_UdpPublisher: Failed to set SO_SNDBUF: %s", strerror(errno));
		// Non-fatal, continue
	}

	// Enable broadcast (optional, for future multicast support)
	int broadcast = 1;
	if (setsockopt(_socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
		PX4_WARN("EKF2_UdpPublisher: Failed to set SO_BROADCAST: %s", strerror(errno));
		// Non-fatal, continue
	}

	_initialized = true;
	_last_stats_log_time_us = hrt_absolute_time();

	PX4_INFO("EKF2_UdpPublisher: Initialized successfully - Target: %s:%u", TARGET_IP, TARGET_PORT);
	return true;
}

bool EKF2_UdpPublisher::publishSample(uint64_t timestamp_us,
                                      const Vector3f &delta_ang,
                                      float delta_ang_dt,
                                      const Vector3f &delta_vel,
                                      float delta_vel_dt)
{
	if (!_initialized) {
		PX4_ERR("EKF2_UdpPublisher: Not initialized");
		return false;
	}

	// Validate delta times to prevent division by zero
	if (delta_ang_dt <= 0.0f || delta_vel_dt <= 0.0f) {
		PX4_ERR("EKF2_UdpPublisher: Invalid delta time (gyro: %.6f, accel: %.6f)",
		        (double)delta_ang_dt, (double)delta_vel_dt);
		_stats.send_failures++;
		return false;
	}

	   // Track first sample time
	   if (_first_sample_time_us == 0) {
		   _first_sample_time_us = timestamp_us;
		   // Convert to wall-clock time (localtime)
		   time_t now_sec = time(nullptr);
		   struct tm local_tm;
		   localtime_r(&now_sec, &local_tm);
		   char time_buf[32];
		   strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &local_tm);
		   PX4_INFO("EKF2_UdpPublisher: First sample received at %s (wall time), timestamp_us=%" PRIu64 ", t=%.3f s from boot", time_buf, timestamp_us, (double)timestamp_us/1e6);
	   }

	// Calculate sample interval for monitoring
	if (_last_sample_time_us > 0) {
		float interval = timestamp_us - _last_sample_time_us;
		_min_sample_interval_us = fminf(_min_sample_interval_us, interval);
		_max_sample_interval_us = fmaxf(_max_sample_interval_us, interval);
	}
	_last_sample_time_us = timestamp_us;

	// Convert delta values to instantaneous rates
	// gyro: rad/s = delta_angle / delta_time
	// accel: m/s² = delta_velocity / delta_time
	Vector3f gyro_rate = delta_ang / delta_ang_dt;
	Vector3f accel = delta_vel / delta_vel_dt;

	// Prepare packet
	ImuUdpPacket pkt{};
	pkt.timestamp_us = timestamp_us;
	pkt.sequence = _sequence_counter++;
	pkt.gyro_x = gyro_rate(0);
	pkt.gyro_y = gyro_rate(1);
	pkt.gyro_z = gyro_rate(2);
	pkt.accel_x = accel(0);
	pkt.accel_y = accel(1);
	pkt.accel_z = accel(2);
	pkt.delta_ang_dt = delta_ang_dt;
	pkt.delta_vel_dt = delta_vel_dt;

	// Calculate CRC for integrity (excluding CRC field itself)
	pkt.crc16 = calculateCrc16(reinterpret_cast<const uint8_t*>(&pkt),
	                           sizeof(ImuUdpPacket) - sizeof(uint16_t));

	// Track enqueue time for latency measurement
	uint64_t enqueue_time = hrt_absolute_time();

	// Send immediately (no buffering for lowest latency)
	// Ring buffer is kept for future expansion if batching is needed
	bool sent = sendPacket(pkt);

	// Update statistics
	_stats.total_samples++;
	_stats_interval_sample_count++;

	if (sent) {
		_stats.total_sent++;

		// Calculate and track latency
		float latency_us = hrt_absolute_time() - enqueue_time;
		_latency_sum_us += latency_us;
		_stats.max_latency_us = fmaxf(_stats.max_latency_us, latency_us);
		_stats.avg_latency_us = _latency_sum_us / static_cast<float>(_stats.total_sent);

		// Warn on high latency
		if (latency_us > 20.0f) { // 20µs threshold
			PX4_WARN("EKF2_UdpPublisher: High latency detected: %.2f µs", (double)latency_us);
		}
	} else {
		_stats.send_failures++;
	}

	_stats.current_sequence = _sequence_counter;

	return sent;
}

bool EKF2_UdpPublisher::sendPacket(const ImuUdpPacket &pkt)
{
	ssize_t bytes_sent = sendto(_socket_fd,
	                            &pkt,
	                            sizeof(ImuUdpPacket),
	                            0,
	                            reinterpret_cast<const struct sockaddr*>(&_target_addr),
	                            sizeof(_target_addr));

	if (bytes_sent < 0) {
		   if (errno == EAGAIN) {
			   // Non-blocking socket buffer full - expected occasional occurrence at 250Hz
			   return false;
		   } else {
			   PX4_ERR("EKF2_UdpPublisher: sendto failed: %s (errno: %d)", strerror(errno), errno);
			   return false;
		   }
	}

	if (bytes_sent != sizeof(ImuUdpPacket)) {
		PX4_ERR("EKF2_UdpPublisher: Partial send - Expected: %zu, Sent: %zd",
		        sizeof(ImuUdpPacket), bytes_sent);
		return false;
	}

	return true;
}

void EKF2_UdpPublisher::logStatistics()
{
	uint64_t now = hrt_absolute_time();

	// Log at configured interval (default 1 second)
	if (now - _last_stats_log_time_us < STATS_LOG_INTERVAL_US) {
		return;
	}

	   float elapsed_s = (now - _last_stats_log_time_us) / 1e6f;
	   float rate_hz = _stats_interval_sample_count / elapsed_s;

	   // Calculate absolute time since publisher start (first sample)
	   float time_since_start_s = 0.0f;
	   if (_first_sample_time_us > 0) {
		   time_since_start_s = (now - _first_sample_time_us) / 1e6f;
	   }

	// Calculate success rate
	float success_rate = 0.0f;
	if (_stats.total_samples > 0) {
		success_rate = 100.0f * _stats.total_sent / _stats.total_samples;
	}

			PX4_INFO_RAW("EKF2_UdpPublisher Statistics:\n");
			PX4_INFO_RAW("  Time since start: %.1f s | Time since last log: %.2f s\n",
						(double)time_since_start_s, (double)elapsed_s);
			PX4_INFO_RAW("  Rate: %.1f Hz (target: 250 Hz)\n", (double)rate_hz);
			PX4_INFO_RAW("  Total: %" PRIu64 " samples | Sent: %" PRIu64 " (%.1f%%)\n",
						_stats.total_samples, _stats.total_sent, (double)success_rate);
			PX4_INFO_RAW("  Failures: %" PRIu64 " | Buffer overruns: %" PRIu64 "\n",
						_stats.send_failures, _stats.buffer_overruns);
			PX4_INFO_RAW("  Sequence: %" PRIu32 " | Avg latency: %.1f µs | Max latency: %.1f µs\n",
						_stats.current_sequence, (double)_stats.avg_latency_us, (double)_stats.max_latency_us);
			PX4_INFO_RAW("  Sample interval: min=%.0f µs, max=%.0f µs (nominal: 4000 µs @ 250Hz)\n",
						(double)_min_sample_interval_us, (double)_max_sample_interval_us);

	// Warnings for anomalies
	if (success_rate < 95.0f) {
		PX4_WARN("EKF2_UdpPublisher: Low success rate: %.1f%% - Check network/receiver",
		         (double)success_rate);
	}

	if (_stats.max_latency_us > 20000.0f) { // 20ms threshold
		PX4_WARN("EKF2_UdpPublisher: High max latency: %.1f ms",
		         (double)(_stats.max_latency_us / 1000.0f));
	}

	if (rate_hz < 240.0f || rate_hz > 260.0f) {
		PX4_WARN("EKF2_UdpPublisher: Sample rate outside expected range: %.1f Hz",
		         (double)rate_hz);
	}

	// Reset interval statistics
	_last_stats_log_time_us = now;
	_stats_interval_sample_count = 0;
	_min_sample_interval_us = 1e6f;
	_max_sample_interval_us = 0;
}

uint16_t EKF2_UdpPublisher::calculateCrc16(const uint8_t *data, size_t len)
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
