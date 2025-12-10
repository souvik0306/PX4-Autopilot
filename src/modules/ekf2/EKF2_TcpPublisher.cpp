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
 * @file EKF2_TcpPublisher.cpp
 * TCP-based IMU publisher implementation
 */

#include "EKF2_TcpPublisher.hpp"

EKF2_TcpPublisher::EKF2_TcpPublisher()
{
	memset(&_server_addr, 0, sizeof(_server_addr));
	memset(&_client_addr, 0, sizeof(_client_addr));
	memset(&_stats, 0, sizeof(_stats));
}

EKF2_TcpPublisher::~EKF2_TcpPublisher()
{
	closeClient();

	if (_server_fd >= 0) {
		close(_server_fd);
		_server_fd = -1;
	}

	PX4_INFO("[EKF2_TcpPublisher %p] Destroyed. Total samples: %llu, Total sent: %llu, Send failures: %llu",
	         this, (unsigned long long)_stats.total_samples, (unsigned long long)_stats.total_sent,
	         (unsigned long long)_stats.send_failures);
}

bool EKF2_TcpPublisher::init()
{
	if (_initialized) {
		PX4_WARN("[EKF2_TcpPublisher %p] Already initialized", this);
		return true;
	}

	// Create TCP socket
	_server_fd = socket(AF_INET, SOCK_STREAM, 0);

	if (_server_fd < 0) {
		PX4_ERR("[EKF2_TcpPublisher %p] Failed to create TCP socket: %s", this, strerror(errno));
		return false;
	}

	// Set socket options: reuse address to avoid "address already in use" errors
	int opt = 1;

	if (setsockopt(_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
		PX4_ERR("[EKF2_TcpPublisher %p] Failed to set SO_REUSEADDR: %s", this, strerror(errno));
		close(_server_fd);
		_server_fd = -1;
		return false;
	}

	// Disable Nagle's algorithm for low-latency streaming
	if (setsockopt(_server_fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)) < 0) {
		PX4_WARN("[EKF2_TcpPublisher %p] Failed to set TCP_NODELAY: %s", this, strerror(errno));
		// Non-fatal, continue
	}

	// Set non-blocking mode
	int flags = fcntl(_server_fd, F_GETFL, 0);

	if (flags < 0) {
		PX4_ERR("[EKF2_TcpPublisher %p] Failed to get socket flags: %s", this, strerror(errno));
		close(_server_fd);
		_server_fd = -1;
		return false;
	}

	if (fcntl(_server_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
		PX4_ERR("[EKF2_TcpPublisher %p] Failed to set non-blocking mode: %s", this, strerror(errno));
		close(_server_fd);
		_server_fd = -1;
		return false;
	}

	// Bind to port 14567
	_server_addr.sin_family = AF_INET;
	_server_addr.sin_addr.s_addr = INADDR_ANY;
	_server_addr.sin_port = htons(LISTEN_PORT);

	if (bind(_server_fd, (struct sockaddr *)&_server_addr, sizeof(_server_addr)) < 0) {
		PX4_ERR("[EKF2_TcpPublisher %p] Failed to bind to port %u: %s", this, LISTEN_PORT, strerror(errno));
		close(_server_fd);
		_server_fd = -1;
		return false;
	}

	// Start listening
	if (listen(_server_fd, MAX_PENDING_CONNECTIONS) < 0) {
		PX4_ERR("[EKF2_TcpPublisher %p] Failed to listen: %s", this, strerror(errno));
		close(_server_fd);
		_server_fd = -1;
		return false;
	}

	_initialized = true;
	_first_sample_time_us = hrt_absolute_time();
	_last_stats_log_time_us = _first_sample_time_us;

	PX4_INFO("[EKF2_TcpPublisher %p] Initialized. Listening on port %u", this, LISTEN_PORT);

	return true;
}

bool EKF2_TcpPublisher::acceptClient()
{
	if (_server_fd < 0) {
		return false;
	}

	if (_client_fd >= 0) {
		// Already have a client
		return false;
	}

	socklen_t client_len = sizeof(_client_addr);
	int new_client_fd = accept(_server_fd, (struct sockaddr *)&_client_addr, &client_len);

	if (new_client_fd < 0) {
		if (errno != EAGAIN) {
			PX4_WARN("[EKF2_TcpPublisher %p] Accept failed: %s", this, strerror(errno));
		}

		return false;
	}

	// Set client socket to non-blocking
	int flags = fcntl(new_client_fd, F_GETFL, 0);

	if (flags >= 0) {
		fcntl(new_client_fd, F_SETFL, flags | O_NONBLOCK);
	}

	// Disable Nagle's algorithm on client socket
	int opt = 1;

	if (setsockopt(new_client_fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)) < 0) {
		PX4_WARN("[EKF2_TcpPublisher %p] Failed to set TCP_NODELAY on client socket: %s", this, strerror(errno));
	}

	_client_fd = new_client_fd;
	_last_client_connect_time_us = hrt_absolute_time();

	char client_ip[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &_client_addr.sin_addr, client_ip, sizeof(client_ip));
	PX4_INFO("[EKF2_TcpPublisher %p] Client connected from %s:%u", this, client_ip, ntohs(_client_addr.sin_port));

	return true;
}

void EKF2_TcpPublisher::closeClient()
{
	if (_client_fd >= 0) {
		close(_client_fd);
		_client_fd = -1;
		_stats.client_disconnects++;

		char client_ip[INET_ADDRSTRLEN];
		inet_ntop(AF_INET, &_client_addr.sin_addr, client_ip, sizeof(client_ip));
		PX4_INFO("[EKF2_TcpPublisher %p] Client disconnected: %s:%u", this, client_ip, ntohs(_client_addr.sin_port));
	}
}

bool EKF2_TcpPublisher::sendPacket(const ImuNetworkPacket &pkt)
{
	if (_client_fd < 0) {
		return false;
	}

	const uint8_t *data = reinterpret_cast<const uint8_t *>(&pkt);
	size_t total_sent = 0;
	size_t len = sizeof(pkt);

	// TCP send loop (handle partial sends)
	while (total_sent < len) {
		ssize_t sent = send(_client_fd, data + total_sent, len - total_sent, MSG_NOSIGNAL);

		if (sent < 0) {
			if (errno == EAGAIN) {
				// Would block, buffer full - drop sample to avoid blocking EKF2
				return false;
			}

			// Connection error (broken pipe, reset, etc.)
			PX4_WARN("[EKF2_TcpPublisher %p] Send error: %s. Closing client.", this, strerror(errno));
			closeClient();
			return false;
		}

		total_sent += sent;
	}

	return true;
}

bool EKF2_TcpPublisher::publishSample(uint64_t timestamp_us,
                                      const Vector3f &delta_ang,
                                      float delta_ang_dt,
                                      const Vector3f &delta_vel,
                                      float delta_vel_dt)
{
	if (!_initialized) {
		return false;
	}

	_stats.total_samples++;

	// Try to accept new client if none connected
	if (_client_fd < 0) {
		acceptClient();

		if (_client_fd < 0) {
			// No client connected yet, silently drop sample
			return false;
		}
	}

	// Track sample timing
	uint64_t now = hrt_absolute_time();

	if (_last_sample_time_us > 0) {
		float interval_us = (float)(now - _last_sample_time_us);

		if (interval_us < _min_sample_interval_us) {
			_min_sample_interval_us = interval_us;
		}

		if (interval_us > _max_sample_interval_us) {
			_max_sample_interval_us = interval_us;
		}
	}

	_last_sample_time_us = now;

	// Build packet
	ImuNetworkPacket pkt;
	pkt.timestamp_us = timestamp_us;
	pkt.sequence = _sequence_counter++;

	// Convert delta_ang to gyro (rad/s)
	if (delta_ang_dt > 1e-6f) {
		pkt.gyro_x = delta_ang(0) / delta_ang_dt;
		pkt.gyro_y = delta_ang(1) / delta_ang_dt;
		pkt.gyro_z = delta_ang(2) / delta_ang_dt;
	} else {
		pkt.gyro_x = pkt.gyro_y = pkt.gyro_z = 0.0f;
	}

	pkt.delta_ang_dt = delta_ang_dt;

	// Convert delta_vel to accel (m/s^2)
	if (delta_vel_dt > 1e-6f) {
		pkt.accel_x = delta_vel(0) / delta_vel_dt;
		pkt.accel_y = delta_vel(1) / delta_vel_dt;
		pkt.accel_z = delta_vel(2) / delta_vel_dt;
	} else {
		pkt.accel_x = pkt.accel_y = pkt.accel_z = 0.0f;
	}

	pkt.delta_vel_dt = delta_vel_dt;

	// Calculate CRC (exclude crc16 field itself)
	pkt.crc16 = calculateCrc16(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt) - sizeof(pkt.crc16));

	// Send packet
	bool sent = sendPacket(pkt);

	if (sent) {
		_stats.total_sent++;
		_stats_interval_sample_count++;

		// Calculate latency
		uint64_t send_time = hrt_absolute_time();
		float latency_us = (float)(send_time - timestamp_us);
		_latency_sum_us += latency_us;

		if (latency_us > _stats.max_latency_us) {
			_stats.max_latency_us = latency_us;
		}

		// Log first 4 samples for debugging
		if (_stats.total_sent <= 4) {
			PX4_INFO("[EKF2_TcpPublisher %p] Sample %u sent: ts=%llu seq=%u gyro=[%.3f,%.3f,%.3f] accel=[%.3f,%.3f,%.3f] dt_ang=%.6f dt_vel=%.6f crc=0x%04X latency=%.1fus",
			         this, (unsigned int)_stats.total_sent,
			         (unsigned long long)pkt.timestamp_us, pkt.sequence,
			         (double)pkt.gyro_x, (double)pkt.gyro_y, (double)pkt.gyro_z,
			         (double)pkt.accel_x, (double)pkt.accel_y, (double)pkt.accel_z,
			         (double)pkt.delta_ang_dt, (double)pkt.delta_vel_dt,
			         pkt.crc16, (double)latency_us);
		}
	} else {
		_stats.send_failures++;
	}

	_stats.current_sequence = _sequence_counter;

	// Log statistics periodically
	if (now - _last_stats_log_time_us > STATS_LOG_INTERVAL_US) {
		logStatistics();
		_last_stats_log_time_us = now;
		_stats_interval_sample_count = 0;
		_latency_sum_us = 0;
	}

	return sent;
}

void EKF2_TcpPublisher::logStatistics()
{
	uint64_t now = hrt_absolute_time();
	float uptime_s = (float)(now - _first_sample_time_us) / 1e6f;

	float avg_rate = 0;

	if (uptime_s > 0) {
		avg_rate = (float)_stats.total_samples / uptime_s;
	}

	_stats.avg_latency_us = 0;

	if (_stats_interval_sample_count > 0) {
		_stats.avg_latency_us = _latency_sum_us / (float)_stats_interval_sample_count;
	}

	float success_rate = 0;

	if (_stats.total_samples > 0) {
		success_rate = 100.0f * (float)_stats.total_sent / (float)_stats.total_samples;
	}

	PX4_INFO("[EKF2_TcpPublisher %p] Stats: uptime=%.1fs samples=%llu sent=%llu failed=%llu rate=%.1fHz success=%.1f%% "
	         "latency_avg=%.1fus latency_max=%.1fus client_connected=%d disconnects=%llu",
	         this, (double)uptime_s,
	         (unsigned long long)_stats.total_samples,
	         (unsigned long long)_stats.total_sent,
	         (unsigned long long)_stats.send_failures,
	         (double)avg_rate,
	         (double)success_rate,
	         (double)_stats.avg_latency_us,
	         (double)_stats.max_latency_us,
	         isClientConnected() ? 1 : 0,
	         (unsigned long long)_stats.client_disconnects);
}

uint16_t EKF2_TcpPublisher::calculateCrc16(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;

		for (int j = 0; j < 8; j++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;

			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}
