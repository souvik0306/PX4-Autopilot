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
 * @file EKF2_TcpSubscriber.cpp
 * TCP-based AI-processed IMU subscriber implementation
 */

#include "EKF2_TcpSubscriber.hpp"

EKF2_TcpSubscriber::EKF2_TcpSubscriber()
{
	memset(&_server_addr, 0, sizeof(_server_addr));
	memset(&_client_addr, 0, sizeof(_client_addr));
	memset(&_stats, 0, sizeof(_stats));
	memset(_rx_ring_buffer, 0, sizeof(_rx_ring_buffer));
	memset(_ai_queue, 0, sizeof(_ai_queue));
	memset(_tcp_buffer, 0, sizeof(_tcp_buffer));
}

EKF2_TcpSubscriber::~EKF2_TcpSubscriber()
{
	// Stop receiver thread
	if (_receiver_running) {
		_receiver_running = false;
		pthread_join(_receiver_thread, nullptr);
	}

	closeClient();

	if (_server_fd >= 0) {
		close(_server_fd);
		_server_fd = -1;
	}

	PX4_INFO("[EKF2_TcpSubscriber %p] Destroyed. Total received: %llu, CRC failures: %llu",
	         this, (unsigned long long)_stats.total_received, (unsigned long long)_stats.crc_failures);
}

bool EKF2_TcpSubscriber::init()
{
	if (_initialized) {
		PX4_WARN("[EKF2_TcpSubscriber %p] Already initialized", this);
		return true;
	}

	// Create TCP socket
	_server_fd = socket(AF_INET, SOCK_STREAM, 0);

	if (_server_fd < 0) {
		PX4_ERR("[EKF2_TcpSubscriber %p] Failed to create TCP socket: %s", this, strerror(errno));
		return false;
	}

	// Set socket options
	int opt = 1;

	if (setsockopt(_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
		PX4_ERR("[EKF2_TcpSubscriber %p] Failed to set SO_REUSEADDR: %s", this, strerror(errno));
		close(_server_fd);
		_server_fd = -1;
		return false;
	}

	// Disable Nagle's algorithm for low-latency
	if (setsockopt(_server_fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)) < 0) {
		PX4_WARN("[EKF2_TcpSubscriber %p] Failed to set TCP_NODELAY: %s", this, strerror(errno));
	}

	// Set non-blocking mode
	int flags = fcntl(_server_fd, F_GETFL, 0);

	if (flags < 0) {
		PX4_ERR("[EKF2_TcpSubscriber %p] Failed to get socket flags: %s", this, strerror(errno));
		close(_server_fd);
		_server_fd = -1;
		return false;
	}

	if (fcntl(_server_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
		PX4_ERR("[EKF2_TcpSubscriber %p] Failed to set non-blocking mode: %s", this, strerror(errno));
		close(_server_fd);
		_server_fd = -1;
		return false;
	}

	// Bind to port 14568
	_server_addr.sin_family = AF_INET;
	_server_addr.sin_addr.s_addr = INADDR_ANY;
	_server_addr.sin_port = htons(LISTEN_PORT);

	if (bind(_server_fd, (struct sockaddr *)&_server_addr, sizeof(_server_addr)) < 0) {
		PX4_ERR("[EKF2_TcpSubscriber %p] Failed to bind to port %u: %s", this, LISTEN_PORT, strerror(errno));
		close(_server_fd);
		_server_fd = -1;
		return false;
	}

	// Start listening
	if (listen(_server_fd, MAX_PENDING_CONNECTIONS) < 0) {
		PX4_ERR("[EKF2_TcpSubscriber %p] Failed to listen: %s", this, strerror(errno));
		close(_server_fd);
		_server_fd = -1;
		return false;
	}

	// Start receiver thread
	_receiver_running = true;

	if (pthread_create(&_receiver_thread, nullptr, receiverThreadEntry, this) != 0) {
		PX4_ERR("[EKF2_TcpSubscriber %p] Failed to create receiver thread", this);
		close(_server_fd);
		_server_fd = -1;
		_receiver_running = false;
		return false;
	}

	_initialized = true;
	_first_packet_time_us = hrt_absolute_time();
	_last_stats_log_time_us = _first_packet_time_us;

	PX4_INFO("[EKF2_TcpSubscriber %p] Initialized. Listening on port %u", this, LISTEN_PORT);

	return true;
}

void *EKF2_TcpSubscriber::receiverThreadEntry(void *arg)
{
	EKF2_TcpSubscriber *subscriber = static_cast<EKF2_TcpSubscriber *>(arg);
	subscriber->receiverLoop();
	return nullptr;
}

void EKF2_TcpSubscriber::receiverLoop()
{
	PX4_INFO("[EKF2_TcpSubscriber %p] Receiver thread started", this);

	while (_receiver_running) {
		// Try to accept client if none connected
		if (_client_fd < 0) {
			acceptClient();

			if (_client_fd < 0) {
				// No client yet, sleep briefly
				usleep(100000); // 100ms
				continue;
			}
		}

		// Receive and process packets
		receivePackets();

		// Transfer from RX buffer to AI queue
		transferToAiQueue();

		// Small sleep to prevent busy loop
		usleep(1000); // 1ms
	}

	PX4_INFO("[EKF2_TcpSubscriber %p] Receiver thread stopped", this);
}

bool EKF2_TcpSubscriber::acceptClient()
{
	if (_server_fd < 0 || _client_fd >= 0) {
		return false;
	}

	socklen_t client_len = sizeof(_client_addr);
	int new_client_fd = accept(_server_fd, (struct sockaddr *)&_client_addr, &client_len);

	if (new_client_fd < 0) {
		if (errno != EAGAIN) {
			PX4_WARN("[EKF2_TcpSubscriber %p] Accept failed: %s", this, strerror(errno));
		}

		return false;
	}

	// Set client socket to non-blocking
	int flags = fcntl(new_client_fd, F_GETFL, 0);

	if (flags >= 0) {
		fcntl(new_client_fd, F_SETFL, flags | O_NONBLOCK);
	}

	// Disable Nagle's algorithm
	int opt = 1;

	if (setsockopt(new_client_fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)) < 0) {
		PX4_WARN("[EKF2_TcpSubscriber %p] Failed to set TCP_NODELAY on client socket: %s", this, strerror(errno));
	}

	_client_fd = new_client_fd;

	inet_ntop(AF_INET, &_client_addr.sin_addr, _client_ip, sizeof(_client_ip));
	_client_port = ntohs(_client_addr.sin_port);
	_client_info_logged = false;

	PX4_INFO("[EKF2_TcpSubscriber %p] Client connected from %s:%u", this, _client_ip, _client_port);

	return true;
}

void EKF2_TcpSubscriber::closeClient()
{
	if (_client_fd >= 0) {
		close(_client_fd);
		_client_fd = -1;
		_stats.client_disconnects++;
		_tcp_buffer_used = 0; // Clear partial data

		PX4_INFO("[EKF2_TcpSubscriber %p] Client disconnected: %s:%u", this, _client_ip, _client_port);
	}
}

void EKF2_TcpSubscriber::receivePackets()
{
	if (_client_fd < 0) {
		return;
	}

	// Receive data into buffer
	while (true) {
		ssize_t received = recv(_client_fd, _tcp_buffer + _tcp_buffer_used,
		                        sizeof(_tcp_buffer) - _tcp_buffer_used, 0);

		if (received < 0) {
			if (errno == EAGAIN) {
				// No more data available
				break;
			}

			// Connection error
			PX4_WARN("[EKF2_TcpSubscriber %p] Recv error: %s. Closing client.", this, strerror(errno));
			closeClient();
			return;
		}

		if (received == 0) {
			// Connection closed by peer
			PX4_INFO("[EKF2_TcpSubscriber %p] Client closed connection", this);
			closeClient();
			return;
		}

		_tcp_buffer_used += received;

		// Process complete packets
		while (_tcp_buffer_used >= sizeof(ImuNetworkPacket)) {
			ImuNetworkPacket pkt;
			memcpy(&pkt, _tcp_buffer, sizeof(pkt));

			// Validate packet
			if (!validatePacket(pkt)) {
				_stats.crc_failures++;
				// Discard one byte and try to resync
				memmove(_tcp_buffer, _tcp_buffer + 1, _tcp_buffer_used - 1);
				_tcp_buffer_used--;
				continue;
			}

			// Remove packet from buffer
			memmove(_tcp_buffer, _tcp_buffer + sizeof(pkt), _tcp_buffer_used - sizeof(pkt));
			_tcp_buffer_used -= sizeof(pkt);

			// Convert to AiProcessedSample
			AiProcessedSample sample;
			sample.timestamp_us = pkt.timestamp_us;
			sample.sequence = pkt.sequence;
			sample.gyro = matrix::Vector3f(pkt.gyro_x, pkt.gyro_y, pkt.gyro_z);
			sample.accel = matrix::Vector3f(pkt.accel_x, pkt.accel_y, pkt.accel_z);
			sample.delta_ang_dt = pkt.delta_ang_dt;
			sample.delta_vel_dt = pkt.delta_vel_dt;
			sample.received_time_us = hrt_absolute_time();

			// Check sequence
			if (_first_packet_received) {
				if (pkt.sequence != _expected_sequence) {
					uint32_t gap = (pkt.sequence - _expected_sequence);
					_stats.sequence_gaps += gap;
				}
			} else {
				_first_packet_received = true;

				// Log first packet info
				if (!_client_info_logged) {
					PX4_INFO("[EKF2_TcpSubscriber %p] First packet from %s:%u - seq=%u ts=%llu",
					         this, _client_ip, _client_port, pkt.sequence, (unsigned long long)pkt.timestamp_us);
					_client_info_logged = true;
				}
			}

			_expected_sequence = pkt.sequence + 1;
			_stats.last_sequence = pkt.sequence;

			// Calculate latency
			float latency_us = (float)(sample.received_time_us - sample.timestamp_us);
			_latency_sum_us += latency_us;

			if (latency_us > _stats.max_latency_us) {
				_stats.max_latency_us = latency_us;
			}

			_stats.total_received++;
			_stats_interval_packet_count++;

			// Log first few samples
			if (_stats.total_received <= 4) {
				PX4_INFO("[EKF2_TcpSubscriber %p] RX Sample %llu: seq=%u ts=%llu gyro=[%.3f,%.3f,%.3f] accel=[%.3f,%.3f,%.3f] latency=%.1fus",
				         this, (unsigned long long)_stats.total_received, pkt.sequence,
				         (unsigned long long)pkt.timestamp_us,
				         (double)pkt.gyro_x, (double)pkt.gyro_y, (double)pkt.gyro_z,
				         (double)pkt.accel_x, (double)pkt.accel_y, (double)pkt.accel_z,
				         (double)latency_us);
			}

			// Push to RX ring buffer
			if (!pushToRxBuffer(sample)) {
				_stats.queue_overflows++;
			}
		}
	}
}

bool EKF2_TcpSubscriber::validatePacket(const ImuNetworkPacket &pkt) const
{
	uint16_t calculated_crc = calculateCrc16(reinterpret_cast<const uint8_t *>(&pkt),
	                          sizeof(pkt) - sizeof(pkt.crc16));
	return calculated_crc == pkt.crc16;
}

uint16_t EKF2_TcpSubscriber::calculateCrc16(const uint8_t *data, size_t len)
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

bool EKF2_TcpSubscriber::pushToRxBuffer(const AiProcessedSample &sample)
{
	uint32_t head = _rx_head.load();
	uint32_t next_head = (head + 1) % RX_RING_BUFFER_SIZE;

	if (next_head == _rx_tail.load()) {
		// Buffer full
		return false;
	}

	_rx_ring_buffer[head] = sample;
	_rx_head.store(next_head);

	return true;
}

bool EKF2_TcpSubscriber::popFromRxBuffer(AiProcessedSample &sample)
{
	uint32_t tail = _rx_tail.load();

	if (tail == _rx_head.load()) {
		// Buffer empty
		return false;
	}

	sample = _rx_ring_buffer[tail];
	_rx_tail.store((tail + 1) % RX_RING_BUFFER_SIZE);

	return true;
}

void EKF2_TcpSubscriber::transferToAiQueue()
{
	AiProcessedSample sample;

	while (popFromRxBuffer(sample)) {
		uint32_t head = _ai_head.load();
		uint32_t next_head = (head + 1) % AI_QUEUE_SIZE;

		if (next_head == _ai_tail.load()) {
			// AI queue full, drop sample
			_stats.queue_overflows++;
			continue;
		}

		_ai_queue[head] = sample;
		_ai_head.store(next_head);
	}
}

bool EKF2_TcpSubscriber::getAiSample(AiProcessedSample &sample)
{
	uint32_t tail = _ai_tail.load();

	if (tail == _ai_head.load()) {
		// Queue empty
		return false;
	}

	sample = _ai_queue[tail];
	_ai_tail.store((tail + 1) % AI_QUEUE_SIZE);

	return true;
}

bool EKF2_TcpSubscriber::hasSamples() const
{
	return _ai_tail.load() != _ai_head.load();
}

uint32_t EKF2_TcpSubscriber::getQueueDepth() const
{
	uint32_t head = _ai_head.load();
	uint32_t tail = _ai_tail.load();

	if (head >= tail) {
		return head - tail;

	} else {
		return AI_QUEUE_SIZE - tail + head;
	}
}

void EKF2_TcpSubscriber::logStatistics()
{
	uint64_t now = hrt_absolute_time();
	float uptime_s = (float)(now - _first_packet_time_us) / 1e6f;

	_stats.avg_receive_rate_hz = 0;

	if (uptime_s > 0) {
		_stats.avg_receive_rate_hz = (float)_stats.total_received / uptime_s;
	}

	_stats.avg_latency_us = 0;

	if (_stats_interval_packet_count > 0) {
		_stats.avg_latency_us = _latency_sum_us / (float)_stats_interval_packet_count;
	}

	uint32_t queue_depth = getQueueDepth();

	PX4_INFO("[EKF2_TcpSubscriber %p] Stats: uptime=%.1fs received=%llu rate=%.1fHz crc_fail=%llu seq_gaps=%llu "
	         "queue_depth=%u overflows=%llu latency_avg=%.1fus latency_max=%.1fus client_connected=%d disconnects=%llu",
	         this, (double)uptime_s,
	         (unsigned long long)_stats.total_received,
	         (double)_stats.avg_receive_rate_hz,
	         (unsigned long long)_stats.crc_failures,
	         (unsigned long long)_stats.sequence_gaps,
	         queue_depth,
	         (unsigned long long)_stats.queue_overflows,
	         (double)_stats.avg_latency_us,
	         (double)_stats.max_latency_us,
	         isClientConnected() ? 1 : 0,
	         (unsigned long long)_stats.client_disconnects);

	_last_stats_log_time_us = now;
	_stats_interval_packet_count = 0;
	_latency_sum_us = 0;
}
