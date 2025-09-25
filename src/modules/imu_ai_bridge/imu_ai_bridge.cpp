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
 * @file imu_ai_bridge.cpp
 * @author Your Name
 *
 * IMU AI Bridge - Receives AI-processed IMU data and publishes as vehicle_imu_ai
 */

#include "imu_ai_bridge.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/vehicle_imu_ai.h>
#include <mathlib/mathlib.h>

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

namespace
{
	// Legacy packet layout (dt fields in microseconds, no delta_angle_clipping byte)
	struct __attribute__((packed)) AIBridgePacketV0 {
		uint64_t timestamp;
		uint64_t timestamp_sample;
		uint32_t accel_device_id;
		uint32_t gyro_device_id;
		float delta_angle[3];
		float delta_velocity[3];
		uint16_t delta_angle_dt;    // microseconds
		uint16_t delta_velocity_dt; // microseconds
		uint8_t delta_velocity_clipping;
		uint8_t accel_calibration_count;
		uint8_t gyro_calibration_count;
	};

	static_assert(sizeof(AIBridgePacketV0) == 55, "Unexpected legacy AI IMU packet size");

	// Current python helper layout (dt fields in seconds, explicit delta_angle_clipping byte)
	struct __attribute__((packed)) AIBridgePacketV1 {
		uint64_t timestamp;
		uint64_t timestamp_sample;
		uint32_t accel_device_id;
		uint32_t gyro_device_id;
		float delta_angle[3];
		float delta_velocity[3];
		float delta_angle_dt;      // seconds
		float delta_velocity_dt;   // seconds
		uint8_t delta_angle_clipping;
		uint8_t delta_velocity_clipping;
		uint8_t accel_calibration_count;
		uint8_t gyro_calibration_count;
	};

	static_assert(sizeof(AIBridgePacketV1) == 60, "Unexpected AI IMU helper packet size");
}

ImuAIBridge::ImuAIBridge() :
	ModuleParams(nullptr),
	ScheduledWorkItem("imu_ai_bridge", px4::wq_configurations::hp_default)
{
}

ImuAIBridge::~ImuAIBridge()
{
	if (_socket_fd >= 0) {
		close(_socket_fd);
	}
	}

bool ImuAIBridge::init()
{
	// Create UDP socket
	_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_socket_fd < 0) {
		PX4_ERR("socket creation failed: %s", strerror(errno));
		return false;
	}

	// Set socket to non-blocking
	int flags = fcntl(_socket_fd, F_GETFL, 0);
	fcntl(_socket_fd, F_SETFL, flags | O_NONBLOCK);

	// Bind socket
	struct sockaddr_in addr {};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(_param_imu_ai_port.get());

	if (bind(_socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("bind failed on port %d: %s", _param_imu_ai_port.get(), strerror(errno));
		close(_socket_fd);
		_socket_fd = -1;
		return false;
	}

	PX4_INFO("IMU AI Bridge listening on UDP port %d", _param_imu_ai_port.get());

	// Start the work queue (1 kHz polling cadence)
	ScheduleOnInterval(1_ms);

	return true;
	}

void ImuAIBridge::Run()
{
	if (should_exit()) {
		ScheduleClear();
		return;
	}

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// Read AI IMU data from UDP socket
	receive_ai_imu_data();
	}

void ImuAIBridge::receive_ai_imu_data()
{
	if (_socket_fd < 0) {
		return;
	}

	struct sockaddr_in sender_addr {};
	uint8_t recv_buffer[sizeof(vehicle_imu_ai_s)]{};

	while (true) {
		socklen_t sender_len = sizeof(sender_addr);
		ssize_t bytes_received = recvfrom(_socket_fd, recv_buffer, sizeof(recv_buffer), 0,
					      (struct sockaddr *)&sender_addr, &sender_len);

		if (bytes_received < 0) {
#if EWOULDBLOCK != EAGAIN
			if ((errno != EWOULDBLOCK) && (errno != EAGAIN)) {
#else
			if (errno != EWOULDBLOCK) {
#endif
				PX4_WARN("recvfrom failed: %s", strerror(errno));
			}

			break;
		}

		vehicle_imu_ai_s ai_imu_msg{};
		bool parsed = false;

		if (bytes_received == (ssize_t)sizeof(vehicle_imu_ai_s)) {
			memcpy(&ai_imu_msg, recv_buffer, sizeof(vehicle_imu_ai_s));
			parsed = true;

		} else if (bytes_received == (ssize_t)sizeof(AIBridgePacketV1)) {
			AIBridgePacketV1 packet{};
			memcpy(&packet, recv_buffer, sizeof(packet));

			ai_imu_msg.timestamp = packet.timestamp;
			ai_imu_msg.timestamp_sample = packet.timestamp_sample;
			ai_imu_msg.accel_device_id = packet.accel_device_id;
			ai_imu_msg.gyro_device_id = packet.gyro_device_id;
			memcpy(ai_imu_msg.delta_angle, packet.delta_angle, sizeof(packet.delta_angle));
			memcpy(ai_imu_msg.delta_velocity, packet.delta_velocity, sizeof(packet.delta_velocity));
			ai_imu_msg.delta_angle_dt = packet.delta_angle_dt;
			ai_imu_msg.delta_velocity_dt = packet.delta_velocity_dt;
			ai_imu_msg.delta_velocity_clipping = packet.delta_velocity_clipping;
			ai_imu_msg.accel_calibration_count = packet.accel_calibration_count;
			ai_imu_msg.gyro_calibration_count = packet.gyro_calibration_count;
			parsed = true;

		} else if (bytes_received == (ssize_t)sizeof(AIBridgePacketV0)) {
			AIBridgePacketV0 packet{};
			memcpy(&packet, recv_buffer, sizeof(packet));

			ai_imu_msg.timestamp = packet.timestamp;
			ai_imu_msg.timestamp_sample = packet.timestamp_sample;
			ai_imu_msg.accel_device_id = packet.accel_device_id;
			ai_imu_msg.gyro_device_id = packet.gyro_device_id;
			memcpy(ai_imu_msg.delta_angle, packet.delta_angle, sizeof(packet.delta_angle));
			memcpy(ai_imu_msg.delta_velocity, packet.delta_velocity, sizeof(packet.delta_velocity));
			ai_imu_msg.delta_angle_dt = 1e-6f * packet.delta_angle_dt;
			ai_imu_msg.delta_velocity_dt = 1e-6f * packet.delta_velocity_dt;
			ai_imu_msg.delta_velocity_clipping = packet.delta_velocity_clipping;
			ai_imu_msg.accel_calibration_count = packet.accel_calibration_count;
			ai_imu_msg.gyro_calibration_count = packet.gyro_calibration_count;
			parsed = true;
		}

		if (!parsed) {
			PX4_WARN("Received %zd bytes, expected %zu, %zu or %zu bytes",
				  bytes_received,
				  sizeof(vehicle_imu_ai_s),
				  sizeof(AIBridgePacketV1),
				  sizeof(AIBridgePacketV0));
			continue;
		}

		const hrt_abstime now = hrt_absolute_time();
		ai_imu_msg.timestamp = now;

		if ((ai_imu_msg.timestamp_sample == 0) || (ai_imu_msg.timestamp_sample > now)) {
			ai_imu_msg.timestamp_sample = now;
		}

		bool data_valid = true;

		// Basic sanity checks
		for (int i = 0; i < 3; i++) {
			data_valid &= PX4_ISFINITE(ai_imu_msg.delta_angle[i]);
			data_valid &= PX4_ISFINITE(ai_imu_msg.delta_velocity[i]);

			if (!data_valid) {
				break;
			}

			if ((fabsf(ai_imu_msg.delta_angle[i]) > 2.f * M_PI_F)
			    || (fabsf(ai_imu_msg.delta_velocity[i]) > 100.f)) {
				data_valid = false;
				break;
			}
		}

		if (data_valid) {
			data_valid &= PX4_ISFINITE(ai_imu_msg.delta_angle_dt)
					&& PX4_ISFINITE(ai_imu_msg.delta_velocity_dt)
					&& (ai_imu_msg.delta_angle_dt > 5e-4f) && (ai_imu_msg.delta_angle_dt < 5e-2f)
					&& (ai_imu_msg.delta_velocity_dt > 5e-4f) && (ai_imu_msg.delta_velocity_dt < 5e-2f);
		}

		if (!data_valid) {
			PX4_WARN("Invalid AI IMU data received, ignoring");
			continue;
		}

		_vehicle_imu_ai_pub.publish(ai_imu_msg);
		_msg_count++;

		if ((_msg_count % 200) == 0) {
			PX4_DEBUG("AI IMU: count=%u, dt=%.3f ms, dv=[%.3f,%.3f,%.3f], da=[%.3f,%.3f,%.3f]",
				  _msg_count,
				  (double)(ai_imu_msg.delta_velocity_dt * 1e3f),
				  (double)ai_imu_msg.delta_velocity[0], (double)ai_imu_msg.delta_velocity[1], (double)ai_imu_msg.delta_velocity[2],
				  (double)ai_imu_msg.delta_angle[0], (double)ai_imu_msg.delta_angle[1], (double)ai_imu_msg.delta_angle[2]);
		}
	}
}

int ImuAIBridge::task_spawn(int argc, char *argv[])
{
	ImuAIBridge *instance = new ImuAIBridge();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
	}

int ImuAIBridge::print_status()
{
	PX4_INFO("IMU AI Bridge running");
	PX4_INFO("UDP port: %d", _param_imu_ai_port.get());
	PX4_INFO("Messages received: %u", _msg_count);

	return 0;
	}

int ImuAIBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
	}

int ImuAIBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
IMU AI Bridge receives AI-processed IMU data over UDP and publishes it as vehicle_imu_ai topic.

The AI model should send binary packets matching the vehicle_imu_ai message structure.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("imu_ai_bridge", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
	}

extern "C" __EXPORT int imu_ai_bridge_main(int argc, char *argv[])
{
	return ImuAIBridge::main(argc, argv);
	}
