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
 * @file EKF2_ImuPacket.hpp
 * Shared IMU packet structure for network communication
 *
 * This packet format is used by both UDP and TCP publishers/subscribers
 * for transmitting IMU data to/from AI preprocessing systems.
 */

#ifndef EKF2_IMU_PACKET_HPP
#define EKF2_IMU_PACKET_HPP

#include <stdint.h>

// Compact binary protocol for efficient transmission
#pragma pack(push, 1)
struct ImuNetworkPacket {
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

#endif // EKF2_IMU_PACKET_HPP
