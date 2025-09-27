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
 *
 * Simple bridge that receives AI-processed IMU samples over UDP and
 * republishes them on the vehicle_imu_ai uORB topic.
 */

#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>

#include <drivers/drv_hrt.h>

#include <uORB/topics/vehicle_imu_ai.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstring>

using namespace time_literals;

class ImuAIBridge final : public ModuleBase<ImuAIBridge>
{
public:
        int task_spawn(int argc, char *argv[]) override;
        int custom_command(int argc, char *argv[]) override { return print_usage("unknown command"); }
        int print_usage(const char *reason = nullptr) override;

        int run();

private:
        static constexpr uint16_t kListenPort{14560};

        struct PX4_PACKED VehicleImuAiWire
        {
                uint64_t timestamp;
                uint64_t timestamp_sample;
                uint32_t accel_device_id;
                uint32_t gyro_device_id;
                float delta_angle[3];
                float delta_velocity[3];
                uint16_t delta_angle_dt;
                uint16_t delta_velocity_dt;
                uint8_t delta_velocity_clipping;
                uint8_t accel_calibration_count;
                uint8_t gyro_calibration_count;
        };
};

int ImuAIBridge::task_spawn(int argc, char *argv[])
{
        ImuAIBridge *instance = new ImuAIBridge();

        if (!instance) {
                PX4_ERR("alloc failed");
                return PX4_ERROR;
        }

        _object.store(instance);
        _task_id = task_id_is_work_queue;

        return instance->run();
}

int ImuAIBridge::run()
{
        vehicle_imu_ai_s msg{};
        std::array<uint8_t, sizeof(VehicleImuAiWire)> rx_buffer{};

        const int sock = ::socket(AF_INET, SOCK_DGRAM, 0);

        if (sock < 0) {
                PX4_ERR("socket");
                return PX4_ERROR;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(kListenPort);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (::bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
                PX4_ERR("bind");
                ::close(sock);
                return PX4_ERROR;
        }

        PX4_INFO("imu_ai_bridge listening on UDP port %u", static_cast<unsigned>(kListenPort));

        orb_advert_t pub = nullptr;

        hrt_abstime last_size_warn_us = 0;

        while (!should_exit()) {
                const ssize_t bytes = ::recvfrom(sock, rx_buffer.data(), rx_buffer.size(), MSG_TRUNC, nullptr, nullptr);

                if (bytes < 0) {
                        // socket error, sleep briefly to avoid tight loop
                        px4_usleep(5_ms);
                        continue;
                }

                if (bytes != static_cast<ssize_t>(rx_buffer.size())) {
                        if (hrt_elapsed_time(&last_size_warn_us) > 1_s) {
                                PX4_WARN("discarded imu_ai packet with size %zd (expected %zu)", bytes,
                                         rx_buffer.size());
                                last_size_warn_us = hrt_absolute_time();
                        }

                        continue;
                }

                VehicleImuAiWire wire{};
                std::memcpy(&wire, rx_buffer.data(), sizeof(wire));

                msg = {};
                msg.timestamp = hrt_absolute_time();
                msg.timestamp_sample = wire.timestamp_sample;
                msg.accel_device_id = wire.accel_device_id;
                msg.gyro_device_id = wire.gyro_device_id;
                std::memcpy(msg.delta_angle, wire.delta_angle, sizeof(msg.delta_angle));
                std::memcpy(msg.delta_velocity, wire.delta_velocity, sizeof(msg.delta_velocity));
                msg.delta_angle_dt = wire.delta_angle_dt;
                msg.delta_velocity_dt = wire.delta_velocity_dt;
                msg.delta_velocity_clipping = wire.delta_velocity_clipping;
                msg.accel_calibration_count = wire.accel_calibration_count;
                msg.gyro_calibration_count = wire.gyro_calibration_count;

                if (!pub) {
                        pub = orb_advertise(ORB_ID(vehicle_imu_ai), &msg);

                } else {
                        orb_publish(ORB_ID(vehicle_imu_ai), pub, &msg);
                }
        }

        if (pub) {
                orb_unadvertise(pub);
        }

        ::close(sock);
        return PX4_OK;
}

int ImuAIBridge::print_usage(const char *reason)
{
        if (reason) {
                PX4_WARN("%s", reason);
        }

        PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
### Description

Listens for AI-processed IMU samples on UDP port 14560 and republishes
then on the vehicle_imu_ai topic.

)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("imu_ai_bridge", "system");
        PRINT_MODULE_USAGE_COMMAND("start");

        return PX4_OK;
}

extern "C" __EXPORT int imu_ai_bridge_main(int argc, char *argv[])
{
        return ImuAIBridge::main(argc, argv);
}
