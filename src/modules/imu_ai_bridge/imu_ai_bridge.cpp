/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * Simple UDP bridge that receives AI-denoised IMU delta data and
 * republishes it on the vehicle_imu_ai uORB topic.
 */

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>

#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_imu_ai.h>

#include <arpa/inet.h>
#include <cerrno>
#include <cstdlib>
#include <netinet/in.h>
#include <sys/socket.h>

class ImuAIBridge final : public ModuleBase<ImuAIBridge>
{
public:
ImuAIBridge() = default;
~ImuAIBridge() override;

/** @see ModuleBase */
int init();
void run() override;

static int task_spawn(int argc, char *argv[]);
static ImuAIBridge *instantiate(int argc, char *argv[]);
static int custom_command(int argc, char *argv[]);
static int print_usage(const char *reason = nullptr);

private:
int _port{14560};
int _socket{-1};
orb_advert_t _imu_ai_pub{nullptr};
};

ImuAIBridge::~ImuAIBridge()
{
if (_socket >= 0) {
px4_close(_socket);
_socket = -1;
}
}

int ImuAIBridge::init()
{
_socket = ::socket(AF_INET, SOCK_DGRAM, 0);

if (_socket < 0) {
PX4_ERR("socket open failed (%i)", errno);
return PX4_ERROR;
}

int reuse = 1;
(void)setsockopt(_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

sockaddr_in addr{};
addr.sin_family = AF_INET;
addr.sin_port = htons((uint16_t)_port);
addr.sin_addr.s_addr = htonl(INADDR_ANY);

if (bind(_socket, (sockaddr *)&addr, sizeof(addr)) < 0) {
PX4_ERR("bind failed (%i)", errno);
px4_close(_socket);
_socket = -1;
return PX4_ERROR;
}

PX4_INFO("listening on UDP port %d", _port);

return PX4_OK;
}

void ImuAIBridge::run()
{
if (_socket < 0) {
PX4_ERR("invalid socket");
return;
}

vehicle_imu_ai_s msg{};
sockaddr_in src{};
socklen_t srclen = sizeof(src);

while (!should_exit()) {
px4_pollfd_struct_t fds{};
fds.fd = _socket;
fds.events = POLLIN;

int ret = px4_poll(&fds, 1, 1000);

if (ret < 0) {
PX4_WARN("poll error (%i)", errno);
continue;

} else if (ret == 0) {
// timeout, loop again
continue;
}

srclen = sizeof(src);
ssize_t nbytes = recvfrom(_socket, &msg, sizeof(msg), 0, (sockaddr *)&src, &srclen);

if (nbytes < 0) {
PX4_WARN("recvfrom error (%i)", errno);
continue;
}

if (nbytes != (ssize_t)sizeof(msg)) {
PX4_WARN("unexpected packet size %zd", nbytes);
continue;
}

const hrt_abstime now = hrt_absolute_time();
msg.timestamp = now;

if (msg.timestamp_sample == 0) {
msg.timestamp_sample = now;
}

if (_imu_ai_pub == nullptr) {
_imu_ai_pub = orb_advertise(ORB_ID(vehicle_imu_ai), &msg);

if (_imu_ai_pub == nullptr) {
PX4_ERR("advertise failed");
continue;
}

} else {
orb_publish(ORB_ID(vehicle_imu_ai), _imu_ai_pub, &msg);
}
}
}

int ImuAIBridge::task_spawn(int argc, char *argv[])
{
_task_id = px4_task_spawn_cmd("imu_ai_bridge",
    SCHED_DEFAULT,
    SCHED_PRIORITY_SLOW_DRIVER,
    1700,
    (px4_main_t)&run_trampoline,
    argv);

if (_task_id < 0) {
PX4_ERR("task start failed");
_task_id = -1;
return PX4_ERROR;
}

return PX4_OK;
}

ImuAIBridge *ImuAIBridge::instantiate(int argc, char *argv[])
{
ImuAIBridge *instance = new ImuAIBridge();

if (!instance) {
PX4_ERR("alloc failed");
return nullptr;
}

int ch;
const char *options = "p:";

while ((ch = px4_getopt(argc, argv, options, nullptr, nullptr)) != EOF) {
switch (ch) {
case 'p':
instance->_port = (int)strtol(px4_optarg, nullptr, 10);
break;

default:
print_usage("unrecognized option");
delete instance;
return nullptr;
}
}

if (instance->init() != PX4_OK) {
delete instance;
return nullptr;
}

return instance;
}

int ImuAIBridge::custom_command(int argc, char *argv[])
{
return print_usage("unrecognized command");
}

int ImuAIBridge::print_usage(const char *reason)
{
if (reason) {
PX4_WARN("%s", reason);
}

PRINT_MODULE_USAGE_NAME("imu_ai_bridge", "module");
PRINT_MODULE_USAGE_COMMAND("start");
PRINT_MODULE_USAGE_PARAM_INT('p', 14560, 1, 65535, "UDP listen port", true);
PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

return PX4_OK;
}

extern "C" __EXPORT int imu_ai_bridge_main(int argc, char *argv[])
{
return ImuAIBridge::main(argc, argv);
}
