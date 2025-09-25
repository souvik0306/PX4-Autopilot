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
 * @file ekf2_main.cpp
 *
 * Simplified EKF2 stub that demonstrates IMU source selection between
 * vehicle_imu and vehicle_imu_ai via the EKF2_IMU_SRC parameter.
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>

#include <drivers/drv_hrt.h>
#include <inttypes.h>
#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_imu_ai.h>

class Ekf2 final : public ModuleBase<Ekf2>, public ModuleParams
{
public:
Ekf2();
~Ekf2() override = default;

void run() override;

static Ekf2 *instantiate(int argc, char *argv[]);
static int task_spawn(int argc, char *argv[]);
static int custom_command(int argc, char *argv[]);
static int print_usage(const char *reason = nullptr);

private:
void parameter_update();
void handle_imu_sample(hrt_abstime timestamp_sample,
      const float delta_angle[3],
      const float delta_velocity[3],
      float delta_angle_dt,
      float delta_velocity_dt);

uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
uORB::Subscription _imu_raw_sub{ORB_ID(vehicle_imu)};
uORB::Subscription _imu_ai_sub{ORB_ID(vehicle_imu_ai)};

param_t _param_ekf2_imu_src_handle{PARAM_INVALID};
int32_t _param_ekf2_imu_src{0};

hrt_abstime _last_sample_time{0};
uint64_t _sample_counter{0};
};

Ekf2::Ekf2() : ModuleParams(nullptr)
{
_param_ekf2_imu_src_handle = param_find("EKF2_IMU_SRC");

if (_param_ekf2_imu_src_handle == PARAM_INVALID) {
PX4_ERR("EKF2_IMU_SRC parameter not found");
}
}

void Ekf2::parameter_update()
{
if (_param_ekf2_imu_src_handle != PARAM_INVALID) {
param_get(_param_ekf2_imu_src_handle, &_param_ekf2_imu_src);

if (_param_ekf2_imu_src < 0) {
_param_ekf2_imu_src = 0;
} else if (_param_ekf2_imu_src > 1) {
_param_ekf2_imu_src = 1;
}
}
}

void Ekf2::handle_imu_sample(hrt_abstime timestamp_sample,
     const float delta_angle[3],
     const float delta_velocity[3],
     float delta_angle_dt,
     float delta_velocity_dt)
{
_last_sample_time = timestamp_sample;
_sample_counter++;

if ((_sample_counter % 200) == 0) {
PX4_INFO("EKF2 imu src=%" PRId32 " samples=%" PRIu64 " dt=%.3f/%.3f ms",
 _param_ekf2_imu_src,
 _sample_counter,
 static_cast<double>(delta_angle_dt * 1e3f),
 static_cast<double>(delta_velocity_dt * 1e3f));
}

(void)delta_angle;
(void)delta_velocity;
}

void Ekf2::run()
{
if (_param_ekf2_imu_src_handle == PARAM_INVALID) {
PX4_ERR("EKF2_IMU_SRC handle invalid");
return;
}

parameter_update();
PX4_INFO("EKF2 stub running with imu source %" PRId32, _param_ekf2_imu_src);

while (!should_exit()) {
if (_parameter_update_sub.updated()) {
parameter_update_s p{};

if (_parameter_update_sub.copy(&p)) {
parameter_update();
}
}

if (_param_ekf2_imu_src == 0) {
vehicle_imu_s imu{};

while (_imu_raw_sub.update(&imu)) {
const float da_dt = static_cast<float>(imu.delta_angle_dt) * 1e-6f;
const float dv_dt = static_cast<float>(imu.delta_velocity_dt) * 1e-6f;
handle_imu_sample(imu.timestamp_sample, imu.delta_angle, imu.delta_velocity, da_dt, dv_dt);
}

} else {
vehicle_imu_ai_s imu{};

while (_imu_ai_sub.update(&imu)) {
handle_imu_sample(imu.timestamp_sample, imu.delta_angle, imu.delta_velocity,
 imu.delta_angle_dt, imu.delta_velocity_dt);
}
}

px4_usleep(2000);
}
}

Ekf2 *Ekf2::instantiate(int argc, char *argv[])
{
if (argc > 1) {
return nullptr;
}

Ekf2 *instance = new Ekf2();

if (!instance) {
PX4_ERR("alloc failed");
return nullptr;
}

if (instance->_param_ekf2_imu_src_handle == PARAM_INVALID) {
delete instance;
return nullptr;
}

return instance;
}

int Ekf2::task_spawn(int argc, char *argv[])
{
_task_id = px4_task_spawn_cmd("ekf2",
   SCHED_DEFAULT,
   SCHED_PRIORITY_ESTIMATOR,
   2800,
   (px4_main_t)&run_trampoline,
   argv);

if (_task_id < 0) {
PX4_ERR("task start failed");
_task_id = -1;
return PX4_ERROR;
}

return PX4_OK;
}

int Ekf2::custom_command(int argc, char *argv[])
{
return print_usage("unrecognized command");
}

int Ekf2::print_usage(const char *reason)
{
if (reason) {
PX4_WARN("%s", reason);
}

PRINT_MODULE_USAGE_NAME("ekf2", "module");
PRINT_MODULE_USAGE_SIMPLE("start");
PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

return PX4_OK;
}

extern "C" __EXPORT int ekf2_main(int argc, char *argv[])
{
return Ekf2::main(argc, argv);
}
