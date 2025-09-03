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

#include <px4_log.h>
#include <px4_defines.h>
#include <cmath>
#include "HiwonderEMM.h"

HiwonderEMM::HiwonderEMM(int bus, int addr):
	I2C(DRV_MOTOR_DEVTYPE_HIWONDER_EMM, MODULE_NAME, bus, addr, 400000)
{

}

int HiwonderEMM::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) { return ret; }

	const uint8_t cmd[2] = {MOTOR_TYPE_ADDR, MOTOR_TYPE_JGB37_520_12V_110RPM};
	int motor_type = transfer(cmd, 2, nullptr, 0);
	const uint8_t cmd2[2] = {MOTOR_ENCODER_POLARITY_ADDR, 0};
	int encoder_polarity = transfer(cmd2, 2, nullptr, 0);

	if (motor_type != PX4_OK || encoder_polarity != PX4_OK) {
		PX4_ERR("Hiwonder EMM initialization failed");
		return PX4_ERROR;

	} else {
		PX4_INFO("Hiwonder EMM initialized");
	}

	return PX4_OK;
}

int HiwonderEMM::probe()
{
	const int ret = I2C::probe();

	if (ret != PX4_OK) { return ret; }

	const uint8_t cmd = ADC_BAT_ADDR;
	uint8_t buf[2] = {};
	const int ret2 = transfer(&cmd, 1, buf, 2);

	if (ret2 != PX4_OK) {
		PX4_ERR("probe: i2c::transfer returned %d", ret);

	} else {
		PX4_INFO("HiwonderEMM found");
	}

	return ret2;
}

int HiwonderEMM::read_adc()
{
	const uint8_t cmd = ADC_BAT_ADDR;
	uint8_t buf[2] = {};
	const int ret = transfer(&cmd, 1, buf, 2);

	if (ret != PX4_OK) {
		PX4_ERR("read_adc failed");
		return ret;
	}

	return (buf[0] << 8) | buf[1];
}

int HiwonderEMM::set_motor_pwm(const int16_t pwm_values[4])
{
	uint8_t cmd[9];
	cmd[0] = MOTOR_FIXED_PWM_ADDR;
	cmd[1] = (pwm_values[0] >> 8) & 0xFF;
	cmd[2] = pwm_values[0] & 0xFF;
	cmd[3] = (pwm_values[1] >> 8) & 0xFF;
	cmd[4] = pwm_values[1] & 0xFF;
	cmd[5] = (pwm_values[2] >> 8) & 0xFF;
	cmd[6] = pwm_values[2] & 0xFF;
	cmd[7] = (pwm_values[3] >> 8) & 0xFF;
	cmd[8] = pwm_values[3] & 0xFF;
	const int ret = transfer(cmd, sizeof(cmd), nullptr, 0);

	if (ret != PX4_OK) {
		PX4_ERR("set_motor_pwm failed");

	} else {
		PX4_INFO("PWM set to %d %d %d %d", pwm_values[0], pwm_values[1], pwm_values[2], pwm_values[3]);
	}

	return ret;
}

int HiwonderEMM::set_motor_speed(const uint8_t speed_values[4])
{
	uint8_t cmd[5];
	cmd[0] = MOTOR_FIXED_SPEED_ADDR;
	cmd[1] = speed_values[0];
	cmd[2] = speed_values[1];
	cmd[3] = speed_values[2];
	cmd[4] = speed_values[3];
	const int ret = transfer(cmd, sizeof(cmd), nullptr, 0);

	if (ret != PX4_OK) {
		PX4_ERR("set_motor_speed failed");
	}

	// } else {
	// 	PX4_INFO("Speed set to %d %d %d %d", cmd[1], cmd[2], cmd[3], cmd[4]);
	// }

	return ret;
}
