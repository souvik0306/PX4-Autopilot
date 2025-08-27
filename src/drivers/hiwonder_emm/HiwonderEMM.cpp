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

	// uint8_t buf[2] = {};

	// buf[0] = HiwonderEMM_REG_MODE1;
	// buf[1] = HiwonderEMM_DEFAULT_MODE1_CFG | HiwonderEMM_MODE1_SLEEP_MASK;  // put into sleep mode
	// ret = transfer(buf, 2, nullptr, 0);

	// if (OK != ret) {
	// 	PX4_ERR("init: i2c::transfer returned %d", ret);
	// 	return ret;
	// }

	return PX4_OK;
}

int HiwonderEMM::probe()
{
	int ret = I2C::probe();

	if (ret != PX4_OK) { return ret; }

	// uint8_t buf[2] = {0x00};
	// return transfer(buf, 2, buf, 1);
	return PX4_OK;
}
