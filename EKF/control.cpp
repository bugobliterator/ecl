/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file control.cpp
 * Control functions for ekf attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

void Ekf::controlFusionModes()
{
	// Determine the vehicle status
	calculateVehicleStatus();

	// optical flow fusion mode selection logic
	if(_params.fusion_mode == PV_AID_OF) {
		// decide when to start using optical flow data
		if (!_control_status.flags.opt_flow) {
			if (_control_status.flags.angle_align && (_time_last_imu - _time_last_optflow) < 5e5) {
				_control_status.flags.opt_flow = true;
				_control_status.flags.gps = false;
				//use flow if present and 
				resetPosition();
				resetVelocity();
				_healthy_optical_flow = false;
			}
		}
	}

	// GPS fusion mode selection logic
	// To start use GPS we need angular alignment completed, the local NED origin set and fresh GPS data
	if (!_control_status.flags.gps && !_control_status.flags.opt_flow) {
		if (_control_status.flags.angle_align && (_time_last_imu - _time_last_gps) < 5e5 && _NED_origin_initialised
		    && (_time_last_imu - _last_gps_fail_us > 5e6)) {
			_control_status.flags.gps = true;
			resetPosition();
			resetVelocity();
		}
	}

	// handle the case when we are relying on GPS fusion and lose it
	if (_control_status.flags.gps) {
		// We are relying on GPS aiding to constrain attitude drift so after 10 seconds without aiding we need to do something
		if ((_time_last_imu - _time_last_pos_fuse > 10e6) && (_time_last_imu - _time_last_vel_fuse > 10e6)) {
			if (_time_last_imu - _time_last_gps > 5e5) {
				// if we don't have gps then we need to switch to the non-aiding mode, zero the veloity states
				// and set the synthetic GPS position to the current estimate
				_control_status.flags.gps = false;
				_last_known_posNE(0) = _state.pos(0);
				_last_known_posNE(1) = _state.pos(1);
				_state.vel.setZero();
			} else {
				// Reset states to the last GPS measurement
				resetPosition();
				resetVelocity();
			}
		}
	}

	// handle the case when we are relying on optical flow fusion and lose it
	if (_control_status.flags.opt_flow && !_control_status.flags.gps) {
		// We are relying on flow aiding to constrain attitude drift so after 400ms without aiding we need to do something
		if ((_time_last_imu - _time_last_of_fuse > 4e5)) {
			if (_time_last_imu - _time_last_optflow > 2e5) {
				// if we don't have gps then we need to switch to the non-aiding mode, zero the veloity states
				// and set the synthetic position to the current estimate
				_control_status.flags.opt_flow = false;
				_last_known_posNE(0) = _state.pos(0);
				_last_known_posNE(1) = _state.pos(1);
				_state.vel.setZero();
			} else {
				// Reset states to the last measurement
				resetPosition();
				resetVelocity();
			}
			_healthy_optical_flow = false;
		}
	}

	// Determine if we should use simple magnetic heading fusion which works better when there are large external disturbances
	// or the more accurate 3-axis fusion
	if (!_control_status.flags.armed) {
		// always use simple mag fusion for initial startup
		_control_status.flags.mag_hdg = true;
		_control_status.flags.mag_3D = false;

	} else {
		if (_control_status.flags.in_air) {
			// always use 3-axis mag fusion when airborne
			_control_status.flags.mag_hdg = false;
			_control_status.flags.mag_3D = true;

		} else {
			// always use simple heading fusion when on the ground
			_control_status.flags.mag_hdg = true;
			_control_status.flags.mag_3D = false;
		}
	}
}

void Ekf::calculateVehicleStatus()
{
	// determine if the vehicle is armed
	_control_status.flags.armed = _vehicle_armed;

	// record vertical position whilst disarmed to use as a height change reference
	if (!_control_status.flags.armed) {
		_last_disarmed_posD = _state.pos(2);
	}

	// Transition to in-air occurs when armed and when altitude has increased sufficiently from the altitude at arming
	if (!_control_status.flags.in_air && _control_status.flags.armed && (_state.pos(2) - _last_disarmed_posD) < -1.0f) {
		_control_status.flags.in_air = true;
	}

	// Transition to on-ground occurs when disarmed.
	if (_control_status.flags.in_air && !_control_status.flags.armed) {
		_control_status.flags.in_air = false;
	}
}
