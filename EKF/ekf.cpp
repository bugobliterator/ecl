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
 * @file ekf.cpp
 * Core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include "ekf.h"
#include <drivers/drv_hrt.h>

Ekf::Ekf():
	_control_status{},
	_filter_initialised(false),
	_earth_rate_initialised(false),
	_fuse_height(false),
	_fuse_pos(false),
	_fuse_hor_vel(false),
	_fuse_vert_vel(false),
	_time_last_fake_gps(0),
	_time_last_pos_fuse(0),
	_time_last_vel_fuse(0),
	_time_last_hgt_fuse(0),
	_time_last_of_fuse(0),
	_vel_pos_innov{},
	_mag_innov{},
	_heading_innov{},
	_vel_pos_innov_var{},
	_mag_innov_var{},
	_heading_innov_var{}
{
	_earth_rate_NED.setZero();
	_R_prev = matrix::Dcm<float>();
	_delta_angle_corr.setZero();
	_delta_vel_corr.setZero();
	_vel_corr.setZero();
	_last_known_posNE.setZero();
}


Ekf::~Ekf()
{


}
bool Ekf::init(uint64_t timestamp)
{
	bool ret = initialise_interface(timestamp);
	_state.ang_error.setZero();
	_state.vel.setZero();
	_state.pos.setZero();
	_state.gyro_bias.setZero();
	_state.gyro_scale(0) = 1.0f;
	_state.gyro_scale(1) = 1.0f;
	_state.gyro_scale(2) = 1.0f;
	_state.accel_z_bias = 0.0f;
	_state.mag_I.setZero();
	_state.mag_B.setZero();
	_state.wind_vel.setZero();
	_state.quat_nominal.setZero();
	_state.quat_nominal(0) = 1.0f;

	_output_new.vel.setZero();
	_output_new.pos.setZero();
	_output_new.quat_nominal = matrix::Quaternion<float>();


	_imu_down_sampled.delta_ang.setZero();
	_imu_down_sampled.delta_vel.setZero();
	_imu_down_sampled.delta_ang_dt = 0.0f;
	_imu_down_sampled.delta_vel_dt = 0.0f;
	_imu_down_sampled.time_us = timestamp;

	_q_down_sampled(0) = 1.0f;
	_q_down_sampled(1) = 0.0f;
	_q_down_sampled(2) = 0.0f;
	_q_down_sampled(3) = 0.0f;

	_imu_updated = false;
	_NED_origin_initialised = false;
	_gps_speed_valid = false;
	_mag_healthy = false;
	return ret;
}

bool Ekf::update()
{
	bool ret = false;	// indicates if there has been an update
	if (!_filter_initialised) {
		_filter_initialised = initialiseFilter();

		if (!_filter_initialised) {
			return false;
		}
	}

	printStatesFast();
	//printStoredBaro();
	// prediction
	if (_imu_updated) {
		ret = true;
		predictState();
		predictCovariance();
	}

	// control logic
	controlFusionModes();

	// measurement updates

	// Fuse magnetometer data using the selected fuson method and only if angular alignment is complete
	if (_mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed)) {
		if (_control_status.flags.mag_3D && _control_status.flags.angle_align) {
			fuseMag();
			if (_control_status.flags.mag_dec) {
				// TODO need to fuse synthetic declination measurements if there is no GPS or equivalent aiding
				// otherwise heading will slowly drift
			}
		} else if (_control_status.flags.mag_hdg && _control_status.flags.angle_align) {
			fuseHeading();
		}
	}

	if (_range_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_range_sample_delayed)) {
		_fuse_range_data = true;
		_fuse_height = true;
	} else if (_baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed) && _params.vdist_sensor_type == VDIST_SENSOR_BARO) {
		_fuse_height = true;
	}

	// If we are using GPS aiding and data has fallen behind the fusion time horizon then fuse it
	// if we aren't doing any aiding, fake GPS measurements at the last known position to constrain drift
	// Coincide fake measurements with baro data for efficiency with a minimum fusion rate of 5Hz
	//if (_gps_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_gps_sample_delayed) && _control_status.flags.gps) {
	//	_fuse_pos = true;
	//	_fuse_vert_vel = true;
	//	_fuse_hor_vel = true;
	//	_fuse_flow = false;
	if(_flow_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_flow_sample_delayed) && _control_status.flags.opt_flow) {
		_fuse_pos = false;
		_fuse_vert_vel = false;
		_fuse_hor_vel = false;
		
		_fuse_flow = true;
	}// else if (!_control_status.flags.gps && !_control_status.flags.opt_flow
	//	   && ((_time_last_imu - _time_last_fake_gps > 2e5) || _fuse_height)) {
	//	_fuse_pos = true;
	//	_gps_sample_delayed.pos(0) = _last_known_posNE(0);
	//	_gps_sample_delayed.pos(1) = _last_known_posNE(1);
	//	_time_last_fake_gps = _time_last_imu;
	//}
	
	if (_fuse_height || _fuse_pos || _fuse_hor_vel || _fuse_vert_vel) {
		fuseVelPosHeight();
		_fuse_hor_vel = _fuse_vert_vel = _fuse_pos = _fuse_height = false;
	}

	if (_fuse_flow) {
		fuseOptFlow();
		_fuse_flow = false;
	}

	if (_airspeed_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_airspeed_sample_delayed)) {
		fuseAirspeed();
	}

	calculateOutputStates();

	return ret;
}

bool Ekf::initialiseFilter(void)
{
	_state.ang_error.setZero();
	_state.vel.setZero();
	_state.pos.setZero();
	_state.gyro_bias.setZero();
	_state.gyro_scale(0) = _state.gyro_scale(1) = _state.gyro_scale(2) = 1.0f;
	_state.accel_z_bias = 0.0f;
	_state.mag_I.setZero();
	_state.mag_B.setZero();
	_state.wind_vel.setZero();

	// get initial roll and pitch estimate from accel vector, assuming vehicle is static
	Vector3f accel_init = _imu_down_sampled.delta_vel / _imu_down_sampled.delta_vel_dt;

	float pitch = 0.0f;
	float roll = 0.0f;

	if (accel_init.norm() > 0.001f) {
		accel_init.normalize();

		pitch = asinf(accel_init(0));
		roll = -asinf(accel_init(1) / cosf(pitch));
	}

	matrix::Euler<float> euler_init(roll, pitch, 0.0f);

	// Get the latest magnetic field measurement.
	// If we don't have a measurement then we cannot initialise the filter
	magSample mag_init = _mag_buffer.get_newest();

	if (mag_init.time_us == 0) {
		return false;
	}

	// rotate magnetic field into earth frame assuming zero yaw and estimate yaw angle assuming zero declination
	// TODO use declination if available
	matrix::Dcm<float> R_to_earth_zeroyaw(euler_init);
	Vector3f mag_ef_zeroyaw = R_to_earth_zeroyaw * mag_init.mag;
	float declination = 0.0f;
	euler_init(2) = declination - atan2f(mag_ef_zeroyaw(1), mag_ef_zeroyaw(0));

	// calculate initial quaternion states
	_state.quat_nominal = Quaternion(euler_init);
	_output_new.quat_nominal = _state.quat_nominal;

	// TODO replace this with a conditional test based on fitered angle error states.
	_control_status.flags.angle_align = true;

	// calculate initial earth magnetic field states
	matrix::Dcm<float> R_to_earth(euler_init);
	_state.mag_I = R_to_earth * mag_init.mag;

	resetVelocity();
	resetPosition();

	if (_params.vdist_sensor_type == VDIST_SENSOR_RANGE) {
		rangeSample range_init = _range_buffer.get_newest();
		_state.pos(2) = range_init.rng;
		_output_new.pos(2) = -range_init.rng;
		if (range_init.time_us == 0) {
			return false;
		}
	} else {
		// initialize vertical position with newest baro measurement
		baroSample baro_init = _baro_buffer.get_newest();
		_state.pos(2) = -baro_init.hgt;
		_output_new.pos(2) = -baro_init.hgt;
		if (baro_init.time_us == 0) {
			return false;
		}
	}
	initialiseCovariance();

	return true;
}

void Ekf::predictState()
{
	if (!_earth_rate_initialised) {
		if (_NED_origin_initialised) {
			calcEarthRateNED(_earth_rate_NED, _pos_ref.lat_rad);
			_earth_rate_initialised = true;
		}
	}

	// attitude error state prediciton
	matrix::Dcm<float> R_to_earth(_state.quat_nominal);	// transformation matrix from body to world frame
	Vector3f corrected_delta_ang = _imu_sample_delayed.delta_ang - _R_prev * _earth_rate_NED *
				       _imu_sample_delayed.delta_ang_dt;
	Quaternion dq;	// delta quaternion since last update
	dq.from_axis_angle(corrected_delta_ang);
	_state.quat_nominal = dq * _state.quat_nominal;
	_state.quat_nominal.normalize();

	_R_prev = R_to_earth.transpose();

	Vector3f vel_last = _state.vel;

	// predict velocity states
	_state.vel += R_to_earth * _imu_sample_delayed.delta_vel;
	_state.vel(2) += 9.81f * _imu_sample_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * _imu_sample_delayed.delta_vel_dt * 0.5f;

	constrainStates();
}


bool Ekf::collect_imu(imuSample &imu)
{

	imu.delta_ang(0) = imu.delta_ang(0) * _state.gyro_scale(0);
	imu.delta_ang(1) = imu.delta_ang(1) * _state.gyro_scale(1);
	imu.delta_ang(2) = imu.delta_ang(2) * _state.gyro_scale(2);

	imu.delta_ang -= _state.gyro_bias * imu.delta_ang_dt / (_dt_imu_avg > 0 ? _dt_imu_avg : 0.01f);
	imu.delta_vel(2) -= _state.accel_z_bias * imu.delta_vel_dt / (_dt_imu_avg > 0 ? _dt_imu_avg : 0.01f);;

	// store the new sample for the complementary filter prediciton
	_imu_sample_new = {
		.delta_ang	= imu.delta_ang,
		.delta_vel	= imu.delta_vel,
		.delta_ang_dt	= imu.delta_ang_dt,
		.delta_vel_dt	= imu.delta_vel_dt,
		.time_us	= imu.time_us
	};

	_imu_down_sampled.delta_ang_dt += imu.delta_ang_dt;
	_imu_down_sampled.delta_vel_dt += imu.delta_vel_dt;


	Quaternion delta_q;
	delta_q.rotate(imu.delta_ang);
	_q_down_sampled =  _q_down_sampled * delta_q;
	_q_down_sampled.normalize();

	matrix::Dcm<float> delta_R(delta_q.inversed());
	_imu_down_sampled.delta_vel = delta_R * _imu_down_sampled.delta_vel;
	_imu_down_sampled.delta_vel += imu.delta_vel;

	if ((_dt_imu_avg * _imu_ticks >= (float)(FILTER_UPDATE_PERRIOD_MS) / 1000) || 
		_dt_imu_avg * _imu_ticks >= 0.02f){
		imu = {
			.delta_ang	= _q_down_sampled.to_axis_angle(),
			.delta_vel	= _imu_down_sampled.delta_vel,
			.delta_ang_dt	= _imu_down_sampled.delta_ang_dt,
			.delta_vel_dt	= _imu_down_sampled.delta_vel_dt,
			.time_us	= imu.time_us
		};
		_imu_down_sampled.delta_ang.setZero();
		_imu_down_sampled.delta_vel.setZero();
		_imu_down_sampled.delta_ang_dt = 0.0f;
		_imu_down_sampled.delta_vel_dt = 0.0f;
		_q_down_sampled(0) = 1.0f;
		_q_down_sampled(1) = _q_down_sampled(2) = _q_down_sampled(3) = 0.0f;
		return true;
	}

	return false;
}

bool Ekf::collect_opticalflow(uint64_t time_usec, uint8_t quality, Vector2f *flowdata, Vector2f *gyrodata, uint32_t dt)
{
	if(quality <= 150 || fabsf(_state.pos(2)) < 0.2f) {
		(*flowdata)(0) = 0.0f;
        (*flowdata)(1) = 0.0f;
        (*gyrodata)(0) = 0.0f;
        (*gyrodata)(1) = 0.0f;
	}else {
		//matrix::Euler<float> euler(_state.quat_nominal);
		float integralToRate = 1e6f/dt;
		/*float cosYaw = cosf(euler(0));
		float sinYaw = sinf(euler(0));
		Vector2f gyro = *gyrodata,flow = *flowdata;
		(*flowdata)(0) = 1.0f * integralToRate * (cosYaw * float(flow(0)) - sinYaw * float(flow(1))); // rad/sec measured optically about the X body axis
        (*flowdata)(1) = 1.0f * integralToRate * (sinYaw * float(flow(0)) + cosYaw * float(flow(1))); // rad/sec measured optically about the Y body axis
        (*gyrodata)(0) = integralToRate * (cosYaw * float(gyro(0)) - sinYaw * float(gyro(1))); // rad/sec measured inertially about the X body axis
        (*gyrodata)(1) = integralToRate * (sinYaw * float(gyro(0)) + cosYaw * float(gyro(1))); // rad/sec measured inertially about the Y body axis
		*/
		(*flowdata)(0) *= integralToRate;
		(*flowdata)(1) *= integralToRate;
		(*gyrodata)(0) *= integralToRate;
		(*gyrodata)(1) *= integralToRate;
		
	}
	return true;
}
void Ekf::calculateOutputStates()
{
	imuSample imu_new = _imu_sample_new;
	Vector3f delta_angle;

	// Note: We do no not need to consider any bias or scale correction here
	// since the base class has already corrected the imu sample
	delta_angle(0) = imu_new.delta_ang(0);
	delta_angle(1) = imu_new.delta_ang(1);
	delta_angle(2) = imu_new.delta_ang(2);

	Vector3f delta_vel = imu_new.delta_vel;

	delta_angle += _delta_angle_corr;
	Quaternion dq;
	dq.from_axis_angle(delta_angle);

	_output_new.time_us = imu_new.time_us;
	_output_new.quat_nominal = dq * _output_new.quat_nominal;
	_output_new.quat_nominal.normalize();

	matrix::Dcm<float> R_to_earth(_output_new.quat_nominal);

	Vector3f delta_vel_NED = R_to_earth * delta_vel + _delta_vel_corr;
	delta_vel_NED(2) += 9.81f * imu_new.delta_vel_dt;

	Vector3f vel_last = _output_new.vel;

	_output_new.vel += delta_vel_NED;

	_output_new.pos += (_output_new.vel + vel_last) * (imu_new.delta_vel_dt * 0.5f) + _vel_corr * imu_new.delta_vel_dt;

	if (_imu_updated) {
		_output_buffer.push(_output_new);
		_imu_updated = false;
	}

	_output_sample_delayed = _output_buffer.get_oldest();

	Quaternion quat_inv = _state.quat_nominal.inversed();
	Quaternion q_error =  _output_sample_delayed.quat_nominal * quat_inv;
	q_error.normalize();
	Vector3f delta_ang_error;

	float scalar;

	if (q_error(0) >= 0.0f) {
		scalar = -2.0f;

	} else {
		scalar = 2.0f;
	}

	delta_ang_error(0) = scalar * q_error(1);
	delta_ang_error(1) = scalar * q_error(2);
	delta_ang_error(2) = scalar * q_error(3);

	_delta_angle_corr = delta_ang_error * imu_new.delta_ang_dt;

	_delta_vel_corr = (_state.vel - _output_sample_delayed.vel) * imu_new.delta_vel_dt;

	_vel_corr = (_state.pos - _output_sample_delayed.pos);

}


void Ekf::fuseAirspeed()
{

}

void Ekf::fuseRange()
{

}

void Ekf::printStates()
{
	static int counter = 0;

	if (counter % 50 == 0) {
		printf("quaternion\n");

		for (int i = 0; i < 4; i++) {
			printf("quat %i %.5f\n", i, (double)_state.quat_nominal(i));
		}

		matrix::Euler<float> euler(_state.quat_nominal);
		printf("yaw pitch roll %.5f %.5f %.5f\n", (double)euler(2), (double)euler(1), (double)euler(0));

		printf("vel\n");

		for (int i = 0; i < 3; i++) {
			printf("v %i %.5f\n", i, (double)_state.vel(i));
		}

		printf("pos\n");

		for (int i = 0; i < 3; i++) {
			printf("p %i %.5f\n", i, (double)_state.pos(i));
		}

		printf("gyro_scale\n");

		for (int i = 0; i < 3; i++) {
			printf("gs %i %.5f\n", i, (double)_state.gyro_scale(i));
		}

		printf("mag earth\n");

		for (int i = 0; i < 3; i++) {
			printf("mI %i %.5f\n", i, (double)_state.mag_I(i));
		}

		printf("mag bias\n");

		for (int i = 0; i < 3; i++) {
			printf("mB %i %.5f\n", i, (double)_state.mag_B(i));
		}

		counter = 0;
	}

	counter++;

}

void Ekf::printStatesFast()
{
	static int counter_fast = 0;

	if (counter_fast % 200 == 0) {
		printf("quaternion\n");

		for (int i = 0; i < 4; i++) {
			printf("quat %i %.5f\n", i, (double)_output_new.quat_nominal(i));
		}

		printf("vel\n");

		for (int i = 0; i < 3; i++) {
			printf("v %i %.5f\n", i, (double)_output_new.vel(i));
		}

		printf("pos\n");

		for (int i = 0; i < 3; i++) {
			printf("p %i %.5f\n", i, (double)_output_new.pos(i));
		}

		counter_fast = 0;
	}

	counter_fast++;
}
