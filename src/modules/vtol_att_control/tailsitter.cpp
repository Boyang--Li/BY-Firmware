/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
* @file tailsitter.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#include "tailsitter.h"
#include "vtol_att_control_main.h"

#define ARSP_YAW_CTRL_DISABLE 4.0f	// airspeed at which we stop controlling yaw during a front transition
#define THROTTLE_TRANSITION_MAX 0.25f	// maximum added thrust above last value in transition
#define PITCH_TRANSITION_FRONT_P1 -1.1f	// pitch angle to switch to TRANSITION_P2
#define PITCH_TRANSITION_BACK -0.25f	// pitch angle to switch to MC

Tailsitter::Tailsitter(VtolAttitudeControl *attc) :
	VtolType(attc),
	_thrust_transition_start(0.0f),
	_yaw_transition(0.0f),
	_pitch_transition_start(0.0f)
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

	_flag_was_in_trans_mode = false;

	_params_handles_tailsitter.front_trans_dur_p2 = param_find("VT_TRANS_P2_DUR");
}

void
Tailsitter::parameters_update()
{
	float v;

	/* vtol front transition phase 2 duration */
	param_get(_params_handles_tailsitter.front_trans_dur_p2, &v);
	_params_tailsitter.front_trans_dur_p2 = v;
}

void Tailsitter::update_vtol_state()
{

	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/

	matrix::Eulerf euler = matrix::Quatf(_v_att->q);
	float pitch = euler.theta();

	if (!_attc->is_fixed_wing_requested()) {

		switch (_vtol_schedule.flight_mode) { // user switchig to MC mode
		case MC_MODE:
			break;

		case FW_MODE:
			_vtol_schedule.flight_mode 	= TRANSITION_BACK;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = MC_MODE;
			break;

		case TRANSITION_BACK:

			// check if we have reached pitch angle to switch to MC mode
			if (pitch >= PITCH_TRANSITION_BACK) {
				_vtol_schedule.flight_mode = MC_MODE;
			}

			break;
		}

	} else {  // user switchig to FW mode

		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
			// initialise a front transition
			_vtol_schedule.flight_mode 	= TRANSITION_FRONT_P1;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case FW_MODE:
			break;

		case TRANSITION_FRONT_P1: {

				bool airspeed_condition_satisfied = _airspeed->indicated_airspeed_m_s >= _params->transition_airspeed;
				airspeed_condition_satisfied |= _params->airspeed_disabled;

				// check if we have reached airspeed  and pitch angle to switch to TRANSITION P2 mode
				if ((airspeed_condition_satisfied && pitch <= PITCH_TRANSITION_FRONT_P1) || can_transition_on_ground()) {
					_vtol_schedule.flight_mode = FW_MODE;
				}

				break;
			}

		case TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = FW_MODE;
			break;
		}
	}

	// map tailsitter specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		_vtol_mode = ROTARY_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		_flag_was_in_trans_mode = false;
		break;

	case FW_MODE:
		_vtol_mode = FIXED_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		_flag_was_in_trans_mode = false;
		break;

	case TRANSITION_FRONT_P1:
		_vtol_mode = TRANSITION_TO_FW;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;

	case TRANSITION_BACK:
		_vtol_mode = TRANSITION_TO_MC;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;
	}
}

void Tailsitter::update_transition_state()
{
	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

	// coefficients of optimal transition Forward_Pitch/Throttle (Duration, Coefs)
	int F_group = (int)_params->front_trans_duration - 2;
	// 2s 3s 4s 5s
	double F_P[4][10] = {{-7.4199, 68.4020, -265.1932, 560.3263, -696.2477, 507.6028, -199.8091, 31.6669, 0.8125, -1.1578},
				{-0.2349, 3.2259, -18.6008, 58.5090, -109.0086, 121.7502, -78.0129, 25.4079, -3.4404, -0.7483},
				{-0.0113, 0.2280, -1.9051, 8.5602, -22.4758, 35.0094, -31.3998, 15.2099, -3.8951, -0.5119},
				{-0.0012, 0.0323, -0.3522, 2.0713, -7.1725, 14.9228, -18.2153, 12.2450, -4.1813, -0.4604}};
	double F_T[4][10] = {{-4.1409, 34.2731, -116.7371, 210.9709, -217.3414, 127.2911, -39.5723, 5.2551, -0.0268, 0.5698},
				{-0.1845, 2.2684, -11.4027, 30.1122, -44.5822, 36.3603, -14.5111, 1.5223, 0.4489, 0.5389},
				{0.0010, -0.0259, 0.2512, -1.2290, 3.3790, -5.4857, 5.4397, -3.3604, 1.0260, 0.5247},
				{-0.0016, 0.0346, -0.3199, 1.6069, -4.7498, 8.3301, -8.1908, 3.8157, -0.5911, 0.6037}};

	int B_group = (int)_params->back_trans_duration - 1;
	// 1s 2s 3s
	double B_P[3][10] = {{-3468.4980, 16057.4460, -31597.5265, 34253.6207, -22136.9947, 8627.1658, -1966.5836, 245.5379, -14.3973, 0.3184},
				{-7.2341, 62.3607, -223.4805, 427.4301, -461.2393, 263.4110, -50.7292, -20.4003, 11.2437, -1.3922}, 
				{0.0133, -0.0160, -0.8095, 4.9849, -12.3784, 14.0746, -4.7415, -3.9180, 3.4549, -0.6814}};	 
	double B_T[3][10] = {{-1788.5736, 6147.3816, -7615.7336, 2944.9074, 2076.3554, -2803.5485, 1310.9067, -307.1693, 35.4740, -1.5708},
				{-12.8003, 126.3106, -531.5831, 1240.6703, -1747.5478, 1513.9686, -786.0760, 225.3139, -28.7702, 0.9923},
				{0.0031, 0.0603, -0.5498, 0.7082, 4.7367, -18.3354, 25.7949, -16.0712, 4.0318, 0.0872}};

	if (!_flag_was_in_trans_mode) {
		// save desired heading for transition and last thrust value
		_yaw_transition = _v_att_sp->yaw_body;
		//transition should start from current attitude instead of currBY_t
		matrix::Eulerf euler = matrix::Quatf(_v_att->q);
		_pitch_transition_start = euler.theta();
		_thrust_transition_start = _v_att_sp->thrust;
		_flag_was_in_trans_mode = true;
	}

	if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P1) {
		// create time dependant pitch angle set point + 0.2 rad overlaBY_switch value
		// modify the pitch angle setpoint to enable optimal transitionBY_
		// time will be less than 2s? or fixed to 2s ;;; Fitted with a polynomical by matlab and then put here		
		// variables: _pitch_transition_start ; time_since_trans_start    pitch_cmd = F (t,pitch_start) 
		if (_attc->is_by_optimal_F_transition()) {			
			_v_att_sp->pitch_body = _pitch_transition_start + (float)( F_P[F_group][0]*pow(time_since_trans_start,9) 
				+ F_P[F_group][1]*pow(time_since_trans_start,8) + F_P[F_group][2]*pow(time_since_trans_start,7) 
				+ F_P[F_group][3]*pow(time_since_trans_start,6) + F_P[F_group][4]*pow(time_since_trans_start,5) 
				+ F_P[F_group][5]*pow(time_since_trans_start,4) + F_P[F_group][6]*pow(time_since_trans_start,3) 
				+ F_P[F_group][7]*pow(time_since_trans_start,2) + F_P[F_group][8]*pow(time_since_trans_start,1) 
				+ F_P[F_group][9] );
		} else {
			_v_att_sp->pitch_body = _pitch_transition_start	- fabsf(PITCH_TRANSITION_FRONT_P1 - _pitch_transition_start) *
			time_since_trans_start / _params->front_trans_duration;	
		}
		
		_v_att_sp->pitch_body = math::constrain(_v_att_sp->pitch_body, PITCH_TRANSITION_FRONT_P1 - 0.2f,
							_pitch_transition_start);

		// Front Trans Throttle Setpoint
		// front/back transition uses same throttle setpoint
		//  variables: time_since_trans_start    thrust_cmd = F (t,pitch_start) 
		if (_v_control_mode->flag_control_climb_rate_enabled) {	
			if (_attc->is_by_optimal_F_transition()) {
				_v_att_sp->thrust = 0.2f + (float)( F_T[F_group][0]*pow(time_since_trans_start,9) 
				+ F_T[F_group][1]*pow(time_since_trans_start,8) + F_T[F_group][2]*pow(time_since_trans_start,7) 
				+ F_T[F_group][3]*pow(time_since_trans_start,6) + F_T[F_group][4]*pow(time_since_trans_start,5) 
				+ F_T[F_group][5]*pow(time_since_trans_start,4) + F_T[F_group][6]*pow(time_since_trans_start,3) 
				+ F_T[F_group][7]*pow(time_since_trans_start,2) + F_T[F_group][8]*pow(time_since_trans_start,1) 
				+ F_T[F_group][9] );
			} else {
				_v_att_sp->thrust = _params->front_trans_throttle;
			}
			_v_att_sp->thrust = math::constrain(_v_att_sp->thrust, 0.0f, 1.0f);
		} else {
			_v_att_sp->thrust = _mc_virtual_att_sp->thrust;
		}

		// disable mc yaw control once the plane has picked up speed
		if (_airspeed->indicated_airspeed_m_s > ARSP_YAW_CTRL_DISABLE) {
			_mc_yaw_weight = 0.0f;

		} else {
			_mc_yaw_weight = 1.0f;
		}

		_mc_roll_weight = 1.0f;
		_mc_pitch_weight = 1.0f;

	} else if (_vtol_schedule.flight_mode == TRANSITION_BACK) {

		if (!flag_idle_mc) {
			flag_idle_mc = set_idle_mc();
		}

		// create time dependant pitch angle set point stating at -pi/2 + 0.2 rad overlap over the switch value
		// BACK This equation is meaningless since all data will be constrained by |-2,-0.05|
		//_v_att_sp->pitch_body = M_PI_2_F + _pitch_transition_start + fabsf(PITCH_TRANSITION_BACK + 1.57f) *
		//			time_since_trans_start / _params->back_trans_duration;
		if (_attc->is_by_optimal_B_transition()) {
			_v_att_sp->pitch_body = _pitch_transition_start + (float)( B_P[B_group][0]*pow(time_since_trans_start,9) 
				+ B_P[B_group][1]*pow(time_since_trans_start,8) + B_P[B_group][2]*pow(time_since_trans_start,7) 
				+ B_P[B_group][3]*pow(time_since_trans_start,6) + B_P[B_group][4]*pow(time_since_trans_start,5) 
				+ B_P[B_group][5]*pow(time_since_trans_start,4) + B_P[B_group][6]*pow(time_since_trans_start,3) 
				+ B_P[B_group][7]*pow(time_since_trans_start,2) + B_P[B_group][8]*pow(time_since_trans_start,1) 
				+ B_P[B_group][9] );
		} else { // original modified back transition in  back_trans_duration time
		_v_att_sp->pitch_body = _pitch_transition_start + fabsf(PITCH_TRANSITION_BACK + 1.57f) *
			time_since_trans_start / _params->back_trans_duration;
		}

		_v_att_sp->pitch_body = math::constrain(_v_att_sp->pitch_body, -2.0f, PITCH_TRANSITION_BACK + 0.2f);

		// Back trans throttle setpoint
		if (_v_control_mode->flag_control_climb_rate_enabled) {	
		//  variables: time_since_trans_start    thrust_cmd = F (t,pitch_start) 
			if (_attc->is_by_optimal_B_transition()) {
				_v_att_sp->thrust = 0.2f + (float)( B_T[B_group][0]*pow(time_since_trans_start,9) 
				+ B_T[B_group][1]*pow(time_since_trans_start,8) + B_T[B_group][2]*pow(time_since_trans_start,7) 
				+ B_T[B_group][3]*pow(time_since_trans_start,6) + B_T[B_group][4]*pow(time_since_trans_start,5) 
				+ B_T[B_group][5]*pow(time_since_trans_start,4) + B_T[B_group][6]*pow(time_since_trans_start,3) 
				+ B_T[B_group][7]*pow(time_since_trans_start,2) + B_T[B_group][8]*pow(time_since_trans_start,1) 
				+ B_T[B_group][9] );
			} else {
				_v_att_sp->thrust = _params->back_trans_throttle;
			}
			_v_att_sp->thrust = math::constrain(_v_att_sp->thrust, 0.0f, 1.0f);
		} else {
			_v_att_sp->thrust = _mc_virtual_att_sp->thrust;
		}

		// keep yaw disabled
		_mc_yaw_weight = 0.0f;

		// smoothly move control weight to MC
		_mc_roll_weight = _mc_pitch_weight = time_since_trans_start / _params->back_trans_duration;

	}

	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);
	_mc_pitch_weight = math::constrain(_mc_pitch_weight, 0.0f, 1.0f);

	// compute desired attitude and thrust setpoint for the transition

	_v_att_sp->timestamp = hrt_absolute_time();
	_v_att_sp->roll_body = 0.0f;
	_v_att_sp->yaw_body = _yaw_transition;

	matrix::Quatf q_sp = matrix::Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body);
	q_sp.copyTo(_v_att_sp->q_d);
	_v_att_sp->q_d_valid = true;
}

void Tailsitter::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	_v_att_sp->thrust = _thrust_transition;
}

void Tailsitter::update_mc_state()
{
	VtolType::update_mc_state();
}

void Tailsitter::update_fw_state()
{
	VtolType::update_fw_state();
}

/**
* Write data to actuator output topic.
*/
void Tailsitter::fill_actuator_outputs()
{
	_actuators_out_0->timestamp = hrt_absolute_time();
	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;

	_actuators_out_1->timestamp = hrt_absolute_time();
	_actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;

	switch (_vtol_mode) {
	case ROTARY_WING:
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW];
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];

		if (_params->elevons_mc_lock) {
			_actuators_out_1->control[0] = 0;
			_actuators_out_1->control[1] = 0;

		} else {
			// NOTE: There is no mistake in the line below, multicopter yaw axis is controlled by elevon roll actuation!
			_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
				_actuators_mc_in->control[actuator_controls_s::INDEX_YAW];	//roll elevon
			_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
				_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];	//pitch elevon
		}

		break;

	case FIXED_WING:
		// in fixed wing mode we use engines only for providing thrust, no moments are generated
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = 0;
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] = 0;
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = 0;
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];

		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
			-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];	// roll elevon
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];	// pitch elevon
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];	// yaw
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];	// throttle
		break;

	case TRANSITION_TO_FW:
	case TRANSITION_TO_MC:
		// in transition engines are mixed by weight (BACK TRANSITION ONLY)
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL]
				* _mc_roll_weight;
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW] *
				_mc_yaw_weight;
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];

		// NOTE: There is no mistake in the line below, multicopter yaw axis is controlled by elevon roll actuation!
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = -_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL]
				* (1 - _mc_yaw_weight);
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
		// **LATER** + (_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim) *(1 - _mc_pitch_weight);
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];
		break;
	}
}
