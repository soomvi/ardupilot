#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAltHold::init(bool ignore_checks)
{

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

	// YIG-ADD
	if(copter.control_mode_reason == ModeReason::EKF_FAILSAFE)
	{
		//rtl_bearing = home_bearing();

		_state = SubMode::GlitchAltHold_Starting;
		_loop_timer = AP_HAL::millis();
		gcs().send_text(MAV_SEVERITY_INFO, "AltHold on GPS Fail");
		gcs().send_text(MAV_SEVERITY_INFO, "home bearing = %ld", copter.rtl_bearing);
	}
	else
		_state = SubMode::GlitchAltHold_None;
	//

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAltHold::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

	// YIG-ADD
	if(copter.control_mode_reason == ModeReason::EKF_FAILSAFE)
	{
		switch (_state) {

		case SubMode::GlitchAltHold_None:
			break;

		case SubMode::GlitchAltHold_Starting:
		case SubMode::GlitchAltHold_RotateToHome:
			target_roll = 0; target_pitch = 0; target_yaw_rate = 0; target_climb_rate = 0;
			break;

		case SubMode::GlitchAltHold_Climb:
			target_roll = 0; target_pitch = 0; target_yaw_rate = 0;
			target_climb_rate = g.pilot_speed_up * 0.5f;
			break;

		case SubMode::GlitchAltHold_ReturnToHome:
			target_yaw_rate = 0; target_climb_rate = 0;
    		auto_get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());
			break;
		}
	}

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero

    	// call attitude controller
    	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    	// run the vertical position controller and set output throttle
    	pos_control->update_z_controller();
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero

    	// call attitude controller
    	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    	// run the vertical position controller and set output throttle
    	pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);

    	// call attitude controller
    	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    	// run the vertical position controller and set output throttle
    	pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

		// YIG-ADD : for GPS_Glitch
		if(copter.control_mode_reason == ModeReason::EKF_FAILSAFE)
		{
			switch (_state) {

			case SubMode::GlitchAltHold_None:
				break;

			case SubMode::GlitchAltHold_Starting:
				auto_yaw.set_yaw_angle_rate(copter.rtl_bearing * 0.01f, 0.0f);		// 2nd argument 0.0f 일때 ATC_SLEW_YAW 값을 사용 함
			   	_state = SubMode::GlitchAltHold_RotateToHome;
				gcs().send_text(MAV_SEVERITY_INFO, "starting RotateToHome");
				break;

			case SubMode::GlitchAltHold_RotateToHome:
				pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

				attitude_control->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, auto_yaw.yaw(), true);

				//if ((abs(wrap_180_cd(ahrs.yaw_sensor-copter.rtl_bearing))) <= 3)
				if ((abs(wrap_180_cd(ahrs.yaw_sensor-copter.rtl_bearing))) <= 50)	// 1 degree
				{
#if 0
					takeoff.gps_glitch_start(constrain_float(1000.0f,0.0f,10000.0f));
			   		_state = SubMode::GlitchAltHold_Climb;
					gcs().send_text(MAV_SEVERITY_INFO, "starting Climb");
#else
					_state = SubMode::GlitchAltHold_ReturnToHome;
					gcs().send_text(MAV_SEVERITY_INFO, "starting ReturnToHome");
#endif
				}
			    break;

			case SubMode::GlitchAltHold_Climb:
        		takeoff.do_pilot_gps_glitch_takeoff(target_climb_rate);

    			attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

				if(takeoff.gps_glitch_running() == false)
				{
					_state = SubMode::GlitchAltHold_ReturnToHome;
					gcs().send_text(MAV_SEVERITY_INFO, "starting ReturnToHome");
				}
		       	break;

			case SubMode::GlitchAltHold_ReturnToHome:
        		pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

    			attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

		       	break;
			}

    		pos_control->update_z_controller();

		} // for GPS Glitch

		else
		{
#if AC_AVOID_ENABLED == ENABLED
        	// apply avoidance
        	copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        	// get avoidance adjusted climb rate
        	target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        	// update the vertical offset based on the surface measurement
        	copter.surface_tracking.update_surface_offset();

        	// Send the commanded climb rate to the position controller
        	pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

    		// call attitude controller
    		attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    		// run the vertical position controller and set output throttle
    		pos_control->update_z_controller();
		}
        break;
    }
}
