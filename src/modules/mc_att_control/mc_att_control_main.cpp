/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * Publication for the desired attitude tracking:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_virtual_fw.h>
#include <uORB/topics/actuator_controls_virtual_mc.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/mc_att_ctrl_status.h>
    /** MC* ...........*/
#include <uORB/topics/g_matrix.h>
#include <uORB/topics/B_matrix.h>
#include <uORB/topics/T_bw_matrix.h>
#include <uORB/topics/Bb_tb_i_matrix.h>
#include <uORB/topics/Bt_tb_i_matrix.h>
#include <uORB/topics/Bb_tb_i_pinv_matrix.h>
#include <uORB/topics/am_u_tbeta.h>
#include <uORB/topics/am_flag.h>
#include <uORB/topics/am_tau.h>
#include <uORB/topics/csi.h>
#include <uORB/topics/csi_r.h>
#include <uORB/topics/csi_dot.h>
    /** MC* ...........*/
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>

#include "matrix/Matrix.hpp"

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define RATES_I_LIMIT	0.3f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ATTITUDE_TC_DEFAULT 0.2f

class MulticopterAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControl();

	/**
	 * Destructor, also kills the main task
	 */
	~MulticopterAttitudeControl();

	/**
	 * Start the multicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */

	int		_ctrl_state_sub;		/**< control state subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_armed_sub;				/**< arming status subscription */
	int		_vehicle_status_sub;	/**< vehicle status subscription */
	int 	_motor_limits_sub;		/**< motor limits subscription */

    /** MC* ...........*/
    int     _g_eta_sub;
    int     _B_eta_sub;
    int     _T_bw_sub;
    int     _Bb_tb_i_sub;
    int     _Bt_tb_i_sub;
    int     _Bb_tb_i_pinv_sub;
    int     _am_u_tbeta_sub;
    int     _csi_dot_sub;
    int     _csi_sub;
    int     _csi_r_sub;

    //const int J_m = 0.0168;

    orb_advert_t    _am_flag_pub;
    orb_advert_t    _am_tau_pub;

    struct g_matrix_s               _g_eta;
    struct B_matrix_s               _B_eta;
    struct T_bw_matrix_s            _T_bw;
    struct Bb_tb_i_matrix_s         _Bb_tb_i;
    struct Bt_tb_i_matrix_s         _Bt_tb_i;
    struct Bb_tb_i_pinv_matrix_s    _Bb_tb_i_pinv;
    struct am_u_tbeta_s             _am_u_tbeta;
    struct am_flag_s                _am_flag;
    struct am_tau_s                 _am_tau;
    struct csi_dot_s                _csi_dot;
    struct csi_s                    _csi;
    struct csi_r_s                  _csi_r;

    math::Vector<10> am_g_eta;//(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
    math::Matrix<10,10> am_B_eta;//(0,0,0,0,0,0,0,0,0,0);
    math::Matrix<6,2> am_T_betaw;
    math::Matrix<8,2> am_Bt_tb_i;
    math::Matrix<8,6> am_Bb_tb_i;
    math::Matrix<6,8> am_Bb_tb_i_pinv;

    /** Submatrices: */
    math::Vector<2> am_g_t;
    math::Vector<2> am_g_w;//(0,0,0,0,0,0,0,0,0,0);
    math::Vector<6> am_g_beta;//(0,0,0,0,0,0,0,0,0,0);
    math::Matrix<2,2> am_B_ww;// am_B_ww.set_row(0,Bww_r1); am_B_ww.set_row(1,Bww_r2); am_B_ww.set_row(2,Bww_r3);
    math::Matrix<6,2> am_B_betaw;
    math::Matrix<2,6> am_B_wbeta;

    /** Intermediate variables: */
    math::Vector<6> am_tau_beta_in;
    math::Vector<8> am_effec_tau;
    math::Matrix<8,8> am_B_tb_i;
    math::Matrix<6,6> am_Bbb_tb_i;

    math::Vector<6> am_beta_dot_ref;

    math::Vector<6> am_tau_beta;
    math::Vector<2> am_tau_omega_xy;
    math::Matrix<2,6> am_T_betaw_t;
    math::Vector<2> am_tau_omega_in;

    math::Vector<2> am_tau_quad_int_add_aux;
    math::Vector<2> am_tau_quad_int_add;
    math::Vector<6> am_tau_beta_int_add;

    math::Vector<8> am_u_tbeta_int_add_noZ;
    math::Vector<6> am_BZ_tb_i_pinv;
    math::Vector<6> am_tau_utb_Z_aux;

    math::Vector<6>  alpha;
    math::Vector<6>  alpha_pos;
    math::Matrix<2,2> am_T_betaw_sq;
    math::Matrix<2,2> am_T_betaw_sq_inv;
    math::Vector<2> err_omega_alpha;
    math::Vector<2> err_omega_alpha_int;
    math::Vector<2> err_omega_alpha_int_add;
    math::Vector<2> omega_xy_r;
    math::Vector<2> am_phi_xy_r;
    math::Vector<2> err_omega_alpha_pos;
    math::Vector<6> am_u_tbeta_aux;
    //math::Vector<6> am_tau_beta_rm;
    //    math::Vector<2> err_omega_csi;

    math::Vector<6>  alpha_p;
    math::Vector<6>  alpha_p0;
    math::Vector<6>  beta_p;
    math::Vector<6>  alpha_fb;
    math::Vector<6>  err_alpha;
    math::Vector<6>  beta;
    math::Vector<6>  alpha_0;
    math::Vector<6>  alpha_dot_int_add;
    math::Vector<6>  alpha_dot;
    math::Vector<6>  alpha_dot_int;
    const float Kpp_alpha =  2.4f;
    const float Kpv_alpha = 12.0f;
    const float Kiv_alpha = 48.0f;

    float det_Tbetaw;
//    const float Kp_alpha_pos = 2.0f;
//    const float Kp_alpha = 15.0f;
//    const float Ki_alpha =  40.0f; //10.0f;
    const float W_w2beta = 0.1f;
    /** MC* ...........*/

	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t	_controller_status_pub;	/**< controller status publication */

	orb_id_t _rates_sp_id;	/**< pointer to correct rates setpoint uORB metadata structure */
	orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct control_state_s				_ctrl_state;		/**< control state */
	struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator controls */
	struct actuator_armed_s				_armed;				/**< actuator arming status */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct multirotor_motor_limits_s	_motor_limits;		/**< motor limits */
	struct mc_att_ctrl_status_s 		_controller_status; /**< controller status */

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_controller_latency_perf;

	math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
	math::Vector<3>		_rates_sp_prev; /**< previous rates setpoint */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_int;		/**< angular rates integral error */
	float				_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>		_att_control;	/**< attitude control vector */

	math::Matrix<3, 3>  _I;				/**< identity matrix */

	/** RR* ...........*/

    /** Da Funzione principale*/
    math::Vector<8> am_u_tbeta;
    math::Vector<2> am_omega_xy_dot_ref;
    math::Vector<8> am_u_tbeta_int_add;
    math::Vector<2> am_omega_xy_dot_ref_int_add;
    math::Vector<2> am_att_control_acc_i;
    float am_thrust;
    math::Vector<3> am_tau_quad;
    math::Vector<4> am_tau_robot;

	bool am_saturation_omega_thrust;
	bool am_saturation_omega_tau_quad;
	bool am_saturation_omega_tau_robot;
    bool am_saturation_utb_thrust;
	bool am_saturation_utb_tau_quad;
	bool am_saturation_utb_tau_robot;
    bool am_saturation_utbZ_thrust;
	bool am_saturation_utbZ_tau_quad;
	bool am_saturation_utbZ_tau_robot;

	bool reset_int_flag = true;

    float ControlToActControl_T;
	math::Vector<3> ControlToActControl_tau;

    /** RR* ...........*/

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_d;
		param_t roll_rate_ff;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_d;
		param_t pitch_rate_ff;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_d;
		param_t yaw_rate_ff;
		param_t yaw_ff;
		param_t roll_rate_max;
		param_t pitch_rate_max;
		param_t yaw_rate_max;
		param_t yaw_auto_max;

		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;
		param_t rattitude_thres;

		param_t vtol_type;
		param_t roll_tc;
		param_t pitch_tc;
		param_t vtol_opt_recovery_enabled;

	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		math::Vector<3> att_p;					/**< P gain for angular error */
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> rate_i;				/**< I gain for angular rate error */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		math::Vector<3>	rate_ff;			/**< Feedforward gain for desired rates */
		float yaw_ff;						/**< yaw control feed-forward */

		float roll_rate_max;
		float pitch_rate_max;
		float yaw_rate_max;
		float yaw_auto_max;
		math::Vector<3> mc_rate_max;		/**< attitude rate limits in stabilized modes */
		math::Vector<3> auto_rate_max;		/**< attitude rate limits in auto modes */
		math::Vector<3> acro_rate_max;		/**< max attitude rates in acro mode */
		float rattitude_thres;
		int vtol_type;						/**< 0 = Tailsitter, 1 = Tiltrotor, 2 = Standard airframe */
		bool vtol_opt_recovery_enabled;
	}		_params;

	TailsitterRecovery *_ts_opt_recovery;	/**< Computes optimal rates for tailsitter recovery */

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for attitude setpoint updates.
	 */
	void		vehicle_attitude_setpoint_poll();

	/**
	 * Check for rates setpoint updates.
	 */
	void		vehicle_rates_setpoint_poll();

	/**
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

	/**
	 * Attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

    void        compute_final_torques(float dt);
	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for vehicle motor limits status.
	 */
	void		vehicle_motor_limits_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

namespace mc_att_control
{

MulticopterAttitudeControl	*g_control;
}

MulticopterAttitudeControl::MulticopterAttitudeControl() :

	_task_should_exit(false),
	_control_task(-1),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),
	_vehicle_status_sub(-1),

    /** MC* ...........*/
    _g_eta_sub(-1),
    _B_eta_sub(-1),
    _T_bw_sub(-1),
    _Bb_tb_i_sub(-1),
    _Bt_tb_i_sub(-1),
    _Bb_tb_i_pinv_sub(-1),
    _am_u_tbeta_sub(-1),
    _csi_dot_sub(-1),
    _csi_sub(-1),
    _csi_r_sub(-1),

    _am_flag_pub(nullptr),
    _am_tau_pub(nullptr),

    /** MC* ...........*/

	/* publications */
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_controller_status_pub(nullptr),
	_rates_sp_id(0),
	_actuators_id(0),

	_actuators_0_circuit_breaker_enabled(false),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
	_ts_opt_recovery(nullptr)

{
	memset(&_ctrl_state, 0, sizeof(_ctrl_state));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_armed, 0, sizeof(_armed));
	memset(&_vehicle_status, 0, sizeof(_vehicle_status));
	memset(&_motor_limits, 0, sizeof(_motor_limits));
	memset(&_controller_status, 0, sizeof(_controller_status));
	_vehicle_status.is_rotary_wing = true;

    /** MC* ...........*/
	memset(&_g_eta, 0, sizeof(_g_eta));
	memset(&_B_eta, 0, sizeof(_B_eta));
	memset(&_T_bw, 0, sizeof(_T_bw));
	memset(&_Bb_tb_i, 0, sizeof(_Bb_tb_i));
    memset(&_Bt_tb_i, 0, sizeof(_Bt_tb_i));
	memset(&_Bb_tb_i_pinv, 0, sizeof(_Bb_tb_i_pinv));
	memset(&_am_u_tbeta, 0, sizeof(_am_u_tbeta));
	memset(&_am_flag, 0, sizeof(_am_flag));
	memset(&_am_tau, 0, sizeof(_am_tau));
	memset(&_csi_dot, 0, sizeof(_csi_dot));
	memset(&_csi, 0, sizeof(_csi));
	memset(&_csi_r, 0, sizeof(_csi_r));
    /** MC* ...........*/

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_d.zero();
	_params.rate_ff.zero();
	_params.yaw_ff = 0.0f;
	_params.roll_rate_max = 0.0f;
	_params.pitch_rate_max = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.mc_rate_max.zero();
	_params.auto_rate_max.zero();
	_params.acro_rate_max.zero();
	_params.rattitude_thres = 1.0f;
	_params.vtol_opt_recovery_enabled = false;

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_sp_prev.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();
	/** RR .........*/
    am_att_control_acc_i.zero();
    ControlToActControl_T = 0.0192f;
    ControlToActControl_tau(0) = 0.0653f;
    ControlToActControl_tau(1) = 0.1624f;
    ControlToActControl_tau(2) = 0.5376f;
    /** RR .........*/

	_I.identity();

	_params_handles.roll_p			= 	param_find("MC_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	_params_handles.roll_rate_i		= 	param_find("MC_ROLLRATE_I");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	_params_handles.roll_rate_ff	= 	param_find("MC_ROLLRATE_FF");
	_params_handles.pitch_p			= 	param_find("MC_PITCH_P");
	_params_handles.pitch_rate_p	= 	param_find("MC_PITCHRATE_P");
	_params_handles.pitch_rate_i	= 	param_find("MC_PITCHRATE_I");
	_params_handles.pitch_rate_d	= 	param_find("MC_PITCHRATE_D");
	_params_handles.pitch_rate_ff 	= 	param_find("MC_PITCHRATE_FF");
	_params_handles.yaw_p			=	param_find("MC_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	_params_handles.yaw_rate_i		= 	param_find("MC_YAWRATE_I");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");
	_params_handles.yaw_rate_ff	 	= 	param_find("MC_YAWRATE_FF");
	_params_handles.yaw_ff			= 	param_find("MC_YAW_FF");
	_params_handles.roll_rate_max	= 	param_find("MC_ROLLRATE_MAX");
	_params_handles.pitch_rate_max	= 	param_find("MC_PITCHRATE_MAX");
	_params_handles.yaw_rate_max	= 	param_find("MC_YAWRATE_MAX");
	_params_handles.yaw_auto_max	= 	param_find("MC_YAWRAUTO_MAX");
	_params_handles.acro_roll_max	= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max	= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max	= 	param_find("MC_ACRO_Y_MAX");
	_params_handles.rattitude_thres = 	param_find("MC_RATT_TH");
	_params_handles.vtol_type 		= 	param_find("VT_TYPE");
	_params_handles.roll_tc			= 	param_find("MC_ROLL_TC");
	_params_handles.pitch_tc		= 	param_find("MC_PITCH_TC");
	_params_handles.vtol_opt_recovery_enabled = param_find("VT_OPT_RECOV_EN");

	/* fetch initial parameter values */
	parameters_update();

	if (_params.vtol_type == 0 && _params.vtol_opt_recovery_enabled) {
		// the vehicle is a tailsitter, use optimal recovery control strategy
		_ts_opt_recovery = new TailsitterRecovery();
	}

}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}
	if (_ts_opt_recovery != nullptr) {
		delete _ts_opt_recovery;
	}

	mc_att_control::g_control = nullptr;
}

int
MulticopterAttitudeControl::parameters_update()
{
	float v;

	float roll_tc, pitch_tc;

	param_get(_params_handles.roll_tc, &roll_tc);
	param_get(_params_handles.pitch_tc, &pitch_tc);

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_i, &v);
	_params.rate_i(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_ff, &v);
	_params.rate_ff(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_i, &v);
	_params.rate_i(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_ff, &v);
	_params.rate_ff(1) = v;

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;
	param_get(_params_handles.yaw_rate_ff, &v);
	_params.rate_ff(2) = v;

	param_get(_params_handles.yaw_ff, &_params.yaw_ff);

	/* angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.mc_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.mc_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.mc_rate_max(2) = math::radians(_params.yaw_rate_max);

	/* auto angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.auto_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.auto_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_auto_max, &_params.yaw_auto_max);
	_params.auto_rate_max(2) = math::radians(_params.yaw_auto_max);

	/* manual rate control scale and auto mode roll/pitch rate limits */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);

	/* stick deflection needed in rattitude mode to control rates not angles */
	param_get(_params_handles.rattitude_thres, &_params.rattitude_thres);

	param_get(_params_handles.vtol_type, &_params.vtol_type);

	int tmp;
	param_get(_params_handles.vtol_opt_recovery_enabled, &tmp);
	_params.vtol_opt_recovery_enabled = (bool)tmp;

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	return OK;
}

void
MulticopterAttitudeControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void
MulticopterAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterAttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

void
MulticopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
MulticopterAttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &_motor_limits);
	}
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
MulticopterAttitudeControl::control_attitude(float dt)
{
	vehicle_attitude_setpoint_poll();

	_thrust_sp = _v_att_sp.thrust;

	/* construct attitude setpoint rotation matrix */
	math::Matrix<3, 3> R_sp;
	R_sp.set(_v_att_sp.R_body);

	/* get current rotation matrix from control state quaternions */
	math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	math::Matrix<3, 3> R = q_att.to_dcm();

	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
	math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation */
	math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

	/* calculate angle error */
	float e_R_z_sin = e_R.length();
	float e_R_z_cos = R_z * R_sp_z;

	/* calculate weight for yaw control */
	float yaw_w = R_sp(2, 2) * R_sp(2, 2);

	/* calculate rotation matrix after roll/pitch only rotation */
	math::Matrix<3, 3> R_rp;

	if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

		e_R = e_R_z_axis * e_R_z_angle;

		/* cross product matrix for e_R_axis */
		math::Matrix<3, 3> e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_z_axis(2);
		e_R_cp(0, 2) = e_R_z_axis(1);
		e_R_cp(1, 0) = e_R_z_axis(2);
		e_R_cp(1, 2) = -e_R_z_axis(0);
		e_R_cp(2, 0) = -e_R_z_axis(1);
		e_R_cp(2, 1) = e_R_z_axis(0);

		/* rotation matrix for roll/pitch only rotation */
		R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

	} else {
		/* zero roll/pitch rotation */
		R_rp = R;
	}

	/* R_rp and R_sp has the same Z axis, calculate yaw error */
	math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		math::Quaternion q;
		q.from_dcm(R.transposed() * R_sp);
		math::Vector<3> e_R_d = q.imag();
		e_R_d.normalize();
		e_R_d *= 2.0f * atan2f(e_R_d.length(), q(0));

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	}

	/* calculate angular rates setpoint */
	_rates_sp = _params.att_p.emult(e_R) * 1.25f;

	/* limit rates */
	for (int i = 0; i < 3; i++) {
		if (_v_control_mode.flag_control_velocity_enabled && !_v_control_mode.flag_control_manual_enabled) {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.auto_rate_max(i), _params.auto_rate_max(i));
		} else {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
		}
	}

	/* feed forward yaw setpoint rate */
	_rates_sp(2) += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterAttitudeControl::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_armed.armed || !_vehicle_status.is_rotary_wing) {
		_rates_int.zero();
		am_att_control_acc_i.zero();
	}

	/* current body angular rates */
	math::Vector<3> rates;
	rates(0) = _ctrl_state.roll_rate;
	rates(1) = _ctrl_state.pitch_rate;
	rates(2) = _ctrl_state.yaw_rate;

	/* angular rates error */
	math::Vector<3> rates_err = _rates_sp - rates;

	/** RR* .................... */
    /** Bww solo per controllo quadricottero*/
    math::Vector<3> Bww_r1(0.0429,0,0);
    math::Vector<3> Bww_r2(0,0.0269,0);
    math::Vector<3> Bww_r3(0,0,0.0721);
    math::Matrix<3,3> Bww;
    Bww.set_row(0,Bww_r1);
    Bww.set_row(1,Bww_r2);
    Bww.set_row(2,Bww_r3);

    /** Accelerazioni */
	math::Vector<3> am_att_control_acc_pd = _params.rate_p.emult(rates_err)* 1.25f +  _params.rate_d.emult(_rates_prev - rates) / dt + _params.rate_ff.emult(_rates_sp - _rates_sp_prev) / dt;

    /** add to integral action?*/
    math::Vector<3> _att_control_acc_i_add =  _params.rate_i.emult(rates_err) * dt * 1.5625f;

    math::Vector<2> am_omega_xy_dot_ref_int_add;
    am_omega_xy_dot_ref_int_add(0) = _att_control_acc_i_add(0);
    am_omega_xy_dot_ref_int_add(1) = _att_control_acc_i_add(1);

    am_omega_xy_dot_ref(0) = am_att_control_acc_pd(0) + am_att_control_acc_i(0);
    am_omega_xy_dot_ref(1) = am_att_control_acc_pd(1) + am_att_control_acc_i(1);

    if (_v_control_mode.flag_control_offboard_enabled){
        if (reset_int_flag) {
            printf("Reset omega! \n");
            am_omega_xy_dot_ref_int_add.zero();
            am_omega_xy_dot_ref.zero();
            am_att_control_acc_i.zero();
            err_omega_alpha_int.zero();
            alpha_dot_int.zero();
            alpha_dot_int_add.zero();
            reset_int_flag = false;
            }

        /** Calcola coppie finali e saturazioni*/
        compute_final_torques(dt);

        /** Assegna coppie*/
            for (int i = 0; i < 3; i++) {
                    _att_control(i) = am_tau_quad(i);
            }

//            printf("am_tau_beta: \n");
//            for (size_t i = 0; i < 6; i++) {
//                printf(" %-2.4g ", (double)am_tau_beta(i));
//            }
            //am_thrust = am_thrust;
//            printf("rates_err: \n");
//            for (size_t i = 0; i < 3; i++) {
//                printf(" %-2.4g ", (double)rates_err(i));
//            }
//            printf("\n");
//
//            printf("csi: \n");
//            for (size_t i = 0; i < 10; i++) {
//                printf(" %-2.4g ", (double)_csi.csi[i]);
//            }
//            printf("\n");
//            printf("csi_r: \n");
//            for (size_t i = 0; i < 10; i++) {
//                printf(" %-2.4g ", (double)_csi_r.csi_r[i]);
//            }
//            printf("\n");
//
//            printf("csi_dot: \n");
//            for (size_t i = 0; i < 10; i++) {
//                printf(" %-2.4g ", (double)_csi_dot.csi_dot[i]);
//            }
//            printf("\n");
//
//            printf("u_tbeta (att_control): \n");
//            for (size_t i = 0; i < 8; i++) {
//                printf(" %-2.4g ", (double)am_u_tbeta(i));
//            }
//            printf("\n");

//            printf("err_alpha: \n");
//            for (size_t i = 0; i < 6; i++) {
//                printf(" %-2.4g ", (double)err_alpha(i));
//            }
//            printf("\n");
//
//            printf("alpha_dot_int: \n");
//            for (size_t i = 0; i < 6; i++) {
//                printf(" %-2.4g ", (double)alpha_dot_int(i));
//            }
//            printf("\n");
//
//            printf("T_bw: \n");
//            for (size_t i = 0; i < 6; i++) {
//                for (size_t j = 0; j < 2; j++) {
//                    printf(" %-2.4g ", (double)am_T_betaw(i, j));
//                }
//                printf("\n");
//            }
//            printf("\n");

//            printf("tau_beta: \n");
//            for (size_t j = 0; j < 6; j++) {
//                printf(" %-2.4g ", (double)am_tau_beta(j));
//            }
//            printf("\n");
//
//            printf("tau_quad: \n");
//            for (size_t j = 0; j < 3; j++) {
//                printf(" %-2.4g ", (double)am_tau_quad(j));
//            }
//            printf("\n");

//            printf("g beta = \n");
//            for (int j = 0; j < 6; j++) {
//                printf(" %-2.4g ", (double)am_g_beta(j));
//            }
//            printf("\n");
//
//            printf("tau beta in = \n");
//            for (int j = 0; j < 6; j++) {
//                printf(" %-2.4g ", (double)am_tau_beta_in(j));
//            }
//            printf("\n");
//
//            printf("B_betaw: \n");
//            for (size_t i = 0; i < 6; i++) {
//                for (size_t j = 0; j < 2; j++) {
//                    printf(" %-2.4g ", (double)am_B_betaw(i, j));
//                }
//                printf("\n");
//            }
//            printf("\n");
//
//            printf("omega_xy_dot_ref: \n");
//            for (size_t j = 0; j < 2; j++) {
//                printf(" %-2.4g ", (double)am_omega_xy_dot_ref(j));
//            }
//            printf("\n");
//
//            printf("am_tau_omega_xy: \n");
//            for (size_t j = 0; j < 2; j++) {
//                printf(" %-2.4g ", (double)am_tau_omega_xy(j));
//            }
//            printf("\n");

//            printf("B: \n");
//            for (size_t i = 0; i < 10; i++) {
//                for (size_t j = 0; j < 10; j++) {
//                    printf(" %-2.4g ", (double)am_B_eta(i, j));
//                }
//                printf("\n");
//            }
//            printf("\n");


            _thrust_sp = am_thrust;
        /** Aggiorna Integrali*/
            //if (fabsf(am_thrust) > MIN_TAKEOFF_THRUST && !_motor_limits.lower_limit && !_motor_limits.upper_limit) { !!!!!!!!!
            //    if (!am_saturation_omega_tau_quad && !am_saturation_omega_tau_robot && !am_saturation_omega_thrust){
                    am_att_control_acc_i += am_omega_xy_dot_ref_int_add;
            //    }
            //}

    }else{
        math::Vector<3> _att_control_pd; // =
        math::Vector<3> am_tau_pd_aux = Bww*am_att_control_acc_pd;
        for (int i = 0; i < 3; i++) {
            _att_control_pd(i) = ControlToActControl_tau(i)*am_tau_pd_aux(i);
        }
        //math::Vector<3> _att_control_pd = (Bww*am_att_control_acc_pd);
        _att_control = _att_control_pd  + _rates_int ;
    }


	_rates_sp_prev = _rates_sp;
	_rates_prev = rates;

	/* update integral only if not saturated on low limit and if motor commands are not saturated */
	if (_thrust_sp > MIN_TAKEOFF_THRUST && !_motor_limits.lower_limit && !_motor_limits.upper_limit) {

		for (int i = 0; i < 3; i++) {
			if (fabsf(_att_control(i)) < _thrust_sp) {
                //float rate_i = _rates_int(i) +  _params.rate_i(i) * rates_err(i) * dt;
				//float rate_i = _rates_int(i) + ControlToActControl_tau(i) * _params.rate_i(i) * rates_err(i) * dt;
				float rate_i = _rates_int(i);
				for (int j = 0; j < 3; j++) {
                     rate_i += ControlToActControl_tau(i) * (Bww(i,j)*_att_control_acc_i_add(j));
                     //rate_i += (Bww(i,j)*_att_control_acc_i(j));
                }
                /** RR* .................... */
				if (PX4_ISFINITE(rate_i) && rate_i > -RATES_I_LIMIT && rate_i < RATES_I_LIMIT &&
				    _att_control(i) > -RATES_I_LIMIT && _att_control(i) < RATES_I_LIMIT) {
					_rates_int(i) = rate_i;
				}
			}
		}
	}
return;
}

void
MulticopterAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	mc_att_control::g_control->task_main();
}

void
MulticopterAttitudeControl::compute_final_torques(float dt)
{
/** Ho B_csi, g_csi, T_bw. Dentro la funzione mi si deve passare tau_b_inertia,beta_dot_ref, omega_dot_ref_xy */

/** Input: */
    /** Da ORB */
    bool updated;
    orb_check(_am_u_tbeta_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(am_u_tbeta), _am_u_tbeta_sub, &_am_u_tbeta);
    }
    orb_check(_g_eta_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(g_matrix), _g_eta_sub, &_g_eta);
    }
    orb_check(_B_eta_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(B_matrix), _B_eta_sub, &_B_eta);
    }
    orb_check(_T_bw_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(T_bw_matrix), _T_bw_sub, &_T_bw);
    }
    orb_check(_csi_dot_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(csi_dot), _csi_dot_sub, &_csi_dot);
    }
    orb_check(_csi_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(csi), _csi_sub, &_csi);
    }


    for (int i = 0; i < 8; ++i) {
        am_u_tbeta(i)=_am_u_tbeta.am_u_tbeta[i];
        am_u_tbeta_int_add(i)=_am_u_tbeta.am_u_tbeta_int_add[i];
        }

    for (int i = 0; i < 6; ++i) {
        beta(i)=_am_u_tbeta.am_beta[i];
        beta_p(i)=_am_u_tbeta.am_beta_p[i];
        }

    for (int i = 0; i < 10; ++i) {
        am_g_eta(i)=_g_eta.g[i];
        for (int j = 0; j < 10; ++j) {
            am_B_eta(i,j)=_B_eta.B[i+10*j];
        }
    }

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 2; ++j) {
            am_T_betaw(i,j)=_T_bw.T_bw[i+6*j];
        }
    }

    alpha_p0(0)=_csi.csi[2]; //!!!!
    for (int i = 0; i < 5; ++i) {
        alpha_p0(i+1)=_csi.csi[i+5];
    }

    alpha_0(0)=_csi_dot.csi_dot[2]; //!!!
    for (int i = 0; i < 5; ++i) {
        alpha_0(i+1)=_csi_dot.csi_dot[i+5];
    }


    orb_check(_Bt_tb_i_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(Bt_tb_i_matrix), _Bt_tb_i_sub, &_Bt_tb_i);
    }
    orb_check(_Bb_tb_i_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(Bb_tb_i_matrix), _Bb_tb_i_sub, &_Bb_tb_i);
    }
    orb_check(_Bb_tb_i_pinv_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(Bb_tb_i_pinv_matrix), _Bb_tb_i_pinv_sub, &_Bb_tb_i_pinv);
    }
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 2; ++j) {
            am_Bt_tb_i(i,j)=_Bt_tb_i.Bt_tb_i[i+8*j];
        }
    }

    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 6; ++j) {
            am_Bb_tb_i(i,j)=_Bb_tb_i.Bb_tb_i[i+8*j];
        }
    }

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 8; ++j) {
            am_Bb_tb_i_pinv(i,j)=_Bb_tb_i_pinv.Bb_tb_i_pinv[i+6*j];
        }
    }

/** Extract Submatrices: */
    //C_betav =
    //C_wv =
        /** am_g_t; */
    for (int i = 0; i < 2; i++) {
            am_g_t(i) = am_g_eta(i);
    }
    for (int j = 2; j < 4; j++) {
        am_g_w(j-2) = am_g_eta(j);
    }
    for (int j = 4; j < 10; j++) {
        am_g_beta(j-4) = am_g_eta(j);
    }

    for (int i = 2; i < 4; i++) {
        for (int j = 2; j < 4; j++) {
            am_B_ww(i-2,j-2) = am_B_eta(i,j);
        }
    }
    for (int i = 4; i < 10; i++) {
        for (int j = 2; j < 4; j++) {
            am_B_betaw(i-4,j-2) = am_B_eta(i,j);
            am_B_wbeta(j-2,i-4) = am_B_eta(i,j);
        }
    }

    /** am_B_tb_i; */
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            if(j<2){
                am_B_tb_i(i,j) = am_Bt_tb_i(i,j);
            }
            else{
                am_B_tb_i(i,j) = am_Bb_tb_i(i,j-2);
                if (i>1){
                    am_Bbb_tb_i(i-2,j-2) = am_Bb_tb_i(i,j-2);
                }
            }
        }
    }


/** Calcola le tau_beta_in con Bb_tb_i, Bb_tb_i_pinv, g. */
//    for (int i = 0; i < 6; ++i) {
//        am_tau_beta_in(i) = am_u_tbeta(i+2); //am_Bb_tb_i_pinv*(am_u_tbeta + am_Bt_tb_i*am_g_t);
//    }

//    am_T_betaw_sq = am_T_betaw_t*am_T_betaw;
//    am_T_betaw_sq_inv(0,0) = am_T_betaw_sq(1,1); am_T_betaw_sq_inv(0,1) = -am_T_betaw_sq(0,1);
//    am_T_betaw_sq_inv(1,0) = -am_T_betaw_sq(1,0); am_T_betaw_sq_inv(1,1) = am_T_betaw_sq(0,0);
//    det_Tbetaw = (am_T_betaw_sq(0,0)*am_T_betaw_sq(1,1))-(am_T_betaw_sq(1,0)*am_T_betaw_sq(0,1));
//    am_T_betaw_sq_inv = am_T_betaw_sq_inv/det_Tbetaw;

    /* construct attitude setpoint rotation matrix */
    math::Matrix<3, 3> R_sp;
    R_sp.set(_v_att_sp.R_body);
    math::Vector<3>am_rpy_sp;
    am_rpy_sp = R_sp.to_euler();


    am_phi_xy_r(0)= am_rpy_sp(0); //_csi.csi[3];
    am_phi_xy_r(1)= 0.0f; //am_rpy_sp(1); //_csi.csi[4];

    omega_xy_r(0)= _rates_sp(0);//_ctrl_state.roll_rate;
    omega_xy_r(1)= 0.0f; //_rates_sp(1);//_ctrl_state.pitch_rate;

    alpha_p = alpha_p0 - beta_p;
    alpha_fb = (-am_T_betaw*am_phi_xy_r - alpha_p)*Kpp_alpha;
    err_alpha = beta - alpha_0 - am_T_betaw*omega_xy_r + alpha_fb;
    alpha_dot_int_add = err_alpha*Kiv_alpha*dt;
    alpha_dot = err_alpha*Kpv_alpha + alpha_dot_int;
    alpha_dot_int += alpha_dot_int_add;

    for (int i = 0; i < 6; ++i) {
        am_u_tbeta(i+2) += alpha_dot(i);
    }

//    //err_omega_csi = phi_r - am_T_betaw_sq_inv*am_T_betaw_t*csi_alpha; //usa riferimento, non valore attuale
//    //omega_alpha_fb = Kp_w_alpha*err_omega_csi;
//    err_omega_alpha_pos = (-am_phi_xy_r - am_T_betaw_sq_inv*am_T_betaw_t*alpha_pos)*Kp_alpha_pos;
//
//    err_omega_alpha = err_omega_alpha_pos - omega_xy_r - am_T_betaw_sq_inv*am_T_betaw_t*alpha;
//
//    err_omega_alpha_int_add = err_omega_alpha*Ki_alpha*dt;
//
//    am_u_tbeta_aux = am_T_betaw*(err_omega_alpha*Kp_alpha+err_omega_alpha_int);
//
//    //if (!_am_flag.am_saturation_utb_tau_quad && !_am_flag.am_saturation_utb_tau_robot && !_am_flag.am_saturation_utb_thrust){
//        err_omega_alpha_int += err_omega_alpha_int_add;
//    //}
//
//    for (int i = 0; i < 6; ++i) {
//        am_u_tbeta(i+2) += am_u_tbeta_aux(i);
//    }

    am_tau_beta_in = am_Bb_tb_i_pinv*(am_u_tbeta + am_Bt_tb_i*am_g_t);

    /** am_effec_tau; */
    for (int i = 0; i < 8; i++) {
        if(i<2){
            am_effec_tau(i) = -am_g_t(i);
        }
        else{
            am_effec_tau(i) = am_tau_beta_in(i-2);
        }
    }

    /** Calcola le tb_dot_ref con Bb_tb_i, Bt_tb_i, g, tau_beta_in. */
    am_beta_dot_ref = am_Bb_tb_i.transposed()*am_effec_tau;

    am_omega_xy_dot_ref(1) = 0; // !!!

    /** Tau_beta = tau_b_inertia + (C_betav + g_beta) + (B_betaw * omega_dot_ref) */
    am_tau_beta = am_g_beta + am_tau_beta_in + am_B_betaw*am_omega_xy_dot_ref;

//    printf("tau_beta: \n");
//    for (size_t j = 0; j < 6; j++) {
//        printf(" %-2.4g ", (double)am_tau_beta(j));
//    }
//    printf("\n");
//
//    printf("g beta = \n");
//    for (int j = 0; j < 6; j++) {
//        printf(" %-2.4g ", (double)am_g_beta(j));
//    }
//    printf("\n");
//
//    printf("tau beta in = \n");
//    for (int j = 0; j < 6; j++) {
//        printf(" %-2.4g ", (double)am_tau_beta_in(j));
//    }
//    printf("\n");
//
//    printf("B_betaw: \n");
//    for (size_t i = 0; i < 6; i++) {
//        for (size_t j = 0; j < 2; j++) {
//            printf(" %-2.4g ", (double)am_B_betaw(i, j));
//        }
//        printf("\n");
//    }
//    printf("\n");
//
//    printf("omega_xy: \n");
//    for (size_t j = 0; j < 2; j++) {
//        printf(" %-2.4g ", (double)am_omega_xy_dot_ref(j));
//    }
//    printf("\n");
//    printf("am_B_betaw*am_omega_xy_dot_ref: \n");
//    for (size_t j = 0; j < 2; j++) {
//        printf(" %-2.4g ", (double)am_omega_xy_dot_ref(j));
//    }
//    printf("\n");

    /** Tau_omega_xy = (B_ww * omega_dot_ref_xy) + (B_wbeta * beta_dot_ref) + (C_wv + g_w) */
    am_tau_omega_xy = am_B_ww*am_omega_xy_dot_ref + (am_B_wbeta*am_beta_dot_ref) +  am_g_w;

    am_T_betaw_t = am_T_betaw.transposed();

    /** Restituisce tau_quad, Thrust, Tau_robot*/
    const float am_thr_max = 0.9f; // CONTROLLA!!!!!
    const float am_thr_min = 0.1f; // CONTROLLA!!!!!
    const float am_tau_quad_max = 0.3f;
    const float am_tau_robot_max = 1.0f; //!!!

    /** Restituisce Tau_quad*/

            am_tau_quad(0)  =   ControlToActControl_tau(0)*am_tau_omega_xy(0);
            am_tau_quad(1)  =   ControlToActControl_tau(1)*am_tau_omega_xy(1);

        /** Tau_omega_xy_0 = Tau_omega_xy + T_bw^T * Tau_beta; */
        for (int j = 0; j < 6; j++) {
            am_tau_quad(0) += ControlToActControl_tau(0)*am_T_betaw_t(0,j)*am_tau_beta(j);
            am_tau_quad(1) += ControlToActControl_tau(1)*am_T_betaw_t(1,j)*am_tau_beta(j);
        }

        am_tau_quad(1) = 0.0f; //!!!

     //am_tau_beta_rm = am_B_betaw*am_omega_xy_dot_ref*(1.0f-W_w2beta);
    am_tau_beta -= am_B_betaw*am_omega_xy_dot_ref*(1.0f-W_w2beta);

    /** Restituisce Thrust*/

    am_thrust = ControlToActControl_T*am_tau_beta(0);
    am_tau_quad(2)  =   ControlToActControl_tau(2)*am_tau_beta(1);

        /** CONTROLLO su thrust max */
        if (fabsf(am_thrust) > am_thr_max){
            am_thrust = -am_thr_max;
        }
        else if (fabsf(am_thrust) < am_thr_min){
            am_thrust = -am_thr_min;
        }


    /** CONTROLLO su Tau_quad max */
        for (int i = 0; i < 3; i++) {
            if (am_tau_quad(i) > am_tau_quad_max){
                am_tau_quad(i)  = am_tau_quad_max;
            }
            else if (am_tau_quad(i)  < -am_tau_quad_max){
                am_tau_quad(i)  = -am_tau_quad_max;
            }
        }

    /** Restituisce Tau_robot*/
    for (int j = 0; j < 4; j++) {
            am_tau_robot(j) = am_tau_beta(j+2); //-am_tau_beta_rm(j+2); // + am_beta_dot_ref(j+2)*J_m;
    }

//    am_tau_robot(1) = 0.6f*((_csi_r.csi_r[7] - _csi.csi[7])*6.0f - _csi_dot.csi_dot[7]) + am_g_beta(3) ;
//    am_tau_robot(2) = 0.6f*((_csi_r.csi_r[8] - _csi.csi[8])*6.0f - _csi_dot.csi_dot[8]) + am_g_beta(4);

    /** CONTROLLO su Tau_robot max */
    for (int i = 0; i < 4; i++) {
        if (am_tau_robot(i) > am_tau_robot_max){
            am_tau_robot(i)  = am_tau_robot_max;
        }
        else if (am_tau_robot(i)  < -am_tau_robot_max){
            am_tau_robot(i)  = -am_tau_robot_max;
        }
    }

    /** Saturation of Integral Action */

        /** Effect of am_u_tbeta_Z_int_add */
            float am_u_tbeta_Z_int_add = am_u_tbeta_int_add(2);
            am_u_tbeta_int_add_noZ = am_u_tbeta_int_add;
            am_u_tbeta_int_add_noZ(2) = 0.0f;

                for (int j = 0; j < 6; j++) {
                    am_BZ_tb_i_pinv(j) = am_Bb_tb_i_pinv(j,2);
                }

            am_tau_utb_Z_aux = am_BZ_tb_i_pinv*am_u_tbeta_Z_int_add;

            am_tau_quad_int_add_aux = (am_B_wbeta*am_Bbb_tb_i + am_T_betaw_t)*am_tau_utb_Z_aux;
            am_tau_quad_int_add(0)  =   am_tau_quad(0) + ControlToActControl_tau(0)*am_tau_quad_int_add_aux(0);
            am_tau_quad_int_add(1)  =   am_tau_quad(1) + ControlToActControl_tau(1)*am_tau_quad_int_add_aux(1);
            am_tau_beta_int_add     =   am_tau_beta + am_tau_utb_Z_aux;

            /** SATURA?*/
            // TODO
                am_saturation_utbZ_thrust = false;
                if (ControlToActControl_T*fabsf(am_tau_beta_int_add(0))>am_thr_max){
                        am_saturation_utbZ_thrust = true;
                }

                am_saturation_utbZ_tau_quad = false;
                if (fabsf(am_thrust) > MIN_TAKEOFF_THRUST && !_motor_limits.lower_limit && !_motor_limits.upper_limit) {
                    for (int j = 0; j < 2; j++) {
                        if (fabsf(am_tau_quad_int_add(j))>am_tau_quad_max || fabsf(am_tau_quad_int_add(j)) > fabsf(am_thrust)){
                            am_saturation_utbZ_tau_quad = true;
                        }
                    }
                        float am_tau_yaw_aux = fabsf(ControlToActControl_tau(2)*am_tau_beta_int_add(1));
                        if (am_tau_yaw_aux>am_tau_quad_max || fabsf(am_tau_yaw_aux) > fabsf(am_thrust)){
                            am_saturation_utbZ_tau_quad = true;
                        }
                }

                am_saturation_utbZ_tau_robot = false;
                for (int j = 2; j < 6; j++) {
                    if (fabsf(am_tau_beta_int_add(j))>am_tau_robot_max){
                        am_saturation_utbZ_tau_robot = true;
                    }
                }

        /** Effect of omega_dot_ref */

                am_tau_quad_int_add_aux = (am_B_ww + am_T_betaw_t*am_B_betaw)*am_omega_xy_dot_ref_int_add;
                if (!am_saturation_utbZ_thrust && !am_saturation_utbZ_tau_robot && !am_saturation_utbZ_thrust){
                    am_tau_quad_int_add(0)  +=   ControlToActControl_tau(0)*am_tau_quad_int_add_aux(0);
                    am_tau_quad_int_add(1)  +=   ControlToActControl_tau(1)*am_tau_quad_int_add_aux(1);
                    am_tau_beta_int_add     +=   am_B_betaw*am_omega_xy_dot_ref_int_add;
                }else{
                    am_tau_quad_int_add(0)  =   am_tau_quad(0) + ControlToActControl_tau(0)*am_tau_quad_int_add_aux(0);
                    am_tau_quad_int_add(1)  =   am_tau_quad(1) + ControlToActControl_tau(1)*am_tau_quad_int_add_aux(1);
                    am_tau_beta_int_add     =   am_tau_beta + am_B_betaw*am_omega_xy_dot_ref_int_add;
                }


            /** SATURA?*/
                //TODO
                am_saturation_omega_thrust = false;

                if (ControlToActControl_T*fabsf(am_tau_beta_int_add(0))>am_thr_max){
                        am_saturation_omega_thrust = true;
                }
                am_saturation_omega_tau_quad = false;
                if (fabsf(am_thrust) > MIN_TAKEOFF_THRUST && !_motor_limits.lower_limit && !_motor_limits.upper_limit) {
                    for (int j = 0; j < 2; j++) {
                        if (fabsf(am_tau_quad_int_add(j))>am_tau_quad_max || fabsf(am_tau_quad_int_add(j)) > fabsf(am_thrust) ){
                            am_saturation_omega_tau_quad = true;
                        }
                    }
                    float am_tau_yaw_aux = fabsf(ControlToActControl_tau(2)*am_tau_beta_int_add(1));
                        if (am_tau_yaw_aux>am_tau_quad_max || fabsf(am_tau_yaw_aux) > fabsf(am_thrust)){
                            am_saturation_omega_tau_quad = true;
                        }
                }

                am_saturation_omega_tau_robot = false;
                for (int j = 2; j < 6; j++) {
                    if (fabsf(am_tau_beta_int_add(j))>am_tau_robot_max){
                        am_saturation_omega_tau_robot = true;
                    }
                }

        /** Effect of tbeta_dot_ref */
                 if (am_saturation_omega_tau_quad || am_saturation_omega_tau_robot || am_saturation_omega_thrust){
                     // Leva l'incremento di coppia dovuto a omega
                    am_tau_quad_int_add(0)  -=   ControlToActControl_tau(0)*am_tau_quad_int_add_aux(0);
                    am_tau_quad_int_add(1)  -=   ControlToActControl_tau(1)*am_tau_quad_int_add_aux(1);
                    am_tau_beta_int_add     -=   am_B_betaw*am_omega_xy_dot_ref_int_add;
                }

                math::Vector<6> am_tau_utb_aux = am_Bb_tb_i_pinv*am_u_tbeta_int_add_noZ;
                    am_tau_quad_int_add_aux = (am_B_wbeta*am_Bbb_tb_i + am_T_betaw_t)*am_tau_utb_aux;

                am_tau_quad_int_add(0)  +=   ControlToActControl_tau(0)*am_tau_quad_int_add_aux(0);
                am_tau_quad_int_add(1)  +=   ControlToActControl_tau(1)*am_tau_quad_int_add_aux(1);
                am_tau_beta_int_add     +=   am_tau_utb_aux;

            /** SATURA?*/
            // TODO
                am_saturation_utb_thrust = false;
                if (ControlToActControl_T*fabsf(am_tau_beta_int_add(0))>am_thr_max){
                        am_saturation_utb_thrust = true;
                }

                am_saturation_utb_tau_quad = false;
                if (fabsf(am_thrust) > MIN_TAKEOFF_THRUST && !_motor_limits.lower_limit && !_motor_limits.upper_limit) {
                    for (int j = 0; j < 2; j++) {
                        if (fabsf(am_tau_quad_int_add(j))>am_tau_quad_max || fabsf(am_tau_quad_int_add(j)) > fabsf(am_thrust)){
                            am_saturation_utb_tau_quad = true;
                        }
                    }
                        float am_tau_yaw_aux = fabsf(ControlToActControl_tau(2)*am_tau_beta_int_add(1));
                        if (am_tau_yaw_aux>am_tau_quad_max || fabsf(am_tau_yaw_aux) > fabsf(am_thrust)){
                            am_saturation_utb_tau_quad = true;
                        }
                }

                am_saturation_utb_tau_robot = false;
                for (int j = 2; j < 6; j++) {
                    if (fabsf(am_tau_beta_int_add(j))>am_tau_robot_max){
                        am_saturation_utb_tau_robot = true;
                    }
                }

                _am_flag.am_saturation_utbZ_tau_quad = am_saturation_utbZ_tau_quad;
                _am_flag.am_saturation_utbZ_tau_robot = am_saturation_utbZ_tau_robot;
                _am_flag.am_saturation_utbZ_thrust = am_saturation_utbZ_thrust;
                _am_flag.am_saturation_utb_tau_quad = am_saturation_utb_tau_quad;
                _am_flag.am_saturation_utb_tau_robot = am_saturation_utb_tau_robot;
                _am_flag.am_saturation_utb_thrust = am_saturation_utb_thrust;

                for (int i = 0; i < 4; ++i) {
                    _am_tau.tau_robot[i]=am_tau_robot(i);
                }

                if (_am_flag_pub != nullptr) {
                        orb_publish(ORB_ID(am_flag), _am_flag_pub, &_am_flag);
                    } else {
                        _am_flag_pub = orb_advertise(ORB_ID(am_flag), &_am_flag);
                    }
                if (_am_tau_pub != nullptr) {
                        orb_publish(ORB_ID(am_tau), _am_tau_pub, &_am_tau);
                    } else {
                        _am_tau_pub = orb_advertise(ORB_ID(am_tau), &_am_tau);
                    }
return;
}

void
MulticopterAttitudeControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));

    /*MC----------*/
	_g_eta_sub = orb_subscribe(ORB_ID(g_matrix));
    _B_eta_sub = orb_subscribe(ORB_ID(B_matrix));
    _T_bw_sub = orb_subscribe(ORB_ID(T_bw_matrix));
    _Bb_tb_i_sub = orb_subscribe(ORB_ID(Bb_tb_i_matrix));
    _Bt_tb_i_sub = orb_subscribe(ORB_ID(Bt_tb_i_matrix));
    _Bb_tb_i_pinv_sub = orb_subscribe(ORB_ID(Bb_tb_i_pinv_matrix));
    _am_u_tbeta_sub = orb_subscribe(ORB_ID(am_u_tbeta));
    _csi_r_sub = orb_subscribe(ORB_ID(csi_r));
    _csi_sub = orb_subscribe(ORB_ID(csi));
    _csi_dot_sub = orb_subscribe(ORB_ID(csi_dot));

    /*MC----------*/

	/* initialize parameters cache */
	parameters_update();

	/* wakeup source: vehicle attitude */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _ctrl_state_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		/* wait for up to 100ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
		    printf("[Attitude] pret == 0\n");
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		bool updated;
	    orb_check(_csi_r_sub, &updated);
	    if (updated) {
	        orb_copy(ORB_ID(csi_r), _csi_r_sub, &_csi_r);
	    }

		/* run controller on attitude changes */
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* copy attitude and control state topics */
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

			/* check for updates in other topics */
			parameter_update_poll();
			vehicle_control_mode_poll();
			arming_status_poll();
			vehicle_manual_poll();
			vehicle_status_poll();
			vehicle_motor_limits_poll();

			/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
			 * or roll (yaw can rotate 360 in normal att control).  If both are true don't
			 * even bother running the attitude controllers */
			if (_vehicle_status.main_state == vehicle_status_s::MAIN_STATE_RATTITUDE) {
				if (fabsf(_manual_control_sp.y) > _params.rattitude_thres ||
				    fabsf(_manual_control_sp.x) > _params.rattitude_thres) {
					_v_control_mode.flag_control_attitude_enabled = false;
				}
			}

			if (_v_control_mode.flag_control_attitude_enabled) {

				if (_ts_opt_recovery == nullptr) {
					// the  tailsitter recovery instance has not been created, thus, the vehicle
					// is not a tailsitter, do normal attitude control
					control_attitude(dt);

				} else {
					vehicle_attitude_setpoint_poll();
					_thrust_sp = _v_att_sp.thrust;
					math::Quaternion q(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
					math::Quaternion q_sp(&_v_att_sp.q_d[0]);
					_ts_opt_recovery->setAttGains(_params.att_p, _params.yaw_ff);
					_ts_opt_recovery->calcOptimalRates(q, q_sp, _v_att_sp.yaw_sp_move_rate, _rates_sp);

					/* limit rates */
					for (int i = 0; i < 3; i++) {
						_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
					}
				}

				/* publish attitude rates setpoint */
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

				} else if (_rates_sp_id) {
					_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
				}

				//}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_control_mode.flag_control_manual_enabled) {
					/* manual rates control - ACRO mode */
					_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x,
								    _manual_control_sp.r).emult(_params.acro_rate_max);
					_thrust_sp = math::min(_manual_control_sp.z, MANUAL_THROTTLE_MAX_MULTICOPTER);

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}

				} else {
					/* attitude controller disabled, poll rates setpoint topic */
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
				}
			}

			//MC*
			if (!reset_int_flag && !_v_control_mode.flag_control_offboard_enabled) {
                printf("set flag att \n");
                reset_int_flag = true;
            }
			if (_v_control_mode.flag_control_rates_enabled) {
			    control_attitude_rates(dt);

				/* publish actuator controls */
				_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
				_actuators.control[1] = 0.0f; // (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
				_actuators.control[2] = 0.0f; // (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
				_actuators.control[3] = 0.25f; // (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
				_actuators.timestamp = hrt_absolute_time();
				_actuators.timestamp_sample = _ctrl_state.timestamp;

				_controller_status.roll_rate_integ = _rates_int(0);
				_controller_status.pitch_rate_integ = _rates_int(1);
				_controller_status.yaw_rate_integ = _rates_int(2);
				_controller_status.timestamp = hrt_absolute_time();
				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub != nullptr) {
						orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
						perf_end(_controller_latency_perf);

					} else if (_actuators_id) {
						_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
					}

				}
				/* publish controller status */
				if (_controller_status_pub != nullptr) {
					orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

				} else {
					_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
				}
			}
		}
		perf_end(_loop_perf);
	}
	_control_task = -1;
	return;
}

int
MulticopterAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&MulticopterAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (mc_att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		mc_att_control::g_control = new MulticopterAttitudeControl;

		if (mc_att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != mc_att_control::g_control->start()) {
			delete mc_att_control::g_control;
			mc_att_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (mc_att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete mc_att_control::g_control;
		mc_att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (mc_att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
