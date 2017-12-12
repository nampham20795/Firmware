/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file usv_control_main.cpp
 * Unmanned Surface Vehicle controller.
 *  
 * CDT12
 *
 * 
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_getopt.h>
#include <px4_defines.h>
#include <px4_log.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
//#include <poll.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#ifdef __PX4_NUTTX
#include <nuttx/fs/ioctl.h>
#endif

#include "systemlib/systemlib.h"
#include "systemlib/err.h"
#include "systemlib/param/param.h"
#include "drivers/drv_pwm_output.h"




#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>

/**
 *  Unmanned Surface Vehicle control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int usv_control_main(int argc, char *argv[]);



class USVControl
{
public:
	/**
	 * Constructor
	 */
	USVControl();

	/**
	 * Destructor, also kills the main task
	 */
	~USVControl();

	/**
	 * Start the usv control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int   	_control_task;      /**< task handle */
	
	int	_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	


	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */


	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_controller_latency_perf;

	
	TailsitterRecovery *_ts_opt_recovery;	/**< Computes optimal rates for tailsitter recovery */


	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	int		task_main();
};

namespace usv_control
{

USVControl	*g_control;
}

USVControl::USVControl() :

	_task_should_exit(false),
	_control_task(-1),

	/* subscriptions */
	_v_rates_sp_sub(-1),


	_v_rates_sp{},

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
	_ts_opt_recovery(nullptr)

{


}

USVControl::~USVControl()
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

	usv_control::g_control = nullptr;
}





void
USVControl::task_main_trampoline(int argc, char *argv[])
{
	usv_control::g_control->task_main();
}

int 
USVControl::task_main()
{

	/*
	 * do subscriptions
	 */
	
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	

	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	const char *dev= PWM_OUTPUT0_DEVICE_PATH;

 	 /* open for ioctl only */
  	int fd = px4_open(dev, 0);
  	if (fd < 0) {
      		PX4_ERR("can't open %s", dev);
      		return 1;
  	}


	while (!_task_should_exit) {

		poll_fds.fd = _v_rates_sp_sub;
    		int pret = px4_poll(&poll_fds, 1, 10);
    		

    		/* this is undesirable but not much we can do - might want to flag unhappy status */
   		if (pret < 0) {
      			warn("auv att ctrl: poll error %d, %d", pret, errno);
      			/* sleep a bit before next try */
      			usleep(100000);
      			continue;
    		}

    		perf_begin(_loop_perf);

    		//CDT12 debug: Comment for avoiding joystick change checking!!!
    		//if (poll_fds.revents & POLLIN) {
    		if (true){
                	orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
    			
        		
        		int pwm_value[6] = {1500, 1500, 1500, 1500, 1500, 1500 }; //debug, for testing approximation function

        		pwm_value[0] = 1500 +  (int)(50.0*(double)_v_rates_sp.roll);
        		pwm_value[1] = 1500 +  (int)(50.0*(double)_v_rates_sp.pitch);



        		for (unsigned i = 0; i < 6; i++) {  
                        
      				PX4_INFO("PWM_VALUE %d   %d", i+1, pwm_value[i]);
      				int ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value[i]);       

      				if (ret != OK) {
        				PX4_ERR("PWM_SERVO_SET(%d)", i);
        				return 1;
      				}                 
    			}

   	 		#ifdef __PX4_NUTTX
      			/* Trigger all timer's channels in Oneshot mode to fire
     	 		* the oneshots with updated values.	
      			*/
      			up_pwm_update();
    			#endif



   			 perf_end(_loop_perf);
  		}

  	}
	_control_task = -1;
	return 0;
        
}


int
USVControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("usv_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1700,
					   (px4_main_t)&USVControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int usv_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: usv_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (usv_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		usv_control::g_control = new USVControl;

		if (usv_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != usv_control::g_control->start()) {
			delete usv_control::g_control;
			usv_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (usv_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete usv_control::g_control;
		usv_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (usv_control::g_control) {
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
