/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file dynamixel_deamon.c
 *
 * @author Example User <mail@example.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <drivers/drv_hrt.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <poll.h>
#include <string.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include "dynamixel.h"
#include "dxl_hal.h"
#include <uORB/uORB.h>

#include <uORB/topics/csi.h>
#include <uORB/topics/csi_dot.h>
#include <uORB/topics/csi_r.h>
#include <uORB/topics/csi_dot_r.h>
#include <uORB/topics/dynamixel_state.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
__EXPORT int dynamixel_daemon_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int dynamixel_daemon_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int dynamixel_daemon_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("Dynamixel daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("dynamixel_daemon",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 dynamixel_daemon_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int dynamixel_daemon_thread_main(int argc, char *argv[])
{
	warnx("[Dynamixel] starting\n");
	thread_running = true;

	//uOrb Initialization
    struct csi_r_s csi_r;
    memset(&csi_r, 0, sizeof(csi_r));
	int csi_r_sub_fd = orb_subscribe(ORB_ID(csi_r));

    struct csi_dot_r_s csi_r_dot;
    memset(&csi_r_dot, 0, sizeof(csi_r_dot));
	int csi_r_dot_sub_fd = orb_subscribe(ORB_ID(csi_dot_r));

    struct dynamixel_state_s dynamixel_state;
    memset(&dynamixel_state, 0, sizeof(dynamixel_state));
    orb_advert_t dynamixel_state_pub_fd = orb_advertise(ORB_ID(dynamixel_state), &dynamixel_state);

    int *state;
    int res;
    double torque[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double torque_old[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double err_v[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double err_pos[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double err_v_old[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    int q_old[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
//    float q_dot_old[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    hrt_abstime Ts_prev = 0;
//    const float  c_dev = 60.0f; //60;

    double Kpp[5] = {4.0, 4.0, 4.0, 4.0, 4.0}; 		
    double Kpv[5] = {0.5293, 0.35055, 0.5293, 0.5293, 0.5293}; 
    double Kiv[5] = {0.936, 0.6199, 0.8268, 0.8268, 0.8268}; 
    int bit_torque[5];
    //int reset_flag = 1;

    //Dynamixel Initialization
    res = dxl_initialize(1,9);
    printf("Initialization result: %d \n",res);

    //SyncWrite Initialization
    int NUM_ACTUATOR = 5;
    int i;
    int id[5];
    for( i=0; i<NUM_ACTUATOR; i++ )
    {
        id[i] = i+1;
    }

    //2 sec delay
    sleep(2);

    //Present speed = 0 e passaggio in wheel mode
    dxl_write_word(254,32,0);
    dxl_write_word(254,6,0);
    dxl_write_word(254,8,0);

    printf("Wheel Mode \n",res);
    sleep(1);

    //Inizializzazione tv_old
    struct timeval tv;
    gettimeofday (&tv, NULL);

    hrt_abstime t = hrt_absolute_time();
    float Ts = Ts_prev != 0 ? (t - Ts_prev) * 0.000001f : 0.02f;

    //Main Loop
	while (!thread_should_exit) {
        //Calculate Ts
        hrt_abstime t = hrt_absolute_time();
        Ts = Ts_prev != 0 ? (t - Ts_prev) * 0.000001f : 0.02f;
        Ts_prev = t;

	//Read position and speed from uOrb
        //Read cycle
        for (int k = 1; k < 6; ++k) { 
            state = dxl_read_state(k);
            if (dxl_get_result() != 1) {
                printf( "[Dynamixel] Errore %d \n",dxl_get_result());
                dxl_terminate();
                usleep(1000);
                res = dxl_initialize(1,9);
            } else {
                dynamixel_state.q[k-1]=(float)(state[0]-512)*0.005113281f;
                dynamixel_state.q_dot[k-1] = (float)(state[0] - q_old[k-1])*0.005113281f/((float)Ts*2);
                q_old[k-1] = state[0];
            }
        }

            orb_copy(ORB_ID(csi_r), csi_r_sub_fd, &csi_r);
            orb_copy(ORB_ID(csi_dot_r), csi_r_dot_sub_fd, &csi_r_dot);
            //PPI Controller
            for (int i = 0; i < 5; ++i) {
                //Calcolo errore di posizione
                err_pos[i]=(double)(csi_r.csi_r[i+6] - dynamixel_state.q[i]);
                //Calcolo errore velocità + FeedForward
                err_v[i] = err_pos[i]*Kpp[i] + (double)csi_r_dot.csi_r_dot[i+6] - (double)dynamixel_state.q_dot[i];
                //Controllore PI discretizzato con EI
                torque[i] = torque_old[i] + (err_v[i]-err_v_old[i])*Kpv[i] + err_v[i]*(double)Ts*Kiv[i];

                //Limit torque 1.5 Nm
                if (torque[i] > 1.4)
                    torque[i] = 1.4;
                if (torque[i] < -1.4)
                    torque[i] = -1.4;
                //Salvo per passo successivo
                torque_old[i]=torque[i];
                err_v_old[i]=err_v[i];
                //Conversione in bit
                bit_torque[i] = (int)(torque[i] / 1.5 * 1000.0);
                if (bit_torque[i]< 0)
                    bit_torque[i] = - bit_torque[i] + 1024;
        }

        //SyncWrite
        dxl_set_txpacket_id(254);
        dxl_set_txpacket_instruction(131); //INST_SYNC_WRITE
        dxl_set_txpacket_parameter(0, 32); //MOVING_SPEED_L
        dxl_set_txpacket_parameter(1, 2);
        for( i=0; i<NUM_ACTUATOR; i++ )
        {
            dxl_set_txpacket_parameter(2+3*i, id[i]);
            dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(bit_torque[i]));
            dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(bit_torque[i]));
        }
        dxl_set_txpacket_length((2+1)*NUM_ACTUATOR+4);
        dxl_txrx_packet();

        //Scrittura su uorb
        orb_publish(ORB_ID(dynamixel_state), dynamixel_state_pub_fd, &dynamixel_state);
	}

	dxl_write_word(254,32,0);
	dxl_terminate();

	warnx("[Dynamixel] exiting.\n");
	thread_running = false;
	return 0;
}
