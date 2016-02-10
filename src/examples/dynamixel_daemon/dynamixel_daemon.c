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
	int csi_r_sub_fd = orb_subscribe(ORB_ID(csi_r));

	struct csi_s csi;
	memset(&csi, 0, sizeof(csi));
	orb_advert_t csi_pub_fd = orb_advertise(ORB_ID(csi), &csi);
	int csi_sub_fd = orb_subscribe(ORB_ID(csi));

	struct csi_dot_s csi_dot;
    memset(&csi_dot, 0, sizeof(csi_dot));
    orb_advert_t csi_dot_pub_fd = orb_advertise(ORB_ID(csi_dot), &csi_dot);
    int csi_dot_sub_fd = orb_subscribe(ORB_ID(csi_dot));

    int *state;

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
	    { .fd = csi_r_sub_fd,   .events = POLLIN },
	};

	//Dynamixel Initialization
    int res;
    res = dxl_initialize(1,9);
    if (!res)
        printf("Error in initialization Dynamixel!\n");
    printf("Initialization result: %d \n",res);

    //Set Moving Speed
    dxl_write_word(254,32,30);

    //SyncWrite Initialization
    int NUM_ACTUATOR = 5;
    int i;
    int id[5];
    int GoalPos[5];
    for( i=0; i<NUM_ACTUATOR; i++ )
    {
        id[i] = i+1;
    }

    sleep(3);
    //Main Loop
	while (!thread_should_exit) {
	    /* wait for sensor update of 1 file descriptor for 1000 ms */
	    int poll_ret = px4_poll(fds, 1, 1);

	    if ((poll_ret>0) & fds[0].revents & POLLIN) {
	        /* obtained data for the first file descriptor */
	        struct csi_r_s raw;
	        /* copy sensors raw data into local buffer */
	        orb_copy(ORB_ID(csi_r), csi_r_sub_fd, &raw);
	        //printf("[Dynamixel] SyncWrite: %g %g %g %g %g \n", (double)raw.csi_r[6], (double)raw.csi_r[7], (double)raw.csi_r[8], (double)raw.csi_r[9], (double)raw.csi_r[10]);

	        //Conversion from rad to bit
	        GoalPos[0] = (int)(raw.csi_r[6]*195.5696f+512);
	        GoalPos[1] = (int)(raw.csi_r[7]*195.5696f+512);
	        GoalPos[2] = (int)(raw.csi_r[8]*195.5696f+512);
	        GoalPos[3] = (int)(raw.csi_r[9]*195.5696f+512);
	        GoalPos[4] = (int)(raw.csi_r[10]*195.5696f+512);

	        //printf("[Dynamixel] GoalPos: %d %d %d %d %d \n", GoalPos[0], GoalPos[1], GoalPos[2], GoalPos[3], GoalPos[4]);

	        //SyncWrite
	        dxl_set_txpacket_id(254);
	        dxl_set_txpacket_instruction(131); //INST_SYNC_WRITE
	        dxl_set_txpacket_parameter(0, 30); //P_GOAL_POSITION_L
	        dxl_set_txpacket_parameter(1, 2);
	        for( i=0; i<NUM_ACTUATOR; i++ )
	        {
	            dxl_set_txpacket_parameter(2+3*i, id[i]);
	            dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(GoalPos[i]));
	            dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(GoalPos[i]));
	        }
	        dxl_set_txpacket_length((2+1)*NUM_ACTUATOR+4);
	        dxl_txrx_packet();
	    }

	    //Read present position and speed
	    orb_copy(ORB_ID(csi), csi_sub_fd, &csi);
	    orb_copy(ORB_ID(csi_dot), csi_dot_sub_fd, &csi_dot);

	    //Read cycle
        for (int k = 1; k < 6; ++k) {
            state = dxl_read_state(k);
            //printf("%d: %5d, %5d, %5d", k,state[0], state[1], state[2]);
            if (dxl_get_result() != 1) {
                printf( "CommStatus %d \n",dxl_get_result());
                dxl_terminate();
                res = dxl_initialize(1,9);
                dxl_write_word(254,32,30);
            } else {
                csi.csi[k+5]=((float)state[0]-512)/195.5696f;
                if (state[1]>1023)
                    state[1]=-(float)(state[1]-1024);
                csi_dot.csi_dot[k+5]=(float)state[1]*0.0119f;
            }
        }
	    orb_publish(ORB_ID(csi), csi_pub_fd, &csi);
	    orb_publish(ORB_ID(csi_dot), csi_dot_pub_fd, &csi_dot);
	}

	warnx("[Dynamixel] exiting.\n");
	thread_running = false;

	return 0;
}
