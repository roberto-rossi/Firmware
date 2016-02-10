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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <stdlib.h>

#include <uORB/uORB.h>

#include <uORB/topics/csi.h>
#include <uORB/topics/csi_dot.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Inizio programma di test!");

    /* advertise csi topic */
    struct csi_s csi;
    memset(&csi, 0, sizeof(csi));
    orb_advert_t csi_pub = orb_advertise(ORB_ID(csi), &csi);

    /* advertise csi_dot topic */
       struct csi_dot_s csi_dot;
       memset(&csi_dot, 0, sizeof(csi_dot));
       orb_advert_t csi_dot_pub = orb_advertise(ORB_ID(csi_dot), &csi_dot);

    for (int var = 0; var < 10; ++var) {
        for (int var = 0; var < 20; ++var) {
            for (int i = 0; i < 10; ++i) {
                csi.csi[i] = i+var;
                csi_dot.csi_dot[i] = i+var;
            }
            int err = orb_publish(ORB_ID(csi), csi_pub, &csi);
            orb_publish(ORB_ID(csi_dot), csi_dot_pub, &csi_dot);
            printf("%d \n",err);
            usleep(10000);
        }
    }

	PX4_INFO("exiting");

	return 0;
}
