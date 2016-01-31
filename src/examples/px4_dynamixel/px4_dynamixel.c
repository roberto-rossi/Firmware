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

#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <uORB/uORB.h>
#include "dynamixel.h"

__EXPORT int px4_dynamixel_main(int argc, char *argv[]);

int px4_dynamixel_main(int argc, char *argv[])
{
    /*   if (argc < 2) {
            printf("Missing command \n");
            return 1;
        }

    if (!strcmp(argv[1], "initialize")) {
        int res;
        res = dxl_initialize(1,1);
        if (!res)
            printf("Error in initialization Dynamixel!\n");
        printf("Initialization result: %d \n",res);
        return 0;
    }

    if (!strcmp(argv[1], "terminate")) {
        dxl_terminate();
        return 0;
    }

    if (!strcmp(argv[1], "readword")) {
        int res;
        res = dxl_read_word(1,36);
        printf("Present position: %d \n", res);
        return 0;
    }

    if (!strcmp(argv[1], "writeword")) {
        dxl_write_word(1,30,450);
        printf("Write position setpoint \n");
        return 0;
    }
  printf("Wrong command \n");
  */
//
    int res;
    int c = 0;
	printf("Test Dynamixel!\n");

    res = dxl_initialize(1,200);
    if (!res)
        printf("Error in initialization Dynamixel!\n");
    printf("Initialization result: %d \n",res);

	while (c<1){
	//dxl_write_word(1,32,10);
	//dxl_write_word(1,30,450);
	//dxl_write_word(1,32,100);
	c++;
    }
	res = dxl_read_word(1,36);
	printf("Read present position: %d \n",res);
//
	return 0;
}
