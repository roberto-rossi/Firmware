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
    int res;
    int c = 0;
	printf("Test Dynamixel!\n");

    res = dxl_initialize(1,9);
    if (!res)
        printf("Error in initialization Dynamixel!\n");
    printf("Initialization result: %d \n",res);

    dxl_write_word(254,32,15);

      int NUM_ACTUATOR = 5;
      int i;
      int id[5];
      int GoalPos;

        for( i=0; i<NUM_ACTUATOR; i++ )
        {
            id[i] = i+1;
        }

        dxl_set_txpacket_id(254);
        dxl_set_txpacket_instruction(131); //INST_SYNC_WRITE
        dxl_set_txpacket_parameter(0, 30); //P_GOAL_POSITION_L
        dxl_set_txpacket_parameter(1, 2);
        for( i=0; i<NUM_ACTUATOR; i++ )
        {
            dxl_set_txpacket_parameter(2+3*i, id[i]);
            GoalPos = (int)(500);
            dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(GoalPos));
            dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(GoalPos));
        }
        dxl_set_txpacket_length((2+1)*NUM_ACTUATOR+4);

        dxl_txrx_packet();

        printf("Fine SyncWrite \n");

    int SyncRead[15];

    while (c<1000){

        for (int k = 1; k < 6; ++k) {
            int *state;
            state = dxl_read_state(k);
            SyncRead[0+(k-1)*3]=state[0];
            SyncRead[1+(k-1)*3]=state[1];
            SyncRead[2+(k-1)*3]=state[2];
            //printf("%d: %5d, %5d, %5d \n", k,state[0], state[1], state[2]);
            if (dxl_get_result() != 1)
            printf( "CommStatus %d Pos: %d \n",dxl_get_result(),state[0] );
        }

        for (int k = 0; k < 15; ++k) {
            printf("%d ",SyncRead[k]);
        }
        printf("\n");

	c++;
    }
	return 0;
}
