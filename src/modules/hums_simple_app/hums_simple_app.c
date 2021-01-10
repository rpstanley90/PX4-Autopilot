/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file hums_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/hums_nn_msg.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>


__EXPORT int hums_simple_app_main(int argc, char *argv[]);

int hums_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Starting HUMS topic app");

	/* subscribe to vehicle_attitude topic */
	int attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	
	/* subscribe to vehicle_attitude_setpoint topic */
	int att_sp_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	
	/* limit the update rate to 100 Hz */
	//orb_set_interval(attitude_sub_fd, 10);
	//orb_set_interval(att_sp_sub_fd, 10);
	
	/* create hums advertiser object */
	struct hums_nn_msg_s hums_nn;
	memset(&hums_nn, 0, sizeof(hums_nn));
	orb_advert_t hums_nn_pub = orb_advertise(ORB_ID(hums_nn_msg), &hums_nn);

	/* one could wait for multiple topics with this technique */
	px4_pollfd_struct_t fds[] = {
		{ .fd = attitude_sub_fd,   .events = POLLIN },
		{ .fd = att_sp_sub_fd,   .events = POLLIN }
		
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;
	int max_loop_counter = 10000;

	for (int i = 0; i < max_loop_counter; i++) {
		
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			/* Check and see if new data has arrived for both topics */ 
			if (fds[0].revents & POLLIN && fds[1].revents) {
				
				//PX4_INFO("Got data!");
				
				/* Create structure for copying attitude topic data */
				struct vehicle_attitude_s att;
				
				/* Create structure for copying attitude setpoint topic data */
				struct vehicle_attitude_setpoint_s att_sp;
				
				/* copy attitude raw data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude), attitude_sub_fd, &att);
				
				/* copy attitude raw data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub_fd, &att_sp);

				/* Add copied data to hums_nn structure fields */
				hums_nn.timestamp = hrt_absolute_time();
				
                /*hums_nn.q[0] = att.q[0];
				hums_nn.q[1] = att.q[1];
				hums_nn.q[2] = att.q[2];
				hums_nn.q[3] = att.q[3];
				
				hums_nn.q_d[0] = att_sp.q_d[0];
				hums_nn.q_d[1] = att_sp.q_d[1];
				hums_nn.q_d[2] = att_sp.q_d[2];
                hums_nn.q_d[3] = att_sp.q_d[3];*/

				/* Publish hums_nn_msg topic */ 
				orb_publish(ORB_ID(hums_nn_msg), hums_nn_pub, &hums_nn);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}
