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
 * @file hums_topic_logger.cpp
 *
 *
 * @author Ryan Stanley
 */

//#include <drivers/drv_hrt.h>
//#include <px4_platform_common/px4_config.h>
//#include <px4_platform_common/tasks.h>
//#include <px4_platform_common/posix.h>
//#include <unistd.h>
//#include <stdio.h>
//#include <poll.h>
//#include <string.h>
//#include <math.h>

//#include <px4_platform_common/px4_config.h>
//#include <px4_platform_common/tasks.h>
//#include <systemlib/err.h>

//#include <uORB/Subscription.hpp>
//#include <uORB/uORB.h>
//#include <uORB/topics/vehicle_attitude_setpoint.h>


//******************************
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/hums_nn_msg.h>
#include "hums_topic_logger.h"

int HumsTopicLogger::print_status()
{
    PX4_INFO("Running");
    // TODO: print additional runtime information about the state of the module

    return 0;
}

int HumsTopicLogger::custom_command(int argc, char *argv[])
{
    /*
    if (!is_running()) {
        print_usage("not running");
        return 1;
    }

    // additional custom commands can be handled like this:
    if (!strcmp(argv[0], "do-something")) {
        get_instance()->do_something();
        return 0;
    }
     */

    return print_usage("unknown command");
}


int HumsTopicLogger::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("hums_topic_logger",
                      SCHED_DEFAULT,
                      SCHED_PRIORITY_DEFAULT,
                      1024,
                      (px4_main_t)&run_trampoline,
                      (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

HumsTopicLogger *HumsTopicLogger::instantiate(int argc, char *argv[])
{
    int example_param = 0;
    bool example_flag = false;
    bool error_flag = false;

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    // parse CLI arguments
    while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'p':
            example_param = (int)strtol(myoptarg, nullptr, 10);
            break;

        case 'f':
            example_flag = true;
            break;

        case '?':
            error_flag = true;
            break;

        default:
            PX4_WARN("unrecognized flag");
            error_flag = true;
            break;
        }
    }

    if (error_flag) {
        return nullptr;
    }

    HumsTopicLogger *instance = new HumsTopicLogger(example_param, example_flag);

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

HumsTopicLogger::HumsTopicLogger(int example_param, bool example_flag)
    : ModuleParams(nullptr)
{
}

void HumsTopicLogger::run()
{

    PX4_INFO("starting HUMS topic publisher");

    // Example: run the loop synchronized to the sensor_combined topic publication
    /* subscribe to topics of interest */
    int angrates_sub_fd = orb_subscribe(ORB_ID(vehicle_angular_velocity)); //vehicle angular velocity
    int angrates_sp_sub_fd = orb_subscribe(ORB_ID(vehicle_rates_setpoint)); //vehicle angular velocity setpoints
    int act_controls_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0)); //actuator control values

//    int pub_interval = 1; //ms between published samples
//    orb_set_interval(angrates_sub_fd, pub_interval);
//    orb_set_interval(angrates_sp_sub_fd, pub_interval);
//    orb_set_interval(act_controls_sub_fd, pub_interval);

    px4_pollfd_struct_t fds[] = {
        { .fd = angrates_sub_fd,   .events = POLLIN },
        { .fd = angrates_sp_sub_fd,   .events = POLLIN },
        { .fd = act_controls_sub_fd,   .events = POLLIN }

        /* there could be more file descriptors here, in the form like:
         * { .fd = other_sub_fd,   .events = POLLIN },
         */
    };

    // initialize parameters
    parameters_update(true);

    /* create hums advertiser object */
    struct hums_nn_msg_s hums_nn;
    memset(&hums_nn, 0, sizeof(hums_nn));
    orb_advert_t hums_nn_pub = orb_advertise(ORB_ID(hums_nn_msg), &hums_nn);

    while (!should_exit()) {

        // wait for up to 1000ms for data
        int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

        if (pret == 0) {
            // Timeout: let the loop run anyway, don't do `continue` here

        } else if (pret < 0) {
            // this is undesirable but not much we can do
            PX4_ERR("poll error %d, %d", pret, errno);
            px4_usleep(50000);
            continue;

            //Publish hums topic whenever an update to field is received. Field chosen updates at 250 Hz so hums_topic will too
        } else if (fds[2].revents & POLLIN) {

            /* Create structure for copying topic data */
            struct vehicle_angular_velocity_s ang_vel;
            struct vehicle_rates_setpoint_s ang_vel_sp;
            struct actuator_controls_s act_control_sp;

            /* copy raw data into local buffers */
            orb_copy(ORB_ID(vehicle_angular_velocity), angrates_sub_fd, &ang_vel);
            orb_copy(ORB_ID(vehicle_rates_setpoint), angrates_sp_sub_fd, &ang_vel_sp);
            orb_copy(ORB_ID(actuator_controls), act_controls_sub_fd, &act_control_sp);

            //Time
            hums_nn.timestamp = hrt_absolute_time();

            //Angular velocity
            hums_nn.roll_rate = ang_vel.xyz[0];
            hums_nn.pitch_rate = ang_vel.xyz[1];
            hums_nn.yaw_rate = ang_vel.xyz[2];

            //Angular rates setpoint
            hums_nn.roll_rate_sp = ang_vel_sp.roll;
            hums_nn.pitch_rate_sp = ang_vel_sp.pitch;
            hums_nn.yaw_rate_sp = ang_vel_sp.yaw;

            //Actuator controls
            hums_nn.control[0] = act_control_sp.control[0];
            hums_nn.control[1] = act_control_sp.control[1];
            hums_nn.control[2] = act_control_sp.control[2];
            hums_nn.control[3] = act_control_sp.control[3];

            /* Publish hums_nn_msg topic */
            orb_publish(ORB_ID(hums_nn_msg), hums_nn_pub, &hums_nn);

        }

        parameters_update();
    }

    orb_unsubscribe(angrates_sub_fd);
    orb_unsubscribe(angrates_sp_sub_fd);
    orb_unsubscribe(act_controls_sub_fd);

}

void HumsTopicLogger::parameters_update(bool force)
{
    // check for parameter updates
    if (_parameter_update_sub.updated() || force) {
        // clear update
        parameter_update_s update;
        _parameter_update_sub.copy(&update);

        // update parameters from storage
        updateParams();
    }
}

int HumsTopicLogger::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("module", "hums_topic_logger");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
    PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int hums_topic_logger_main(int argc, char *argv[])
{
    return HumsTopicLogger::main(argc, argv);
}
