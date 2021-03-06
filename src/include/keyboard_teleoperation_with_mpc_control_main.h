/*!*******************************************************************************************
 * \brief     Keyboard teleoperation with mpc control implementation file.
 * \authors   Alberto Rodelgo
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef KEYBOARD_TELEOPERATION_WITH_MPC_CONTROL_H
#define KEYBOARD_TELEOPERATION_WITH_MPC_CONTROL_H

#include <string>
#include "ros/ros.h"
#include <sstream>
#include <stdio.h>
#include <boost/thread/thread.hpp>
#include <curses.h>
#include <thread>
#include <locale.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "mav_msgs/RollPitchYawrateThrust.h"
#include "aerostack_msgs/FlightActionCommand.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_srvs/Empty.h"
#include <std_msgs/Int8.h>

//Inputs
#define ASCII_KEY_UP 65
#define ASCII_KEY_DOWN 66
#define ASCII_KEY_RIGHT 67
#define ASCII_KEY_LEFT 68

// Define controller commands define constants
#define CTE_SPEED (1.00)
#define CTE_POSE (1.00)
#define CTE_COMMANDS (0.20)
#define CTE_COMMANDS_TIME (0.50)
#define CTE_ALTITUDE (1.00)
#define CTE_YAW (0.1)
//Loop rate
#define FREQ_INTERFACE 50.0

int miliseconds = CTE_COMMANDS_TIME * 1000;
const int GROUND_SPEED = 1;
const int POSE = 2;
const int ATTITUDE = 3;

const int LEFT = 1;
const int RIGHT = 2;
const int UP = 3;
const int DOWN = 4;
//Publishers
ros::Publisher pose_reference_publ;
ros::Publisher flightaction_pub;
//Subscribers
ros::Subscriber self_pose_sub;
ros::Subscriber ground_speed_sub;

//MSG
geometry_msgs::PoseStamped self_localization_pose_msg; 
geometry_msgs::PoseStamped motion_reference_pose_msg; 
geometry_msgs::PoseStamped motion_reference_pose_2_msg; 
geometry_msgs::TwistStamped self_speed_msg;
aerostack_msgs::FlightActionCommand flight_action_msg;



//Functions
void printoutPoseControls();
void toEulerianAngle(geometry_msgs::PoseStamped q, double *roll, double *pitch, double *yaw);
double current_commands_roll,current_commands_pitch,current_commands_yaw;
tf2::Quaternion q_rot;
void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void selfSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);


std::string drone_id_namespace;

#endif
