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

#include "../include/keyboard_teleoperation_with_mpc_control_main.h"

void spinnerThread(){
  ros::spin();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "KEYBOARD TELEOPERATION WITH MPC CONTROL");
  ros::NodeHandle n("~");
  n.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");
  std::thread thr(&spinnerThread);

  // ncurses initialization
  setlocale(LC_ALL, "");
  std::setlocale(LC_NUMERIC, "C");
  initscr();
  start_color();
  use_default_colors();  
  curs_set(0);
  noecho();
  nodelay(stdscr, TRUE);
  erase();
  refresh();
  init_pair(1, COLOR_BLUE, -1);
  init_pair(2, COLOR_GREEN, -1);
  init_pair(3, COLOR_CYAN, -1);
  init_pair(4, COLOR_RED, -1);
  init_pair(5, COLOR_YELLOW, -1);


  //Input variable
  char command = 0;

  //Publishers
  pose_reference_publ = n.advertise<geometry_msgs::PoseStamped>("/"+drone_id_namespace+"/motion_reference/pose", 1, true);
  flightaction_pub = n.advertise<aerostack_msgs::FlightActionCommand>("/"+drone_id_namespace+"/actuator_command/flight_action", 1, true);

  //Subscribers
  self_pose_sub = n.subscribe("/"+drone_id_namespace+"/self_localization/pose", 1, selfLocalizationPoseCallback);
  ground_speed_sub = n.subscribe("/"+drone_id_namespace+"/self_localization/speed", 1, selfSpeedCallback);
  //Wait 3sec for initialization
  sleep(3);
  

  move(0,0);clrtoeol();
  printw("                  - KEYBOARD TELEOPERATION WITH MPC CONTROL -");
  move(3,0);clrtoeol();
  printw("--------------------------------------------------------------------------------");
  //Print controls
  printoutPoseControls(); 
  //LOOP
  ros::Rate loop_rate(FREQ_INTERFACE);

  while (ros::ok()){
    // Read messages
    ros::spinOnce();
    move(16,0);
    printw("                        Last key pressed: ");
    //Read command
    command = getch();
    switch (command){
      case 't':  // Take off
        printw("t       ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Take off           ");clrtoeol();
        flight_action_msg.action = aerostack_msgs::FlightActionCommand::TAKE_OFF;
        flightaction_pub.publish(flight_action_msg);
        motion_reference_pose_msg.pose.position = self_localization_pose_msg.pose.position;
        motion_reference_pose_msg.pose.position.z = 1;
        pose_reference_publ.publish(motion_reference_pose_msg);
        break;
      case 'y':  // Land
        flight_action_msg.action = aerostack_msgs::FlightActionCommand::LAND;
        flightaction_pub.publish(flight_action_msg);
        motion_reference_pose_msg.pose.position = self_localization_pose_msg.pose.position;
        motion_reference_pose_msg.pose.position.z = 0;
        pose_reference_publ.publish(motion_reference_pose_msg);
        printw("y        ");clrtoeol(); 
        move(17, 0); 
        printw("                        Last command:     Land             ");clrtoeol();            
        break;
      case 'h':  // Hover   
      {
        printw("h      ");clrtoeol();
        move(17, 0); printw("                        Last command:     Keep hovering             ");clrtoeol();refresh();
        flight_action_msg.action = aerostack_msgs::FlightActionCommand::HOVER;
        flightaction_pub.publish(flight_action_msg);
        motion_reference_pose_msg.pose = self_localization_pose_msg.pose;                                             
        pose_reference_publ.publish(motion_reference_pose_msg);        
        break;
      }
      case 'q':  // Move upwards
        flight_action_msg.action = aerostack_msgs::FlightActionCommand::MOVE;
        flightaction_pub.publish(flight_action_msg);
        if(motion_reference_pose_msg.pose.position.z == 0 && motion_reference_pose_msg.pose.position.x == 0 && motion_reference_pose_msg.pose.position.y == 0){
          motion_reference_pose_msg.pose = self_localization_pose_msg.pose;
        }
        motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z + CTE_ALTITUDE;
        pose_reference_publ.publish(motion_reference_pose_msg);
        printw("q      ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase altitude         ");clrtoeol();
        break;
      case 'a':  //Move downwards
        flight_action_msg.action = aerostack_msgs::FlightActionCommand::MOVE;
        flightaction_pub.publish(flight_action_msg);
        if(motion_reference_pose_msg.pose.position.z == 0 && motion_reference_pose_msg.pose.position.x == 0 && motion_reference_pose_msg.pose.position.y == 0){
          motion_reference_pose_msg.pose = self_localization_pose_msg.pose;
        }
        motion_reference_pose_msg.pose.position.z = motion_reference_pose_msg.pose.position.z - CTE_ALTITUDE;
        pose_reference_publ.publish(motion_reference_pose_msg);
        printw("a        ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Decrease altitude         ");clrtoeol(); 
        break;         
      case 'z':  //(yaw) turn counter-clockwise
      {             
        flight_action_msg.action = aerostack_msgs::FlightActionCommand::MOVE;
        flightaction_pub.publish(flight_action_msg);  
        double yaw,r,p;
        toEulerianAngle(motion_reference_pose_msg, &r,&p,&yaw);
        q_rot.setRPY(r, p, yaw + CTE_YAW);
        current_commands_yaw = yaw + CTE_YAW;

        if(motion_reference_pose_msg.pose.position.z == 0 && motion_reference_pose_msg.pose.position.x == 0 && motion_reference_pose_msg.pose.position.y == 0){
          motion_reference_pose_msg.pose = self_localization_pose_msg.pose;
        }
        motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
        motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
        motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
        motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();

        pose_reference_publ.publish(motion_reference_pose_msg);
        printw("z       ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Turn counter-clockwise        ");clrtoeol();  
        break;    
      }      
      case 'x':  // (yaw) turn clockwise
      {
        flight_action_msg.action = aerostack_msgs::FlightActionCommand::MOVE;
        flightaction_pub.publish(flight_action_msg);
        double yaw,r,p;
        toEulerianAngle(motion_reference_pose_msg, &r,&p,&yaw);
        q_rot.setRPY(r, p, yaw - CTE_YAW);
        current_commands_yaw = yaw - CTE_YAW;
        if(motion_reference_pose_msg.pose.position.z == 0 && motion_reference_pose_msg.pose.position.x == 0 && motion_reference_pose_msg.pose.position.y == 0){
          motion_reference_pose_msg.pose = self_localization_pose_msg.pose;
        }
        motion_reference_pose_msg.pose.orientation.w = q_rot.getW();
        motion_reference_pose_msg.pose.orientation.x = q_rot.getX();
        motion_reference_pose_msg.pose.orientation.y = q_rot.getY();
        motion_reference_pose_msg.pose.orientation.z = q_rot.getZ();
        pose_reference_publ.publish(motion_reference_pose_msg);
        printw("x      ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Turn clockwise          ");clrtoeol();
      }
        break;                 
      case ASCII_KEY_RIGHT:
        flight_action_msg.action = aerostack_msgs::FlightActionCommand::MOVE;
        flightaction_pub.publish(flight_action_msg);
        if(motion_reference_pose_msg.pose.position.z == 0 && motion_reference_pose_msg.pose.position.x == 0 && motion_reference_pose_msg.pose.position.y == 0){
          motion_reference_pose_msg.pose = self_localization_pose_msg.pose;
        }
        motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y - CTE_POSE;
        pose_reference_publ.publish(motion_reference_pose_msg);
        printw("\u2192            ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase movement to the right        ");clrtoeol();  
        break;               
      case ASCII_KEY_LEFT:
        flight_action_msg.action = aerostack_msgs::FlightActionCommand::MOVE;
        flightaction_pub.publish(flight_action_msg);
        if(motion_reference_pose_msg.pose.position.z == 0 && motion_reference_pose_msg.pose.position.x == 0 && motion_reference_pose_msg.pose.position.y == 0){
          motion_reference_pose_msg.pose = self_localization_pose_msg.pose;
        }
        motion_reference_pose_msg.pose.position.y = motion_reference_pose_msg.pose.position.y + CTE_POSE;
        pose_reference_publ.publish(motion_reference_pose_msg);       
        printw("\u2190            ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase movement to the left         ");clrtoeol();
        break;       
      case ASCII_KEY_DOWN:
        flight_action_msg.action = aerostack_msgs::FlightActionCommand::MOVE;
        flightaction_pub.publish(flight_action_msg);
        if(motion_reference_pose_msg.pose.position.z == 0 && motion_reference_pose_msg.pose.position.x == 0 && motion_reference_pose_msg.pose.position.y == 0){
          motion_reference_pose_msg.pose = self_localization_pose_msg.pose;
        }
        motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x - CTE_POSE;
        pose_reference_publ.publish(motion_reference_pose_msg);          
        printw("\u2193     ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase backward      ");clrtoeol();
        break;        
      case ASCII_KEY_UP:
        flight_action_msg.action = aerostack_msgs::FlightActionCommand::MOVE;
        flightaction_pub.publish(flight_action_msg);
        if(motion_reference_pose_msg.pose.position.z == 0 && motion_reference_pose_msg.pose.position.x == 0 && motion_reference_pose_msg.pose.position.y == 0){
          motion_reference_pose_msg.pose = self_localization_pose_msg.pose;
        }
        motion_reference_pose_msg.pose.position.x = motion_reference_pose_msg.pose.position.x + CTE_POSE;
        pose_reference_publ.publish(motion_reference_pose_msg);    
        printw("\u2191    ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase forward            ");clrtoeol();
        break;         
    case 'r':
    {
      printw("r        ");clrtoeol();
      move(17, 0); 
      flight_action_msg.action = aerostack_msgs::FlightActionCommand::MOVE;
      flightaction_pub.publish(flight_action_msg);
      printw("                        Last command:     Reset orientation           ");clrtoeol(); 
      if(motion_reference_pose_msg.pose.position.z == 0 && motion_reference_pose_msg.pose.position.x == 0 && motion_reference_pose_msg.pose.position.y == 0){
        motion_reference_pose_msg.pose = self_localization_pose_msg.pose;
      }
      current_commands_yaw = 0;
      motion_reference_pose_msg.pose.orientation.w = 0;
      motion_reference_pose_msg.pose.orientation.x = 0;
      motion_reference_pose_msg.pose.orientation.y = 0;
      motion_reference_pose_msg.pose.orientation.z = 0;
      pose_reference_publ.publish(motion_reference_pose_msg);     
    }
    break;            
    }
    refresh();
    loop_rate.sleep();
  }

  endwin();
  return 0;
}

//Pose mode controls
void printoutPoseControls(){
  move(4,0);clrtoeol();
  printw(" BASIC MOTIONS                     POSE CONTROL");
  move(5,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   t");attroff(COLOR_PAIR(5)); printw("      Take off              ");  
  attron(COLOR_PAIR(5));printw("    \u2191");attroff(COLOR_PAIR(5));printw("  Increase forward position %.2f m  ",CTE_POSE);
  
  move(6,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   y");attroff(COLOR_PAIR(5)); printw("      Land                  ");
  attron(COLOR_PAIR(5));printw("    \u2193");attroff(COLOR_PAIR(5));printw("  Increase backward position %.2f m  ",CTE_POSE);
  
  move(7,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   h");attroff(COLOR_PAIR(5)); printw("      Keep hovering         ");
  attron(COLOR_PAIR(5));printw("    \u2192");attroff(COLOR_PAIR(5));printw("  Increase position to the right %.2f m  ",CTE_POSE);  

  move(8,0);clrtoeol();
  attron(COLOR_PAIR(5));printw("   r");attroff(COLOR_PAIR(5));printw("      Reset orientation    ");
  attron(COLOR_PAIR(5));printw("     \u2190");attroff(COLOR_PAIR(5));printw("  Increase position to the left %.2f m  ",CTE_POSE);
  
  move(9,36);clrtoeol();
  attron(COLOR_PAIR(5));printw("q");attroff(COLOR_PAIR(5));printw("  Increase altitude %.2f m ",CTE_ALTITUDE);
  
  move(10,36);clrtoeol();
  attron(COLOR_PAIR(5));printw("a");attroff(COLOR_PAIR(5));printw("  Decrease altitude %.2f m ",CTE_ALTITUDE);

  move(11,36);clrtoeol();
  attron(COLOR_PAIR(5));printw("z");attroff(COLOR_PAIR(5));printw("  Turn counter-clockwise %.2f rad      ",CTE_YAW);

  move(12,36);clrtoeol();
  attron(COLOR_PAIR(5));printw("x");attroff(COLOR_PAIR(5));printw("  Turn clockwise %.2f rad        ",CTE_YAW);

  move(13,0);clrtoeol();
  printw("--------------------------------------------------------------------------------");
  refresh();
}

//Self localization callback
void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  self_localization_pose_msg = (*msg);
}

void selfSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
  self_speed_msg=*msg;
}

void toEulerianAngle(geometry_msgs::PoseStamped q, double *roll, double *pitch, double *yaw){
    if(q.pose.orientation.w == 0 && q.pose.orientation.x == 0 && q.pose.orientation.y == 0 && q.pose.orientation.z == 0){
        *roll   = 0; 
        *pitch = 0;
        *yaw = 0;
    }else{
        *roll  = atan2(2.0 * (q.pose.orientation.z * q.pose.orientation.y + q.pose.orientation.w * q.pose.orientation.x) , 1.0 - 2.0 * (q.pose.orientation.x * q.pose.orientation.x + q.pose.orientation.y * q.pose.orientation.y));
        *pitch = asin(2.0 * (q.pose.orientation.y * q.pose.orientation.w - q.pose.orientation.z * q.pose.orientation.x));
        *yaw   = atan2(2.0 * (q.pose.orientation.z * q.pose.orientation.w + q.pose.orientation.x * q.pose.orientation.y) , - 1.0 + 2.0 * (q.pose.orientation.w * q.pose.orientation.w + q.pose.orientation.x * q.pose.orientation.x));    
    }
}