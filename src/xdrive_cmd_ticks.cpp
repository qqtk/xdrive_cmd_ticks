
#include <vector>
#include <string>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "xdriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// #include "xdrive_cmd_ticks/WheelSpeed.h"
#include "robbase_msg/WheelSpeed.h"
#include "robbase_msg/encoders.h"
// #include "encoder_test/ticks.h"

// #include <map>
// #include "xdrive_cmd_ticks/xdriver.h"

ros::Time current_time, last_time;

#define M_PIpi 3.1415926
#define wheel_base 0.496
int i_gear_ratio;
double d_drive_freq;
double d_wheel_diameter;
double d_diam;

ros::NodeHandle *private_n;
robbase_msg::WheelSpeed wheelspeed_msg;
// xdrive_cmd_ticks::WheelSpeed wheelspeed_msg;

using namespace std;
int right_wheel_speed;
int left_wheel_speed;
unsigned int numA;
unsigned int numB;

typedef struct {
	float Vlin;
	float Vang;
} twist_struct;
twist_struct twist_vec;

typedef struct{
	float tick_lb;
	float tick_lf;
	float tick_rb;
	float tick_rf;
} tick_vec_struct;
tick_vec_struct tick_vec;

float lwheelmotor, rwheelmotor;
float reduction_ratio;
  ros::Publisher cmd_vel_pub;

/*
void wheel_update_Speed_callback(const xdrive_unit::WheelSpeed& vel_motors_msg){
    d_diam = 0.14;
    left_wheel_speed = vel_motors_msg.lwheelmotor/d_diam*M_PIpi*reduction_ratio*60;
    right_wheel_speed = vel_motors_msg.rwheelmotor/d_diam*M_PIpi*reduction_ratio*60;
} */
// #if 1

void cmdvel_Callback(const geometry_msgs::Twist::ConstPtr& cmdvel_msg)
{
  twist_vec.Vlin = cmdvel_msg->linear.x;
  twist_vec.Vang = cmdvel_msg->angular.z;

  // Vlinr = (lmotor + rmotor) / 2
  // Vangular = (rmotor - lmotor) / w
  lwheelmotor = twist_vec.Vlin- twist_vec.Vang * wheel_base *0.5;
  rwheelmotor = twist_vec.Vlin + twist_vec.Vang * wheel_base *0.5;
  // .header.frame_id = "/qch";
  wheelspeed_msg.lwheel = lwheelmotor /d_diam*M_PIpi*reduction_ratio*60 ;
  wheelspeed_msg.rwheel = d_diam*M_PIpi*reduction_ratio*60;
  wheelspeed_msg.header.stamp = ros::Time::now();
  // header
  cmd_vel_pub.publish(wheelspeed_msg);
}
// #if 1 # endif // a=JunZhen

int main(int argc, char** argv)
{
  char strBuf[100];

	// struct twist_struct twist_vec;
	 twist_vec.Vlin =0.0;
	 twist_vec.Vang =0.0;

    ros::init(argc, argv, "xdrive_ticks_cmd_test" );
    ROS_INFO("xdrive_ticks_cmd_test starting");
    ros::NodeHandle nh;

    // ros::NodeHandle private_n.
    private_n= new ros::NodeHandle("~");

    double d_exec_rate;
    if(!private_n->getParam("exec_rate", d_exec_rate))
    {
        ROS_WARN("No exec_rate provided - default: 20 Hz");
        d_exec_rate = 20;
    }

    // float reduction_ratio;
    if(!private_n->getParam("f_reduction_ratio", reduction_ratio))
    {
        ROS_WARN("No reduction_ratio provided - default: 65");
        d_exec_rate = 65;
    }

 float d_diam;
    if(!private_n->getParam("wheel_diam", d_diam)) {
        ROS_WARN("Not provided: d_diam. Default=0.262");
        d_diam  = 0.262;
    }

  ROS_INFO("serial connection _ not-use");
  ros::Subscriber cmdvel_sub = nh.subscribe("cmd_vel", 20, cmdvel_Callback);

//  ros::Publisher ticksLR_pub = nh.advertise<robbase_msg::ticks>("/ticks", 20);
   ros::Publisher ticksLR_pub = nh.advertise<robbase_msg::encoders>("/ticksLR", 20);
 // ros::Publisher ticksLR4_pub = nh.advertise<encoder_test::ticks>("/ticks", 20);
//  ros::Publisher ticksMLR_pub = nh.advertise<robbase_msg::RazorImu>("/ticksMLR", 20);
  // showing motor_speed:
  cmd_vel_pub  = nh.advertise<robbase_msg::WheelSpeed>("/wheelspeed", 10);
//  cmd_vel_pub  = nh.advertise<robbase_msg::WheelSpeed>("/wheelspeed", 10);

  // struct qch.rwheelmotor

  // ros::Duration dur_time;
 robbase_msg::encoders ticksMsg;
 //  encoder_test::ticks ticksMsg;
  ros::Rate loop_rate(d_exec_rate);
  ros::Time read_time = ros::Time::now();

    while(ros::ok())
    {
	xdriver_struct_set("Vlin",&(twist_vec.Vlin));
	xdriver_struct_set("Vang",&(twist_vec.Vang));

	tick_vec.tick_lb = xdriver_getValue("tlb");
	tick_vec.tick_lf  = xdriver_getValue("tlf");
	tick_vec.tick_rb =  xdriver_getValue("trb");
	tick_vec.tick_rf = xdriver_getValue("trf");

	ticksMsg.ticks_l = int( (tick_vec.tick_lb + tick_vec.tick_lf )*0.5 );
	ticksMsg.ticks_r = int( (tick_vec.tick_rb + tick_vec.tick_rf) *0.5 );

	sleep(5);

	twist_vec.Vlin =0.0;
	xdriver_struct_set("Vlin",&(twist_vec.Vlin));
	xdriver_struct_set("Vang",&(twist_vec.Vang));

	// xdrive_motor();
	// ticksMsg.rticks = 99; // numA; // 'L'+numA==rwheel
	ticksLR_pub.publish(ticksMsg);
//    ticksMLRmsg.yaw = 99;
//    ticksMLRmsg.pitch = 99;
//    ticksMLR_pub.publish(ticksMLRmsg);

        ros::spinOnce();
        loop_rate.sleep();
    }// end.mainloop
}// end.main()
