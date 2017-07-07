/**
 * @file website_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */
#include <graphics.h>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
mavros_msgs::State current_state;
sensor_msgs::Joy joyvar;
sensor_msgs::Imu imu;
bool fl_manual=false ;
bool fl_auto=false ;
bool fl_no_sig=true ;
bool fl_land=false ;
bool fl_disarm=false;
bool fl_T_init=false;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void joyfn(const sensor_msgs::Joy::ConstPtr& joy){
    //ROS_INFO("1");
    joyvar = *joy;

if (joyvar.buttons[5] == 1 && fl_manual==true)
	{fl_manual=false;
	fl_auto=true;
	fl_land=false;	
    fl_disarm=false;
ROS_INFO("Manual OFF");}

if (joyvar.buttons[0] == 1 && fl_manual==false)
	{fl_manual=true;
        fl_T_init=false;
		fl_auto=false;
	fl_land=false;
    fl_disarm=false;
	ROS_INFO("Manual ON");}
if (joyvar.buttons[1] == 1 )
	{fl_manual=false;
		fl_auto=false;
	fl_land=true;
    fl_T_init=false;
    fl_disarm=false;
	ROS_INFO("Landing Commanded");}
if (joyvar.buttons[3] == 1 && fl_land==true)
    {fl_disarm=true; 
ROS_INFO("Disarm Commanded");}

}



void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    imu = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "website_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber joy_ip = nh.subscribe("joy", 10, joyfn);
    
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 10, imu_cb);

    ros::ServiceClient  land_client = nh.serviceClient<mavros_msgs::CommandTOL> 
	    ("mavros/cmd/land");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped vel;
  //  pose.pose.position.x = 0;
   // pose.pose.position.y = 0;
   // pose.pose.position.z = 1;
	
     vel.twist.linear.x = 0;
     vel.twist.linear.y = 0;
     vel.twist.linear.z = 0;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
          local_vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
	mavros_msgs::CommandTOL land;
    mavros_msgs::CommandBool arm_cmd;
    float q[4];
    float ang;
    int x_d = 0, y_d = 0, z_d = 2;
	 float v_xi, v_yi;
	 double to,t;
	 //bool fl_T_init=false;
	 float x_cmd, y_cmd, z_cmd, y_dot, yaw;
     float V_max=0.5;
	 float A=2.5,B=2.5, a=1, b=2;
         float s_dot, s=0;

         s_dot=V_max/sqrt(A*A*a*a+B*B*b*b);
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))&& fl_land==false){
                   arm_cmd.request.value = true;
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

//ROS_INFO("in loop");
//if  (fl_no_sig==true)
  //   {local_pos_pub.publish(pose);}
if  (fl_manual==true)
    {q[0] = imu.orientation.w;
    q[1] = imu.orientation.x;
    q[2] = imu.orientation.y;
    q[3] = imu.orientation.z;
    ang = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
	
    //cur_yaw.data = cur_yaw.data-90;
   //ROS_INFO("%lf,%lf,%lf,%f",joyvar.axes[3],joyvar.axes[2],joyvar.axes[1],ang*180/3.14);
   
    v_xi= joyvar.axes[3]*1.5;
    v_yi= joyvar.axes[4]*1.5;

    vel.twist.linear.y = v_xi*cos(-ang) - v_yi*sin(-ang);
    vel.twist.linear.x = v_yi*cos(-ang) + v_xi*sin(-ang);
    vel.twist.linear.z = joyvar.axes[1]*1.5;
    vel.twist.angular.z = joyvar.axes[0]*1.0;
/*
    if(joyvar.buttons[6] != 0)
    	vel.twist.angular.z = joyvar.buttons[6]*0.25;
    elsel
       vel.twist.angular.z = -joyvar.buttons[7]*0.25;
*/
 //vel.twist.linear.y = 0;//v_xi*cos(-ang) - v_yi*sin(-ang);
   // vel.twist.linear.x = 0;//v_yi*cos(-ang) + v_xi*sin(-ang);
  //  vel.twist.linear.z = 0;

}

  


     if  (fl_auto==true)
	{  
		if (fl_T_init==false)
		{
			to=ros::Time::now().toSec();
			fl_T_init=true;
    			vel.twist.linear.y = 0;
      		   	vel.twist.linear.x = 0;
         		vel.twist.linear.z = 0;
    			local_vel_pub.publish(vel);
		}
		t=ros::Time::now().toSec();	
		
		//ROS_INFO("%lf",t-to);
		y_dot = ((B*b)/(A*a))*(cos(b*s)/sin(a*s))*(-1);
		yaw = atan2((B*b)*(cos(b*s))*(-1),(A*a)*(sin(a*s)));
		s=s_dot*(t-to);
		x_cmd=A*cos(a*s);
		y_cmd=B*sin(b*s);
 		pose.pose.position.x = x_cmd;
    		pose.pose.position.y = y_cmd;
    		pose.pose.position.z = 1.5;
		pose.pose.orientation.w = cos(yaw/2);
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = sin(yaw/2);
		pose.pose.orientation.x = 0;
		local_pos_pub.publish(pose);
		/*
		vel.twist.linear.y = 0;
         	vel.twist.linear.x = 0;
         	vel.twist.linear.z = 0;
    		local_vel_pub.publish(vel);*/
	}

  
   

if (fl_land==true)
      { vel.twist.linear.x = 0;
	vel.twist.linear.y = 0;
	vel.twist.linear.z = -0.25;
	local_vel_pub.publish(vel);
    //    land.request.altitude = 0;
 	//land.request.min_pitch = 0;
    //	land_client.call(land);


if(current_state.armed && fl_disarm==true)
{     arm_cmd.request.value = false;
if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle disarmed");
                }
}


      }


if (fl_auto==false)
local_vel_pub.publish(vel);



  ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
