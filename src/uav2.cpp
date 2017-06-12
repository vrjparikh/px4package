/**
 * @file uav2.cpp
 * @Lissajous Implementation of Assem's paper UAV#2
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <px4package/InitStatus.h>
#define uavid "uav2"
#define uavid1 uav2
mavros_msgs::State current_state;
sensor_msgs::Joy joyvar;
sensor_msgs::Imu imu;
geometry_msgs::PoseStamped feedback;
px4package::InitStatus updater;
px4package::InitStatus checker;

bool fl_manual=false ;
bool fl_auto=false ;
bool fl_no_sig=true ;
bool fl_land=false ;
bool fl_disarm=false;
bool fl_T_init=false;
bool i_reached = false;
bool all_reached = false;
int id = 2;
int offset_x = 2;
int offset_y = 2; 
float threshold = 0.1;
float A=3,B=3, a=3, b=2;
float s_dot, s=0;
float pi = 3.14;  
float pd = ((2*(id-1)*pi)/(a+b));
float x_init=A*cos(pd)-offset_x;    
float y_init=B*sin(pd)-offset_y;
double to;
void reached_cb(const px4package::InitStatus::ConstPtr& msg)
{
  //gdsgfsf
  checker = *msg;
  if(checker.uav1==true&&checker.uav2==true&&checker.uav3==true&&checker.uav4==true&&checker.uav5==true&&all_reached==false)
  {
    all_reached = true;
    to=ros::Time::now().toSec();
  }
  else if (all_reached==false)
  {
    all_reached = false;
  }
}
void whether_reached()
{
  
  if (i_reached==true)
  {
    updater.uav1 = checker.uav1;
    updater.uav2 = checker.uav2;
    updater.uav3 = checker.uav3;
    updater.uav4 = checker.uav4;
    updater.uav5 = checker.uav5;
    updater.uavid1 = true;
    // reached_pub.publish(updater);
  }
  else
  {
    updater.uav1 = checker.uav1;
    updater.uav2 = checker.uav2;
    updater.uav3 = checker.uav3;
    updater.uav4 = checker.uav4;
    updater.uav5 = checker.uav5;
    updater.uavid1 = false;
  }
}
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
void joyfn(const sensor_msgs::Joy::ConstPtr& joy)
{
    joyvar = *joy;
  if (joyvar.buttons[5] == 1 && fl_manual==true)
  {
    fl_manual=false;
    fl_auto=true;
    fl_land=false;  
      fl_disarm=false;
    ROS_INFO("Manual OFF");
  }

  if (joyvar.buttons[0] == 1 && fl_manual==false)
  {
    fl_manual=true;
        fl_T_init=false;
    fl_auto=false;
    fl_land=false;
      fl_disarm=false;
    ROS_INFO("Manual ON");
  }
  if (joyvar.buttons[1] == 1 )
  {
    fl_manual=false;
    fl_auto=false;
    fl_land=true;
      fl_T_init=false;
      fl_disarm=false;
    ROS_INFO("Landing Commanded");
  }
  if (joyvar.buttons[3] == 1 && fl_land==true)
  {
    fl_disarm=true; 
    ROS_INFO("Disarm Commanded");
  }
}
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu = *msg;
}
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    feedback = *msg;
    ROS_INFO("in position_cb");
    if(fabs(feedback.pose.position.x-x_init)<= threshold && fabs(feedback.pose.position.y-y_init)<=threshold && i_reached==false)
    {
      i_reached=true;
      //whether_reached();
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, uavid);
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            (uavid"/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            (uavid"/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (uavid"/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (uavid"/mavros/set_mode");
    ros::Subscriber joy_ip = nh.subscribe("joy", 10, joyfn);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            (uavid"/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            (uavid"/mavros/imu/data", 10, imu_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            (uavid"/mavros/local_position/pose", 10, position_cb);
    ros::ServiceClient  land_client = nh.serviceClient<mavros_msgs::CommandTOL> 
      (uavid"/mavros/cmd/land");
    ros::Publisher reached_pub = nh.advertise<px4package::InitStatus>
           ("/reached", 10);
    ros::Subscriber reached_sub = nh.subscribe<px4package::InitStatus>
            ("/reached", 10, reached_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    // whether_reached();
    
    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped vel;
    vel.twist.linear.x = 0;
    vel.twist.linear.y = 0;
    vel.twist.linear.z = 0;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
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
  double t;
  float x_cmd, y_cmd, z_cmd, y_dot, yaw;
    float V_max=0.5;
    s_dot=V_max/sqrt(A*A*a*a+B*B*b*b);
    ros::Time last_request = ros::Time::now();
    while(ros::ok())
    {
        whether_reached();
        reached_pub.publish(updater);
        if(current_state.mode!="OFFBOARD"&&(ros::Time::now()-last_request>ros::Duration(5.0)))  //Enable Offboard Mode
        {
          if(set_mode_client.call(offb_set_mode)&&offb_set_mode.response.success)       //Echo that Offboard Mode is Enabled
          {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        else                                          //Arming Vehicle
        {
            if(!current_state.armed&&(ros::Time::now()-last_request>ros::Duration(5.0))&&fl_land==false)
            {
              arm_cmd.request.value = true;
                if(arming_client.call(arm_cmd)&&arm_cmd.response.success)           //Echo if Vehicle is Armed
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
    if  (fl_manual==true)       //IF Quad is in Manual Flight Mode
      {
        q[0] = imu.orientation.w;   //Calculating Euler Angles based on Quaternions
        q[1] = imu.orientation.x;
        q[2] = imu.orientation.y;
        q[3] = imu.orientation.z;
        ang = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
        v_xi= joyvar.axes[3]*1.5;
        v_yi= joyvar.axes[4]*1.5;
        vel.twist.linear.y = v_xi*cos(-ang) - v_yi*sin(-ang);   //Commanding Linear Velocity
        vel.twist.linear.x = v_yi*cos(-ang) + v_xi*sin(-ang);
        vel.twist.linear.z = joyvar.axes[1]*1.5;
        vel.twist.angular.z = joyvar.axes[0]*1.0;         //Commanding Yaw
    }
      if  (fl_auto==true)            //If quad has Enetered Auto Mode for navigation on given curve
    {  
      if (fl_T_init==false)                //check if Takeoff has initiated in Auto Mode
      {
        
        fl_T_init=true;
          vel.twist.linear.y = 0;
          vel.twist.linear.x = 0;
            vel.twist.linear.z = 0;
          local_vel_pub.publish(vel);
      }
      
      if (all_reached==true)
      {
        //Stay at Starting Position
        t=ros::Time::now().toSec(); 
      y_dot = ((B*b)/(A*a))*(cos(b*s)/sin(a*s))*(-1);
      yaw = atan2((B*b)*(cos(pd+(b*s))*(-1)),(A*a)*(sin(pd-(a*s))));
      s=s_dot*(t-to);             //compute paramter to set position setpoints
      x_cmd=A*cos(pd-(a*s))-offset_x;     //Calculate Commanding X
      y_cmd=B*sin(pd+(b*s))-offset_y;     //Calculate Commanding Y
        pose.pose.position.x = x_cmd;     //Setpoint Position
          pose.pose.position.y = y_cmd;       
          pose.pose.position.z = 1;
        pose.pose.orientation.w = cos(yaw/2); //Setpoint Orientation
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = sin(yaw/2);
        pose.pose.orientation.x = 0;
        local_pos_pub.publish(pose);      //Publishing Pose
      }
      else
      {
        //Maintain the initial position
        pose.pose.position.x = x_init;      //Setpoint Position
          pose.pose.position.y = y_init;        
          pose.pose.position.z = 1;
        pose.pose.orientation.w = 0;      //Setpoint Orientation
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.x = 0;
        local_pos_pub.publish(pose);      //Publishing Pose
        }
    }
    if (fl_land==true)                //Landing Mode
      { 
        vel.twist.linear.x = 0;
      vel.twist.linear.y = 0;
      vel.twist.linear.z = -0.25;
      local_vel_pub.publish(vel);
      if(current_state.armed && fl_disarm==true)  //Disarm Quad, Emergency Kill
      {
          arm_cmd.request.value = false;
        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
                  ROS_INFO("Vehicle disarmed");
              }
      }
      }
    if (fl_auto==false)               //If quad is in Manual Mode, Velocity COmmanded
      local_vel_pub.publish(vel);

      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}