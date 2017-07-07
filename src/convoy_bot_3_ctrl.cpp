/**
 * @file website_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/Joy.h>

#define pi 3.14159265359
int bot_id =3;
bool fl_reached=false;
bool fl_T_init=false, fl_oriented=false, fl_start=false;

geometry_msgs::Twist Vel;
nav_msgs::Odometry feedback;
float heading,x_bot,y_bot;
sensor_msgs::Joy joyvar;

void Odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    feedback = *msg;
     float q[4];

    //printf("%lf\n",feedback.pose.pose.position.x);
    q[0] = feedback.pose.pose.orientation.w;
     q[1] = feedback.pose.pose.orientation.x;
     q[2] = feedback.pose.pose.orientation.y;
     q[3] = feedback.pose.pose.orientation.z;
       heading = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
       x_bot=feedback.pose.pose.position.x;
      y_bot=feedback.pose.pose.position.y;
}

void joyfn(const sensor_msgs::Joy::ConstPtr& joy){
    //ROS_INFO("1");
    joyvar = *joy;

if (joyvar.buttons[4] == 1 && fl_oriented==true)
  {fl_start=true;
 ROS_INFO("ROBOT %d Start tracking",bot_id);}
}
// if (joyvar.buttons[3] == 1 && fl_manual==true)
//  {fl_manual=false;
//  fl_auto=true;
//  fl_land=false;  
// ROS_INFO("Manual OFF");}

// if (joyvar.buttons[1] == 1 && fl_manual==false)
//  {fl_manual=true;
//    fl_auto=false;
//  fl_land=false;
//  ROS_INFO("Manual ON");}
// if (joyvar.buttons[2] == 1 )
//  {fl_manual=false;
//    fl_auto=false;
//  fl_land=true;
//  ROS_INFO("Landing Commanded");}
// }


float get_distance(float x1, float y1, float x2, float y2)
{   return(sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
}
float get_heading_error(float heading_cmd, float heading_cur)
{ float heading_err;
  heading_err=heading_cmd-heading_cur;
        if (heading_err>pi)
            heading_err=heading_err-2*pi;
        if (heading_err<=-pi)
            heading_err=heading_err+2*pi;
return (heading_err);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "convoy_bot_3_ctrl");
    ros::NodeHandle nh;

    ros::Subscriber Odom_sub = nh.subscribe<nav_msgs::Odometry>
            ("syscon_bot_3/odom", 10, Odom_cb);
    ros::Publisher Vel_pub = nh.advertise<geometry_msgs::Twist>
            ("syscon_bot_3/cmd_vel", 10);
    ros::Subscriber joy_ip = nh.subscribe("joy", 10, joyfn);   
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

 

   
    float q[4];
    float ang;
    int x_d = 0, y_d = 0, z_d = 2;
   float v_xi, v_yi;
   float V_max=0.12;
   float s_dot, s=0,s0;
   double to,t,T;
   float x_l, y_l,D_lb=0, theta, heading_cmd, heading_err , heading_l;
   float A=3,B=3, a=1, b=2;
   float yl_dot, xl_dot;
    ros::Time last_request = ros::Time::now();

    s_dot=V_max/sqrt(A*A*a*a+B*B*b*b);
to=ros::Time::now().toSec();
s0=(bot_id-1)*pi/(6*a*b);
   
while(ros::ok() && to<2 ){to=ros::Time::now().toSec();}
    while(ros::ok()){
 
       
if (fl_reached==false)
{ yl_dot=B*b*cos(b*(s0));
  xl_dot=-A*a*sin(a*(s0));
    heading_l=atan2(yl_dot,xl_dot);
    x_l=A*cos(a*(s0))+0.18*cos(heading_l+pi);
    y_l=B*sin(b*(s0))+0.18*sin(heading_l+pi);
   heading_cmd=atan2(y_l-y_bot,x_l-x_bot);
   D_lb=get_distance(x_bot,y_bot,x_l,y_l);
   heading_err=get_heading_error(heading_cmd,heading);
    Vel.linear.x = 0.5* D_lb;
   Vel.angular.z=-0.5*heading_err;
   if (D_lb<=0.1 )
    {fl_reached=true;
      ROS_INFO("ROBOT %d REACHED",bot_id);
    }
}

if (fl_oriented==false && fl_reached==true)
{
 
 x_l=A*cos(a*(s0));
  y_l=B*sin(b*(s0));
   heading_cmd=atan2(y_l-y_bot,x_l-x_bot);
    heading_err=get_heading_error(heading_cmd,heading);
   Vel.linear.x = 0;  
  Vel.angular.z=-0.5*heading_err;
  if (abs(heading_err)<=pi/12)
    {fl_oriented=true;
      Vel.angular.z=0;
      ROS_INFO("ROBOT %d ORIENTED",bot_id);}
}


if (fl_start==true)
{
  if (fl_T_init==false)
      {to=ros::Time::now().toSec();
      fl_T_init=true;
        printf("init time = %f",to);}

    t=ros::Time::now().toSec();
    T=t-to;
  

    s=s_dot*T;

    x_l=A*cos(a*(s0+s));
    y_l=B*sin(b*(s0+s));

            heading_cmd=atan2(y_l-y_bot,x_l-x_bot);
            D_lb=get_distance(x_bot,y_bot,x_l,y_l);     //printf(" to=%f, t=%f , T=%f \n",to,t,T );

     //printf("t = %f, x_l =%f, y_l=%f, x_bot =%f, y_bot=%f , Distance = %f\n",T,x_l,y_l,x_bot,y_bot,D_lb );
       heading_err=get_heading_error(heading_cmd,heading);

            Vel.linear.x = 0.5* D_lb;

         Vel.linear.y = 0;
        //*cos(heading_err);
         Vel.linear.z = 0;

         Vel.angular.z=-0.5*heading_err;
  //}
}
    if (Vel.linear.x>=0.12)
      Vel.linear.x=0.12;
    if (abs(Vel.angular.z)>=1.5)
      Vel.angular.z=Vel.angular.z/abs(Vel.angular.z);

    Vel_pub.publish(Vel);
   





  ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
