/**
 * @file website_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandTOL.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <px4_mavros_fly/cmd_ellipse.h>

#define PI 3.14159
 px4_mavros_fly::cmd_ellipse ellipse_cmd;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped quad_pose;
sensor_msgs::Joy joyvar;
sensor_msgs::Imu imu;
bool fl_manual=false ;
bool fl_auto=false ;
bool fl_no_sig=true ;
bool fl_land=false ;
bool fl_disarm=false;
bool fl_T_init=false;
bool  fl_reg_init =false;
    float x_q,y_q,z_q,alpha_q;
nav_msgs::Odometry feedback[5];
    float q[4];



float v_xi, v_yi,v_zi;


void Odom_1_cb(const nav_msgs::Odometry::ConstPtr& msg){
    feedback[0] = *msg;
    //printf("%lf\n",feedback.pose.pose.position.x);
}
void Odom_2_cb(const nav_msgs::Odometry::ConstPtr& msg){
    feedback[1] = *msg;
    //printf("%lf\n",feedback.pose.pose.position.x);
}
void Odom_3_cb(const nav_msgs::Odometry::ConstPtr& msg){
    feedback[2] = *msg;
    //printf("%lf\n",feedback.pose.pose.position.x);
}
void Odom_4_cb(const nav_msgs::Odometry::ConstPtr& msg){
    feedback[3] = *msg;
    //printf("%lf\n",feedback.pose.pose.position.x);
}
void Odom_5_cb(const nav_msgs::Odometry::ConstPtr& msg){
    feedback[4] = *msg;
    //printf("%lf\n",feedback.pose.pose.position.x);
}

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
    q[0] = imu.orientation.w;
    q[1] = imu.orientation.x;
    q[2] = imu.orientation.y;
    q[3] = imu.orientation.z;
    alpha_q = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
    //ROS_INFO("alpha %f", alpha_q );
}


void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    quad_pose = *msg;
    x_q = quad_pose.pose.position.x;
    y_q = quad_pose.pose.position.y;
    z_q = quad_pose.pose.position.z;

   // ROS_INFO("x %f y %f z %f \n", x_q,y_q,z_q);
   }

int main(int argc, char **argv)
{
    int j=0,i=0,N=5;
    int x_d = 0, y_d = 0, z_d = 2;
    float  init_value_thrust;
    float cmd_height, V_cmd;
    float x_q_ef,y_q_ef,alpha_q_ef,x_O,y_O, Ro,omega_max;

    float a_el,b_el,ma,mi,df,Df1,Df2,xf1,yf1,xf2,yf2,rho;
    float alpha_t,alpha_err, alpha_d,k_l,omega_q,theta,dtheta;
    float x_c[N],y_c[N],x_sum,y_sum,x_bar,y_bar,l1,l2;
    float xx_sum,yy_sum,xy_sum,m,c,xo,yo,p1,p2,D_proj,dp_max,dl_max,xe1,xe2,ye1,ye2,x_tr,y_tr,m_tr;

    ros::init(argc, argv, "convoy_sitl_node");
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

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

    ros::ServiceClient  land_client = nh.serviceClient<mavros_msgs::CommandTOL> 
	    ("mavros/cmd/land");

    ros::Subscriber Odom_1_sub = nh.subscribe<nav_msgs::Odometry>
            ("syscon_bot_1/odom", 10, Odom_1_cb);

     ros::Subscriber Odom_2_sub = nh.subscribe<nav_msgs::Odometry>
            ("syscon_bot_2/odom", 10, Odom_2_cb);
            
     ros::Subscriber Odom_3_sub = nh.subscribe<nav_msgs::Odometry>
            ("syscon_bot_3/odom", 10, Odom_3_cb);
      
      ros::Subscriber Odom_4_sub = nh.subscribe<nav_msgs::Odometry>
            ("syscon_bot_4/odom", 10, Odom_4_cb);

      ros::Subscriber Odom_5_sub = nh.subscribe<nav_msgs::Odometry>
            ("syscon_bot_5/odom", 10, Odom_5_cb);                          

      ros::Publisher ellipse_pub = nh.advertise<px4_mavros_fly::cmd_ellipse>
            ("/ellipse_params", 20);
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
    float ang;
	 float v_xi, v_yi;
	 double to,t;
	 //bool fl_T_init=false;
	 float x_cmd, y_cmd, z_cmd;
     float V_max=0.5;
	 float A=2.5,B=2.5, a=1, b=1;
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
    {
	
    //cur_yaw.data = cur_yaw.data-90;
   //ROS_INFO("%lf,%lf,%lf,%f",joyvar.axes[3],joyvar.axes[2],joyvar.axes[1],ang*180/3.14);
   
    v_xi= joyvar.axes[3]*1.5;
    v_yi= joyvar.axes[4]*1.5;

    vel.twist.linear.y = v_xi*cos(-alpha_q) - v_yi*sin(-alpha_q);
    vel.twist.linear.x = v_yi*cos(-alpha_q) + v_xi*sin(-alpha_q);
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


   /* if (fl_T_init==false)
		{to=ros::Time::now().toSec();
		fl_T_init=true;
    vel.twist.linear.y = 0;
         vel.twist.linear.x = 0;
         vel.twist.linear.z = 0;
    local_vel_pub.publish(vel);}

		t=ros::Time::now().toSec();


		s=s_dot*(t-to);
		x_cmd=A*cos(a*s);
		y_cmd=B*sin(b*s);
 		 pose.pose.position.x = x_cmd;
    	pose.pose.position.y = y_cmd;
    	pose.pose.position.z = 1;

		local_pos_pub.publish(pose);
*/
          cmd_height=1.5;
          V_cmd=0.5;
          omega_max=0.5;
          Ro=V_cmd/omega_max;
          k_l=1;


    for(i=0;i<N;i++)
      {x_c[i]=feedback[i].pose.pose.position.x;
       y_c[i]=feedback[i].pose.pose.position.y;
      }

        x_sum=0;
          y_sum=0;
      for(i=0;i<N;i++)
       {  x_sum=x_sum+x_c[i];
          y_sum=y_sum+y_c[i];
        }
        x_bar=x_sum/N;
        y_bar=y_sum/N;
 if (fl_reg_init==false)
        {fl_reg_init=true;
        
          
          xx_sum=0;
          yy_sum=0;
          xy_sum=0;

      for(i=0;i<N;i++)
       { 
          xx_sum=xx_sum+x_c[i]*x_c[i];
          yy_sum=yy_sum+y_c[i]*y_c[i];
          xy_sum=xy_sum+x_c[i]*y_c[i];
        }
        


        c=(y_bar*xx_sum-x_bar*xy_sum)/(xx_sum-N*x_bar*x_bar);
        m=(xy_sum-N*x_bar*y_bar)/(xx_sum-N*x_bar*x_bar);
        dp_max=0;
        xe1=x_bar;
        xe2=x_bar;

      for(i=0;i<N;i++)
        {xo=x_c[i];
        yo=y_c[i];
        p1=(xo - c*m + m*yo)/(m*m + 1);
        p2=(yo*m*m + xo*m + c)/(m*m + 1);
         D_proj=sqrt((xo-p1)*(xo-p1)+(yo-p2)*(yo-p2));

        if (D_proj>=dp_max)
            dp_max=D_proj;
          if (p1>=xe1)
            {xe1=p1;
            ye1=p2;}
          if(p1<=xe2)
            {xe2=p1;
              ye2=p2;}
        }
              dl_max=sqrt((xe1-xe2)*(xe1-xe2)+(ye1-ye2)*(ye1-ye2));
            

             theta=atan(m);
             if (dl_max<dp_max*2)
              theta=PI/2-theta;

            if (theta >PI/2)
              theta=theta-PI;
            if (theta<=-PI/2)
              theta=theta+PI;
          	}
 if (fl_reg_init==true)
         {
         // x_sum=0;
         // y_sum=0;
          xx_sum=0;
          yy_sum=0;
          xy_sum=0;

      for(i=0;i<N;i++)
       {  x_tr=cos(theta)*(x_c[i]-x_bar)+sin(theta)*(y_c[i]-y_bar);
          y_tr=-sin(theta)*(x_c[i]-x_bar)+cos(theta)*(y_c[i]-y_bar);
          // x_sum=x_sum+x_tr;
          // y_sum=y_sum+y_tr;
          xx_sum=xx_sum+x_tr*x_tr;
        //  yy_sum=yy_sum+y_c[i]*y_c[i];
          xy_sum=xy_sum+x_tr*y_tr;
        }

        //c=(y_bar*xx_sum-x_bar*xy_sum)/(xx_sum-N*x_bar*x_bar);
        m_tr=xy_sum/xx_sum;
        dtheta=atan(m_tr);
        theta= theta+ dtheta;
                m=tan (theta);
                c=y_bar-m*x_bar;

        dp_max=0;
        xe1=x_bar;
        xe2=x_bar;

      for(i=0;i<N;i++)
        {xo=x_c[i];
        yo=y_c[i];
        p1=(xo - c*m + m*yo)/(m*m + 1);
        p2=(yo*m*m + xo*m + c)/(m*m + 1);
         D_proj=sqrt((xo-p1)*(xo-p1)+(yo-p2)*(yo-p2));

        if (D_proj>=dp_max)
            dp_max=D_proj;
          if (p1>=xe1)
            {xe1=p1;
            ye1=p2;}
          if(p1<=xe2)
            {xe2=p1;
              ye2=p2;}
       }
              dl_max=sqrt((xe1-xe2)*(xe1-xe2)+(ye1-ye2)*(ye1-ye2));
              l1=dl_max;
              l2=2*dp_max;
              x_O=(xe1+xe2)/2;
              y_O=(ye1+ye2)/2;
              a_el=l1/sqrt(2);
              b_el=l2/sqrt(2);
              if (b_el<=sqrt(l1*Ro/sqrt(2)))
                b_el=sqrt(l1*Ro/sqrt(2));


               ellipse_cmd.a=a_el;
                ellipse_cmd.b=b_el;
               ellipse_cmd.xO=x_O;
                ellipse_cmd.yO=y_O;
               ellipse_cmd.theta=theta;
               ellipse_pub.publish(ellipse_cmd);
            }

           
        /*  v_xi= 0;
          v_yi= V_cmd;

         vel.twist.linear.y = v_xi*cos(-alpha_q) - v_yi*sin(-alpha_q);
          vel.twist.linear.x = v_yi*cos(-alpha_q) + v_xi*sin(-alpha_q);
       */


      
          //theta=PI/4;
          //x_O=0;
          //y_O=5;


          alpha_q_ef=alpha_q- theta;
          x_q_ef=(x_q-x_O)*cos(theta)+(y_q-y_O)*sin(theta);
          y_q_ef=-(x_q-x_O)*sin(theta)+(y_q-y_O)*cos(theta);

          vel.twist.linear.x =V_cmd*cos(alpha_q);
          vel.twist.linear.y =V_cmd*sin(alpha_q);
          vel.twist.linear.z=1.5*(cmd_height- z_q);
          //a_el=5;
          //b_el=3;

          ma=2*a_el;
          mi=ma*b_el/a_el;
          df=sqrt((ma/2)*(ma/2)-(mi/2)*(mi/2));
          xf1=df;
          xf2=-df;
          yf1=0;
          yf2=0;
          rho=((x_q_ef*x_q_ef)/(a_el*a_el))+((y_q_ef*y_q_ef)/(b_el*b_el));
        //Df1=sqrt((x_q_ef-xf1)*(x_q_ef-xf1)+(y_q_ef-yf1)*(y_q_ef-yf1));
        //Df2=sqrt((x_q_ef-xf2)*(x_q_ef-xf2)+(y_q_ef-yf2)*(y_q_ef-yf2));
        //rho=(ma/(Df1+Df2));
         alpha_t= atan2(1*b_el*b_el*x_q_ef,(-1*a_el*a_el*y_q_ef));

          alpha_d=alpha_t+atan(k_l*(rho-1));
            if (alpha_d>=PI)
              alpha_d=alpha_d-2*PI;
            if (alpha_d<-PI)
               alpha_d= alpha_d+2*PI;
            
            alpha_err=alpha_d-alpha_q_ef;

            if (alpha_err>=PI)
              alpha_err=alpha_err-2*PI;
            
            if (alpha_err<-PI)
              alpha_err= alpha_err+2*PI;


            omega_q=1*alpha_err;

            if (abs(omega_q)>=omega_max)
            {omega_q=omega_q/abs(omega_q);}

           ROS_INFO("omega %f \n",omega_q);
          vel.twist.angular.z=omega_q;
           local_vel_pub.publish(vel);
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
