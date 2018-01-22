#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
//#include "Aria.h"
#include <time.h>
#include <stdio.h>
#include <math.h>

#define PI 3.14159265

struct OdomData
{
    double x;
    double y;
    double heading;
};

class RobotDriver
{
	private:
	  //! The node handle we'll be using
	  ros::NodeHandle nh_;
	  //! We will be publishing to the "cmd_vel" topic to issue commands
          //ros::Publisher cmd_vel_pub_;
	  ros::Subscriber odom_pub;
          ros::Subscriber virtual_point_sub;

	public:
          ros::Publisher cmd_vel_pub_;
          ros::Publisher ready_to_track_pub;
          void OdomCallback(const nav_msgs::Odometry::ConstPtr & odomsg);
          void VirtualPointCallback(const geometry_msgs::Point::ConstPtr & pointmsg);
	  //! ROS node initialization
	  RobotDriver(ros::NodeHandle &nh)
	  {
	    nh_ = nh;
	    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);         
            odom_pub = nh_.subscribe("/turtle1/pose", 1, &RobotDriver::OdomCallback,this); ///RosAria/pose
            virtual_point_sub = nh_.subscribe("/virtualposition", 1, &RobotDriver::VirtualPointCallback,this);
            ready_to_track_pub = nh_.advertise<std_msgs::String>("/readytotrack", 1);

          }
          double Angle;
          double RobotHeading;
          struct OdomData OdomDataSeekur;
          struct OdomData VirtualVehicle;
          bool Wantstotrack;

      void GoToPoint(struct OdomData RobotPos,struct OdomData VirtualVehiclePos)
      {
          double arcsin,arctan,roboh;
          arctan = atan2((RobotPos.y - VirtualVehiclePos.y),(RobotPos.x - VirtualVehiclePos.x));
          arctan = arctan * 180 / PI;
          roboh = RobotPos.heading * 180 /PI;
          ROS_INFO("Heading of robot %f Relative heading %f \n", roboh,arctan);
      }
};

void RobotDriver::OdomCallback(const nav_msgs::Odometry::ConstPtr& odomsg)
{
    OdomDataSeekur.x = odomsg->pose.pose.position.x;
    OdomDataSeekur.y = odomsg->pose.pose.position.y;
}

void RobotDriver::VirtualPointCallback(const geometry_msgs::Point::ConstPtr & pointmsg)
{
    VirtualVehicle.x = pointmsg->x;
    VirtualVehicle.y = pointmsg->y;
    GoToPoint(OdomDataSeekur,VirtualVehicle);
}

//***********MAIN PROGRAM OF THE PATH PLANNING USING ODOMETRY**************
int main(int argc, char** argv) 
{
	  //init the ROS node
	  ros::init(argc, argv, "robot_driver");
	  ros::NodeHandle nh;
	  RobotDriver driver(nh);
          ros::Rate rate(20.0);

          // publish message to tell the target that robot is ready to track
          std_msgs::String msg;
          std::stringstream ss;
          ss << "track";
          msg.data = ss.str();
          //driver.ready_to_track_pub.publish(msg);

          double velangular;
          double vellinear;
          double relative_heading_seekur_target;
          double relative_distance_seekur_target;
          double heading_target;
          double distance_fake;
          tf::TransformListener listener;
          geometry_msgs::Twist base_cmd;
          while (nh.ok()){
            driver.ready_to_track_pub.publish(msg);
            tf::StampedTransform transform;
            // find out the heading of the target
            try{
              //listener.lookupTransform("/virtual_vehicle", "/virtual_vehicle2",
              //                         ros::Time(0), transform);
	      listener.lookupTransform("/VirtualTarget", "/world",
                                       ros::Time(0), transform);

               }
            catch (tf::TransformException ex){
              ROS_ERROR("%s",ex.what());
            }
            heading_target = atan2(transform.getOrigin().y(),
                                        transform.getOrigin().x());
            distance_fake = sqrt(pow(transform.getOrigin().x(), 2) +
                                         pow(transform.getOrigin().y(), 2));


            // find out the relative heading & distance of robot to target
            try{
		
              listener.lookupTransform("/turtle1", "/VirtualTarget",
                                       ros::Time(0), transform);

               }
            catch (tf::TransformException ex){
		std::cout<<"I'm here";
              ROS_ERROR("%s",ex.what());
            }
            relative_heading_seekur_target = atan2(transform.getOrigin().y(),
                                        transform.getOrigin().x());
            relative_distance_seekur_target = sqrt(pow(transform.getOrigin().x(), 2) +
                                         pow(transform.getOrigin().y(), 2));

            // Calculate the heading control
            double Theta_R;
            double a,b;
            double Pmt,Pr;
            double debug;
            double heading_input;
            double speed_input;
            Pmt = 0.15;
            Pr = 1;
            a = Pmt * sin(heading_target - relative_heading_seekur_target);
            b = Pr;
            debug = asin(a);
            Theta_R = relative_heading_seekur_target + asin(a/b);
            //ROS_INFO("Theta_R: %f Theta_mt: %f Phi: %f debug: %f distance %f \n", Theta_R,heading_target,relative_heading_seekur_target,debug,relative_distance_seekur_target);
            //ROS_INFO("%f %f %f %f \n", heading_target,distance_fake,relative_heading_seekur_target,relative_distance_seekur_target);
            heading_input = Theta_R/3;
            speed_input = 0.05;

            // For safety reason, if the virtual vehicle behind the robot..stop tracking
            // relative_heading_seekur_target -0.8 rad n 0.8 are the view of the robot

            if (relative_heading_seekur_target < -0.7 || relative_heading_seekur_target > 0.7)
            {
                ROS_INFO("target is behind the robot! No Tracking!!!");
                heading_input = 0;
                speed_input = 0;
            }

            // The goal!
            if (relative_distance_seekur_target < 0.1)
            {
                ROS_INFO("distance of robot n target is close..around 50 cm");
                speed_input = 0;
                heading_input = 0;
            }
            // For safety reason, the maximum angular speed is 0.15;
            if (heading_input < -0.15 || heading_input > 0.15)
            {
                ROS_INFO("angular speed is exceed the maximum value");
                if (heading_input > 0)
                {
                    heading_input = 0.15;
                }
                else
                {
                    heading_input = -0.15;
                }
            }
            base_cmd.angular.z = heading_input;
            base_cmd.linear.x = speed_input;
	    
	    //std::cout<<heading_input;

            /*
            if (relative_distance_seekur_target < 0.01)
            {
                base_cmd.linear.x = 0;
            }
            */
            driver.cmd_vel_pub_.publish(base_cmd);
            //rate.sleep();



            rate.sleep();
            ros::spinOnce();
          }


          //ros::spin();
         // ros::Subscriber sub = nh.subscribe("/RosAria/pose", 1000, driver.OdomCallback,this)
         //ros::spin();
}
