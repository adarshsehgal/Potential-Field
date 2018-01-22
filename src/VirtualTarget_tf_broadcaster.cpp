#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  ros::Publisher virtualposition_pub = node.advertise<nav_msgs::Odometry>("virtualPosition", 50);

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok()){
    ros::spinOnce();
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = 1.0*sin(ros::Time::now().toSec());
    odom.pose.pose.position.y = 1.0*cos(ros::Time::now().toSec());
    virtualposition_pub.publish(odom);
	
    transform.setOrigin( tf::Vector3(1.0*sin(ros::Time::now().toSec()), 1.0*cos(ros::Time::now().toSec()), 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "VirtualTarget"));
    rate.sleep();
  }
  return 0;
};


