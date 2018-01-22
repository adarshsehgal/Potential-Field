#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

//std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
	tf::Quaternion q;
	q.setRPY(0, 0, msg->theta);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Origin_Virtual_Target"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "my_tf_broadcaster");
	
	ros::NodeHandle node;
	ros:: Subscriber sub = node.subscribe("turtle1/pose", 10, &poseCallback);

	ros::spin();
	return 0;
}
