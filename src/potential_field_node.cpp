#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h> 
#include <math.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Pose2D current_pose;
ros::Publisher pub_pose2d;

void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
//std::cout<<"Hello";	
    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
//std::cout<<"msg:"<<msg;    

    // quaternion to RPY conversion
    //ROS uses quaternions to track and apply rotations. A quaternion has 4 components (x,y,z, w). That's right, 'w' is last. The commonly-used unit quaternion that yields no rotation about the x/y/z axes is (0,0,0,1): 
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // angular position
    current_pose.theta = yaw;
    pub_pose2d.publish(current_pose);

}

int main(int argc, char **argv){

	//const double PI = 3.14159265358979323846;
    
    	ROS_INFO("start");

	ros::init(argc, argv, "potential_field_node");
	ros::NodeHandle n;
    	ros::Subscriber sub_odometry = n.subscribe("/summit_xl_control/odom", 1, callback); //summit_xl_control/odom
    	ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("/summit_xl_control/cmd_vel",1); // turtle1/cmd_vel - summit_xl_control/cmd_vel
    	pub_pose2d = n.advertise<geometry_msgs::Pose2D>("bot_pose2d", 1);
    	ros::Rate rate(10);


	while (ros::ok()){
				
		int n=2;
		float delta_t=0.5;

		//t varies from 0 to 10 with difference of delta_t
		int length_t=200;
		
		float lambda=8.5;
		float pr_max=50;		


		//SET VIRTUAL TARGET
		double qv[length_t][n];

		for(int i=0;i<length_t;i++){
			for(int j=0;j<n;j++){
				qv[i][j]=0;
			}
		}
		qv[0][0]=current_pose.x;//x of actual robot and virtual target	
		qv[0][1]=current_pose.y;// y of actual robot and virtual target
	
		float pv=0.1;

		double theta_t[length_t][1];

		for(int i=0;i<length_t;i++){
			theta_t[i][0]=0;
		}

		//SET ROBOT
		double qr[length_t][n];
		
		for(int i=0;i<length_t;i++){
                        for(int j=0;j<n;j++){
                                qr[i][j]=0;
                        }
                }
		qr[0][0]=current_pose.x;//x of actual robot	
		qr[0][1]=current_pose.y;// y of actual robot

                float pr[length_t][1];

		float theta_r[length_t][1];

                for(int i=0;i<length_t;i++){
                	pr[i][0]=pv;
			theta_r[i][0]=0;
                }

		theta_r[0][0]=current_pose.theta; // heading of actual robot
		
		//Set reletive states between robot and virtual target
		float qrv[length_t][n];
		float prv[length_t][n];

                for(int i=0;i<length_t;i++){
                        for(int j=0;j<n;j++){
                                qrv[i][j]=0;
				prv[i][j]=0;
                        }
                }

		//compute initial reletive states between robot and virtual target
		for(int j=0;j<n;j++){
			qrv[0][j]=qv[0][j] - qr[0][j];
		}
		//std::cout<<qrv[0][1];
		prv[0][0]=pv*cos(theta_t[0][0]) - pr[0][0]*cos(theta_r[0][0]);    
                prv[0][1]=pv*sin(theta_t[0][0]) - pr[0][0]*sin(theta_r[0][0]);

		
		//set noide mean and standard deviation
		float noise_mean=0.5;
		float noise_std=0.5;

		//Main Program
		float qv_x,qv_y;
		float t[200];
		t[0]=0;
		float qt_diff[length_t][n];
		float phi[200];	//dimension of phi matrix
		phi[0]=0;
		float norm_qrv;

		for(int i=1;i<200;i++){
			t[i]=t[i-1]+0.05;
		}	
		for(int i=1;i<10;i++){
			
			qv_x=60 - 25*cos(t[i]);
			qv_y=30 + 25*sin(t[i]);
			
			qv[i][0]=qv_x;
			qv[i][1]=qv_y;

			//std::cout<<qv[2][0];
			
			
			for(int j=0;j<n;j++){
				qt_diff[i][j]=qv[i][j] - qv[i-1][j];
				
			}
			theta_t[i][0]=atan2(qt_diff[i][1], qt_diff[i][0]);
			
			//problem - returning 'nan'			
			phi[i]=atan2(qrv[i-1][1], qrv[i-1][0]);
			//std::cout<<phi[3];

			norm_qrv=sqrt(pow(qrv[i-1][0],2)+pow(qrv[i-1][1],2));
			//std::cout<<sqrt(pow(qrv[1][0],2)+pow(qrv[1][1],2));


			//fixing the robot velocity to fixed value as virtual target velocity
			//pr[i][0]=sqrt(pow(pv,2) + 2*lambda*norm_qrv*pv*cos(theta_t[i][0]-phi[i] + pow(lambda,2)*pow(norm_qrv,2)));	
			//std::cout<<pr[2][0];
					
			theta_r[i][0]=phi[i] + asin(pv*sin(theta_t[i][0] - phi[i])/pr[i][0]);
			//std::cout<<phi[i];
			//std::cout<<(pv*sin(theta_t[1][0] - phi[1])/pr[1][0])<<"-";

			//update position and velocity of robot
			//qr[i][0]=qr[i-1][0] + pr[i][0]*delta_t*cos(theta_r[i-1][0]);
			//qr[i][1]=qr[i-1][1] + pr[i][0]*delta_t*sin(theta_r[i-1][0]);

			//std::cout<<qr[i][1];

			qrv[i][0]=qv[i][0] - qr[i][0];
			qrv[i][1]=qv[i][1] - qr[i][1];
			//std::cout<<pr[2][0]; //<- returning nan -1
			//std::cout<<qrv[1][0];
			//std::cout<<qr[i][0];

			geometry_msgs::Twist msg;
           
	   		//msg.linear.x=0.2;
	   		//msg.angular.z=-0.3;

			//msg.linear.x=qr[i][0];
			//msg.linear.y=qr[i][1];
	   		//msg.angular.z=qr[i][1];
			msg.linear.x=pv;
			msg.linear.y=pv;
	   		msg.angular.z=theta_r[i][0];
	   		movement_pub.publish(msg);
			
			for(int j=0; j<n;j++){
					qrv[i][j]=qv[i][j] - qr[i][j];
			}
			
			prv[i][0]=pv*cos(theta_t[i][0]) - pr[i][0]*cos(theta_r[i][0]);
			prv[i][1]=pv*sin(theta_t[i][0]) - pr[i][0]*sin(theta_r[i][0]);

						
		}
		ros::spinOnce(); 
		rate.sleep();
	   	
	}
	
}
