#include<ros/ros.h>
#include<string>
#include<sensor_msgs/JointState.h>
#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<chapter5_tutorials/odom_arduino.h>

ros::Time current_time_encoder;
ros::Time last_time_encoder;

const double d=0.14;
double dist_per_count=0.041; //in meters
double previous_left_ticks=0;
double previous_right_ticks=0;
	// initial position
double x = 0.0; 
double y = 0.0;
double th = 0;

	// velocity
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
void callback(const chapter5_tutorials::odom_arduino& ticks)
{   
	double V,W,theta;
    current_time_encoder = ros::Time::now();
	ROS_INFO_STREAM("Time:"<<current_time_encoder<<std::endl);

    double delta_left = ticks.vel_left - previous_left_ticks;
    double delta_right = ticks.vel_right - previous_right_ticks;
	
    // dist_per_count = distance traveled per count, delta_left = ticks moved
    double vel_left = (delta_left * dist_per_count) / (current_time_encoder - last_time_encoder).toSec(); // Left velocity
    double vel_right = (delta_right * dist_per_count) / (current_time_encoder - last_time_encoder).toSec(); // Right velocity

    // Getting Translational and Rotational velocities from Left and Right wheel velocities
    // V = Translation vel. W = Rotational vel.
    if (vel_left == vel_right)
    {
        V = vel_left;
        W = 0;
    }

    else
    {
            // Assuming the robot is rotating about point A   
        // W = vel_left/r = vel_right/(r + d), see the image below for r and d
        double r = (vel_left * d) / (vel_right - vel_left); // Anti Clockwise is positive
        W = vel_left/r; // Rotational velocity of the robot
		theta=W*(current_time_encoder - last_time_encoder).toSec();
        V = W * (r + d/2); // Translation velocity of the robot
    }

    vth = W;
    vx=V*cos(theta);
	vy=V*sin(theta);
    th=theta;
  /*  ROS_INFO_STREAM("Vx:"<<vx<<std::endl);
	ROS_INFO_STREAM("Vy:"<<vy<<std::endl);
	ROS_INFO_STREAM("Vth:"<<vth<<std::endl);
	ROS_INFO_STREAM("th"<<theta<<std::endl);*/
    previous_left_ticks = ticks.vel_left;
    previous_right_ticks = ticks.vel_right;

    last_time_encoder = current_time_encoder;
}
 
 
int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
	ros::Subscriber arduino_subscriber=n.subscribe("odom_data",10,callback);

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
 
	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	const double degree = M_PI/180;

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "link_chassis";
	while (ros::ok()) 
	{
		current_time = ros::Time::now(); 

		double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		geometry_msgs::Quaternion odom_quat;	
		odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

		// update transform
		odom_trans.header.stamp = current_time; 
		odom_trans.transform.translation.x = x; 
		odom_trans.transform.translation.y = y; 
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

		//filling the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "link_chassis";

		// position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//velocity
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = vth;

		last_time = current_time;

		// publishing the odometry and the new tf
		broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);

		loop_rate.sleep();
	}
	//ros::spin();
    return 0;
}