/**
* @file offb_node.cpp
* @brief offboard example node, written with mavros version 0.14.2, px4 flight
* stack and tested in Gazebo SITL
*/

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <string.h>
#include <math.h>


tf::TransformBroadcaster* tfbr;

std::string inputFrame = "/fcu";
std::string localFrame = "/world";
std::string robotName = "/uav";

std::string baseFrame;
std::string flatFrame;
std::string attFrame;


void pos_cb(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	ROS_INFO_ONCE( "...Broadcasting!" );

	tf::StampedTransform worldTransform;
	tf::transformStampedMsgToTF(*msg, worldTransform);

	tf::Transform baseTransform;
	tf::Transform flatTransform;
	tf::Transform attTransform;
	//========================================================================

	baseTransform.setOrigin( worldTransform.getOrigin() );

	baseTransform.setRotation( tf::Quaternion(0.0, 0.0, 0.0, 1.0) ); //Set it to zero heading

	//flat_link		================================================================
	flatTransform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );

	geometry_msgs::Vector3 rot;
	tf::Matrix3x3(worldTransform.getRotation()).getRPY(rot.x, rot.y, rot.z);
	flatTransform.setRotation(tf::createQuaternionFromYaw(rot.z));

	//att_link		================================================================
	attTransform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );

	attTransform.setRotation(worldTransform.getRotation());

	//Send Transforms	============================================================
	tfbr->sendTransform( tf::StampedTransform( baseTransform, worldTransform.stamp_, localFrame, baseFrame ) );
	tfbr->sendTransform( tf::StampedTransform( flatTransform, worldTransform.stamp_, baseFrame, flatFrame ) );
	tfbr->sendTransform( tf::StampedTransform( attTransform, worldTransform.stamp_, baseFrame, attFrame ) );
}

//================================//
// Main Function                  //
//================================//
int main(int argc, char **argv)
{
	//================================//
	// Initialize node                //
	//================================//
	ros::init(argc, argv, "mavframe" );
	ros::NodeHandle nh( ros::this_node::getName() );

	if( !nh.getParam( "input_frame", inputFrame ) ) {
		ROS_WARN( "No parameter set for \"input_frame\", using: %s", inputFrame.c_str() );
	}
	ROS_INFO( "Input transformation: %s", inputFrame.c_str() );

	if( !nh.getParam( "local_frame", localFrame ) ) {
		ROS_WARN( "No parameter set for \"local_frame\", using: %s", localFrame.c_str() );
	}
	ROS_INFO( "Local frame: %s", localFrame.c_str() );

	if( !nh.getParam( "robot_name", robotName ) ) {
		ROS_WARN( "No parameter set for \"robot_name\", using: %s", robotName.c_str() );
	}
	ROS_INFO( "Robot name: %s", robotName.c_str() );

	baseFrame = robotName + "/base_link";
	flatFrame = robotName + "/hdg_link";
	attFrame = robotName + "/att_link";

	ROS_INFO( "Ouput Frame (base_link): %s", baseFrame.c_str() );
	ROS_INFO( "Ouput Frame (hdg_link): %s", flatFrame.c_str() );
	ROS_INFO( "Ouput Frame (attitude): %s", attFrame.c_str() );

	//Initialize TF
	tfbr = new tf::TransformBroadcaster();

	ros::Subscriber waypoint_sub = nh.subscribe<geometry_msgs::TransformStamped>
		(inputFrame, 10, pos_cb);

	ROS_INFO( "Waiting for transforms..." );

	ros::spin();

	return 0;
}

