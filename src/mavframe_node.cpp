/**
* @file offb_node.cpp
* @brief offboard example node, written with mavros version 0.14.2, px4 flight
* stack and tested in Gazebo SITL
*/

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <string.h>
#include <math.h>

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

	double loopRate = 50.0;
	bool sendData = false;

	std::string inputFrame = "/fcu";
	std::string localFrame = "/world";
	std::string robotName = "/uav";

	if( !nh.getParam( "loop_rate", loopRate ) ) {
		ROS_WARN( "No parameter set for \"loop_rate\", using: %0.2f", loopRate );
	}
	ROS_INFO( "Update rate: %0.2f", loopRate );

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

	std::string baseFrame = robotName + "/base_link";
	std::string flatFrame = robotName + "/hdg_link";
	std::string attFrame = robotName + "/att_link";

	ROS_INFO( "Ouput Frame (base_link): %s", baseFrame.c_str() );
	ROS_INFO( "Ouput Frame (hdg_link): %s", flatFrame.c_str() );
	ROS_INFO( "Ouput Frame (attitude): %s", attFrame.c_str() );

	//Initialize TF
	tf::TransformListener tfln;
	tf::TransformBroadcaster tfbr;

	ros::Rate rate(loopRate);

	bool tfReady = false;

	ROS_INFO( "Waiting for transforms..." );

	while( ros::ok() && !tfReady ) {
		tfReady = tfln.waitForTransform( localFrame, inputFrame, ros::Time::now(), ros::Duration(1.0) );
		ros::spinOnce();
		rate.sleep();
	}

	sleep(1);	//Just to let the TF buffer fill a little

	ROS_INFO( "...Broadcasting!" );
	sendData = true;

	//================================//
	// Main Loop                      //
	//================================//
	while( ros::ok() ) {

		tf::StampedTransform worldTransform;
		tf::Transform baseTransform;
		tf::Transform flatTransform;
		tf::Transform attTransform;

		ros::Time readTime = ros::Time::now();
		//========================================================================

		try {
			//Wait a maximum of 1 timestep for the new rotation
			tfln.waitForTransform(localFrame, inputFrame, readTime, ros::Duration(1/loopRate));
			tfln.lookupTransform(localFrame, inputFrame, readTime, worldTransform);
		}

		catch (tf::TransformException ex) {
			ROS_ERROR( "%s",ex.what() );
			sendData = false;
		}

		if( sendData ) {
			//base_link		================================================================
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
			tfbr.sendTransform( tf::StampedTransform( baseTransform, readTime, localFrame, baseFrame ) );
			tfbr.sendTransform( tf::StampedTransform( flatTransform, readTime, baseFrame, flatFrame ) );
			tfbr.sendTransform( tf::StampedTransform( attTransform, readTime, baseFrame, attFrame ) );

			//Info			================================================================
		} else {
			ROS_ERROR_THROTTLE(1.0, "Some data cannot be obtained, not publishing data..." );
		}

		//Sleep				================================================================
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

