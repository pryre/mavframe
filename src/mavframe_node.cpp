/**
* @file offb_node.cpp
* @brief offboard example node, written with mavros version 0.14.2, px4 flight
* stack and tested in Gazebo SITL
*/

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <string.h>
#include <math.h>

geometry_msgs::Vector3 toEuler(geometry_msgs::Quaternion q) {
    geometry_msgs::Vector3 e;

    double q2sqr = q.y * q.y;
    double t0 = -2.0 * (q2sqr + q.z * q.z) + 1.0;
    double t1 = +2.0 * (q.x * q.y + q.w * q.z);
    double t2 = -2.0 * (q.x * q.z - q.w * q.y);
    double t3 = +2.0 * (q.y * q.z + q.w * q.x);
    double t4 = -2.0 * (q.x * q.x + q2sqr) + 1.0;

    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    e.x = atan2(t3, t4);
    e.y = asin(t2);
    e.z = atan2(t1, t0);

    return e;
}

geometry_msgs::Quaternion toQuaternion(geometry_msgs::Vector3 e) {
    geometry_msgs::Quaternion q;

    double t0 = cos(e.z * 0.5);
    double t1 = sin(e.z * 0.5);
    double t2 = cos(e.x * 0.5);
    double t3 = sin(e.x * 0.5);
    double t4 = cos(e.y * 0.5);
    double t5 = sin(e.y * 0.5);

    q.w = t2 * t4 * t0 + t3 * t5 * t1;
    q.x = t3 * t4 * t0 - t2 * t5 * t1;
    q.y = t2 * t5 * t0 + t3 * t4 * t1;
    q.z = t2 * t4 * t1 - t3 * t5 * t0;

	return q;
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

	double loopRate = 20.0;
	double hdg = 0.0;
	bool sendData = false;

	//Initialize TF
	tf::TransformListener tfln;
	tf::TransformBroadcaster tfbr;

	//Publishers
	ros::Publisher body_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("body_twist", 10);
	ros::Publisher world_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("world_twist", 10);

	geometry_msgs::TwistStamped bodyVelOut;
	bodyVelOut.header.frame_id = "fcu";
	geometry_msgs::TwistStamped worldVelOut;
	worldVelOut.header.frame_id = "world";
	ros::Rate rate(loopRate);

	bool tfReady = false;

	ROS_INFO( "Waiting for transforms..." );

	while( ros::ok() && !tfReady ) {
		tfReady = tfln.waitForTransform( "/world", "/fcu", ros::Time::now(), ros::Duration(1.0) );
		ros::spinOnce();
		rate.sleep();
	}

	sleep(1);	//Just to let the TF buffer fill a little

	ROS_INFO( "Ready to begin broadcasting!" );

	//================================//
	// Main Loop                      //
	//================================//
	while( ros::ok() ) {
		sendData = true;

		tf::StampedTransform worldTransform;
		tf::Transform baseTransform;
		tf::Transform refTransform;
		geometry_msgs::Twist worldVelocity;
		geometry_msgs::Twist bodyVelocity;

		//Listen:
		//	UAV in would frame
		//Broadcast:
		//	uav/ref_link	- UAV position, no heading
		//	uav/base_link	- UAV position, current heading only
		//Publish:
		//	~/twist_world	- Current velocity of the UAV in the world frame
		//	~/twist_body	- Current velocity of the UAV in the body frame

		//========================================================================

		try {
			//Get the latest
			tfln.lookupTransform("/world", "/fcu", ros::Time(0), worldTransform);

			//Get the latest twist (velocity) estimate of the fcu relative to the world
			tfln.lookupTwist("/fcu", "/world", "/fcu", tf::Point(), "/world", ros::Time(0), ros::Duration(0.5), worldVelocity);
					}

		catch (tf::TransformException ex) {
			ROS_ERROR( "%s",ex.what() );
			sendData = false;
		}

		if( sendData ) {
			//ref_link		================================================================
			refTransform.setOrigin( worldTransform.getOrigin() );
			refTransform.setRotation( tf::Quaternion(0.0, 0.0, 0.0, 1.0) ); //Set it to zero heading

			//base_link		================================================================
			baseTransform.setOrigin( tf::Vector3() );

			geometry_msgs::Quaternion q;
			tf::quaternionTFToMsg( worldTransform.getRotation(), q );
			tf::Quaternion qtf;
			qtf.setRPY( 0.0, 0.0, toEuler(q).z );
			baseTransform.setRotation( qtf ); //Set it to current heading


			//Twist World	================================================================
			//This is gotten when we look up the original twist
			//So no work to do here

			tfbr.sendTransform( tf::StampedTransform( refTransform, ros::Time::now(), "world", "fcu/ref" ) );
			tfbr.sendTransform( tf::StampedTransform( baseTransform, ros::Time::now(), "fcu/ref", "fcu/base_link" ) );
			/*
			try {
					//Get the latest twist (velocity) estimate of the fcu relative to the world
					tfln.lookupTwist("/world", "/fcu/base_link", "/world", tf::Point(), "/fcu/base_link", ros::Time(0), ros::Duration(0.5), bodyVelocity);
			}

			catch (tf::TransformException ex) {
				ROS_ERROR( "%s",ex.what() );
				sendData = false;
			}*/

			//Twist Body	================================================================
			//Flip the obtained velocity to align with the body frame
			//bodyVelocity.linear.x = -bodyVelocity.linear.x;
			//bodyVelocity.linear.y = -bodyVelocity.linear.y;
			//bodyVelocity.linear.z = -bodyVelocity.linear.z;


			//Output		================================================================
			//bodyVelOut.header.seq ++;
			//bodyVelOut.header.stamp = ros::Time::now();
			//bodyVelOut.twist = bodyVelocity;
			worldVelOut.header.seq ++;
			worldVelOut.header.stamp = ros::Time::now();
			worldVelOut.twist = worldVelocity;

			//body_vel_pub.publish(bodyVelOut);
			world_vel_pub.publish(worldVelOut);

			//Info			================================================================
			ROS_INFO_THROTTLE(1.0, "World Frame Position: [%0.2f, %0.2f, %0.2f]", worldTransform.getOrigin().getX(), worldTransform.getOrigin().getY(), worldTransform.getOrigin().getZ());
			ROS_INFO_THROTTLE(1.0, "World Frame Velocity:\n\tLinear: [%0.2f, %0.2f, %0.2f]", worldVelocity.linear.x, worldVelocity.linear.y, worldVelocity.linear.z);
			ROS_INFO_THROTTLE(1.0, "Body Frame Velocity:\n\tLinear: [%0.2f, %0.2f, %0.2f]", bodyVelocity.linear.x, bodyVelocity.linear.y, bodyVelocity.linear.z);
		} else {
			ROS_ERROR_THROTTLE(1.0, "Some data cannot be obtained, not publishing data..." );
		}

		//Sleep				================================================================
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

