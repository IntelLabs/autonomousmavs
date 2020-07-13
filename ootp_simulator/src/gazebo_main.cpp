/**
 * @file   gazebo_main.hpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   May, 2018
 * @brief  gazebo simulation
 * @section LICENSE
 *
 *  BSD-3-Clause License
 *
 * @copyright Copyright (C) 2020 Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *  
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
**/ 

#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <thread>         // std::thread

#include <ootp/actions.hpp>
#include <ootp_ros/initializer.hpp>
#include <rlmap_ros/initializer.hpp>
#include <rlmap_ros/visualizer.hpp>
#include <ootp_simulator/simulation_tools.hpp>

namespace ootp
{

void toEulerAngle(const quat& q, double& roll, double& pitch, double& yaw);

class OOTPNode
{
private:
	rlmap::Visualizer 		map_visualizer_;

	nav_msgs::Odometry 		current_pose_;
	std::thread 			*spin_drone_thread_;

	bool initialized_pos_;
	bool initialized_ori_;
	void publish_pose(void);

public:
	OOTPNode();
	virtual~OOTPNode();
	void depthCb(const sensor_msgs::ImageConstPtr& msg);
	void poseCb(const nav_msgs::Odometry& msg);
	void start_drone_pose_thread(void);

	void set_goal(const Point &goal);

	ros::Publisher 			traj_pub_;
	ros::Publisher 			state_machine_;
	Point 					position_goal_;
	Point 					orientation_goal_;
	double					rate_;

	Actions					*actions_hand;
};

OOTPNode::OOTPNode(): orientation_goal_(0.0,0.0,0.0)
{
	initialized_pos_ = false;
	initialized_ori_ = false;
	rate_ = 0.01;
	spin_drone_thread_ = NULL;
}
OOTPNode::~OOTPNode()
{
	if(spin_drone_thread_ != NULL)
	{
		spin_drone_thread_->join();
	}

}

void OOTPNode::set_goal(const Point &goal)
{
	position_goal_ = goal;
}

void OOTPNode::poseCb(const nav_msgs::Odometry& msg)
{
	current_pose_ 		= msg;
	initialized_pos_ 	= true;
	initialized_ori_	= true;
}
void OOTPNode::start_drone_pose_thread(void)
{
	spin_drone_thread_ 	= new std::thread(&OOTPNode::publish_pose, this);
}
void OOTPNode::publish_pose(void)
{
	MatrixXD states;
	double yaw;

	geometry_msgs::PoseStamped msg;
	geometry_msgs::PoseStamped msg_pub;
	ootp::eStates 	state_value;

	while(ros::ok())
	{
		actions_hand->GetMovement(states, yaw);
		Point	pose = states.col(0);

		ootp::publish_goal( pose, yaw, traj_pub_);
		actions_hand->GetState(state_value);

		msg_pub.header.stamp = ros::Time::now();
		msg_pub.pose.position.x = (int)state_value;
		msg_pub.pose.orientation.x = pose(0);
		msg_pub.pose.orientation.y = pose(1);
		msg_pub.pose.orientation.z = pose(2);
		msg_pub.pose.orientation.w = yaw;

		state_machine_.publish(msg_pub);

		PointList path;
		actions_hand->get_local_path(path);
		map_visualizer_.PublishPath(path);

		actions_hand->get_path_consumed(path);
		map_visualizer_.PublishPathConsumed(path);

		PointList points;
		actions_hand->get_raw_points(points);
		map_visualizer_.PublishRawPoints(points);


		ros::Duration(rate_).sleep(); // sleep for 10 ms
	}
}


void OOTPNode::depthCb(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		if(initialized_pos_ && initialized_ori_)
		{
			initialized_pos_ = false;
			initialized_ori_ = false;

			rlmap::FrameObj dst = (cv_bridge::toCvShare(msg, "16UC1")->image).clone();

			Point position;
			position(0) = current_pose_.pose.pose.position.x;
			position(1) = current_pose_.pose.pose.position.y;
			position(2) = current_pose_.pose.pose.position.z;
			quat orientation(current_pose_.pose.pose.orientation.w, current_pose_.pose.pose.orientation.x,current_pose_.pose.pose.orientation.y,
				current_pose_.pose.pose.orientation.z);

			double roll, pitch, yaw;
			toEulerAngle(orientation, roll, pitch, yaw);

			Transform	pose(position, Point(roll, pitch, yaw) );

			actions_hand->AddScene(dst, pose, msg->header.stamp.toSec());

			rlmap::stdVoxelVector map_occup;
			actions_hand->get_occupied_map(map_occup);
			map_visualizer_.PublishOccupiedMap(map_occup);

		}

	}catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
}

} /* namespace ootp */

int main(int argc, char **argv)
{

	ros::init(argc, argv, "ootp_gazebo");
	ROS_INFO( "created ootp_gazebo \n");

	ros::NodeHandle node_nh("~");


	rlmap::InitializeParams(node_nh);
	ootp::InitializeParams(node_nh);

	ROS_INFO( "params initiated \n");

	ootp::OOTPNode ootp_node;

	node_nh.getParam("dtime", ootp_node.rate_);

	//check for ros log status...
	bool is_debug;
	node_nh.getParam("debugging_msg", is_debug);
	if(is_debug)
	{
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	}else
	{
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
	}

	ros::NodeHandle node_n;
	ros::Subscriber sub 		= node_n.subscribe("/rs0r200/camera/depth/image_raw", 1, &ootp::OOTPNode::depthCb, &ootp_node);
	ros::Subscriber sub_pose 	= node_n.subscribe("/hummingbird/odometry_sensor1/odometry", 1, &ootp::OOTPNode::poseCb, &ootp_node);
	ootp_node.traj_pub_ 		= node_n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/hummingbird/command/trajectory", 10);

	ootp_node.state_machine_	= node_n.advertise<geometry_msgs::PoseStamped>("/state_machine", 1);

	ROS_DEBUG("publishing goal in : %s", "/hummingbird/command/trajectory");

	// initializing gazebo physics engine
	if ( !ootp::init_gazebo_engine() )
	{
		ROS_FATAL("Could not wake up Gazebo.");
		return 0;
	}
	// sending initial position
	ootp::Point position(0.0,0.0,0.7);
	ootp::publish_goal( position, 0.0, ootp_node.traj_pub_);

	ootp_node.actions_hand = new ootp::Actions( position );


	ROS_INFO(" Starting drone's initial position at origin");
	// Wait for drone to start
	ros::Duration(2).sleep();

	// starting new goal ootp algo
	ootp::Point goal(25.0, 25.0, 1.0);
	ROS_INFO( "starting exploration algorithm with goal at : %f, %f, %f \n", goal(0), goal(1), goal(2));

	ootp_node.set_goal(goal);
	ootp_node.actions_hand->UpdateGoal(goal, true);

	ootp_node.start_drone_pose_thread();

	ros::spin();

	ROS_INFO("ootp_gazebo terminated.\n");
	return 0;
}
