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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <thread>         // std::thread
#include <termios.h>

#include <ootp/actions.hpp>
#include <ootp_ros/initializer.hpp>
#include <rlmap_ros/initializer.hpp>
#include <rlmap_ros/visualizer.hpp>

namespace ootp
{

class OOTPNode
{
private:
	rlmap::Visualizer 		map_visualizer_;
	std::thread 			*spin_drone_thread_;

	Point position_;
	Point orientation_;

	bool initialized_pos_;
	bool initialized_ori_;

	void publish_pose(void);

public:
	OOTPNode();
	virtual~OOTPNode();
	void depthCb(const sensor_msgs::ImageConstPtr& msg);
	void EulerAnglesCb(const geometry_msgs::Vector3Stamped& msg );
	void PosCb(const geometry_msgs::PoseStamped& msg);
	void start_drone_pose_thread(void);

	void set_goal(const Point &goal);
	void UpZRoutine(void);

	ros::Publisher 			reference_;
	ros::Publisher 			dreference_;
	ros::Publisher 			ddreference_;

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
void OOTPNode::PosCb(const geometry_msgs::PoseStamped& msg)
{
	position_(0) = msg.pose.position.x;
	position_(1) = msg.pose.position.y;
	position_(2) = msg.pose.position.z;
	initialized_pos_ 	= true;
}

void OOTPNode::EulerAnglesCb(const geometry_msgs::Vector3Stamped& msg )
{
	orientation_(0) = msg.vector.x;
	orientation_(1) = msg.vector.y;
	orientation_(2) = msg.vector.z;
	initialized_ori_	= true;
}

void OOTPNode::start_drone_pose_thread(void)
{
	spin_drone_thread_ 	= new std::thread(&OOTPNode::publish_pose, this);
}
void OOTPNode::UpZRoutine(void)
{
	double point = 0.1;
	while(true)
	{
		geometry_msgs::PoseStamped msg_control;
		msg_control.header.stamp = ros::Time::now();
		if(reference_.getNumSubscribers() > 0)
		{
			msg_control.pose.position.x =  0.0;
			msg_control.pose.position.y =  0.0;
			msg_control.pose.position.z =  point;
			point += 0.05;
			reference_.publish(msg_control);
			ROS_DEBUG("Send : %f mts", point);
			if(point >= 0.7)
			{
				ROS_INFO("UpZ routine end ...");
				return;
			}
		}

		ros::Duration(0.2).sleep(); // sleep for 10 ms
	}
}
void OOTPNode::publish_pose(void)
{
	MatrixXD states;
	double yaw;

	geometry_msgs::PoseStamped msg;
	geometry_msgs::PoseStamped msg_pub;
	ootp::eStates 	state_value;
	Point			state_yaw;

	actions_hand->StartTimer();

	while(ros::ok())
	{
		actions_hand->GetMovement(states, state_yaw);

		geometry_msgs::PoseStamped msg_control;
		msg_control.header.stamp = ros::Time::now();
		if(reference_.getNumSubscribers() > 0)
		{
			Point hand = states.col(kPos);
			msg_control.pose.position.x =  hand(0);
			msg_control.pose.position.y =  hand(1);
			msg_control.pose.position.z =  hand(2);

			msg_control.pose.orientation.z = state_yaw(0);
			reference_.publish(msg_control);
		}
		if(dreference_.getNumSubscribers() > 0)
		{
			Point hand = states.col(kVel);
			msg_control.pose.position.x =  hand(0);
			msg_control.pose.position.y =  hand(1);
			msg_control.pose.position.z =  hand(2);

			msg_control.pose.orientation.z = state_yaw(1);
			dreference_.publish(msg_control);
		}
		if(ddreference_.getNumSubscribers() > 0)
		{
			Point hand = states.col(kAcc);
			msg_control.pose.position.x =  hand(0);
			msg_control.pose.position.y =  hand(1);
			msg_control.pose.position.z =  hand(2);

			msg_control.pose.orientation.z = state_yaw(2);
			ddreference_.publish(msg_control);
		}

		ros::Duration(rate_).sleep();
	}
}

void OOTPNode::depthCb(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{

		if(initialized_pos_ && initialized_ori_)
		{
			//ROS_INFO("Received new frame with timestamp %f", msg->header.stamp.toSec());

			initialized_pos_ = false;
			initialized_ori_ = false;

			rlmap::FrameObj dst = (cv_bridge::toCvShare(msg, "16UC1")->image).clone();

			Transform	pose(position_, orientation_ );

			actions_hand->AddScene(dst, pose, msg->header.stamp.toSec());

			rlmap::stdVoxelVector map_occup;
			actions_hand->get_occupied_map(map_occup);
			map_visualizer_.PublishOccupiedMap(map_occup);

			PointList points;
			actions_hand->get_raw_points(points);
			map_visualizer_.PublishRawPoints(points);

			//PointList points;
			actions_hand->get_local_path(points);
			map_visualizer_.PublishPath(points);

			actions_hand->get_path_consumed(points);
			map_visualizer_.PublishPathConsumed(points);
		}

	}catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
}

} /* namespace ootp */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ootp_node");
	ROS_INFO( "created ootp_node \n");
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
	ros::Subscriber sub 		= node_n.subscribe("/depth/image_raw", 1, &ootp::OOTPNode::depthCb, &ootp_node);

	ros::Subscriber sub_euler 	= node_n.subscribe("/euler", 1, &ootp::OOTPNode::EulerAnglesCb, &ootp_node);

	ros::Subscriber sub_pos 	=  node_n.subscribe("/odom_control/pose", 1, &ootp::OOTPNode::PosCb, &ootp_node);

	ootp_node.reference_ 		= node_n.advertise<geometry_msgs::PoseStamped>("/aero/ref_pos", 1, true);
	ootp_node.dreference_ 		= node_n.advertise<geometry_msgs::PoseStamped>("/aero/ref_vel", 1, true);
	ootp_node.ddreference_ 		= node_n.advertise<geometry_msgs::PoseStamped>("/aero/ref_acc", 1, true);

	ROS_DEBUG("publishing goals in : %s", "/aero/ref_pos");

	std::cout<<"starting node ..."<<std::endl;

	// sending initial position
	ootp::Point position(0.0,0.0,0.7);
	ootp_node.actions_hand = new ootp::Actions( position );

	std::vector<double> goal_vector;
	node_nh.getParam("goal", goal_vector);
	// starting new goal ootp algo
	ootp::Point goal(goal_vector[0], goal_vector[1], goal_vector[2]);
	ROS_INFO( "starting exploration algorithm with goal at : %f, %f, %f \n", goal(0), goal(1), goal(2));
	ootp_node.actions_hand->UpdateGoal(goal, true);

	ootp_node.start_drone_pose_thread();

	ros::spin();

	ROS_INFO("ootp_node terminated.\n");
	return 0;
}
