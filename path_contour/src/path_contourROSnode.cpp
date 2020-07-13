/**
 * @file   path_contourROSnode.cpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   May, 2018
 * @brief  Ros wrapper for trajectory generation algorithm implementation
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
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>
#include "path_contour/path_contour.h"

typedef std::vector<Eigen::Vector3d> PointList;

contour::PathDynamics			*traj_handler;
PointList						path_waypoints;

Eigen::Vector3d					yaw_waypoint;
Eigen::Vector3d					position_waypoint;
double							waypoint_lengh;

Eigen::VectorXd pos;
Eigen::VectorXd vel;
Eigen::VectorXd acc;
double yaw;

ros::Publisher pos_error;
ros::Publisher vel_error;
ros::Publisher yaw_error;

Eigen::VectorXd Vector2Eigen(std::vector<double> vector)
{
	Eigen::VectorXd value(vector.size());
	for(int idx=0; idx < vector.size() ; idx++)
	{
		value(idx) = vector[idx];
	}
	return value;
}

void posCb(const geometry_msgs::PoseStamped& msg)
{
	geometry_msgs::PoseStamped msg_pub;
	msg_pub.header.stamp = ros::Time::now();

	msg_pub.pose.position.x = msg.pose.position.x + pos(0);
	msg_pub.pose.position.y = msg.pose.position.y + pos(1);
	msg_pub.pose.position.z = msg.pose.position.z - pos(2);
	pos_error.publish(msg_pub);
}
void velCb(const geometry_msgs::PoseStamped& msg)
{
	geometry_msgs::PoseStamped msg_pub;
	msg_pub.header.stamp = ros::Time::now();

	msg_pub.pose.position.x = msg.pose.position.x + vel(0);
	msg_pub.pose.position.y = msg.pose.position.y + vel(1);
	msg_pub.pose.position.z = msg.pose.position.z - vel(2);
	vel_error.publish(msg_pub);
}
void yawCb(const geometry_msgs::Vector3Stamped& msg)
{
	geometry_msgs::Vector3Stamped euler_msg;
	euler_msg.header.stamp = ros::Time::now();

	euler_msg.vector.x = 0.0;
	euler_msg.vector.y = 0.0;
	euler_msg.vector.z = msg.vector.z - yaw;

	yaw_error.publish(euler_msg);
}

bool GetWaypoint2Filter(Eigen::Vector3d &point, Eigen::Vector3d &yaw);
std::vector<Eigen::VectorXd> ParametrizePath(const PointList &path );

int main(int argc, char *argv[])
{
	ros::init(argc, argv,"path_contour_node");
	ros::NodeHandle n;
	ROS_INFO("created path_contour_node");

	ros::Publisher reference = n.advertise<geometry_msgs::PoseStamped>("/aero/ref_pos", 1, true);
	ros::Publisher dreference = n.advertise<geometry_msgs::PoseStamped>("/aero/ref_vel", 1, true);
	ros::Publisher ddreference = n.advertise<geometry_msgs::PoseStamped>("/aero/ref_acc", 1, true);

	ros::NodeHandle nh("~");
	std::vector<double> vector_hand;

	double arc_velocity;
	double path_error;
	double yaw_error;
	double rho;
	double y_rho;
	double amax;
	double vmax;
	double gain;

	Eigen::VectorXd path_gains(5);
	Eigen::VectorXd yaw_gains(2);
	Eigen::VectorXd position(3);
	nh.getParam("arc_velocity", arc_velocity);
	nh.getParam("path_error", path_error);
	nh.getParam("yaw_error", yaw_error);
	nh.getParam("yaw", yaw);
	nh.getParam("rho", rho);
	nh.getParam("y_rho", y_rho);

	nh.getParam("amax", amax);
	nh.getParam("vmax", vmax);
	nh.getParam("gain", gain);

	nh.getParam("path_gains", vector_hand);
	path_gains = Vector2Eigen(vector_hand);

	nh.getParam("yaw_gains", vector_hand);
	yaw_gains << vector_hand[0], vector_hand[1];

	nh.getParam("position", vector_hand);
	position << vector_hand[0], vector_hand[1], vector_hand[2];

	traj_handler		= new contour::PathDynamics(3, arc_velocity, rho, y_rho, vmax, amax, gain );
	traj_handler->SetGains( path_gains, yaw_gains, path_error );


	double dynamics_iterations;
	nh.getParam("dynamics_iterations", dynamics_iterations);
	double rate;
	nh.getParam("dtime", rate);
	nh.getParam("waypoint_lengh", waypoint_lengh);

	position_waypoint 	= position;
	yaw_waypoint		<< cos(yaw), sin(yaw),0.0;

	traj_handler->SetInitialPose( position, path_error, yaw, yaw_waypoint, yaw_error );

	path_waypoints.push_back(Eigen::Vector3d( 0.0, 0.0, position(2)));
	path_waypoints.push_back(Eigen::Vector3d( 1.0, 0.0, position(2)));
	path_waypoints.push_back(Eigen::Vector3d( 1.0, 1.0, position(2)));
	path_waypoints.push_back(Eigen::Vector3d( 0.0, 1.0, position(2)));
	path_waypoints.push_back(Eigen::Vector3d( 0.0, 0.0, position(2)));
	path_waypoints.push_back(Eigen::Vector3d( 1.0, 0.0, position(2)));
	path_waypoints.push_back(Eigen::Vector3d( 1.0, 1.0, position(2)));
	path_waypoints.push_back(Eigen::Vector3d( 0.0, 1.0, position(2)));
	path_waypoints.push_back(Eigen::Vector3d( 0.0, 0.0, position(2)));

	int counter = 0;
	pos = position;
	vel = Eigen::Vector3d::Zero();
	acc = Eigen::Vector3d::Zero();
	while(counter < 1000)
	{
		geometry_msgs::PoseStamped msg_control;
		msg_control.header.stamp = ros::Time::now();
		//if(reference.getNumSubscribers() > 0)
		{
			msg_control.pose.position.x =  pos(0);
			msg_control.pose.position.y =  pos(1);
			msg_control.pose.position.z =  pos(2);

			msg_control.pose.orientation.z = yaw;
			reference.publish(msg_control);
		}
		//if(dreference.getNumSubscribers() > 0)
		{
			msg_control.pose.position.x =  vel(0);
			msg_control.pose.position.y =  vel(1);
			msg_control.pose.position.z =  vel(2);

			msg_control.pose.orientation.z = 0;
			dreference.publish(msg_control);
		}
		//if(ddreference.getNumSubscribers() > 0)
		{
			msg_control.pose.position.x =  acc(0);
			msg_control.pose.position.y =  acc(1);
			msg_control.pose.position.z =  acc(2);

			msg_control.pose.orientation.z = 0;
			ddreference.publish(msg_control);
		}
		ros::Duration(rate).sleep(); // sleep
		counter++;
	}

	while(ros::ok())
	{
		Eigen::Vector3d	position_ref;
		Eigen::Vector3d yaw_ref;
		if( GetWaypoint2Filter(position_ref, yaw_ref))
		{
			yaw_waypoint = yaw_ref;
			position_waypoint = position_ref;
			traj_handler->PushWaypoint(position_ref, path_error, yaw_ref, yaw_error );
		}

		contour::State st;
		for(int step = 0 ; step < dynamics_iterations ; step++)
		{
			st = traj_handler->UpdateDynamics(rate); // fix step time
		}
		pos = st.position;
		vel = st.velocity*dynamics_iterations;
		acc = st.acceleration*dynamics_iterations*dynamics_iterations;
		yaw = st.yaw;

		geometry_msgs::PoseStamped msg_control;
		msg_control.header.stamp = ros::Time::now();
		//if(reference.getNumSubscribers() > 0)
		{
			msg_control.pose.position.x =  pos(0);
			msg_control.pose.position.y =  pos(1);
			msg_control.pose.position.z =  pos(2);

			msg_control.pose.orientation.z = yaw;
			reference.publish(msg_control);
		}
		//if(dreference.getNumSubscribers() > 0)
		{
			msg_control.pose.position.x =  vel(0);
			msg_control.pose.position.y =  vel(1);
			msg_control.pose.position.z =  vel(2);

			msg_control.pose.orientation.z = st.omega;
			dreference.publish(msg_control);
		}
		//if(ddreference.getNumSubscribers() > 0)
		{
			msg_control.pose.position.x =  acc(0);
			msg_control.pose.position.y =  acc(1);
			msg_control.pose.position.z =  acc(2);

			msg_control.pose.orientation.z = st.alpha;
			ddreference.publish(msg_control);
		}
		//ros::spinOnce();
		ros::Duration(rate).sleep(); // sleep
	}

	ROS_INFO("ld_obs terminated");
	return 0;
}
std::vector<Eigen::VectorXd> ParametrizePath(const PointList &path )
{

}



bool GetWaypoint2Filter(Eigen::Vector3d &point, Eigen::Vector3d &yaw_direction)
{
	double virtual_waypoint_lengh;
	if( !traj_handler->addWaypoint(3)  )
	{
		return false;
	}

	if( !path_waypoints.size() )
	{
		return false;
	}

	if( path_waypoints.size() < 2 )
	{
		point 			= path_waypoints[0];
		yaw_direction 	= Eigen::Vector3d::Zero();
		path_waypoints.erase( path_waypoints.begin() );
		return true;
	}


	Eigen::Vector3d difference		= path_waypoints[1] - position_waypoint;
	double distance = difference.norm();
	int times_path 	= floor( distance / waypoint_lengh );

	virtual_waypoint_lengh = (distance - times_path*waypoint_lengh)/times_path + waypoint_lengh;

	if( times_path  )
	{
		point				= position_waypoint + virtual_waypoint_lengh * ( difference / distance );
		yaw_direction		= difference / distance;
		yaw_direction(2)	= 0.0;

	}else
	{
		path_waypoints.erase( path_waypoints.begin() );
		return false;
	}
	return true;
}
