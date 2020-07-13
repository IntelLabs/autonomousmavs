/**
 * @file   simulation_tools.cpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   May, 2018
 * @brief  gazebo simulation tools
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
#include <ootp_simulator/simulation_tools.hpp>

#include <std_srvs/Empty.h>
#include <chrono>
#include <thread>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include <termios.h>

namespace ootp
{

char getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if(rv != 0)
        //ROS_INFO("no_key_pressed");

    //else
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}

bool init_gazebo_engine(void)
{
	std_srvs::Empty srv;
	bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
	unsigned int i = 0;
	// Trying to unpause Gazebo for 10 seconds.
	while (i <= 10 && !unpaused)
	{
		ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
		std::this_thread::sleep_for(std::chrono::seconds(1));
		unpaused = ros::service::call("/gazebo/unpause_physics", srv);
		++i;
	}
	if (!unpaused)
	{
		ROS_FATAL("Could not wake up Gazebo.");
		return false;
	}
	ROS_INFO("Unpaused the Gazebo simulation.");
	// Wait for Gazebo GUI show up.
	ros::Duration(10).sleep();
	return true;
}

void publish_goal(const Point &point, double yaw, ros::Publisher traj_pub)
{
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	trajectory_msg.header.stamp = ros::Time::now();
	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(point, yaw, &trajectory_msg);
	traj_pub.publish(trajectory_msg);
}

Point toEuler(double x, double y, double z, double w)
{
	Point orientation;

	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (w * x + y * z);
	double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
	orientation(0) = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (w * y - z * x);
	if (fabs(sinp) >= 1)
		orientation(1) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		orientation(1) = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (w * z + x * y);
	double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
	orientation(2) = atan2(siny_cosp, cosy_cosp);

    return orientation;
}

void toEulerAngle(const quat& q, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
	yaw = atan2(siny_cosp, cosy_cosp);
}

}/* namespace ootp */
