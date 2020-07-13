/**
 * @file   visualizer.cpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   June, 2018
 * @brief  A visualizer for rviz
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

#include <rlmap_ros/visualizer.hpp>
#include <rlmap/voxel.hpp>
#include <rlmap/config.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <nav_msgs/Path.h>

namespace rlmap
{
Visualizer::Visualizer():
nh_("~")
{
	// init ROS publishers
	map_						= nh_.advertise<visualization_msgs::MarkerArray>("/map", 1, true);
	path_						= nh_.advertise<nav_msgs::Path>("/path", 1, true);
	path_consumed_				= nh_.advertise<nav_msgs::Path>("/path_consumed", 1, true);
	publisher_pointCloud_		= nh_.advertise<sensor_msgs::PointCloud2>("/raw_image", 1, true);
}
Visualizer::~Visualizer()
{

}
void Visualizer::PublishRawPoints(const PointList &points)
{

	sensor_msgs::PointCloud2 pointCloud;

	pcl::PointCloud<pcl::PointXYZ> cloud;
	uint32_t auxiliaryCount;

	cloud.width = points.size();
	cloud.height = 1;
	cloud.points.resize(cloud.width * cloud.height);
	for (auxiliaryCount = 0; auxiliaryCount < points.size(); auxiliaryCount++)
	{
		cloud.points[auxiliaryCount].x = ((float)points.at(auxiliaryCount)(0));
		cloud.points[auxiliaryCount].y = ((float)points.at(auxiliaryCount)(1));
		cloud.points[auxiliaryCount].z = ((float)points.at(auxiliaryCount)(2));
	}
	pcl::toROSMsg(cloud, pointCloud);

	pointCloud.header.frame_id = "world";
	publisher_pointCloud_.publish(pointCloud);
}
void Visualizer::PublishPathConsumed(PointList &result)
{
	if(result.size())
	{

		nav_msgs::Path path;
		geometry_msgs::PoseStamped point;


		path.header.frame_id = "world";
		path.header.stamp = ros::Time::now();

		point.header.frame_id = "world";
		point.header.stamp = ros::Time::now();

		for(int32_t ind = 0; ind < result.size(); ind ++)
		{
			point.pose.position.x = result[ind](0);
			point.pose.position.y = result[ind](1);
			point.pose.position.z = result[ind](2);


			path.poses.push_back(point);
		}
		path_consumed_.publish(path);
	}

}
void Visualizer::PublishPath(PointList &result)
{
	if(result.size())
	{

		nav_msgs::Path path;
		geometry_msgs::PoseStamped point;


		path.header.frame_id = "world";
		path.header.stamp = ros::Time::now();

		point.header.frame_id = "world";
		point.header.stamp = ros::Time::now();

		for(int32_t ind = 0; ind < result.size(); ind ++)
		{
			point.pose.position.x = result[ind](0);
			point.pose.position.y = result[ind](1);
			point.pose.position.z = result[ind](2);


			path.poses.push_back(point);
		}
		path_.publish(path);
	}
}


void Visualizer::PublishOccupiedMap(const stdVoxelVector &map )
{
	visualization_msgs::MarkerArray marker_array;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time::now();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  1.0;
	marker.action = visualization_msgs::Marker::ADD;
	marker.type =  visualization_msgs::Marker::CUBE;

	for(int32_t ind =0; ind < map.size();ind++)
	{
		marker.id = ind;
		Voxel vox = map[ind];
		Point center = vox.GetCenter();
		uint32_t level= vox.GetLevel();
		double step 	= 	pow(2, (double)level)*Config::voxel_leaf_size();

		marker.scale.x = step;
		marker.scale.y = step;
		marker.scale.z = step;
		marker.pose.position.x = center(0);
		marker.pose.position.y =center(1);

		double color_r;
		double color_g;
		double color_b;

		GetColor(2.0, center(2), color_r, color_g, color_b);

		marker.color.g = color_g;
		marker.color.b = color_b;
		marker.color.r = color_r;
		marker.color.a = 1.0;

		if(center.rows() > 2)
		{
			marker.pose.position.z = center(2);
		}else
		{
			marker.pose.position.z = 1.0;
		}
		marker_array.markers.push_back(marker);
	}
	map_.publish(marker_array);

}

void GetColor(double max_value, double value, double &color_r, double &color_g, double &color_b)
{
	value > max_value ? value = max_value : value;
	value < 0 ? value = 0 : value;
	double H = ( ( value )/(max_value)) * 315.0 + 225.0;
	double S = 1.0;
	double V = 1.0;

	if (S == 0)
	{
		color_r = V;
		color_g = V;
		color_b = V;
	}
	else
	{
		int i;
		double f, p, q, t;
		if (H == 360)
			H = 0;
		else
			H = H / 60;

		i = (int)trunc(H);
		f = H - i;

		p = V * (1.0 - S);
		q = V * (1.0 - (S * f));
		t = V * (1.0 - (S * (1.0 - f)));

		switch (i)
		{
		case 0:
			color_r = V;
			color_g = t;
			color_b = p;
			break;

		case 1:
			color_r = q;
			color_g = V;
			color_b = p;
			break;

		case 2:
			color_r = p;
			color_g = V;
			color_b = t;
			break;

		case 3:
			color_r = p;
			color_g = q;
			color_b = V;
			break;

		case 4:
			color_r = t;
			color_g = p;
			color_b = V;
			break;

		default:
			color_r = V;
			color_g = p;
			color_b = q;
			break;
		}

	}

	(color_r > 1.0)?color_r=1.0:color_r;
	(color_g > 1.0)?color_g=1.0:color_g;
	(color_b > 1.0)?color_b=1.0:color_b;
}




}/* namespace rlmap */
