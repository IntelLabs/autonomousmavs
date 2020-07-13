/**
 * @file   initializer.cpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   June, 2018
 * @brief  A yaml parser
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

#include <rlmap_ros/initializer.hpp>
#include <rlmap/config.hpp>

namespace rlmap
{

void InitializeParams(ros::NodeHandle nh)
{
	uint8_t		dimensions = 3;

	double 		voxel_leaf_size;
	nh.getParam("voxel_leaf_size", voxel_leaf_size);

	double map_size;
	double virtual_map_size;
	nh.getParam("map_size", virtual_map_size);
	double map_count_ = 0.;
	do
	{
		map_size = voxel_leaf_size*pow(2., map_count_);
		map_count_ += 1.;
	}while(map_size < virtual_map_size);
	std::cout<<"Map size : "<<map_size / 2<<"\n";

	double		occupancy_threshold;
	nh.getParam("occupancy_threshold", occupancy_threshold);
	double		simple_probability;
	nh.getParam("simple_probability", simple_probability);

	// depth image processing constants
	int 	frame_width;
	int 	frame_height;
	nh.getParam("frame_width", frame_width);
	nh.getParam("frame_height", frame_height);

	double 		min_depth_distance;
	double 		max_depth_distance;
	double 		depth_factor;
	nh.getParam("min_depth_distance", min_depth_distance);
	nh.getParam("max_depth_distance", max_depth_distance);
	nh.getParam("depth_factor", depth_factor);

	double 		calibration_rfx;
	double 		calibration_rfy;
	double 		calibration_rpx;
	double 		calibration_rpy;
	nh.getParam("calibration_rfx", calibration_rfx);
	nh.getParam("calibration_rfy", calibration_rfy);
	nh.getParam("calibration_rpx", calibration_rpx);
	nh.getParam("calibration_rpy", calibration_rpy);

	double 		image_percent;
	nh.getParam("image_percent", image_percent);

	Point  		camera_translation_wrt_drone;
	Rotation3D  camera_rotation_wrt_drone;
	std::vector<double> 	camera_translation;
	std::vector<double> 	camera_rotation;
	nh.getParam("camera_translation", camera_translation);
	nh.getParam("camera_rotation", camera_rotation);

	camera_translation_wrt_drone << camera_translation[0], camera_translation[1], camera_translation[2];
	camera_rotation_wrt_drone << camera_rotation[0], camera_rotation[1], camera_rotation[2],
			camera_rotation[3], camera_rotation[4], camera_rotation[5],
			camera_rotation[6], camera_rotation[7], camera_rotation[8];


		Config::init_params_externally(
				dimensions,
				map_size,
				voxel_leaf_size,
				occupancy_threshold,
				simple_probability,
				uint32_t(frame_width),
				uint32_t(frame_height),
				min_depth_distance,
				max_depth_distance,
				depth_factor,
				calibration_rfx,
				calibration_rfy,
				calibration_rpx,
				calibration_rpy,
				image_percent,
				camera_translation_wrt_drone,
				camera_rotation_wrt_drone
				);
}

} /* namespace rlmap */
