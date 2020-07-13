/**
 * @file   config.cpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   June, 2018
 * @brief  This is the implementation of constant values of the algorithm
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
#include <rlmap/config.hpp>



namespace rlmap
{
Config::Config() :
		dimensions_(3),
		map_size_(51.2), // must be voxel_leaf_size_ multiple
		voxel_leaf_size_(0.2),
		occupancy_threshold_(0.8),
		simple_probability_(0.001),
		frame_width_(640),
		frame_height_(480),
		min_depth_distance_(0.001),	//(0.3),
		max_depth_distance_(3.0),	//(25.0),
		depth_factor_(0.001),
		calibration_rfx_( (double)307.215  ),
		calibration_rfy_( calibration_rfx_ ),
		calibration_rpx_( (double)frame_width_ / (double)2.0),
		calibration_rpy_( (double)frame_height_ / (double)2.0 ),
		image_percent_(0.9),
		camera_translation_wrt_drone_(0.0,0.0,0.0),
		camera_rotation_wrt_drone_( GetZYXRotation(0.0, 0.0, 0.0) )
{
}
Config& Config::getInstance()
{
  static Config instance; // Instantiated on first use and guaranteed to be destroyed
  return instance;
}

void Config::map_params(double map_size, double voxel_leaf_size)
{
	getInstance().map_size_ 							= 	map_size;
	getInstance().voxel_leaf_size_ 						= 	voxel_leaf_size;
}
void Config::init_params_externally(
		// map storage constants
		uint8_t		dimensions,
		double 		map_size,
		double 		voxel_leaf_size,
		double		occupancy_threshold,
		double		simple_probability,
		// depth image processing constants
		uint32_t 	frame_width,
		uint32_t 	frame_height,
		double 		min_depth_distance,
		double 		max_depth_distance,
		double 		depth_factor,
		double 		calibration_rfx,
		double 		calibration_rfy,
		double 		calibration_rpx,
		double 		calibration_rpy,
		double 		image_percent,

		Point  		camera_translation_wrt_drone,
		Rotation3D  camera_rotation_wrt_drone)
{

	getInstance().dimensions_					= dimensions;
	getInstance().map_size_						= map_size;
	getInstance().voxel_leaf_size_				= voxel_leaf_size;
	getInstance().occupancy_threshold_			= occupancy_threshold;
	getInstance().simple_probability_			= simple_probability;
	getInstance().frame_width_					= frame_width;
	getInstance().frame_height_					= frame_height;
	getInstance().min_depth_distance_			= min_depth_distance;
	getInstance().max_depth_distance_			= max_depth_distance;
	getInstance().depth_factor_					= depth_factor;
	getInstance().calibration_rfx_				= calibration_rfx;
	getInstance().calibration_rfy_				= calibration_rfy;
	getInstance().calibration_rpx_				= calibration_rpx;
	getInstance().calibration_rpy_				= calibration_rpy;
	getInstance().image_percent_				= image_percent;
	getInstance().camera_translation_wrt_drone_	= camera_translation_wrt_drone;
	getInstance().camera_rotation_wrt_drone_	= camera_rotation_wrt_drone;

}

}/* namespace rlmap */
