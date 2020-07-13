/**
 * @file   config.hpp
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

#ifndef RLCONFIG_HPP_
#define RLCONFIG_HPP_

#include <rlmap/global.hpp>

namespace rlmap
{

class Config
{
public:
	static Config& getInstance();

	// map storage constants
	static uint8_t& dimensions() { return getInstance().dimensions_; }
	static double& map_size() { return getInstance().map_size_; }
	static double& voxel_leaf_size() { return getInstance().voxel_leaf_size_; }
	static double& occupancy_threshold() { return getInstance().occupancy_threshold_; }
	static double& simple_probability() { return getInstance().simple_probability_; }
	// depth image processing constants
	static uint32_t& frame_width() { return getInstance().frame_width_; }
	static uint32_t& frame_height() { return getInstance().frame_height_; }
	static double& min_depth_distance() { return getInstance().min_depth_distance_; }
	static double& max_depth_distance() { return getInstance().max_depth_distance_; }
	static double& depth_factor() { return getInstance().depth_factor_; }
	static double& calibration_rfx() { return getInstance().calibration_rfx_; }
	static double& calibration_rfy() { return getInstance().calibration_rfy_; }
	static double& calibration_rpx() { return getInstance().calibration_rpx_; }
	static double& calibration_rpy() { return getInstance().calibration_rpy_; }
	static double& image_percent() { return getInstance().image_percent_; }
	static Point& camera_translation_wrt_drone() { return getInstance().camera_translation_wrt_drone_; }
	static Rotation3D& camera_rotation_wrt_drone() { return getInstance().camera_rotation_wrt_drone_; }


	static void map_params(double map_size, double voxel_leaf_size);
	static void init_params_externally(
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
			Rotation3D  camera_rotation_wrt_drone
		);

private:
	Config(void);
	Config(Config const&);
	void operator=(Config const&);
	// map storage constants
	uint8_t		dimensions_;
	double 		map_size_;
	double 		voxel_leaf_size_;
	double		occupancy_threshold_;
	double		simple_probability_;

	// depth image processing constants
	uint32_t 	frame_width_;
	uint32_t 	frame_height_;
	double 		min_depth_distance_;
	double 		max_depth_distance_;
	double 		depth_factor_;
	double 		calibration_rfx_;
	double 		calibration_rfy_;
	double 		calibration_rpx_;
	double 		calibration_rpy_;
	double 		image_percent_;

	Point  		camera_translation_wrt_drone_;
	Rotation3D  camera_rotation_wrt_drone_;

};

}/* namespace rlmap */

#endif /* RLCONFIG_HPP_ */
