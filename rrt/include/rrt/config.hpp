/**
 * @file   config.hpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   May 22, 2019
 * @brief  
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
#ifndef RRT_CONFIG_HPP_
#define RRT_CONFIG_HPP_

#include <rrt/global.hpp>

namespace rrt
{


class Config
{
public:
	 static Config& getInstance();

	 static uint8_t& dimensions() { return getInstance().dimensions_; }

	 static Point& rrt_max() { return getInstance().rrt_max_; }

	 static Point& rrt_min() { return getInstance().rrt_min_; }

	 static double& rrt_wall_time() { return getInstance().rrt_wall_time_; }

	 static double& agent_radius() { return getInstance().agent_radius_; }

	 static double& waypoint_lengh() { return getInstance().waypoint_lengh_; }

	 static uint32_t& rrt_informed_iterations() { return getInstance().rrt_informed_iterations_; }

	 static double& hfov() { return getInstance().hfov_; }

	 static double& vfov() { return getInstance().vfov_; }

	 static double& tvfov() { return getInstance().tvfov_; }

	 static double& max_depth_distance() { return getInstance().max_depth_distance_; }


	 static void init_params_externally(uint8_t dimensions,
										Point rrt_max,
										Point rrt_min,
										double rrt_wall_time,
										uint32_t rrt_informed_iterations,
										double agent_radius,
										double hfov,
										double vfov,
										double max_depth_distance,
										double waypoint_lengh);

private:
	Config(void);
	Config(Config const&);
	void operator=(Config const&);

	uint8_t dimensions_;
	Point rrt_max_;
	Point rrt_min_;
	double rrt_wall_time_;
	uint32_t rrt_informed_iterations_;
	double agent_radius_;



	// connect FoV variables
	double hfov_;
	double vfov_;
	double tvfov_;
	double max_depth_distance_;

	double waypoint_lengh_;
};

} /* namespace rrt */


#endif /* RRT_CONFIG_HPP_ */
