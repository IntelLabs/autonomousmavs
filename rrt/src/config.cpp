/**
 * @file   config.cpp
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
#include <rrt/config.hpp>

namespace rrt
{

Config::Config() :
	dimensions_(3),
	rrt_max_(3.0,3.0,0.95),
	rrt_min_(-3.0,-3.0,0.85),
	rrt_wall_time_(0.1),
	rrt_informed_iterations_(9),
	agent_radius_(0.5),
	hfov_(3.1416/2.),
	vfov_(3.1416/2.),
	tvfov_(tan(vfov_ / 2.0 )),
	max_depth_distance_(3.0),
	waypoint_lengh_(0.2)
{

}
Config& Config::getInstance()
{
  static Config instance; // Instantiated on first use and guaranteed to be destroyed
  return instance;
}

void Config::init_params_externally(uint8_t dimensions,
										Point rrt_max,
										Point rrt_min,
										double rrt_wall_time,
										uint32_t rrt_informed_iterations,
										double agent_radius,
										double hfov,
										double vfov,
										double max_depth_distance,
										double waypoint_lengh)
{
	getInstance().dimensions_				= dimensions;
	getInstance().rrt_max_					= rrt_max;
	getInstance().rrt_min_					= rrt_min;
	getInstance().rrt_wall_time_			= rrt_wall_time;
	getInstance().rrt_informed_iterations_	= rrt_informed_iterations;
	getInstance().agent_radius_				= agent_radius;
	getInstance().hfov_ 					= hfov;
	getInstance().vfov_ 					= vfov;
	getInstance().tvfov_ 					= tan(vfov / 2.0);
	getInstance().max_depth_distance_ 		= max_depth_distance;
	getInstance().waypoint_lengh_ 			= waypoint_lengh;
}

} /* namespace rrt */
