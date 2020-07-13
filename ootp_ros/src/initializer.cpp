/**
 * @file   initializer.cpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   June, 2018
 * @brief  a yaml wrapper
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
#include <ootp_ros/initializer.hpp>
#include <ootp/config.hpp>
#include <rrt/config.hpp>

namespace ootp
{

void InitializeParams(ros::NodeHandle nh)
{
	std::vector<double> path_gains;
	std::vector<double> yaw_gains;
	ootp::VectorX	Vpath_gains;
	ootp::VectorX	Vyaw_gains;

	double path_error;
	double arc_velocity;
	double waypoint_lengh;
	double agent_radius;
	double safe_radius_person;
	int max_lost_turns;
	double yaw_velocity;
	double yaw_error;
	double rho;
	double y_rho;
	double dtime;

	double amax;
	double vmax;
	double gain;

	std::string log_files;

	// trajectory
	nh.getParam("path_gains", path_gains);
	nh.getParam("yaw_gains", yaw_gains);
	nh.getParam("path_error", path_error);
	nh.getParam("arc_velocity", arc_velocity);
	nh.getParam("waypoint_lengh", waypoint_lengh);
	nh.getParam("agent_radius", agent_radius);

	nh.getParam("safe_radius_person", safe_radius_person);
	nh.getParam("max_lost_turns", max_lost_turns);
	nh.getParam("yaw_velocity", yaw_velocity);
	nh.getParam("yaw_error", yaw_error);

	nh.getParam("rho", rho);
	nh.getParam("y_rho", y_rho);
	nh.getParam("dtime", dtime);

	nh.getParam("amax", amax);
	nh.getParam("vmax", vmax);
	nh.getParam("gain", gain);

	nh.getParam("log_files", log_files);


	Vpath_gains.resize( path_gains.size() );
	Vpath_gains << path_gains[0],  path_gains[1],  path_gains[2],  path_gains[3],  path_gains[4];

	Vyaw_gains.resize( yaw_gains.size() );
	Vyaw_gains << yaw_gains[0],  yaw_gains[1];



	/* rrt parameters! */

	std::vector<double> 	vrrt_max;
	std::vector<double> 	vrrt_min;
	double 					rrt_wall_time;
	double 					rrt_informed_iteration;
	Point 		rrt_max;
	Point 		rrt_min;
	double horizontal_fov;
	double vertical_fov;
	double max_depth_distance;

	nh.getParam("rrt_max", vrrt_max);
	nh.getParam("rrt_min", vrrt_min);
	nh.getParam("rrt_wall_time", rrt_wall_time);
	nh.getParam("rrt_informed_iteration", rrt_informed_iteration);

	nh.getParam("horizontal_fov", horizontal_fov);
	nh.getParam("vertical_fov", vertical_fov);
	nh.getParam("max_depth_distance", max_depth_distance);

	rrt_max << vrrt_max[0], vrrt_max[1], vrrt_max[2];
	rrt_min << vrrt_min[0], vrrt_min[1], vrrt_min[2];

	rrt::Config::init_params_externally(3, rrt_max, rrt_min, rrt_wall_time, rrt_informed_iteration, agent_radius,
			horizontal_fov, vertical_fov, max_depth_distance, waypoint_lengh);


	Config::init_params_externally(
					Vpath_gains,
					Vyaw_gains,
					path_error,
					arc_velocity,
					waypoint_lengh,
					agent_radius,
					safe_radius_person,
					max_lost_turns,
					yaw_velocity,
					yaw_error,
					rho,
					y_rho,
					dtime,
					vmax,
					amax,
					gain,
					horizontal_fov,
					vertical_fov,
					max_depth_distance,
					log_files
				);
}

} /* namespace ootp */
