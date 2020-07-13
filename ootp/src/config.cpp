/**
 * @file   config.cpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   June, 2018
 * @brief  constant values for exploration algorithm
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
#include <ootp/config.hpp>

namespace ootp
{

Config::Config() :
	path_gains_(),
	yaw_gains_(),
	path_error_(0.1),
	arc_velocity_(1.0),
	waypoint_lengh_(0.2),
	agent_radius_(0.3),
	safe_radius_person_(1.0),
	max_lost_turns_(3),
	yaw_velocity_(0.1),
	yaw_error_(0.005),
	rho_(2.0),
	y_rho_(2.0),
	dtime_(0.01),
	vmax_(0.8),
	amax_(0.1),
	gain_(0.5),

	log_files_("/home/leointel/")
{
	max_lost_turns_counter_ =  (int)( ((double)max_lost_turns_)*( 2.0*M_PI / yaw_velocity_) ) ;
	path_gains_.resize(5);
	path_gains_ << -384.0, -400.0, -140.0, -20.0, 1.0;
	yaw_gains_.resize(2);
	yaw_gains_ << -1.0,0.0;

};

Config& Config::getInstance()
{
  static Config instance; // Instantiated on first use and guaranteed to be destroyed
  return instance;
}

void Config::init_params_externally(
	VectorX path_gains,
	VectorX yaw_gains,
	double path_error,
	double arc_velocity,
	double waypoint_lengh,
	double agent_radius,
	double safe_radius_person,
	int 	max_lost_turns,
	double yaw_velocity,
	double yaw_error,
	double rho,
	double y_rho,
	double dtime,

	double vmax,
	double amax,
	double gain,

	double hfov,
	double vfov,
	double max_depth_distance,
	std::string log_files)
{
	getInstance().path_gains_						=	path_gains;
	getInstance().yaw_gains_						=	yaw_gains;
	getInstance().path_error_						=	path_error;
	getInstance().arc_velocity_						=	arc_velocity;
	getInstance().waypoint_lengh_					=	waypoint_lengh;
	getInstance().agent_radius_						=	agent_radius;
	getInstance().safe_radius_person_ 				= 	safe_radius_person;
	getInstance().max_lost_turns_ 					= 	max_lost_turns;
	getInstance().yaw_velocity_ 					= 	yaw_velocity;
	getInstance().yaw_error_ 						= 	yaw_error;
	getInstance().rho_								=   rho;
	getInstance().y_rho_							=   y_rho;
	getInstance().dtime_							=   dtime;

	getInstance().vmax_								=   vmax;
	getInstance().amax_								=   amax;
	getInstance().gain_								=   gain;

	getInstance().max_depth_distance_ 				= max_depth_distance;
	getInstance().hfov_ 							= hfov;
	getInstance().vfov_ 							= vfov;
	getInstance().tvfov_ 							= tan(vfov / 2.0);
	getInstance().log_files_						= log_files;
}

}/* ootp */
