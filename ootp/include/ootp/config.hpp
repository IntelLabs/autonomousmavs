/**
 * @file   config.hpp
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

#ifndef CONFIG_HPP_
#define CONFIG_HPP_
#include <ootp/global.hpp>

namespace ootp
{
class Config
{
public:
	 static Config& getInstance();
	 static VectorX& path_gains() { return getInstance().path_gains_; }
	 static VectorX& yaw_gains() { return getInstance().yaw_gains_; }
	 static double& path_error() { return getInstance().path_error_; }
	 static double& arc_velocity() { return getInstance().arc_velocity_; }
	 static double& waypoint_lengh() { return getInstance().waypoint_lengh_; }
	 static double& agent_radius() { return getInstance().agent_radius_; }
	 static double& rho() { return getInstance().rho_; }
	 static double& y_rho() { return getInstance().y_rho_; }

	 static double& vmax() { return getInstance().vmax_; }
	 static double& amax() { return getInstance().amax_; }
	 static double& gain() { return getInstance().gain_; }


	 static double& safe_radius_person() { return getInstance().safe_radius_person_; }
	 static double& yaw_velocity() { return getInstance().yaw_velocity_; }
	 static int& max_lost_turns() { return getInstance().max_lost_turns_; }
	 static int& max_lost_turns_counter() { return getInstance().max_lost_turns_counter_; }
	 static double& yaw_error() { return getInstance().yaw_error_; }
	 static double& dtime() { return getInstance().dtime_; }

	 static double& hfov() { return getInstance().hfov_; }
	 static double& tvfov() { return getInstance().tvfov_; }
	 static double& max_depth_distance() { return getInstance().max_depth_distance_; }

	 static std::string& log_files() { return getInstance().log_files_; }


	 static void init_params_externally(
		VectorX path_gains,
		VectorX yaw_gains,
		double path_error,
		double arc_velocity,
		double waypoint_lengh,
		double agent_radius,

		double safe_radius_person,
		int max_lost_turns,
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
		std::string log_files );

private:
	Config(void);
	Config(Config const&);
	void operator=(Config const&);

	VectorX path_gains_;
	VectorX yaw_gains_;
	double path_error_;
	double arc_velocity_;
	double waypoint_lengh_;
	double agent_radius_;	

	double dtime_;
	double rho_;
	double y_rho_;

	double vmax_;
	double amax_;
	double gain_;


	double safe_radius_person_;
	// TODO :: yaw_velocity_ and number of lost turns give this :
	int max_lost_turns_;
	int max_lost_turns_counter_;
	double yaw_velocity_;
	double yaw_error_;

	// added for FoV point
	double hfov_;
	double vfov_;
	double tvfov_;
	double max_depth_distance_;

	// log file
	std::string log_files_;
};


}/* ootp */


#endif /* CONFIG_HPP_ */
