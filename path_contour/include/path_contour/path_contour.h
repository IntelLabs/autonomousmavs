/**
 * @file   path_contour.h
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   May, 2018
 * @brief  Trajectory generation algorithm implementation
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
#ifndef PATH_CONTOUR_H
#define PATH_CONTOUR_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <map>

namespace contour
{

typedef enum{use_ellipse,use_error}invariant_set_mode_t;

struct State
{
	Eigen::Vector3d position;
	Eigen::Vector3d velocity;
	Eigen::Vector3d acceleration;
	Eigen::Vector3d jerk;
	Eigen::Vector3d snap;
	double yaw;
	double omega;
	double alpha;
};

struct Waypoint
{
	Eigen::VectorXd position;
	double error;
	double yaw;
	double yaw_error;

};

struct InvariantSet
{
	Eigen::MatrixXd ellipse;
	Eigen::VectorXd gains;
	Eigen::VectorXd yaw_gains;
	invariant_set_mode_t mode;
};

class PathDynamics
{

public:
	PathDynamics();
	PathDynamics(uint8_t space_dimension, double arc_velocity);
	~PathDynamics();
	void SetGains(Eigen::VectorXd gains, Eigen::VectorXd yaw_gains, double error);
	void SetEllipse(Eigen::MatrixXd ellipse, Eigen::VectorXd gains, double error);
	void PushWaypoint(Eigen::VectorXd waypoint, double error, double yaw_ref, double yaw_error);
	State UpdateDynamics(double dt);
	State GetState();
	void SetInitialPosition(const Eigen::VectorXd &position);
	void SetInitialPose(const Eigen::VectorXd &waypoint, const double &path_error, const double &yaw, const double &yaw_error);
	bool addWaypoint(uint32_t size);

	Eigen::Vector3d getLastWaypoint(void);
	Eigen::Vector3d getReference(void);

	double getLastYawWaypoint(void);
	double getYawReference(void);

	uint32_t waypointListSize(void);
	Eigen::Vector3d getLastVel(void);

private:
	Eigen::Vector3d last_direction_;
	Eigen::Vector3d reference_;
	double yaw_ref_;
	std::vector<Waypoint> waypoint_list_;
	State State_;
	double arc_;
	double arc_velocity_;
	InvariantSet gains_;
};

}//contour

#endif
