/**
 * @file   path_contour.cpp
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
#include "path_contour/path_contour.h"
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

#define maxf(x,y) (x>y?x:y)

namespace contour
{

PathDynamics::PathDynamics()
{
	yaw_ref_=0.0;
	last_direction_= Eigen::Vector3d::Zero();
	reference_= Eigen::Vector3d::Zero();
	State_.position = Eigen::Vector3d::Zero();
	State_.velocity = Eigen::Vector3d::Zero();
	State_.acceleration = Eigen::Vector3d::Zero();
	State_.jerk = Eigen::Vector3d::Zero();
	State_.snap = Eigen::Vector3d::Zero();
	State_.yaw = 0.0;
	State_.omega = 0.0;
	State_.alpha = 0.0;
	arc_ = 0;
	arc_velocity_ = 1;
	waypoint_list_.resize(0);
}

PathDynamics::PathDynamics(uint8_t space_dimension, double arc_velocity)
{
	yaw_ref_=0.0;
	last_direction_= Eigen::Vector3d::Zero();
	reference_= Eigen::VectorXd::Zero(space_dimension);
	State_.position = Eigen::VectorXd::Zero(space_dimension);
	State_.velocity = Eigen::VectorXd::Zero(space_dimension);
	State_.acceleration = Eigen::VectorXd::Zero(space_dimension);
	State_.jerk = Eigen::VectorXd::Zero(space_dimension);
	State_.snap = Eigen::VectorXd::Zero(space_dimension);
	State_.yaw = 0.0;
	State_.omega = 0.0;
	State_.alpha = 0.0;
	arc_ = 0;
	arc_velocity_ = arc_velocity;
	waypoint_list_.resize(0);
}
bool PathDynamics::addWaypoint(uint32_t size)
{
	if(waypoint_list_.size() < size)
	{
		return true;
	}
	return false;
}

Eigen::Vector3d PathDynamics::getReference(void)
{
	return reference_;
}

Eigen::Vector3d PathDynamics::getLastWaypoint(void)
{
	return waypoint_list_.back().position;
}

double PathDynamics::getLastYawWaypoint(void)
{
	return waypoint_list_.back().yaw;
}

double PathDynamics::getYawReference(void)
{
	return yaw_ref_;
}

Eigen::Vector3d PathDynamics::getLastVel(void)
{
	Eigen::Vector3d direction;
	direction << cos(State_.yaw), sin(State_.yaw), 0.0f;
	return direction;
}

void PathDynamics::SetInitialPosition(const Eigen::VectorXd &position)
{
	State_.position = position;
	PushWaypoint(position, 0.1, 0.0, 0.1);
}

void PathDynamics::SetInitialPose(const Eigen::VectorXd &waypoint, const double &path_error, const double &yaw, const double &yaw_error)
{
	State_.position = waypoint;
	State_.yaw = yaw;
	yaw_ref_=yaw;
	PushWaypoint(waypoint, path_error, yaw, yaw_error);
}


void PathDynamics::SetGains(Eigen::VectorXd gains, Eigen::VectorXd yaw_gains, double error)
{
	gains_.yaw_gains = yaw_gains;
	gains_.gains = gains;
	gains_.mode = use_error;
}

void PathDynamics::SetEllipse(Eigen::MatrixXd ellipse, Eigen::VectorXd gains, double error)
{
	gains_.ellipse = ellipse;
	gains_.mode = use_ellipse;
	gains_.gains = gains;
}

PathDynamics::~PathDynamics()
{
}

void PathDynamics::PushWaypoint(Eigen::VectorXd waypoint, double error, double yaw_ref, double yaw_error)
{
	if(waypoint_list_.size())
	{
		if( (waypoint - waypoint_list_[0].position).norm() < error && fabs(yaw_ref - waypoint_list_[0].yaw) < yaw_error)
		{
			return;
		}
	}
	Waypoint wp;
	wp.position = waypoint;
	wp.error = error;
	wp.yaw = yaw_ref;
	wp.yaw_error = yaw_error;
	waypoint_list_.push_back(wp);
}
uint32_t PathDynamics::waypointListSize(void)
{
	return waypoint_list_.size();
}

State PathDynamics::UpdateDynamics(double dt)
{
	double distance;
	double yaw_error = 0.0;
	if(waypoint_list_.size())
	{
		double control = arc_velocity_;
		double ellipse = 0.0f;
		for(int index = 0; index<State_.position.size() ;index++)
		{
			if(use_error==gains_.mode)
			{
				Eigen::VectorXd state;
				state.resize(4);
				state<<State_.position(index),State_.velocity(index),State_.acceleration(index),State_.jerk(index);
				control *= maxf(0.0,(waypoint_list_[0].error - fabs(state(0) - reference_(index)))/waypoint_list_[0].error);
			}
			else
			{

				Eigen::VectorXd state;
				state.resize(4);

				state<<State_.position(index),State_.velocity(index),State_.acceleration(index),State_.jerk(index);
				ellipse += state.transpose()*gains_.ellipse*state;

			}

		}
		Eigen::Vector3d direction_ref, direction_state;
		direction_state << cos(State_.yaw), sin(State_.yaw),0.0;
		direction_ref << cos(yaw_ref_), sin(yaw_ref_),0.0;
		yaw_error = -(direction_state(0)*direction_ref(1) - direction_state(1)*direction_ref(0));
		control *= maxf(0.0,(waypoint_list_[0].yaw_error - fabs(yaw_error))/waypoint_list_[0].yaw_error);
		if(use_ellipse==gains_.mode)
		{
			control *=  1.0 - ellipse;
			control = control<0.0f?0.0f:control;
		}
		if(1==waypoint_list_.size())
		{
			reference_ = waypoint_list_[0].position;
			yaw_ref_ = waypoint_list_[0].yaw;
		}
		else
		{
			if((reference_ - State_.position).norm() < waypoint_list_[0].error &&
					(direction_ref - direction_state).norm() < waypoint_list_[0].yaw_error)
			{
					arc_+= dt*control;
			}

			distance = (waypoint_list_[1].position - waypoint_list_[0].position).norm();

			if(distance != 0.0)
			{
				reference_ = (waypoint_list_[1].position - waypoint_list_[0].position)*(arc_/distance) + waypoint_list_[0].position;

				yaw_ref_ = (waypoint_list_[1].yaw - waypoint_list_[0].yaw)*(arc_/distance) + waypoint_list_[0].yaw;
				if(arc_ - distance > 0.0 )
				{
					arc_-=distance;
					waypoint_list_.erase(waypoint_list_.begin());
				}
			}
		}

		Eigen::VectorXd gains = gains_.gains;
		Eigen::VectorXd yaw_gains = gains_.yaw_gains;
		State_.snap = gains(0)*(State_.position - reference_) + gains(1)*State_.velocity + gains(2)*State_.acceleration +
				gains(3)*State_.jerk;
		State_.alpha =  yaw_gains(0)*(yaw_error) + yaw_gains(1)*State_.omega;
		State_.position+=dt*State_.velocity + dt*dt*State_.acceleration/2.0 + pow(dt,3)*State_.jerk/6.0 + pow(dt,4)*State_.snap/24.0;
		State_.velocity+=dt*State_.acceleration + dt*dt*State_.jerk/2.0 + pow(dt,3)*State_.snap/6.0;
		State_.acceleration+=dt*State_.jerk + dt*dt*State_.snap/2.0;
		State_.jerk+=dt*State_.snap;

		State_.yaw+=dt*State_.omega + dt*dt*State_.alpha/2.0;
		State_.omega+=dt*State_.alpha;

	}
	return State_;
}

State PathDynamics::GetState()
{
	return State_;
}

}
