/**
 * @file   actions.cpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   Jun 27, 2019
 * @brief  State machine interface implementation
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
#include <ootp/actions.hpp>
#include <ootp/config.hpp>
#include <rrt/rrt.hpp>
#include <rrt/rrt_connect.hpp>
#include <timer/tictoc_timer.h>

namespace ootp
{

Actions::Actions() :
		current_state_(kIdle),
		new_action_(false),
		new_scene_(false),
		goal_(0.0, 0.0, 0.0),
		yaw_(0),
		yaw_waypoint_(0),
		lost_turns_counter_(0),
		valid_goal_(false) 		// goal_lost_(false)
{

	map_handler_ 		= new rlmap::OOTP_Map();
	traj_handler_		= new contour::PathDynamics(k_dimension, Config::arc_velocity() );

	states_				= MatrixXD::Zero(k_dimension, kNDynbamics);
	position_waypoint_	= states_.col(kPos);

	traj_handler_->SetGains( Config::path_gains(), Config::yaw_gains(), Config::path_error() );
	traj_handler_->SetInitialPose( position_waypoint_, Config::path_error(), yaw_, Config::yaw_error() );

	CurrentState[kIdle]		 		= &Actions::Idle;
	CurrentState[kAngleFollowing] 	= &Actions::AngleFollowing;
	CurrentState[kWalkFollowing] 	= &Actions::WalkFollowing;
	CurrentState[kLost] 			= &Actions::Lost;

	actions_thread_ 				= new std::thread(&Actions::RunActions, this);
}
Actions::Actions(const Point &initial_position) :
				current_state_(kIdle),
				new_action_(false),
				new_scene_(false),
				goal_(0.0, 0.0, 0.0),
				yaw_(0),
				yaw_waypoint_(0),
				lost_turns_counter_(0),
				valid_goal_(false) 		// goal_lost_(false)
{

	map_handler_ 		= new rlmap::OOTP_Map();
	traj_handler_		= new contour::PathDynamics(k_dimension, Config::arc_velocity() );

	states_				= MatrixXD::Zero(k_dimension, kNDynbamics);
	states_.col(kPos)	= initial_position;
	position_waypoint_	= states_.col(kPos);

	traj_handler_->SetGains( Config::path_gains(), Config::yaw_gains(), Config::path_error() );
	traj_handler_->SetInitialPose( position_waypoint_, Config::path_error(), yaw_, Config::yaw_error() );



	CurrentState[kIdle]		 		= &Actions::Idle;
	CurrentState[kAngleFollowing] 	= &Actions::AngleFollowing;
	CurrentState[kWalkFollowing] 	= &Actions::WalkFollowing;
	CurrentState[kLost] 			= &Actions::Lost;

	actions_thread_ 				= new std::thread(&Actions::RunActions, this);
}

Actions::~Actions()
{
	actions_thread_->join();
	delete actions_thread_;
	delete map_handler_;
	delete traj_handler_;
}

void Actions::AddScene(const rlmap::FrameObj& scene, const Transform &pose, double time_stamp)
{
	std::lock_guard<std::mutex> lck(mtx_actions_);
	current_pose_ 		= 	pose;
	current_frame_ 		= 	scene;
	new_action_			=	true;
	new_scene_			= 	true;
	thread_update_scene_.notify_one();
}

void Actions::UpdateGoal( const Point& point, bool new_data )
{
	std::lock_guard<std::mutex> lck(mtx_actions_);
	goal_ = point;
	valid_goal_ = new_data;

	// path must be cleaned since the past goal is not valid anymore
	ClearPath();

	new_action_			=	true;
	thread_update_scene_.notify_one();
}

void Actions::RunActions(void)
{
	while(true)
	{
		std::unique_lock<std::mutex> lck(mtx_actions_);
		thread_update_scene_.wait(lck , [this]{return new_action_;});
		new_action_		= false;
		RenewState();
	}
}

void Actions::GetMovement( MatrixXD &states, Point &debug )
{
	(*this.*CurrentState[current_state_])();
	UpdateStates();
	states = states_;
	debug(0) = yaw_;
	//debug(1) = traj_handler_->getYawReference();
}

void Actions::GetMovement(MatrixXD &state, double &yaw)
{
	(*this.*CurrentState[current_state_])();
	UpdateStates();
	state = states_;
	yaw = yaw_;
}
void Actions::GetMovement( MatrixXD &state, VectorX &yaw )
{
	(*this.*CurrentState[current_state_])();
	UpdateStates();
	state = states_;
	yaw = yaw_states_;
}

void Actions::RenewState(void)
{
	if(new_scene_)
	{
		new_scene_ = false;
		rlmap::Transform  local_state = rlmap::Transform(current_pose_.position_, current_pose_.rotation_, current_pose_.yaw_);

		tictoc_timer_t timer;
		timer_tic(&timer);

		map_handler_->AddDepthImage(current_frame_, local_state );
	}
	if(current_state_ == kIdle)
	{
		// reset turning counter
		lost_turns_counter_ = 0;

		// if( goal_lost_ == false) // has some goal
		if( valid_goal_ == true) // has some goal
		{
			if( (position_waypoint_ - goal_).norm() <  Config::safe_radius_person() )
			{
				current_state_ = kAngleFollowing;

			}else
			{
				current_state_ = kWalkFollowing;
			}
		}
		return;
	}
	if(current_state_ == kAngleFollowing)
	{
		// if( goal_lost_ == false )  // has some goal
		if( valid_goal_ == true )  // has some goal
		{
			// goal has come out of the radius
			if( (position_waypoint_ - goal_).norm() >  Config::safe_radius_person() )
			{
				current_state_ = kWalkFollowing;
			}
		}else
		{
			current_state_ = kLost;
		}
		return;
	}
	if(current_state_ == kWalkFollowing)
	{
		// if( goal_lost_ == false )  // has some goal
		if( valid_goal_ == true )  // has some goal

		{
			// goal has come inside of the radius
			if( (position_waypoint_ - goal_).norm() <  Config::safe_radius_person() )
			{
				current_state_ = kAngleFollowing;
			}
		}else
		{
			current_state_ = kLost;
		}
		return;
	}
	if(current_state_ == kLost)
	{
		// if( goal_lost_ == false || lost_turns_counter_ > Config::max_lost_turns_counter() )  // has some goal or the number of turns have been completed
		if( valid_goal_ == true || lost_turns_counter_ > Config::max_lost_turns_counter() )  // has some goal or the number of turns have been completed

		{
			current_state_ = kIdle;
		}
		return;
	}
}

void Actions::Idle(void)
{
	// nothing to do ..
}
void Actions::AngleFollowing(void)
{
	if( !traj_handler_->addWaypoint(2)  )
	{
		// same position, new yaw
		double yaw_reference =  traj_handler_->getLastYawWaypoint() + GetSpinDirection() * Config::yaw_velocity();
		yaw_waypoint_ = yaw_reference;
		PushWaypoint2Filter( position_waypoint_, yaw_waypoint_);
	}
}
void Actions::WalkFollowing(void)
{
	if( IsPathOccupied() )
	{
		// A new path must be computed
		GenerateNewPath();
	}
	Point ref_point;
	double ref_yaw;
	if( GetWaypoint2Filter(ref_point, ref_yaw) )
	{
		position_waypoint_ 	= ref_point;
		yaw_waypoint_		= ref_yaw;
		PushWaypoint2Filter(position_waypoint_, yaw_waypoint_);
	}
}
void Actions::Lost(void)
{
	if( !traj_handler_->addWaypoint(2)  )
	{
		lost_turns_counter_++;
		// same position, new yaw
		double yaw_reference 	= traj_handler_->getLastYawWaypoint() + (double)lost_turns_counter_ * Config::yaw_velocity();
		yaw_waypoint_			= yaw_reference;
		PushWaypoint2Filter( position_waypoint_, yaw_waypoint_);
	}
}

// helper functions for AngleFollowing
double Actions::GetSpinDirection(void)
{
	Point next = goal_ - position_waypoint_;
	// cross product just for z direction
	return sign( sin(yaw_waypoint_)*next(1) - cos(yaw_waypoint_)*next(0) );
}

// helper functions for WalkFollowing
bool Actions::IsPathOccupied(void)
{
	if( path_waypoints_.size() )
	{
		// need to take into account actual position and next waypoint..
		if( !map_handler_->IsLineCollisionFreeOLD(position_waypoint_ ,  path_waypoints_[1], Config::agent_radius()) )
		{
			return true;
		}

		for(uint32_t idx = 2; idx <  path_waypoints_.size(); idx++)
		{
			if( !map_handler_->IsLineCollisionFreeOLD( path_waypoints_[idx-1], path_waypoints_[idx], Config::agent_radius()) )
			{
				return true;
			}
		}

		return false;
	}
	return true;
}

void Actions::ClearPath(void)
{
	mtx_path_waypoints_.lock();
	path_waypoints_.clear();
	mtx_path_waypoints_.unlock();
}
void Actions::GenerateNewPath(void)
{
	PointList waypoints;
	//rrt::RRTConnect path(map_handler_, position_waypoint_, goal_, last_yaw);

	rrt::RRT path(map_handler_, position_waypoint_, goal_);
	path.Run();
	path.GetWaypoints(waypoints);

	mtx_path_waypoints_.lock();
	path_waypoints_ = waypoints;
	mtx_path_waypoints_.unlock();
}
void Actions::StartTimer(void)
{
	sampling_time_.reset( new rlmap::TicToc() );
}
void Actions::UpdateStates(void)
{
	contour::State st;
	for(int step = 0 ; step < 5 ; step++)
	{
		st = traj_handler_->UpdateDynamics(0.005); // fix step time
	}
	states_.col(kPos) = st.position;
	states_.col(kVel) = st.velocity*5;
	states_.col(kAcc) = st.acceleration*5*5;
	states_.col(kJerk) = st.jerk*5*5*5;
	yaw_ = st.yaw;

	path_consumed_.push_back(states_.col(kPos));
}

void Actions::PushWaypoint2Filter(const Point &position_ref, const double &yaw_ref)
{
	traj_handler_->PushWaypoint(position_ref, Config::path_error(), yaw_ref, Config::yaw_error() );
}

bool Actions::GetWaypoint2Filter(Point &point, double &yaw)
{

	if( !traj_handler_->addWaypoint(2)  )
	{
		return false;
	}

	if( !path_waypoints_.size() )
	{
		return false;
	}

	if( path_waypoints_.size() < 2 )
	{
		point 	= path_waypoints_[0];
		yaw 	= yaw_waypoint_;
		return true;
	}

	Point difference		= path_waypoints_[1] - position_waypoint_;
	double distance 		= difference.norm();
	uint32_t times_path 	= floor( distance / Config::waypoint_lengh() );
	if( times_path  )
	{
		point				= position_waypoint_ + Config::waypoint_lengh() * ( difference / distance );
		yaw 				= atan2(difference(1),difference(0));
	}else
	{
		point = path_waypoints_[1];
		mtx_path_waypoints_.lock();
		path_waypoints_.erase( path_waypoints_.begin() );
		mtx_path_waypoints_.unlock();
		yaw = yaw_waypoint_;
	}
	return true;
}
bool Actions::IsPointInsideFoV(const Point &start, const Point &point, const Point &unitary)
{
	Point pnt_dron = point - start;
	double height = pnt_dron(2);
	pnt_dron(2) = 0.;

	double proj = pnt_dron.dot(unitary);

	double min_rad = ( Config::agent_radius()  )/ tan( Config::hfov() / 2.0 );

	if(  proj <= ( Config::max_depth_distance() - Config::agent_radius() )  && proj >= min_rad)
	{

		if( fabs( height ) <= proj*Config::tvfov() )
		{
			Point pnt_triang = point - (start + unitary*min_rad);
			double proj_triang = pnt_triang.dot(unitary);
			double ang = acos( proj_triang/pnt_triang.norm() );
			if( fabs(ang) <= Config::hfov() / 2.0 )
			{
				return true;
			}
		}
	}
	return false;
}

void Actions::GetState(eStates &state)
{
	state = current_state_;
}

void Actions::get_occupied_map(rlmap::stdVoxelVector &points)
{
	mutex_map_.lock();
	map_handler_->GetAllOccupiedMap(points);
	mutex_map_.unlock();
}
void Actions::get_local_goal(Point &goal)
{
	goal = local_point_;
}
void Actions::get_local_path(PointList &result)
{
	result.clear();
	result.push_back( position_waypoint_ );
	for(size_t idx = 1; idx < path_waypoints_.size() ; idx++)
	{
		result.push_back( path_waypoints_[idx] );
	}
}
void Actions::get_path_consumed(PointList &result)
{
	result.clear();
	result = path_consumed_;
}
void Actions::get_raw_points(PointList &raw_points)
{
	mutex_map_.lock();
	map_handler_->GetRawPoints(raw_points);
	mutex_map_.unlock();
}
void Actions::get_reference_to_check(Point &reference_to_check, Point &reference)
{
	reference_to_check = reference_to_check_;
	reference = reference_;
}


} /* namespace ootp*/

