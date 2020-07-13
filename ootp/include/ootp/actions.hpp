/**
 * @file   actions.hpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   Jun 27, 2019
 * @brief  State machine interface
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

#ifndef ACTIONS_HPP_
#define ACTIONS_HPP_


#include <thread>
#include <mutex>
#include <boost/noncopyable.hpp>
#include <condition_variable>

#include <ootp/global.hpp>
#include <rrt/rrt.hpp>
#include <path_contour/path_contour.h>

namespace ootp
{

static const uint8_t k_dimension = 3;

// kNStates = number of states
typedef enum {kIdle =0, kAngleFollowing, kWalkFollowing, kLost, kNStates}eStates;  
typedef enum {kPos =0, kVel, kAcc, kJerk, kNDynbamics}eDynbamics;

class Actions :  boost::noncopyable
{
private:
	// external handlers
	rlmap::OOTP_Map					*map_handler_;
	contour::PathDynamics			*traj_handler_;

	// state machine
	eStates							current_state_;
	bool							valid_goal_; 	//goal_lost_;
	uint32_t						lost_turns_counter_;

	// Perception
	Transform 						current_pose_;
	rlmap::FrameObj					current_frame_;
	bool							new_scene_;

	// new_states variables
	MatrixXD						states_;
	double							yaw_;
	Point							yaw_states_;
	Point							goal_;
	rlmap::TicTocPtr 				sampling_time_;

	Point							position_waypoint_;
	double							yaw_waypoint_;

	//threads!
	std::condition_variable 		thread_update_scene_;
	std::mutex						mtx_actions_;
	std::mutex 						mutex_map_;
	std::thread						*actions_thread_;
	bool							new_action_;

	// Debugging members
	Point							local_point_;
	Point							reference_to_check_;
	Point							reference_;
	PointList						path_consumed_;
	// helper variables for WalkFollowing
	PointList						path_waypoints_;
	std::mutex						mtx_path_waypoints_;

	Point							last_position_rrt_;
	Point							last_direction_rrt_;


	// State machine functions
	void (Actions::*CurrentState[kNStates])(void);

	void Idle(void);
	void AngleFollowing(void);
	void WalkFollowing(void);
	void Lost(void);

	void RenewState(void);

	// general helper functions
	finline void UpdateStates(void);
	finline void PushWaypoint2Filter(const Point &position_ref, const double &yaw_ref);

	// helper functions for WalkFollowing
	finline bool IsPathOccupied(void);
	finline void ClearPath(void);
	finline void GenerateNewPath(void);
	bool GetWaypoint2Filter(Point &point, double &yaw);
	bool IsPointInsideFoV(const Point &start, const Point &point, const Point &unitary);

	// helper functions for AngleFollowing
	finline double GetSpinDirection(void);

	// threads
	void RunActions(void);

	// prevent copy of class
	Actions(const Actions&);
	Actions& operator=(const Actions&){ return *this;}

public:
	Actions();
	Actions(const Point &initial_position);
	virtual ~Actions(void);


	void AddScene( const rlmap::FrameObj& scene, const Transform &pose, double time_stamp );
	void UpdateGoal( const Point& point, bool new_data );
	void GetMovement( MatrixXD &states, double &yaw );
	void GetMovement( MatrixXD &state, VectorX &yaw );

	void StartTimer(void);

	// debugging methods, map visualization
	void GetState(eStates &state);
	void GetMovement( MatrixXD &states, Point &debug );

	// debugging methods
	void get_occupied_map(rlmap::stdVoxelVector &points);
	void get_local_path(PointList &result);
	void get_local_goal(Point &goal);
	void get_raw_points(PointList &raw_points);
	void get_reference_to_check(Point &reference_to_check, Point &reference);
	void get_path_consumed(PointList &result);

};


} /* namespace ootp*/


#endif /* ACTIONS_HPP_ */
