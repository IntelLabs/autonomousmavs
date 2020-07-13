/**
 * @file   rrt_connect.hpp
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

#include <rrt/global.hpp>
#include <rrt/node.hpp>
#include <rlmap/ootp_map.hpp>
#include <rlmap/tictoc.hpp>

#ifndef RRT_RRT_CONNECT_HPP_
#define RRT_RRT_CONNECT_HPP_

namespace rrt
{

class RRTConnect
{

private:
	rlmap::OOTP_Map 	*map_;
	rlmap::TicTocPtr 	rrt_time_;

	Point				start_;
	Point				goal_;
	Point				unitary_;
	Matrix				rrt_world_boundaries_;

	Nodes				tree_[kNTypes];
	Nodes				solution_;

	bool				goal_reached_;
	uint32_t			other_parent_;
	Matrix				waypoints_;

	PointList			pl_waypoints_;
	bool 				time_out_;

	// rrt star members
	double 				connection_radius_;
	double 				max_distance_;

	// inf rrt star members
	bool 				from_ellipse_;
	double 				c_min_;
	Point 				vX_centre_;
	Matrix 				rotation_matrix_;

	// rrt connect methods
	void SolveRrtConnect(void);
	void SolveRrt(eDirection direction);

	Point GetRandomPoint(eDirection direction);
	void RefreshConnectionRadius(eDirection direction);
	uint32_t FindNodesInCircle(eDirection direction, const Point &random_point, std::vector< uint32_t > &nodes);
	uint32_t ClosestVertex(eDirection direction, const Point &random_point, double &distance);
	bool IsCollisionFree(const Point &random_point, const Point &parent_point);

	bool IsPointInsideFoV(const Point &point);

	void IsNearOfGoal(eDirection direction, const Point &random_point);
	void CreateSolution(eDirection direction);


	void CalculateWaypoints(void);

	// rrt informed methods
	void CreateEllipse(void);
	void SolveInformedRrtStar(void);

public:

	RRTConnect(rlmap::OOTP_Map *map, const Point &start, const Point &goal, double yaw);
	RRTConnect( const Point &start, const Point &goal, double yaw);
	virtual ~RRTConnect();
	void Run(void);
	void GetWaypoints(Matrix &points);
	void GetWaypoints(PointList &points);
};


} /* namespace rrt */




#endif /* RRT_RRT_CONNECT_HPP_ */
