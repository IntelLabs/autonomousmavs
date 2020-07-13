/**
 * @file   rrt.hpp
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
#ifndef RRT_RRT_HPP_
#define RRT_RRT_HPP_


#include <rrt/global.hpp>
#include <rrt/node.hpp>
#include <rlmap/ootp_map.hpp>
#include <rlmap/tictoc.hpp>

namespace rrt
{

class RRT
{

private:
	rlmap::OOTP_Map 	*map_;
	rlmap::TicTocPtr rrt_time_;

	Point			start_;
	Point			goal_;
	Matrix			rrt_world_boundaries_;
	Nodes			tree_;
	Nodes			solution_;
	bool			goal_reached_;
	Matrix			waypoints_;
	PointList		pl_waypoints_;
	bool 			time_out_;

	// rrt star members
	double connection_radius_;
	double max_distance_;

	// inf rrt star members
	bool from_ellipse_;
	double c_min_;
	Point vX_centre_;
	Matrix rotation_matrix_;

	// normal rrt methods
	void SolveRRTStar(void);
	Point GetRandomPoint(void);
	void RefreshConnectionRadius(void);
	uint32_t FindNodesInCircle(const Point &random_point, std::vector< uint32_t > &nodes);
	uint32_t ClosestVertex(const Point &random_point, double &distance);
	bool IsCollisionFree(const Point &random_point, const Point &parent_point);
	void IsNearOfGoal(const Point &random_point);
	void CreateSolution(void);
	void SolveInformedRrtStar(void);

	void CalculateWaypoints(void);

	// rrt informed methods
	void CreateEllipse(void);

public:

	RRT(rlmap::OOTP_Map *map, const Point &start, const Point &goal);
	RRT( const Point &start, const Point &goal);
	virtual ~RRT();
	void Run(void);
	void GetWaypoints(Matrix &points);
	void GetWaypoints(PointList &points);
};

} /* namespace rrt */



#endif /* RRT_RRT_HPP_ */
