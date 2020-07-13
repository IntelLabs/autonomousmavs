/**
 * @file   rrt_connect.cpp
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
#include <time.h>
#include <rrt/rrt_connect.hpp>
#include <rrt/config.hpp>
#include <tbb/parallel_for.h>
#include <tbb/mutex.h>
#include <tbb/tbb.h>

namespace rrt
{
RRTConnect::RRTConnect(const Point &start, const Point &goal, double yaw) :
		start_(start),
		goal_(goal),
		goal_reached_(false),
		time_out_(false),
		connection_radius_(0.0),
		max_distance_(0.0),
		from_ellipse_(false),
		c_min_(0)
{
	rrt_world_boundaries_.resize(Config::dimensions(), kNIndex );
	rrt_world_boundaries_.col(kMax) << Config::rrt_max();
	rrt_world_boundaries_.col(kMin) << Config::rrt_min();

	max_distance_ = (goal_ - start_).norm();
	unitary_ << cos(yaw), sin(yaw), 0.;

	srand(time(NULL));
}
RRTConnect::RRTConnect(rlmap::OOTP_Map *map, const Point &start, const Point &goal, double yaw) :
		start_(start),
		goal_(goal),
		goal_reached_(false),
		time_out_(false),
		connection_radius_(0.0),
		max_distance_(0.0),
		map_(map),
		from_ellipse_(false),
		c_min_(0)
{
	rrt_world_boundaries_.resize(Config::dimensions(), kNIndex );
	rrt_world_boundaries_.col(kMax) << Config::rrt_max();
	rrt_world_boundaries_.col(kMin) << Config::rrt_min();

	max_distance_ = (goal_ - start_).norm();
	unitary_ << cos(yaw), sin(yaw), 0.;

	srand(time(NULL));
}

RRTConnect::~RRTConnect()
{

}

void RRTConnect::Run(void)
{
	//SolveRRTStar();
	SolveInformedRrtStar();
}

void RRTConnect::GetWaypoints(Matrix &points)
{
	points = waypoints_;
}
void RRTConnect::GetWaypoints(PointList &points)
{
	points = pl_waypoints_;
}

void RRTConnect::SolveInformedRrtStar(void)
{
	std::map<double, Nodes> solutions;
	uint32_t times_solution = 0;
	do
	{
		// reset core memebers
		tree_[kFov].clear();
		tree_[kMap].clear();
		goal_reached_= false;
		time_out_	= false;

		// initial nodes
		tree_[kFov].push_back( Node( start_, 0.0, 0, unitary_ ) );
		tree_[kMap].push_back( Node( goal_, 0.0, 0, Point(0,0,0)  ) );

		SolveRrtConnect();

		if(goal_reached_ && !time_out_)
		{
			//calculate carried
			double carried_distance = 0;
			for(size_t idx = 1; idx < solution_.size(); idx++)
			{
				carried_distance += ( solution_[idx].coordinates() - solution_[idx - 1].coordinates() ).norm();
			}

			solutions.insert(std::pair<double, Nodes>( carried_distance,  solution_ )); // calculate carried distance

			max_distance_ = solutions.begin()->first;
			if( max_distance_ < ( (goal_ - start_).norm() + k_rrt_solution_error ) )
			{
				times_solution = Config::rrt_informed_iterations();
			}
			//reset
			CreateEllipse();
		}
		times_solution++;

	}while ( times_solution < Config::rrt_informed_iterations() );
	if(solutions.begin() != solutions.end())
	{
		solution_ = solutions.begin()->second;
		CalculateWaypoints();
	}else
	{
		
	}
}
void RRTConnect::SolveRrtConnect(void)
{
	eDirection direction;
	rrt_time_.reset( new rlmap::TicToc() );
	do
	{

		direction = kFov;
		SolveRrt(direction);
		if(time_out_)
		{
			break;
		}
		IsNearOfGoal( direction,  tree_[direction].back().coordinates()  );
		if( goal_reached_  )
		{
			break;
		}
		if(tree_[kFov].size() > 1 )
		{
			direction = kMap;
			SolveRrt(direction);
			if(time_out_)
			{
				break;
			}
			IsNearOfGoal( direction,  tree_[direction].back().coordinates()  );
			if( goal_reached_  )
			{
				break;
			}
		}
		if( rrt_time_->Toc() > Config::rrt_wall_time() )
		{
			time_out_ = true;
			break;
		}
	}while(1);
	if(goal_reached_  && !time_out_)
	{
		CreateSolution(direction);
	}
}
void RRTConnect::SolveRrt(eDirection direction)
{
	Point random_point;
	uint32_t closest_node;
	std::vector< uint32_t > closest_nodes;
	double distance;

	RefreshConnectionRadius( direction );
	random_point = GetRandomPoint( direction );

	if(time_out_)
	{
		return;
	}
	closest_node = FindNodesInCircle(direction, random_point , closest_nodes);

	if ( closest_node )
	{
		if( IsCollisionFree(random_point, tree_[direction][closest_node].coordinates()) )
		{
			Node node;
			distance = ( random_point - tree_[direction][closest_node].coordinates() ).norm();
			node.carried_distance( distance +  tree_[direction][closest_node].carried_distance() );
			node.parent( closest_node );
			node.coordinates( random_point );
			node.parent_vector(random_point - tree_[direction][closest_node].coordinates() );

			tree_[direction].push_back( node );

			// Check for other points to be redirected...
			for (size_t idx =0; idx <  closest_nodes.size(); idx++)
			{
				closest_node = closest_nodes[idx];
				distance = node.carried_distance() + (tree_[direction][closest_node].coordinates() - random_point).norm();

				if ( distance < tree_[direction][closest_node].carried_distance() )
				{
					if( IsCollisionFree(random_point, tree_[direction][closest_node].coordinates() ) )
					{
						tree_[direction][closest_node].parent( (tree_[direction].size() - 1) );
						tree_[direction][closest_node].carried_distance( distance );

						tree_[direction][closest_node].parent_vector(  tree_[direction][closest_node].coordinates() -random_point );
					}

				}
			}
		}
	}
	else
	{
		closest_node = ClosestVertex(direction, random_point, distance);
		if ( IsCollisionFree( random_point, tree_[direction][closest_node].coordinates()) )
		{
			Node node;
			distance = ( random_point - tree_[direction][closest_node].coordinates() ).norm() ;
			node.carried_distance( distance + tree_[direction][closest_node].carried_distance() );
			node.coordinates(  random_point );
			node.parent( closest_node );
			node.parent_vector(random_point- tree_[direction][closest_node].coordinates() );
			tree_[direction].push_back( node );
		}
	}
}
void RRTConnect::RefreshConnectionRadius(eDirection direction)
{
	double pow_distance = max_distance_;
	double pow_PI = M_PI;

	double var_Vn;
	double minimum_factor;

	for(uint8_t ind = 1; ind < Config::dimensions(); ind++)
	{
		// Calculating the Lebesgue measure of the obstacle - free space
		pow_distance = pow_distance*pow_distance;
		pow_PI = pow_PI*pow_PI;
	}
	// Calculating the unit ball in the d - dimensional Euclidean space
	var_Vn = pow_PI / (tgammaf((double)((double)Config::dimensions() / 2.0f) + 1.0f));
	minimum_factor = 2.5f * pow((1.0f +(double)(1.0f / (double)Config::dimensions()))*(pow_distance / var_Vn), (double)(1.0f / (double)Config::dimensions()));

	if (tree_[direction].size() < 2)
	{
		connection_radius_ = minimum_factor;
	}
	else
	{
		connection_radius_ = minimum_factor*pow((log((double)tree_[direction].size()) / (double)tree_[direction].size()), (double)(1.0f / (double)Config::dimensions()));
	}
}

Point RRTConnect::GetRandomPoint(eDirection direction)
{
	Point point;
	if(from_ellipse_)
	{
		Point vX_fr;
		Point vX_ball;
		Matrix mX_L;
		bool inside;
		while(1)
		{

			inside = true;

			vX_fr.resize( Config::dimensions() );
			vX_fr(0) = max_distance_ / 2.;

			for (size_t dim = 1; dim < Config::dimensions(); dim++)
			{
				vX_fr(dim) = sqrt((max_distance_*max_distance_) - (c_min_*c_min_)) / 2.0;
			}
			mX_L= vX_fr.asDiagonal();

			double theta = 2 * M_PI*((double)rand()) / ((double)RAND_MAX);
			double rho = ((double)rand()) / ((double)RAND_MAX);
			vX_ball.resize( Config::dimensions() );
			if ( Config::dimensions() == 2)
			{
				vX_ball << rho*cos(theta), rho*sin(theta);
			}
			else
			{
				double phi = M_PI*((double)rand()) / ((double)RAND_MAX);
				vX_ball << rho*cos(theta)*sin(phi), rho*sin(theta)*sin(phi), rho*cos(phi);
			}
			point = rotation_matrix_*mX_L*vX_ball + vX_centre_;

			if ( ((start_ - point).norm() + (point - goal_).norm()) <= max_distance_)
			{
				for (size_t dim = 0; dim < Config::dimensions(); dim++)
				{
					if( point(dim) >= rrt_world_boundaries_.col(kMax)(dim) || point(dim) <= rrt_world_boundaries_.col(kMin)(dim) )
					{
						inside = false;
					}
				}
				if(inside)
				{

					if(direction == kMap)
					{
						break;
					}else
					{
						if( IsPointInsideFoV(point) )
						{
							break;
						}
					}
				}
			}
			if( rrt_time_->Toc() > Config::rrt_wall_time() )
			{
				time_out_ = true;
				
				break;
			}
		}

	}else
	{
		while(1)
		{
			point << ((double)rand()) / ((double)RAND_MAX),
					((double)rand()) / ((double)RAND_MAX),
					((double)rand()) / ((double)RAND_MAX);

			point = (rrt_world_boundaries_.col(kMax) - rrt_world_boundaries_.col(kMin)).cwiseProduct( point ) +
					rrt_world_boundaries_.col(kMin);

			if(direction == kMap)
			{
				break;
			}else
			{
				if( IsPointInsideFoV(point) )
				{
					break;
				}
			}
			if( rrt_time_->Toc() > Config::rrt_wall_time() )
			{
				time_out_ = true;
				
				break;
			}
		}
	}
	return point;
}

uint32_t RRTConnect::FindNodesInCircle(eDirection direction, const Point &random_point, std::vector< uint32_t > &nodes)
{
	nodes.clear();

	tbb::mutex 		mutex;
	std::vector< std::pair<double, uint32_t> > nodes_inside;
	double carried_distance = std::numeric_limits<double>::infinity();
	uint32_t closest = 0;

	tbb::parallel_for(size_t(0), (const size_t)tree_[direction].size(), [&](size_t ind)
	{

		double node_distance = ( random_point - tree_[direction][ind].coordinates() ).norm();
		if ( node_distance < connection_radius_ )
		{
			mutex.lock();
			nodes_inside.push_back(std::pair<double, uint32_t>(node_distance, ind ));
			mutex.unlock();
		}
	});
	for (uint32_t tree_it = 0; tree_it < nodes_inside.size(); tree_it++)
	{
		if( carried_distance > ( nodes_inside[tree_it].first + tree_[direction][nodes_inside[tree_it].second].carried_distance() ) )
		{
			carried_distance = nodes_inside[tree_it].first + tree_[direction][nodes_inside[tree_it].second].carried_distance();
			closest = nodes_inside[tree_it].second;
		}
		nodes.push_back( nodes_inside[tree_it].second );
	}
	nodes.erase(std::remove(nodes.begin(), nodes.end(), closest ), nodes.end());
	return closest;
}

uint32_t RRTConnect::ClosestVertex(eDirection direction, const Point &random_point, double &distance)
{
	uint32_t 		closest_node = 0;
	tbb::mutex 		mutex;
	distance		= std::numeric_limits<double>::infinity();

	tbb::parallel_for(size_t(0), (const size_t)tree_[direction].size(), [&](size_t ind)
	//for(size_t ind =0; ind < tree_[direction].size(); ind++)
	{
		double node_distance = ( random_point - tree_[direction][ind].coordinates() ).norm();
		if ( node_distance < distance )
		{
			mutex.lock();
			distance = node_distance;
			closest_node = ind;
			mutex.unlock();
		}
	});
	return closest_node;
}

bool RRTConnect::IsCollisionFree(const Point &random_point, const Point &parent_point)
{
	return map_->IsLineCollisionFree(parent_point, random_point, Config::agent_radius() );
	//return map_->IsLineCollisionFree(parent_point, random_point);

	//return true;
}

void RRTConnect::IsNearOfGoal(eDirection direction, const Point &random_point)
{
	double distance = std::numeric_limits<double>::infinity();
	if(direction == kMap)
	{
		for( size_t idx =1; idx < tree_[kFov].size(); idx++)
		{
			if( IsCollisionFree( random_point, tree_[kFov][idx].coordinates() ))
			{
				goal_reached_ = true;
				double local_distance = (random_point -  tree_[kFov][idx].coordinates()).norm();
				if(local_distance < distance )
				{
					distance = local_distance;
					other_parent_ = idx;
				}
			}
		}

	}else
	{
		for( size_t idx =0; idx < tree_[kMap].size(); idx++)
		{
			if( IsCollisionFree(  random_point, tree_[kMap][idx].coordinates() ) )
			{
				goal_reached_ = true;
				double local_distance = (random_point -  tree_[kMap][idx].coordinates()).norm();
				if(local_distance < distance )
				{
					distance = local_distance;
					other_parent_ = idx;
				}
			}
		}
	}
}

void RRTConnect::CreateSolution(eDirection direction)
{
	uint32_t my_parent;
	solution_.clear();

	if(direction == kFov)
	{
		my_parent = tree_[kFov].size() - 1;
		while ( my_parent != 0  )
		{
			solution_.push_back( tree_[kFov][my_parent] );
			my_parent = tree_[kFov][my_parent].parent();
		}
		solution_.push_back( tree_[kFov][0] );
		std::reverse(solution_.begin(), solution_.end());

		my_parent = other_parent_;
		while (  my_parent != 0 )
		{
			solution_.push_back( tree_[kMap][my_parent] );
			my_parent = tree_[kMap][my_parent].parent();
		}
		solution_.push_back( tree_[kMap][0] );
	}else
	{
		my_parent = other_parent_;
		while ( my_parent != 0  )
		{
			solution_.push_back( tree_[kFov][my_parent] );
			my_parent = tree_[kFov][my_parent].parent();
		}
		solution_.push_back( tree_[kFov][0] );
		std::reverse(solution_.begin(), solution_.end());

		my_parent = tree_[kMap].size() - 1;
		while ( my_parent != 0  )
		{
			solution_.push_back( tree_[kMap][my_parent] );
			my_parent = tree_[kMap][my_parent].parent();
		}
		solution_.push_back( tree_[kMap][0] );
	}
}

bool RRTConnect::IsPointInsideFoV(const Point &point)
{
	Point pnt_dron = point - start_;
	double height = pnt_dron(2);
	pnt_dron(2) = 0.;

	double proj = pnt_dron.dot(unitary_);

	double min_rad = ( Config::agent_radius()  )/ tan( Config::hfov() / 2.0 );

	if(  proj <= ( Config::max_depth_distance() - Config::agent_radius() )  && proj >= min_rad)
	{

		if( fabs( height ) <= proj*Config::tvfov() )
		{
			Point pnt_triang = point - (start_ + unitary_*min_rad);
			double proj_triang = pnt_triang.dot(unitary_);
			double ang = acos( proj_triang/pnt_triang.norm() );
			if( fabs(ang) <= Config::hfov() / 2.0 )
			{
				return true;
			}
		}
	}
	return false;
}

void RRTConnect::CreateEllipse(void)
{
	from_ellipse_ = true;

	c_min_ = (goal_ - start_).norm();
	vX_centre_ = (start_ + goal_) / 2.;

	if (Config::dimensions() == 2)
	{
		double yaw = atan2(goal_(0) - start_(0), goal_(1) - start_(1));
		rotation_matrix_ = CalculateRotationMatrix(yaw);
		return;
	}
	double roll = atan2(goal_(1) - start_(1), goal_(2) - start_(2));
	double pitch = atan2(goal_(2) - start_(2), goal_(0) - start_(0));
	double yaw = atan2(goal_(0) - start_(0), goal_(1) - start_(1));
	rotation_matrix_ = CalculateRotationMatrix(roll, pitch, yaw);
}

void RRTConnect::CalculateWaypoints(void)
{
	if(solution_.size())
	{
		waypoints_.resize(Config::dimensions(),  solution_.size());
		pl_waypoints_.clear();
		for( size_t idx = 0; idx < solution_.size(); idx++)
		{
			waypoints_.col(idx) =  solution_[idx].coordinates();
			pl_waypoints_.push_back(solution_[idx].coordinates());
		}
	}
}

} /* namespace rrt */

