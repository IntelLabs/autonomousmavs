/**
 * @file   rrt.cpp
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
#include <rrt/rrt.hpp>
#include <rrt/config.hpp>
#include <tbb/parallel_for.h>
#include <tbb/mutex.h>
#include <tbb/tbb.h>

namespace rrt
{
RRT::RRT(const Point &start, const Point &goal) :
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
	srand(time(NULL));
}
RRT::RRT(rlmap::OOTP_Map *map, const Point &start, const Point &goal) :
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
	srand(time(NULL));
}

RRT::~RRT()
{

}

void RRT::Run(void)
{
	//SolveRRTStar();
	SolveInformedRrtStar();
}

void RRT::GetWaypoints(Matrix &points)
{
	points = waypoints_;
}
void RRT::GetWaypoints(PointList &points)
{
	points = pl_waypoints_;
}
void RRT::SolveRRTStar(void)
{
	Point 						random_point;
	uint32_t 					closest_node;
	std::vector< uint32_t > 	closest_nodes;
	double 						distance;

	time_out_	= false;

	// initial nodes
	tree_.push_back( Node( start_, 0.0, 0, Point(0,0,0) ) );

	rrt_time_.reset( new rlmap::TicToc() );
	do
	{
		RefreshConnectionRadius( );
		random_point = GetRandomPoint();

		if(time_out_)
		{
			return;
		}

		closest_node = FindNodesInCircle( random_point , closest_nodes);

		if ( closest_node )
		{
			distance = ( random_point - tree_[closest_node].coordinates() ).norm();
			if(distance >= Config::waypoint_lengh())
			{
				if( IsCollisionFree( random_point, tree_[closest_node].coordinates() ) )
				{
					Node node;
					node.carried_distance( distance +  tree_[closest_node].carried_distance() );
					node.parent( closest_node );
					node.coordinates( random_point );
					node.parent_vector(random_point - tree_[closest_node].coordinates() );

					tree_.push_back( node );

					// Check for other points to be redirected...
					for (size_t idx =0; idx <  closest_nodes.size(); idx++)
					{
						closest_node = closest_nodes[idx];
						distance = node.carried_distance() + (tree_[closest_node].coordinates() - random_point).norm();

						if ( distance < tree_[closest_node].carried_distance() )
						{

							if( IsCollisionFree( random_point, tree_[closest_node].coordinates() ) )
							{
								tree_[closest_node].parent( (tree_.size() - 1) );
								tree_[closest_node].carried_distance( distance );
								tree_[closest_node].parent_vector(  tree_[closest_node].coordinates() -random_point );
							}

						}
					}
				}
			}

		}
		else
		{
			closest_node = ClosestVertex( random_point, distance);
			distance = ( random_point - tree_[closest_node].coordinates() ).norm();
			if(distance > Config::waypoint_lengh())
			{
				if ( IsCollisionFree( random_point, tree_[closest_node].coordinates() ) )
				{
					Node node;
					node.carried_distance( distance + tree_[closest_node].carried_distance() );
					node.coordinates(  random_point );
					node.parent( closest_node );
					node.parent_vector(random_point- tree_[closest_node].coordinates() );
					tree_.push_back( node );
				}
			}

		}
		IsNearOfGoal(  tree_.back().coordinates()  );

		if( goal_reached_  )
		{

			break;
		}

		if( rrt_time_->Toc() > Config::rrt_wall_time() )
		{
			time_out_ = true;
			
			break;
		}
	}while(1);
	if(goal_reached_  && !time_out_)
	{
		CreateSolution();
		CalculateWaypoints();
	}
}

void RRT::RefreshConnectionRadius(void)
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

	if (tree_.size() < 2)
	{
		connection_radius_ = minimum_factor;
	}
	else
	{
		connection_radius_ = minimum_factor*pow((log((double)tree_.size()) / (double)tree_.size()), (double)(1.0f / (double)Config::dimensions()));
	}
}

Point RRT::GetRandomPoint(void)
{
	double random = ((double)rand()) / ((double)RAND_MAX);

	if (random < 0.45)
	{

		double theta = 2.0f * M_PI*((double)rand()) / ((double)RAND_MAX);
		double rho = (0.40 - 0.25)*((double)rand()) / ((double)RAND_MAX) + 0.25;

		Point point(rho*cos(theta) + goal_(0), rho*sin(theta) + goal_(1), goal_(2));
		return point;
	}
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
					break;
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
		point << ((double)rand()) / ((double)RAND_MAX),
					((double)rand()) / ((double)RAND_MAX),
					((double)rand()) / ((double)RAND_MAX);

		point = (rrt_world_boundaries_.col(kMax) - rrt_world_boundaries_.col(kMin)).cwiseProduct( point ) +
					rrt_world_boundaries_.col(kMin);
	}
	return point;
}

uint32_t RRT::FindNodesInCircle(const Point &random_point, std::vector< uint32_t > &nodes)
{
	nodes.clear();

	tbb::mutex 		mutex;
	std::vector< std::pair<double, uint32_t> > nodes_inside;
	double carried_distance = std::numeric_limits<double>::infinity();
	uint32_t closest = 0;

	tbb::parallel_for(size_t(0), (const size_t)tree_.size(), [&](size_t ind)
	{

		double node_distance = ( random_point - tree_[ind].coordinates() ).norm();
		if ( node_distance < connection_radius_ )
		{
			mutex.lock();
			nodes_inside.push_back(std::pair<double, uint32_t>(node_distance, ind ));
			mutex.unlock();
		}
	});
	for (uint32_t tree_it = 0; tree_it < nodes_inside.size(); tree_it++)
	{
		if( carried_distance > ( nodes_inside[tree_it].first + tree_[nodes_inside[tree_it].second].carried_distance() ) )
		{
			carried_distance = nodes_inside[tree_it].first + tree_[nodes_inside[tree_it].second].carried_distance();
			closest = nodes_inside[tree_it].second;
		}
		nodes.push_back( nodes_inside[tree_it].second );
	}
	nodes.erase(std::remove(nodes.begin(), nodes.end(), closest ), nodes.end());
	return closest;
}

uint32_t RRT::ClosestVertex(const Point &random_point, double &distance)
{
	uint32_t 		closest_node = 0;
	tbb::mutex 		mutex;
	distance		= std::numeric_limits<double>::infinity();

	tbb::parallel_for(size_t(0), (const size_t)tree_.size(), [&](size_t ind)
	//for(size_t ind =0; ind < tree_[direction].size(); ind++)
	{
		double node_distance = ( random_point - tree_[ind].coordinates() ).norm();
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

bool RRT::IsCollisionFree(const Point &random_point, const Point &parent_point)
{
	return map_->IsLineCollisionFreeOLD(parent_point, random_point, 0.0 );
	//return map_->IsLineCollisionFree(parent_point, random_point, Config::agent_radius());

	//return true;
}

void RRT::IsNearOfGoal(const Point &random_point)
{
	if( (random_point - goal_).norm() < 0.40f )
	{
		goal_reached_ = true;
	}
}

void RRT::CreateSolution(void)
{
	uint32_t my_parent;
	solution_.clear();
	my_parent = tree_.size() - 1;

	while ( my_parent != 0  )
	{
		solution_.push_back( tree_[my_parent] );
		my_parent = tree_[my_parent].parent();
	}
	solution_.push_back( tree_[0] );

	std::reverse(solution_.begin(), solution_.end());
}

void RRT::SolveInformedRrtStar(void)
{
	std::map<double, Nodes> solutions;
	uint32_t times_solution = 0;
	do
	{
		// reset core memebers
		tree_.clear();
		goal_reached_= false;
		time_out_	= false;

		// initial nodes
		tree_.push_back( Node( start_, 0.0, 0, Point(0,0,0) ) );

		SolveRRTStar();

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

void RRT::CreateEllipse(void)
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

void RRT::CalculateWaypoints(void)
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
