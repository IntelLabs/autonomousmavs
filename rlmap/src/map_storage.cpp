/**
 * @file   map_storage.cpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   June, 2018
 * @brief  This is the implementation of the functions that handles the map storage
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

#include <rlmap/map_storage.hpp>
#include <rlmap/config.hpp>
#include <rlmap/hash_functions.hpp>
#include <iostream>       // std::cerr
#include <stdexcept>      // std::out_of_range
#include <fstream>

#include <rlmap/tictoc.hpp>
#include <tbb/parallel_for.h>
#include <tbb/mutex.h>

namespace rlmap
{

MapStorage::MapStorage()
{
	InitMap();
	Initialize3dChildren();
}
MapStorage::~MapStorage()
{

}
void MapStorage::InitMap(void)
{
	map_.clear();

	levels_ = data_key_size(((std::log(Config::map_size()) - std::log(Config::voxel_leaf_size()))*invlog2));
	// The biggest voxels have to be added by hand, in order to simplify the hash function
	PointList childrens;
	GetChildren( childrens, GetHashLevel(1),  Point::Zero() ); // must be added at first level
	// the zero level is for the root voxel
	for(size_t cln_ind=0; cln_ind < childrens.size(); cln_ind++ )
	{
		type_key key_children = HashFunct::Hashf( GetHashLevel(1), childrens[cln_ind] );
		// Up level voxels start with 0 probability
		Voxel voxel_hand(childrens[cln_ind], GetHashLevel(1), 0.0 );
		map_.insert(std::pair<type_key, Voxel>(key_children, voxel_hand ));
	}

}
void MapStorage::Initialize3dChildren(void)
{
	children_ = Matrix::Zero(Config::dimensions(), kchildren);
	int32_t dim = 0;
	Point pivot = Point::Ones();
	for(size_t ind=0; ind < kchildren; ind++)
	{
		pivot(dim) = -1*pivot(dim);
		children_.col(ind) = pivot;
		dim++;
		(dim > children_.rows() - 2)?dim=0:dim;
		if((int32_t)ind%(kchildren/2)==0)
		{
			pivot(children_.rows()-1) = -1*pivot(children_.rows()-1);
		}
	}
}

void MapStorage::InsertOccupiedPoints(PointList& points)
{
	FillLevels(points);
}

void MapStorage::FillLevels(PointList& points)
{
	deleted_nodes_.clear();
	current_occupied_nodes_.clear();

	for(size_t pts_ind=0; pts_ind <  points.size(); pts_ind++)
	{
		// try to find me in the map...
		type_key key_level = HashFunct::Hashf( GetHashLevel(levels_), points[pts_ind]); // lowest level
		VoxelMap_it voxel = map_.find(key_level);
		if( voxel != map_.end())
		{
			voxel->second.AddProbability( probability_.ComputeVoxelProbability(voxel->second.GetProbability()) );
			if( voxel->second.GetProbability() > probability_.OccupancyThreshold() )
			{
				current_occupied_nodes_.insert(key_level);
			}
		}else
		{
			// voxels don't appear at leafs levels, we need to look up
			// from leafs parent upwards
			for(int32_t lvs_ind = levels_-1; lvs_ind >= 0; lvs_ind--)
			{
				// check for my nearest parent
				type_key key = HashFunct::Hashf( GetHashLevel(lvs_ind), points[pts_ind] );
				VoxelMap_it voxel = map_.find(key);
				if(voxel !=  map_.end())
				{
					// I found my nearest parent
					// we need to partition the new region
					// with lvs_ind+1 being the children level of lower_lvs
					deleted_nodes_.insert(key); // Also we need to delete from the map the parent
					Point center = voxel->second.GetCenter();
					for(uint32_t lower_lvs = (lvs_ind+1); lower_lvs <= levels_ ; lower_lvs++)
					{
						type_key parent_key = HashFunct::Hashf( GetHashLevel(lower_lvs), points[pts_ind] );
						PointList childrens;
						GetChildren3d( childrens, GetHashLevel(lower_lvs),  center );
						// this children must be added to the tree
						for(size_t cln_ind=0; cln_ind < kchildren; cln_ind++)
						{
							type_key key_children = HashFunct::Hashf( GetHashLevel(lower_lvs), childrens[cln_ind] );
							if( key_children != parent_key ) //I don't want to add my parent
							{
								// Up level voxels start with 0 probability
								Voxel voxel_hand(childrens[cln_ind], GetHashLevel(lower_lvs), 0.0 );
								map_.insert(std::pair<type_key, Voxel>(key_children, voxel_hand ));
							}else
							{
								center = childrens[cln_ind];
							}
						}
					}
					Voxel voxel_hand(center, GetHashLevel(levels_), 0.0 ); // TODO: add probability function
					map_.insert(std::pair<type_key, Voxel>(key_level, voxel_hand ));
					break;
				}
			}
		}
	}
	//old voxel delete
	DeleteOldNodes();
}

void MapStorage::DeleteOldNodes(void)
{
	keySet::iterator it_deleted;
	for(it_deleted = deleted_nodes_.begin(); it_deleted != deleted_nodes_.end(); it_deleted++)
	{
		type_key key = *(it_deleted);
		map_.erase( key );
	}
}

void MapStorage::UpdateFreeSpace(keyVector &free_voxels)
{
	tbb::parallel_for(size_t(0), (const size_t)free_voxels.size(), [&](size_t ind)
	//for(size_t ind =0; ind < free_voxels.size();ind++)
	{
		map_.at(free_voxels[ind]).ResetProbability();
	} );
}

// Gets helper functions
uint32_t MapStorage::GetHashLevel(uint32_t map_level)
{
	return levels_ - map_level;
}

void MapStorage::GetChildren(PointList & points, uint32_t level, const Point& center)
{
	double step 	= 	((double)(1<<level))*Config::voxel_leaf_size()/2.0;
	size_t childs	=	(int32_t)(1<<center.rows());
	int32_t dim 	=	0;
	Point pivot 	= 	Point::Constant(step);

	for(size_t ind=0; ind < childs; ind++)
	{
		pivot(dim) = -1*pivot(dim);
		points.push_back( center + pivot );
		dim++;
		(dim > center.rows() - 2)?dim=0:dim;
		if((int32_t)ind%(childs/2)==0)
		{
			pivot(center.rows()-1) = -1*pivot(center.rows()-1);
		}
	}
}

void MapStorage::GetChildren3d(PointList & points, uint32_t level, const Point& center)
{
	double step 			= 	((double)(1<<level))*Config::voxel_leaf_size()/2.0;
	Matrix local_children 	= children_*step;

	for(size_t ind=0; ind < kchildren; ind++)
	{
		points.push_back( center + local_children.col(ind) );
	}
}

bool MapStorage::GetKeyFromPoint(const Point &point, VoxelMap_it &vox_it)
{
	type_key key;
	for(size_t level = 0; level <= levels_; level++)
	{
		key = HashFunct::Hashf( GetHashLevel(level), point); // lowest level
		vox_it = map_.find(key);
		if( vox_it != map_.end())
		{
			return true;
		}
	}
	return false;
}

Voxel* MapStorage::GetVoxel(const Point &point)
{
	VoxelMap_it vox_it;
	if(GetKeyFromPoint(point, vox_it))
	{
		return &vox_it->second;
	}
	return NULL;
}

Voxel* MapStorage::GetVoxel(const type_key &key)
{
	VoxelMap_it vox_it;
	vox_it = map_.find(key);
	if(vox_it != map_.end())
	{
		return &vox_it->second;
	}
	return NULL;
}

Point MapStorage::GetPointFromKey(const type_key &key)
{
	Point result;
	data_key_size level = key>>3*HashFunct::ksize;
	if(level == levels_)
	{
		return Point(0.0,0.0,0.0);
	}
	for(uint32_t ind = 0; ind < (uint32_t)result.size() ; ind++)
	{
		result(2-ind) = (((double)(1<<level))*Config::voxel_leaf_size())*( (double)(s_data_key_size(key>>ind*HashFunct::ksize)) + 0.5);
	}
	return result;
}

bool MapStorage::GetKey(const Point &point, type_key &key)
{
	VoxelMap_it vox_it;
	if(GetKeyFromPoint(point, vox_it))
	{
		key = vox_it->first;
		return true;
	}
	return false;
}

void MapStorage::GetCurrentOccupiedNodes(keySet &current_occupied_nodes)
{
	current_occupied_nodes = current_occupied_nodes_;
}


// Collision test
bool MapStorage::IsPointCollisionFree(const Point& point, double radius)
{
	Voxel *voxel;
	voxel = GetVoxel( point );
	return IsPointCollisionFree(voxel, radius);
}
bool MapStorage::IsPointCollisionFree(Voxel *in_voxel, double radius)
{
	Voxel *voxel = in_voxel;
	if(voxel != NULL )
	{
		if(voxel->GetProbability() > Config::occupancy_threshold() )
		{
			return true;
		}
	}else
	{
		return true;
	}
	Point center = voxel->GetCenter();
	double step 	= 	radius;
	size_t childs	=	(int32_t)(1<<center.rows());
	int32_t dim 	=	0;
	Point pivot 	= 	Point::Constant(step);

	for(size_t ind=0; ind < childs; ind++)
	{
		pivot(dim) = -1*pivot(dim);
		voxel = GetVoxel( center + pivot );
		if(voxel != NULL )
		{
			if(voxel->GetProbability() > Config::occupancy_threshold() )
			{
				return true;
			}
		}else
		{
			return true;
		}
		dim++;
		(dim > center.rows() - 2)?dim=0:dim;
		if((int32_t)ind%(childs/2)==0)
		{
			pivot(center.rows()-1) = -1*pivot(center.rows()-1);
		}
	}

	return false;
}

bool MapStorage::IsLineCollisionFreeOLD(const Point& start, const Point& end, double radius)
{
	Point pivot = end - start;
	uint32_t line_interpolation = ( (pivot).norm() / (0.1*Config::voxel_leaf_size()) ) + 1.0;
	pivot /= double(line_interpolation);
	Point point = start;

	for(uint32_t ind = 0; ind < line_interpolation; ind++ )
	{
		if( IsPointCollisionFree(point, radius) )
		{
			return false;
		}
		point+=pivot;
	}
	return true;
}

bool MapStorage::IsLineCollisionFree(const Point& start, const Point& end, double radius)
{
	// Precompute some values
	type_key root_key;
	Point direction = end - start;
	Point inv_direction( 1.0/direction(0), 1.0/direction(1), 1.0/direction(2) );
	Point sign(inv_direction(0) < 0.0, inv_direction(1) < 0.0, inv_direction(2) < 0.0 );
	return !DetectCollision( key_root_, start, end, inv_direction, sign, radius);
}

// For voxel collision method
bool MapStorage::DetectCollision(const type_key &key, const Point& start, const Point& end,
		const Point &inv_direction, const Point &sign, double radius)
{
	data_key_size level = key>>3*HashFunct::ksize;
	if( level == 0 )
	{
		Voxel* voxel = GetVoxel(key);
		return IsPointCollisionFree(voxel, radius);

	}
	else
	{
		PointList children;
		Point center = GetPointFromKey(key);
		GetChildren3d( children, level - 1,  center );
		for( uint32_t idx=0; idx < kchildren; idx++)
		{
			type_key key_child = HashFunct::Hashf( level-1, children[idx] );
			if( IsVoxelInLine(key_child, start, end, inv_direction, sign) )
			{
				Voxel* voxel = GetVoxel(key_child);
				if( voxel == NULL || level == 1)
				{
					if ( DetectCollision(key_child, start, end, inv_direction, sign, radius) )
					{
						return true;
					}
				}
			}
		}
		return false;
	}
}

bool MapStorage::IsVoxelInLine(const type_key &key, const Point& start, const Point& end,
		const Point &inv_direction, const Point &sign)
{
	//Get bounds
	Point center 	=	GetPointFromKey(key);
	uint32_t level 	=	key>>3*HashFunct::ksize;
	double step 	= 	((double)(1<<level))*Config::voxel_leaf_size()/2.0;
	Point pivot 	= 	Point::Constant(step);

	Matrix bounds(3 , 2);
	bounds.col(0) = center - pivot;
	bounds.col(1) = center + pivot;

	double tmin = (bounds.col((uint32_t)sign(0))(0) - start(0)) * inv_direction(0);
	double tmax = (bounds.col((uint32_t)(1.0 - sign(0)))(0) - start(0)) * inv_direction(0);

	double tymin = (bounds.col((uint32_t)sign(1))(1) - start(1)) * inv_direction(1);
	double tymax = (bounds.col((uint32_t)(1.0 - sign(1)))(1) - start(1)) * inv_direction(1);

	if ( (tmin > tymax) || (tymin > tmax) )
	{
		return false;
	}
	if (tymin > tmin)
	{
		tmin = tymin;
	}
	if (tymax < tmax)
	{
		tmax = tymax;
	}
	double tzmin = (bounds.col((uint32_t)sign(2))(2) - start(2)) * inv_direction(2);
	double tzmax = (bounds.col((uint32_t)(1.0 - sign(2)))(2) - start(2)) * inv_direction(2);

	if ( (tmin > tzmax) || (tzmin > tmax) )
	{
		return false;
	}
	if (tzmin > tmin)
	{
		tmin = tzmin;
	}
	if (tzmax < tmax)
	{
		tmax = tzmax;
	}
	return ( (tmin < 1.0) && (tmax > 0.0) );
}

// Disk interacting map
void MapStorage::SaveMap(const std::string &file_directory)
{
	VoxelMap::iterator itm;
	std::ofstream file_map;
	file_map.open(file_directory);
	for(itm = map_.begin(); itm != map_.end(); itm++)
	{
		file_map.write((char*)&(itm->second), sizeof(itm->second));
	}
	file_map.close();
}
void MapStorage::LoadMap(const std::string &file_directory)
{
	Voxel voxel;
	std::ifstream file_map;
	file_map.open(file_directory, std::ios::in);
	map_.clear();
	while( !file_map.eof() )
	{
		file_map.read((char*)&voxel, sizeof(voxel));
		type_key key = HashFunct::Hashf( voxel.GetLevel(), voxel.GetCenter() );
		map_.insert(std::pair<type_key,Voxel>(key, voxel));
	}
	file_map.close();
}


// Debugging methods
void MapStorage::GetCompleteMap(stdVoxelVector &map)
{
	VoxelMap_it it;
	map.clear();
	for(it = map_.begin(); it != map_.end(); it++)
	{
		map.push_back(it->second);
	}
}
void MapStorage::GetOccupiedMap(stdVoxelVector &map)
{
	VoxelMap_it it;
	map.clear();

	for(it = map_.begin(); it != map_.end(); it++)
	{
		if(it->second.GetProbability() >  probability_.OccupancyThreshold() )
		{
			map.push_back(it->second);
		}
	}
}
} /* namespace rlmap */
