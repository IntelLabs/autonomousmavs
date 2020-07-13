/**
 * @file   rlmap.cpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   June, 2018
 * @brief  This is the implementation for our customized version of map
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

#include <rlmap/rlmap.hpp>
#include <rlmap/config.hpp>
#include <tbb/parallel_for.h>
#include <tbb/mutex.h>
#include <rlmap/tictoc.hpp>

namespace rlmap
{

RLMap::RLMap()
{
	map_ = new MapStorage();

	min_distance_ = (uint16_t)(Config::min_depth_distance() / Config::depth_factor());
	max_distance_ = (uint16_t)(Config::max_depth_distance() / Config::depth_factor());

	inverse_focal_length_ << 1.0 / Config::calibration_rfx(), 1.0 / Config::calibration_rfy();
	principal_point_ << Config::calibration_rpx(), Config::calibration_rpy();

	Point projected_point_min;
	Point projected_point_max;
	projected_point_max << (Pixel(1,1).cast<double>() - principal_point_).cwiseProduct(inverse_focal_length_) * Config::max_depth_distance() , Config::max_depth_distance();
	projected_point_min << (Pixel(0,0).cast<double>() - principal_point_).cwiseProduct(inverse_focal_length_) * Config::max_depth_distance() , Config::max_depth_distance();

	Point step = projected_point_max - projected_point_min;
	step_size_ = std::floor(Config::voxel_leaf_size() / (2.0*fabs(step(0))));

	PrecalculateRays();
	free_space_image_percent_ = (uint32_t)( (double)ray_points_.size()*Config::image_percent() );

}

RLMap::~RLMap()
{
	delete map_;
}

void RLMap::PrecalculateRays(void)
{
	double depth = Config::max_depth_distance();
	// TODO:: calculate this parameter depending on drone size
	int32_t safe_region = 50;

	for(int32_t rows = safe_region; rows < Config::frame_height() - safe_region; )
	{
		for(int32_t cols = safe_region; cols < Config::frame_width() - safe_region; )
		{
			Point projected_point;
			projected_point << (Pixel(cols, rows).cast<double>() - principal_point_).cwiseProduct(inverse_focal_length_) * depth , depth;

			ray_points_.push_back( Config::camera_rotation_wrt_drone()*(projected_point) + Config::camera_translation_wrt_drone() );

			cols+=step_size_;
		}
		rows+=step_size_;
	}
}

void RLMap::UpdateFreeSpace( const Transform &pose )
{

	//generate index
	std::set<int> index_taken;
	keySet occupied_voxels;
	keyVector free_voxels;
	tbb::mutex mutex;

	uint32_t rays = ray_points_.size();
	map_->GetCurrentOccupiedNodes(occupied_voxels);

	while(  free_space_image_percent_ > index_taken.size() )
	{
		uint32_t random = ((double)rand()) / ((double)RAND_MAX)*( rays - 1);
		index_taken.insert(random);
	}
	keyVector indexs;
	indexs.assign(index_taken.begin(), index_taken.end());

	tbb::parallel_for(size_t(0), (const size_t)indexs.size(), [&](size_t idx)
	{
		Point pivot = pose.rotation_*( ray_points_[indexs[idx]] );
		uint32_t line_interpolation = 1.0 + pivot.norm() / Config::voxel_leaf_size() ;
		pivot /= double(line_interpolation);
		Point point = pose.position_;

		for(uint32_t ind = 1; ind < line_interpolation; ind++ )
		{
			type_key key;
			if ( map_->GetKey(point, key) )
			{
				if( occupied_voxels.find(key) != occupied_voxels.end()  )
				{
					break;
				}

				mutex.lock();
				free_voxels.push_back(key);
				mutex.unlock();
			}else
			{
				break;
			}
			point+=pivot;
		}
	} );
	map_->UpdateFreeSpace(free_voxels);
}

void RLMap::AddOccupiedPoints(PointList& points)
{
	map_->InsertOccupiedPoints(points);

}
// collision methods
bool RLMap::IsLineCollisionFree(const Point& start, const Point& end, double radius)
{
	return map_->IsLineCollisionFree(start, end, radius);
}
bool RLMap::IsLineCollisionFreeOLD(const Point& start, const Point& end, double radius)
{
	return map_->IsLineCollisionFreeOLD(start, end, radius);
}

// Disk interacting map
void RLMap::SaveMap(const std::string &file_directory)
{
	map_->SaveMap(file_directory);
}
void RLMap::LoadMap(const std::string &file_directory)
{
	map_->LoadMap(file_directory);
}
// Debugging Methods
void RLMap::GetOccupiedPoints(stdVoxelVector &points)
{
	map_->GetOccupiedMap(points);
}
void RLMap::GetCompleteMap(stdVoxelVector &map)
{
	map_->GetCompleteMap(map);
}
} /* namespace rlmap */
