/**
 * @file   ootp_map.cpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   July, 2018
 * @brief  This is the implementation for our customized version of map for the ootp application
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

#include <rlmap/ootp_map.hpp>
#include <rlmap/config.hpp>

#include <tuple>
#include <tbb/parallel_for.h>
#include <tbb/mutex.h>


namespace rlmap
{
OOTP_Map::OOTP_Map() : RLMap()
{
	Point step_dummy_one;
	Point step_dummy_two;
	Point step(1, 1, 1);
	if( Calculate3DPointFromDepth(Pixel(1,1), Config::max_depth_distance()/Config::depth_factor(), step_dummy_one)
	 && Calculate3DPointFromDepth(Pixel(0,0), Config::max_depth_distance()/Config::depth_factor(), step_dummy_two))
	{
		step = step_dummy_one - step_dummy_two;
	}
	size_ = std::floor(Config::voxel_leaf_size() / (2.0*fabs(step(0))));
}
OOTP_Map::OOTP_Map(double map_size, double min_step)
{
	Config::map_params(map_size, min_step);
	RLMap();
	Point step_dummy_one;
	Point step_dummy_two;
	Point step(1, 1, 1);
	if( Calculate3DPointFromDepth(Pixel(1,1), Config::max_depth_distance()/Config::depth_factor(), step_dummy_one)
	 && Calculate3DPointFromDepth(Pixel(0,0), Config::max_depth_distance()/Config::depth_factor(), step_dummy_two))
	{
		step = step_dummy_one - step_dummy_two;
	}
	size_ = std::floor(Config::voxel_leaf_size() / (2.0*fabs(step(0))));
}
OOTP_Map::~OOTP_Map()
{

}
void OOTP_Map::AddDepthImage(const FrameObj &image, const Transform &pose)
{
	Depht2Points( image, pose);
	UpdateFreeSpace(pose);

}

void OOTP_Map::Depht2Points(const FrameObj &image, const Transform &pose)
{
	PointList 		points;
	std::vector< std::tuple<Pixel, uint16_t> > 	pixels;
	std::tuple<Pixel, uint16_t> 				pixel;

	for(int32_t rows = 0; rows < image.rows; )
	{
		for(int32_t cols=0; cols < image.cols; )
		{
			// cols -> x rows -> y, to match opencv with rs
			pixels.push_back( std::make_tuple( Pixel(cols, rows),  ((uint16_t*)image.data)[image.cols*rows + cols]) );
			cols+=size_;
		}
		rows+=size_;
	}
	for(size_t ind =0; ind < pixels.size() ;ind++)
	//tbb::parallel_for(size_t(0), (const size_t)pixels.size(), [&](size_t ind)
	{
		pixel = pixels[ind];
		Point local;
		if(Calculate3DPointFromDepth( std::get<0>(pixel), std::get<1>(pixel), local ))
		{
			local = Config::camera_rotation_wrt_drone()*local + Config::camera_translation_wrt_drone();
			Point global = pose.rotation_*local + pose.position_;
			points.push_back(global);
		}
	}//);
	raw_points_ = points;
	AddOccupiedPoints(points);
}

bool OOTP_Map::Calculate3DPointFromDepth(const Pixel &pixel, uint16_t depth_raw, Point &projected_point)
{
    if( isMinDepthValid(depth_raw, min_distance_) && isMaxDepthValid(depth_raw, max_distance_) )
    {
		const double depth = static_cast<double>(depth_raw) * Config::depth_factor();
		projected_point << (pixel.cast<double>() - principal_point_).cwiseProduct(inverse_focal_length_) * depth , depth;
		return true;
    }
    return false;
}

// debugging methods
void OOTP_Map::GetRawPoints(PointList &raw_points)
{
	raw_points = raw_points_;
}
void OOTP_Map::GetAllOccupiedMap(stdVoxelVector &points)
{
	GetOccupiedPoints(points);
}

} /* namespace rlmap */

