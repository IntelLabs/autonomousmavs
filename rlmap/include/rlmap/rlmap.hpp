/**
 * @file   rlmap.hpp
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

#ifndef RLMAP_HPP_
#define RLMAP_HPP_



#include <rlmap/global.hpp>
#include <rlmap/map_storage.hpp>

namespace rlmap
{

class RLMap
{
public:
	RLMap();
	virtual ~RLMap();

	virtual void AddDepthImage(const FrameObj &image, const Transform &pose){};
	void AddOccupiedPoints(PointList& points);

	// collision methods
	bool IsLineCollisionFree(const Point& start, const Point& end, double radius);
	bool IsLineCollisionFreeOLD(const Point& start, const Point& end, double radius);

	// Disk interacting map
	void SaveMap(const std::string &file_directory);
	void LoadMap(const std::string &file_directory);

	//debugging methods
	void GetCompleteMap(stdVoxelVector &map);


protected:


	uint16_t 	min_distance_;
	uint16_t 	max_distance_;
	Point2D 	inverse_focal_length_;
	Point2D 	principal_point_;
	uint32_t 	step_size_;
	uint32_t 	free_space_image_percent_;

	void UpdateFreeSpace( const Transform &pose );

	//debugging methods
	void GetOccupiedPoints(stdVoxelVector &points);




private:
	MapStorage 		*map_;
	PointList 		ray_points_;

	void PrecalculateRays(void);

	// prevent copy of class
	RLMap(const RLMap&);
	RLMap& operator=(const RLMap&){ return *this;}
};

} /* namespace rlmap */

#endif /* RLMAP_HPP_ */
