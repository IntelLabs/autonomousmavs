/**
 * @file   map_storage.hpp
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
#ifndef MAP_STORAGE_HPP_
#define MAP_STORAGE_HPP_

#include <rlmap/global.hpp>
#include <rlmap/voxel.hpp>
#include <rlmap/probability_manager.hpp>
#include <map>

namespace rlmap
{
const double	invlog2 		= 	(1.0/std::log(2.0));
const int32_t 	kchildren		= 1 << 3;

class MapStorage
{
public:
	// general methods
	MapStorage();
	virtual ~MapStorage();
	void InsertOccupiedPoints(PointList& points);
	void UpdateFreeSpace(keyVector &free_voxels);

	// get helper functions
	Voxel* GetVoxel(const Point &point);
	Voxel* GetVoxel(const type_key &key);
	bool GetKey(const Point &point, type_key &key);
	void GetCurrentOccupiedNodes(keySet &current_occupied_nodes);

	// collision methods
	bool IsLineCollisionFree(const Point& start, const Point& end, double radius);
	bool IsLineCollisionFreeOLD(const Point& start, const Point& end, double radius);
	bool IsPointCollisionFree(const Point& point, double radius);
	bool IsPointCollisionFree(Voxel *in_voxel, double radius);
	// TODO:: check that saving is performed correctly
	void SaveMap(const std::string &file_directory);
	void LoadMap(const std::string &file_directory);

	// debbug
	void GetCompleteMap(stdVoxelVector &map);
	void GetOccupiedMap(stdVoxelVector &map);

private:
	VoxelMap	 					map_;
	data_key_size					levels_;
	keySet							deleted_nodes_;
	CProbabilityMan					probability_;
	keySet							current_occupied_nodes_;
	type_key 						key_root_;
	Matrix							children_;
	// general methods
	void InitMap(void);
	void Initialize3dChildren(void);
	void FillLevels(PointList& points);
	void DeleteOldNodes(void);

	// get functions
	void GetChildren(PointList& points, uint32_t level, const Point& center);
	void GetChildren3d(PointList& points, uint32_t level, const Point& center);
	finline uint32_t GetHashLevel(uint32_t map_level);
	finline bool GetKeyFromPoint(const Point &point, VoxelMap_it &vox_it);
	finline Point GetPointFromKey(const type_key &key);

	// for voxel collision method
	bool DetectCollision(const type_key &key, const Point& start, const Point& end,
			const Point &inv_direction, const Point &sign, double radius);
	finline bool IsVoxelInLine(const type_key &key, const Point& start, const Point& end,
			const Point &inv_direction, const Point &sign);

};

} /* namespace rlmap */



#endif /* MAP_STORAGE_HPP_ */
