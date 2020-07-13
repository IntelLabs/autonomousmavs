/**
 * @file   ootp_map.hpp
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


#ifndef OOTP_MAP_HPP_
#define OOTP_MAP_HPP_

#include <rlmap/rlmap.hpp>

namespace rlmap
{

class OOTP_Map : public RLMap
{
public:
	OOTP_Map();
	OOTP_Map(double map_size, double min_step);
	virtual ~OOTP_Map();

	virtual void AddDepthImage(const FrameObj &image, const Transform &pose) override;

	//debugging methods
	void GetAllOccupiedMap(stdVoxelVector &points);
	void GetRawPoints(PointList &raw_points);

private:
	uint32_t size_;
	PointList raw_points_;

	void Depht2Points(const FrameObj &image, const Transform &pose);
	bool Calculate3DPointFromDepth(const Pixel &pixel, uint16_t depth_raw, Point &projected_point);
	finline static bool isMinDepthValid(uint16_t depthMm, uint16_t minDepthMm) { return depthMm >= minDepthMm; }
	finline static bool isMaxDepthValid(uint16_t depthMm, uint16_t maxDepthMm) { return depthMm <= maxDepthMm; }

	OOTP_Map(const OOTP_Map&);
	OOTP_Map& operator=(const OOTP_Map&){ return *this;}

};

} /* namespace rlmap */



#endif /* OOTP_MAP_HPP_ */
