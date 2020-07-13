/**
 * @file   voxel.hpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   June, 2018
 * @brief  This is the implementation of the voxel properties
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

#ifndef VOXEL_HPP_
#define VOXEL_HPP_

#include <rlmap/global.hpp>

namespace rlmap
{

class Voxel
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Voxel(void);
	Voxel(const Point &center, uint32_t level, int32_t probability);
	virtual ~Voxel(void);

	// Returning methods
	Point GetCenter(void) {return center_; }
	uint32_t GetLevel(void) {return level_; }
	double GetProbability(void) {return probability_; }

	// Assigning methods
	void AddProbability(const double probability);
	void ResetProbability(void);

private:
	 Point 					center_;
	 double 				probability_;
	 uint32_t				level_;

};

} /* namespace rlmap */



#endif /* VOXEL_HPP_ */
