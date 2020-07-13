/**
 * @file   hash_functions.hpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   June, 2018
 * @brief  This is the implementation of the hash function for map a point to a key
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

#ifndef HASH_FUNCTIONS_HPP_
#define HASH_FUNCTIONS_HPP_

#include <rlmap/global.hpp>
#include <rlmap/config.hpp>

namespace rlmap
{

class HashFunct
{
public:
	static const uint8_t 		ksize = 2*sizeof(type_key);
	static const data_key_size 	kmask = pow(2, ksize) - 1;

	finline static type_key Hashf(data_key_size level, const Point& point)
	{
		type_key key = level;
		double div_level = 1.0 / (((double)(1<<level))*Config::voxel_leaf_size());
		key <<= ksize;
		key |= (kmask)&data_key_size(std::floor(point(0)*div_level));
		key <<= ksize;
		key |= (kmask)&data_key_size(std::floor(point(1)*div_level));
		key <<= ksize;
		key |= (kmask)&data_key_size(std::floor(point(2)*div_level));
		return key;
	}

	finline static type_key Hashfg(uint32_t level, const Point& point)
	{
		type_key key = level;
		uint8_t tmp_key;
		for(size_t ind = 0; ind < (size_t)point.size() ; ind++)
		{
			key <<= ksize;

			double div = point(ind) / (((double)(1<<level))*Config::voxel_leaf_size());
			tmp_key = std::floor(div);
			key |= (kmask)&tmp_key;
		}
		return key;
	}
};

} /* namespace rlmap */




#endif /* HASH_FUNCTIONS_HPP_ */
