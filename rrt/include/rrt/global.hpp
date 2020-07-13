/**
 * @file   global.hpp
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
#ifndef RRT_GLOBAL_HPP_
#define RRT_GLOBAL_HPP_

#include <Eigen/Core>
#include <vector>
#include <stdint.h>

namespace rrt
{

typedef enum {kMax=0, kMin, kNIndex}eIndex;
typedef enum {kFov=0, kMap, kNTypes}eDirection;

const double k_rrt_solution_error = 0.1;

#ifndef finline
  #ifdef WIN32
    #define finline __forceinline
  #else
    #define finline inline __attribute__((always_inline))
  #endif
#endif

class 	Node;

typedef Eigen::Vector3d 								Point;
typedef std::vector<Point> 								PointList;
typedef Eigen::MatrixXd 								Matrix;
typedef std::vector<Node>								Nodes;


static Matrix CalculateRotationMatrix(const double &roll, const double &pitch, const double &yaw)
{
	Matrix Rz(3,3);
	Rz<< 	cos(yaw), -sin(yaw), 0.0,
			sin(yaw), cos(yaw), 0.0,
			0.0, 0.0, 1.0;

	Matrix Rx(3,3);
	Rx<< 		1.0, 0.0, 0.0,
			0.0, cos(roll), -sin(roll),
			0.0, sin(roll), cos(roll);

	Matrix Ry(3,3);
	Ry<< 	cos(pitch), 0.0, sin(pitch),
			 0.0, 1.0, 0.0,
			 -sin(pitch), 0.0, cos(pitch);
	return Rz*Ry*Rx;
}
static Matrix CalculateRotationMatrix(const double &theta)
{
	Matrix Rz(2,2);
	Rz<< 	cos(theta), -sin(theta),
			sin(theta), cos(theta);
	return Rz;
}

} /* namespace rrt */

#endif /* RRT_GLOBAL_HPP_ */
