/**
 * @file   global.hpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   May, 2018
 * @brief  global definitions for exploration algorithm
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
#ifndef OOTP_GLOBAL_HPP_
#define OOTP_GLOBAL_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <stdint.h>
#include <deque>
#include <vector>

namespace ootp
{

#ifndef finline
  #ifdef WIN32
    #define finline __forceinline
  #else
    #define finline inline __attribute__((always_inline))
  #endif
#endif

typedef enum {VALID_GOAL=0, ZERO_GOAL, NSTATES} image_state_t;
typedef enum {FROM_VELOCITY=0, TURNING, YAW_TRANS, NANGLE_STATES} angle_sate_t;


typedef Eigen::Vector2i 								Pixel;
typedef std::vector<Pixel> 								PixelList;
typedef Eigen::Vector3d 								Point;
typedef Eigen::Vector2d 								Point2D;
typedef std::vector<Point> 								PointList;
typedef	Eigen::Matrix3d									Rotation3D;
typedef Eigen::Quaterniond								quat;
typedef Eigen::VectorXd									VectorX;
typedef	Eigen::MatrixXd									MatrixXD;


template<typename T>
finline T sign(T value)
{
	return value < 0 ? -1. : 1.;
}

Rotation3D finline GetYawRotation(double yaw)
{
	Rotation3D rot; rot<< cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;
	return rot;
}
Rotation3D finline GetRollRotation(double roll)
{
	Rotation3D rot; rot<< 1.0, 0.0, 0.0, 0.0, cos(roll), -sin(roll), 0.0, sin(roll), cos(roll);
	return rot;
}
Rotation3D finline GetPitchRotation(double pitch)
{
	Rotation3D rot; rot<< cos(pitch), 0.0, sin(pitch), 0.0, 1.0, 0.0, -sin(pitch), 0.0, cos(pitch);
	return rot;
}
Rotation3D finline GetZYXRotation(double roll, double pitch, double yaw)
{
	Rotation3D rot; rot=GetYawRotation(yaw)*GetPitchRotation(pitch)*GetRollRotation(roll);
	return rot;
}

struct Transform
{
	Point 				position_;
	Rotation3D			rotation_;
	double				yaw_;
	Transform()
	{

	}
	Transform(Point position, Point rotation) : position_(position)
	{
		rotation_ = GetZYXRotation(rotation(0), rotation(1), rotation(2));
		yaw_ = rotation(2);
	}
	Transform(Point position, double yaw) : position_(position)
	{
		rotation_ = GetYawRotation(yaw);
		yaw_ = yaw;
	}
	Transform(Point position, Rotation3D rotation) : position_(position), rotation_(rotation)
	{
		yaw_ = 0;
	}
};


} /* ootp */




#endif /* OOTP_GLOBAL_HPP_ */
