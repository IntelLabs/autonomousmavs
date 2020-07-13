/**
 * @file   global.hpp
 * @Author Leobardo Campos (leobardo.e.campos.macias@intel.com)
 * @date   June, 2018
 * @brief  Definitions and common headers
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


#ifndef RLGLOBAL_HPP_
#define RLGLOBAL_HPP_

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <inttypes.h>

namespace rlmap
{

#ifndef finline
  #ifdef WIN32
    #define finline __forceinline
  #else
    #define finline inline __attribute__((always_inline))
  #endif
#endif

class Voxel;
typedef unsigned __int128 									uint128_t;
typedef uint64_t											type_key;
typedef uint16_t											data_key_size;
typedef int16_t												s_data_key_size;
typedef std::set<type_key>									keySet;
typedef std::vector<type_key>								keyVector;
typedef std::map<type_key, Voxel> 							VoxelMap;

typedef Eigen::Vector3d 									Point;
typedef Eigen::MatrixXd 									Matrix;
typedef	std::vector<Point>									PointList;
typedef std::vector<Voxel> 									stdVoxelVector;
typedef VoxelMap::iterator									VoxelMap_it;
typedef Eigen::Vector2i 									Pixel;
typedef Eigen::Vector2d 									Point2D;
typedef	std::vector<Pixel>									PixelList;
typedef cv::Mat 											FrameObj;
typedef Eigen::Matrix3d 									Rotation3D;
typedef Eigen::Quaterniond									Quaternion;


template<typename T>
T clamp(T value, T min, T max)
{
	const T tmp = value < min ? min : value;
	return tmp > max ? max : tmp;
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

		Transform(Point position, Quaternion rotation) : position_(position)
		{
			rotation_ = rotation.normalized().toRotationMatrix();
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
		Transform(Point position, Rotation3D rotation, double yaw) : position_(position), rotation_(rotation), yaw_(yaw)
		{
		}
};

}/* namespace rlmap */




#endif /* GLOBAL_HPP_ */
