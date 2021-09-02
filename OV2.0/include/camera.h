#ifndef __CAMERA_H_
#define __CAMERA_H_

#include <iostream>
#include <Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>

#include <memory>

namespace vslam{

class Camera
{
public:
	typedef std::shared_ptr<Camera> Ptr;
	
	float fx_, fy_, cx_, cy_, depth_scale_; // camera intrinsics

	Camera();
	Camera( float fx, float fy, float cx, float cy, float depth_scale = 0 );

	Eigen::Vector3d world2camera( const Eigen::Vector3d &p_w, const Sophus::SE3 &T_c_w );
	
	Eigen::Vector3d camera2world( const Eigen::Vector3d &p_c, const Sophus::SE3 &T_c_w );

	Eigen::Vector2d camera2pixel( const Eigen::Vector3d &p_c );

	Eigen::Vector3d pixel2camera( const Eigen::Vector2d &p_p, double depth = 1 );

	Eigen::Vector3d pixel2world( const Eigen::Vector2d &p_p, const Sophus::SE3 &T_c_w, double depth = 1 );

	Eigen::Vector2d world2pixel( const Eigen::Vector3d &p_w, const Sophus::SE3 &T_c_w );

};

}

#endif
