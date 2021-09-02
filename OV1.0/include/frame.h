#ifndef __FRAME_H_
#define __FRAME_H_

#include <iostream>
#include <Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>

#include <memory>

#include <opencv2/opencv.hpp>

#include "camera.h"

namespace vslam{

class Frame
{
public:
	typedef std::shared_ptr<Frame> Ptr;

	unsigned long id_ = -1; // frame id
	
	double time_stamp_ = -1;
	
	Sophus::SE3 T_c_w_; // transform from world to camera

	Camera::Ptr camera_ = nullptr;

	cv::Mat color_, depth_; // rgb and depth image

	bool is_key_frame_ = false;
public:
	Frame();
	Frame( long id, double time_stamp = 0, Sophus::SE3 T_c_w = Sophus::SE3(), Camera::Ptr camera = nullptr, cv::Mat color = cv::Mat(), cv::Mat depth = cv::Mat() );

	~Frame();

	static Frame::Ptr createFrame();

	double findDepth( const cv::KeyPoint &kp );

	Eigen::Vector3d getCameraCenter() const;

	bool isInFrame( const Eigen::Vector3d &pt_world );

	void setPose( const Sophus::SE3 &T_c_w );
};

}

#endif
