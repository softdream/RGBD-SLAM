#ifndef __MAP_POINT_H_
#define __MAP_POINT_H_

#include <iostream>
#include <Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>

#include <memory>

#include <opencv2/opencv.hpp>

#include "frame.h"

#include <list>

namespace vslam{

class MapPoint
{
public:
	typedef std::shared_ptr<MapPoint> Ptr;
	
	unsigned long id_ = -1;
	
	static unsigned long factory_id_;

	bool good_ = true; // whether a good point

	Eigen::Vector3d pos_ = Eigen::Vector3d( 0, 0, 0 ); // position of the landmark in world
	
	Eigen::Vector3d norm_ = Eigen::Vector3d( 0, 0, 0 ); // Normal of viewing direction

	cv::Mat descriptor_;
	
	std::list<Frame*> observed_frames_; // key frames that can observe this point
	
	int observed_times_ = 0; // 
	
	int correct_times_ = 0;

public:

	MapPoint();
	
	MapPoint( unsigned long id, Eigen::Vector3d position, Eigen::Vector3d norm, Frame *frame = nullptr, const cv::Mat &descriptor = cv::Mat() );	

	inline cv::Point3f getPositionCV() const 
	{
        
		return cv::Point3f( pos_(0, 0), pos_(1, 0), pos_(2, 0) );
    	}

	

	~MapPoint();

	static MapPoint::Ptr createMapPoint();

	static MapPoint::Ptr createMapPoint( const Eigen::Vector3d& pos_world,
        				     const Eigen::Vector3d& norm,
        				     const cv::Mat& descriptor,
        				     Frame* frame );
};

}

#endif
