#ifndef __MAP_H_
#define __MAP_H_

#include <iostream>
#include <Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>

#include <memory>

#include <opencv2/opencv.hpp>

#include <unordered_map>

#include "frame.h"
#include "mapPoint.h"

namespace vslam{

class Map
{
public:
	typedef std::shared_ptr<Map> Ptr;

	std::unordered_map<unsigned long, MapPoint::Ptr> map_points_; // all landmarks
	std::unordered_map<unsigned long, Frame::Ptr> key_frames_; // all key frames

public:
	Map();
	~Map();

	void insertKeyFrame( Frame::Ptr frame ) ;
	void insertMapPoint( MapPoint::Ptr map_point );

};

}

#endif
