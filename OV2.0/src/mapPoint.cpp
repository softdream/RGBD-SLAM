#include "mapPoint.h"

namespace vslam{

MapPoint::MapPoint()
{

}


MapPoint::MapPoint( unsigned long id, 
		    Eigen::Vector3d position, 
		    Eigen::Vector3d norm, 
		    Frame *frame, 
		    const cv::Mat &descriptor ): id_( id ),
						 pos_( position ),
						 norm_( norm ),
						 descriptor_( descriptor )
{	
	observed_frames_.push_back( frame );
}

MapPoint::~MapPoint()
{

}

MapPoint::Ptr MapPoint::createMapPoint()
{
	return MapPoint::Ptr( new MapPoint( factory_id_++, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0) ) );

}

MapPoint::Ptr MapPoint::createMapPoint( const Eigen::Vector3d& pos_world,
                              const Eigen::Vector3d& norm,
                              const cv::Mat& descriptor,
                              Frame* frame )
{
	return MapPoint::Ptr( new MapPoint( factory_id_++, pos_world, norm, frame, descriptor ) );

}

unsigned long MapPoint::factory_id_ = 0;

}
