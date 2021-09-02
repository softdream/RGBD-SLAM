#include "frame.h"

namespace vslam{

Frame::Frame()
{

}

Frame::Frame( long id, 
	      double time_stamp,
              Sophus::SE3 T_c_w,  
	      Camera::Ptr camera,
              cv::Mat color, 
 	      cv::Mat depth ) : id_( id ),
				time_stamp_( time_stamp ),
				T_c_w_( T_c_w ),
				camera_( camera ),
				color_( color ),
				depth_( depth )
{

}

Frame::~Frame()
{

}

double Frame::findDepth( const cv::KeyPoint &kp )
{
	int x = cvRound( kp.pt.x );
	int y = cvRound( kp.pt.y );

	int d = depth_.ptr<int>(y)[x];

	if( d != 0 ){
		return double(d) / camera_->depth_scale_;
	}
	else {
		int dx[4] = { -1, 0, 1, 0 };
		int dy[4] = { 0, -1, 0, 1 };
	
		for( int i = 0; i < 4; i ++ ){
			d = depth_.ptr<int>( y + dy[i] )[x + dx[i]];
			
			if( d != 0 ){
				return double(d) / camera_->depth_scale_;
			}
		}
	}

	return -1.0;
}

void Frame::setPose( const Sophus::SE3 &T_c_w )
{
	T_c_w_ = T_c_w;
}

Eigen::Vector3d Frame::getCameraCenter() const
{
	return T_c_w_.inverse().translation();
}

bool Frame::isInFrame( const Eigen::Vector3d &pt_world )
{
	Eigen::Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
	
	// cout<<"P_cam = "<<p_cam.transpose()<<endl;
	
	if ( p_cam( 2, 0 ) < 0 )
		return false;
    
	Eigen::Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    	
	// cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
    	
	return pixel(0, 0) > 0 && pixel(1, 0) > 0 && pixel(0, 0) < color_.cols && pixel(1, 0) < color_.rows;

}

}
