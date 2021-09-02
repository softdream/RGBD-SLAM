#include "map.h"

namespace vslam{

Map::Map()
{

}

Map::~Map()
{

}

void Map::insertKeyFrame ( Frame::Ptr frame )
{
	std::cout<<"Key frame size = "<<key_frames_.size()<<std::endl;
    
	if ( key_frames_.find(frame->id_) == key_frames_.end() ){
        	key_frames_.insert( std::make_pair(frame->id_, frame) );
    	}
    	else {
        	key_frames_[ frame->id_ ] = frame;
    	}
}

void Map::insertMapPoint ( MapPoint::Ptr map_point )
{
    	if ( map_points_.find(map_point->id_) == map_points_.end() ) {
		map_points_.insert( std::make_pair(map_point->id_, map_point) );
    	}
    	else {
        	map_points_[map_point->id_] = map_point;
    	}
}


}
