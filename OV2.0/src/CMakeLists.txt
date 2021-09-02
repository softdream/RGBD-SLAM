# generate the lib files

# headers of the source files
#INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include" )

# add the source files
AUX_SOURCE_DIRECTORY( ${PROJECT_SOURCE_DIR}/src slam_src )

ADD_LIBRARY( vslam SHARED ${slam_src} )

ADD_LIBRARY( vslam_static STATIC ${slam_src} )

target_link_libraries( vslam
    ${THIRD_PARTY_LIBS}
)

SET_TARGET_PROPERTIES( vslam_static PROPERTIES OUTPUT_NAME "vslam" )
