CMAKE_MINIMUM_REQUIRED( VERSION 2.8)

PROJECT(useSophus)

FIND_PACKAGE(Sophus REQUIRED)

INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS})

add_executable(useSophus test.cpp)

TARGET_LINK_LIBRARIES(useSophus ${Sophus_LIBRARIES})
