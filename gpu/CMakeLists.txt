add_executable(tx1 tx1.cpp)
target_link_libraries(tx1 ${PROJECT_NAME})

add_executable(gpu_stereo_kitti gpu_stereo_kitti.cc)
target_link_libraries(gpu_stereo_kitti ${PROJECT_NAME})
