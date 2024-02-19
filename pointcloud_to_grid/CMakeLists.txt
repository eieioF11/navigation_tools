cmake_minimum_required(VERSION 3.8)
project(pointcloud_to_grid)

find_package(extension_ament_node REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
extension_ament_init()

add_compile_options(-O3)

extension_ament_create_node(${PROJECT_NAME}_component
  NODE_NAME ${PROJECT_NAME}
  NODE_PLUGIN "PointCloudToGrid"
)
target_link_libraries(${PROJECT_NAME}_component
  tbb
)

target_include_directories(${PROJECT_NAME}_component PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_component ${PCL_LIBRARIES})
target_compile_definitions(${PROJECT_NAME}_component PRIVATE ${PCL_DEFINITIONS})

install(
  DIRECTORY  launch config
  DESTINATION share/${PROJECT_NAME}/
)

extension_ament_final()
