cmake_minimum_required(VERSION 3.8)
project(mpc_path_planning)

add_compile_options(-O3)
add_compile_options(-pg)

find_package(extension_ament_node REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
extension_ament_init()

# MPCPathPlanning
extension_ament_create_node(${PROJECT_NAME}_component
  NODE_NAME ${PROJECT_NAME}
  NODE_PLUGIN "MPCPathPlanning"
)
target_link_libraries(${PROJECT_NAME}_component
  tbb
)
# GlobalPathPlanning
extension_ament_create_node(global_path_planning_component
  NODE_NAME "global_path_planning"
  NODE_PLUGIN "GlobalPathPlanning"
)
target_link_libraries(global_path_planning_component
  tbb
)
# Controller
extension_ament_create_node(control_component
  NODE_NAME "control"
  NODE_PLUGIN "Control"
)
target_link_libraries(control_component
  tbb
)

install(
  DIRECTORY  launch config map rviz
  DESTINATION share/${PROJECT_NAME}/
)

extension_ament_final()
