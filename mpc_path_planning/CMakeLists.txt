cmake_minimum_required(VERSION 3.8)
project(mpc_path_planning)

find_package(extension_ament_node REQUIRED)

extension_ament_init()

add_compile_options(-O3)

extension_ament_create_node(${PROJECT_NAME}_component
  NODE_NAME ${PROJECT_NAME}
  NODE_PLUGIN "MPCPathPlanning"
)
extension_ament_create_node(global_path_planning_component
  NODE_NAME "global_path_planning"
  NODE_PLUGIN "GlobalPathPlanning"
)
extension_ament_create_node(control_component
  NODE_NAME "control"
  NODE_PLUGIN "Control"
)
target_link_libraries(${PROJECT_NAME}_component
  tbb
)
target_link_libraries(global_path_planning_component
  tbb
)
target_link_libraries(control_component
  tbb
)

install(
  DIRECTORY  launch config map rviz
  DESTINATION share/${PROJECT_NAME}/
)

extension_ament_final()
