cmake_minimum_required(VERSION 3.8)
project(mpc_path_planning)

find_package(extension_ament_node REQUIRED)

extension_ament_init()

add_compile_options(-O3)

extension_ament_create_node(${PROJECT_NAME}_component
  NODE_NAME ${PROJECT_NAME}
  NODE_PLUGIN "MPCPathPlanning"
)
target_link_libraries(${PROJECT_NAME}_component
  tbb
)

install(
  DIRECTORY  launch config
  DESTINATION share/${PROJECT_NAME}/
)

extension_ament_final()
