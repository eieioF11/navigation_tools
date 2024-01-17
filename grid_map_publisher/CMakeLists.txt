cmake_minimum_required(VERSION 3.8)
project(grid_map_publisher)

find_package(extension_ament_node REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)
find_package(ament_cmake_auto REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)
extension_ament_init()

add_compile_options(-O3)

extension_ament_create_node(${PROJECT_NAME}_component
  NODE_NAME ${PROJECT_NAME}
  NODE_PLUGIN "GridMapPublisher"#grid_map_publisher
)

target_include_directories(${PROJECT_NAME}_component SYSTEM PRIVATE
  ${GRAPHICSMAGICKCPP_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_component
  tbb
  ${GRAPHICSMAGICKCPP_LIBRARIES}
)

install(
  DIRECTORY  map
  DESTINATION share/${PROJECT_NAME}/
)

extension_ament_final()
