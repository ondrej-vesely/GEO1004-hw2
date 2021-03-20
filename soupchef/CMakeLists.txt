cmake_minimum_required(VERSION 3.17)
project(geo1004.soupchef)

add_executable(soupchef 
  main.cpp
)
set_target_properties(soupchef PROPERTIES 
  CXX_STANDARD 17
)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../data/bk_soup.obj ${CMAKE_CURRENT_BINARY_DIR}/bk_soup.obj COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../data/cube.obj ${CMAKE_CURRENT_BINARY_DIR}/cube.obj COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../data/cube_soup.obj ${CMAKE_CURRENT_BINARY_DIR}/cube_soup.obj COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../data/isolated_cubes.obj ${CMAKE_CURRENT_BINARY_DIR}/isolated_cubes.obj COPYONLY)

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT soupchef)

