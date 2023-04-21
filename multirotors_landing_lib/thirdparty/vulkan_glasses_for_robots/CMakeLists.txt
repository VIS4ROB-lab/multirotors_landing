cmake_minimum_required(VERSION 3.0.0)
project(vulkan_glasses_for_robots VERSION 0.0.1)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(Vulkan REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${TINYOBJLOADER_INCLUDE_DIRS})

include_directories(include)

#hack to add headers in the qtcreator
FILE(GLOB_RECURSE hearderFiles "include/vrglasses_for_robots/*.h")
add_custom_target(headers SOURCES ${hearderFiles})

add_custom_target(
    vrglasses4robots_shader.frag.spv
    #ALL
    COMMAND glslangValidator -V -o vrglasses4robots_shader.frag.spv shaders/vrglasses4robots_shader.frag
    SOURCES shaders/vrglasses4robots_shader.frag
)


#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}_lib SHARED
  src/VulkanTools.cc
  src/vulkan_renderer.cc
)
target_link_libraries(${PROJECT_NAME}_lib
   glog gflags ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Vulkan_LIBRARY})


############
# BINARIES #
############
add_executable(${PROJECT_NAME}_fs
  src/fs_main.cc
  src/visim_processor.cc
  src/data_source.cc
)
target_link_libraries(${PROJECT_NAME}_fs ${PROJECT_NAME}_lib)

# Example script
add_executable(vulkan_renderer_example src/example/vulkan_renderer_example.cpp)
target_link_libraries(vulkan_renderer_example ${PROJECT_NAME}_lib)
