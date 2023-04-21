cmake_minimum_required(VERSION 3.0)
project(multirotors_landing_lib VERSION 0.0.1)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)

###########
# Options #
###########
option(BUILD_TESTS "Building the tests" ON)
option(PYTHON_BINDINGS "Whether to build python bindings" ON)
option(BUILD_COMMUNICATION_EXAMPLES "Whether to build communication examples" ON)

########################
# Finding Dependencies #
########################
message(STATUS "Setup Dependencies")

# Eigen
find_package(Eigen3 REQUIRED)

# YAML
find_package(yaml-cpp REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED)

# OctoMap (for collision detections)
find_package(octomap REQUIRED)

# OpenMP
find_package(OpenMP REQUIRED)
if(NOT TARGET OpenMP::OpenMP_CXX)
  find_package(Threads REQUIRED)
  add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
  set_property(TARGET OpenMP::OpenMP_CXX
               PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
  # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
  set_property(TARGET OpenMP::OpenMP_CXX
               PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
endif()

# Pybind11
if (PYTHON_BINDINGS)
  # We need the latest version to have the support for enum types
  find_package(pybind11 2.6.2 EXACT QUIET)
  if (NOT pybind11_FOUND)
    include(cmake/pybind11.cmake)
  endif()
endif()

# Vulkan glasses (custom library)
set(VULKAN_DIR
  $ENV{ML_PATH}/multirotors_landing_lib/thirdparty/vulkan_glasses_for_robots
)
include(cmake/vulkan_glasses.cmake)

# Utility for linking
set(THIRDPARTIES_LIBRARIES
  ${VULKAN_DIR}/build/libvulkan_glasses_for_robots_lib.so
)

set(DEP_LIBRARIES
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  OpenMP::OpenMP_CXX
  ${THIRDPARTIES_LIBRARIES}
)

# Utility for include
set(THIRDPARTIES_INCLUDE
  ${VULKAN_DIR}/include
)

set(DEP_INCLUDES
  ${YAML_CPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenMP_INCLUDE_PATH}
  ${THIRDPARTIES_INCLUDE}
)

################
# Set up build #
################
file(GLOB ML_RESOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/collider/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/common/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/communication/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/envs/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/renderer/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robots/*.cpp
)

# Create file lists for tests
file(GLOB_RECURSE ML_TEST_SOURCES
  tests/envs/*.cpp
)

# Library and Executables
include_directories(include ${DEP_INCLUDES})

add_library(${PROJECT_NAME} ${ML_RESOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC "include")
target_link_libraries(${PROJECT_NAME} ${DEP_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

# Create Python bindings
if (PYTHON_BINDINGS)
  # Get the source files
  file(GLOB ML_GYM_RESOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/wrapper/*.cpp
  )

  # Use pybind11
  pybind11_add_module(mlgym MODULE ${ML_GYM_RESOURCES})
  target_include_directories(mlgym PRIVATE
    ${PYBIND11_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(mlgym PRIVATE ${PROJECT_NAME})
endif()

# Build tests
if(BUILD_TESTS AND ML_TEST_SOURCES)
  # GTest include
  include(cmake/gtest.cmake)
  enable_testing()

  # Tests
  add_executable(test_lib ${ML_TEST_SOURCES})
  target_link_libraries(test_lib PUBLIC
    ${PROJECT_NAME}
    gtest
    gtest_main
  )
  add_test(test_lib test_lib)
endif()

# Build executables to test
if (BUILD_COMMUNICATION_EXAMPLES)

  # Sockets
  add_executable(server_socket 
    tests/communication/test_socket_server.cpp
  )
  target_link_libraries(server_socket PUBLIC
    ${PROJECT_NAME}
  )

  add_executable(client_socket 
    tests/communication/test_socket_client.cpp
  )
  target_link_libraries(client_socket PUBLIC
    ${PROJECT_NAME}
  )

  # Communicators
  add_executable(client_communicator 
    tests/communication/test_communicator_client.cpp
  )
  target_link_libraries(client_communicator PUBLIC
    ${PROJECT_NAME}
  )

  add_executable(server_communicator 
    tests/communication/test_communicator_server.cpp
  )
  target_link_libraries(server_communicator PUBLIC
    ${PROJECT_NAME}
  )

  add_executable(python_socket_client 
    tests/communication/python_socket_client.cpp
  )
  target_link_libraries(python_socket_client PUBLIC
    ${PROJECT_NAME}
  )

endif()
