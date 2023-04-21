# Generate the build folder
set(VULKAN_BUILD_DIR ${VULKAN_DIR}/build)
file(MAKE_DIRECTORY ${VULKAN_BUILD_DIR})

# cmake ..
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" ..
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${VULKAN_BUILD_DIR}
)
if(result)
  message(FATAL_ERROR "CMake step for Vulkan Glasses failed: ${result}")
endif()

# make
execute_process(COMMAND ${CMAKE_COMMAND} --build . -- -j8
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${VULKAN_BUILD_DIR}
)
if(result)
  message(FATAL_ERROR "Build step for Vulkan Glasses failed: ${result}")
endif()

message(STATUS "Vulkan Glasses built")
