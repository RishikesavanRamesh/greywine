cmake_minimum_required(VERSION 3.8)
project(claw_api)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable compiler warnings for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Create the library (header-only)
add_library(claw_api INTERFACE)

# Specify include directories for the interface library
target_include_directories(claw_api INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/claw_api>
  $<INSTALL_INTERFACE:include/claw_api>
)

# Export the library
ament_export_targets(claw_api HAS_LIBRARY_TARGET)

# Install header files
install(
  DIRECTORY include/claw_api/
  DESTINATION include/claw_api
)

# Install the library target (no binary for header-only)
install(
  TARGETS claw_api
  EXPORT claw_api
  DESTINATION lib
)

# Testing setup
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Add the test executable
  ament_add_gtest(claw_api_test test/claw_api_test.cpp)

  # Link the claw_api library with the test executable
  target_link_libraries(claw_api_test claw_api)

  # Specify include directories for the test
  target_include_directories(claw_api_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

ament_package()
