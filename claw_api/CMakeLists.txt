cmake_minimum_required(VERSION 3.8)
project(claw_api)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

add_library(claw_api INTERFACE)



target_include_directories(claw_api INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/claw_api>
  $<INSTALL_INTERFACE:include/claw_api>
)

ament_export_targets(claw_api HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/claw_api/
  DESTINATION include/claw_api
)

install(
  TARGETS claw_api
  EXPORT claw_api
  DESTINATION lib
)


if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(claw_api_test test/claw_api_test.cpp)

  target_link_libraries(claw_api_test claw_api)
  
  target_link_libraries(claw_api_test  serial)
  

  # Specify include directories for the test
  target_include_directories(claw_api_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${libserial_INCLUDE_DIRS}
  )

endif()

ament_package()
