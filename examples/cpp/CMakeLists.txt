cmake_minimum_required(VERSION 3.10)
project(kcp_cpp_example VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)

option(FIND_KCP_FROM_SYSTEM "Find KCP from system" OFF)

if (FIND_KCP_FROM_SYSTEM)
  find_package(Eigen3 REQUIRED QUIET)
  find_package(nanoflann REQUIRED QUIET)
  find_package(teaserpp REQUIRED QUIET)
  find_package(KCP REQUIRED)
else()
  option(KCP_BUILD_TESTS "" OFF)
  option(KCP_BUILD_PYTHON_WRAPPER "" OFF)
  option(KCP_BUILD_DOC "" OFF)
  add_subdirectory(${CMAKE_SOURCE_DIR}/../.. ${CMAKE_BINARY_DIR}/kcp)
endif()

# PCL
find_package(PCL 1.8 REQUIRED COMPONENTS io)

include_directories(
  ${PCL_INCLUDE_DIRS}
)
add_executable(main main.cpp)
target_link_libraries(main PRIVATE KCP::kcp ${PCL_LIBRARIES})

add_custom_target(data
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/../data ${CMAKE_BINARY_DIR}/data
)
add_dependencies(main data)
