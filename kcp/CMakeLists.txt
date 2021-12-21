project(kcp_src)

include(GNUInstallDirs)

add_library(kcp SHARED src/solver.cpp src/keypoint.cpp src/utility.cpp)
target_include_directories(kcp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(kcp Eigen3::Eigen nanoflann::nanoflann ${TEASER_LIBRARIES})
add_library(KCP::kcp ALIAS kcp)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
)
install(TARGETS kcp
  EXPORT KCPConfig
  LIBRARY DESTINATION lib
)

export(TARGETS kcp
  NAMESPACE KCP::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/KCPConfig.cmake"
)
install(EXPORT KCPConfig
  DESTINATION "${CMAKE_INSTALL_DATADIR}/KCP/cmake"
  NAMESPACE KCP::
)