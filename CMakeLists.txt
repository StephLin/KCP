cmake_minimum_required(VERSION 3.11)
project(KCP VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_VERSION VERSION_LESS "3.15")
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/ListPrepend.cmake")
  list_prepend(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
else()
  list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
endif()

# option(KCP_BUILD_TESTS "Build integration tests" OFF)
option(KCP_BUILD_PYTHON_BINDING "Build Python binding for KCP" OFF)
option(KCP_BUILD_DOC "Build documentation of KCP" OFF)

# Third-party libraries
# ---------------------

include(ExternalProject)
include(FetchContent)

# Eigen
find_package(Eigen3 REQUIRED QUIET)

# nanoflann
find_package(nanoflann REQUIRED QUIET)

# TEASER++
find_package(teaserpp REQUIRED QUIET)
set(TEASER_LIBRARIES teaserpp::teaser_registration)

# # GoogleTest
# if (KCP_BUILD_TESTS)
#   include(gtest)
# endif()

# pybind11
if (KCP_BUILD_PYTHON_BINDING)
  include(pybind11)
endif()

# doxygen
if (KCP_BUILD_DOC)
  include(doxygenTheme)
  find_package(Doxygen OPTIONAL_COMPONENTS dot)
endif()

# Building KCP
# ------------

add_subdirectory(kcp)

if (KCP_BUILD_PYTHON_BINDING)
  add_subdirectory(python)
endif()

# if (KCP_BUILD_TESTS)
#   enable_testing()
#   add_subdirectory(test)
# endif()

if (KCP_BUILD_DOC)
  if (DOXYGEN_FOUND)
    set(DOXYGEN_IN "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Doxyfile.in")
    set(DOXYGEN_OUT "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "${CMAKE_SOURCE_DIR}/README.md")

    add_custom_target(docs ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
  else()
    message("Doxygen need to be installed to generate the doxygen documentation")
  endif()
endif()
