project(kcp_python_wrapper)

pybind11_add_module(pykcp kcp/wrapper.cpp)

target_link_libraries(pykcp PUBLIC KCP::kcp)

# https://github.com/pybind/pybind11/issues/1818
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(pykcp PUBLIC -fsized-deallocation)
endif()