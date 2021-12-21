include(FetchContent)

FetchContent_Declare(
  doxygentheme
  GIT_REPOSITORY https://github.com/jothepro/doxygen-awesome-css
  GIT_TAG        v1.5.0
)

FetchContent_GetProperties(doxygentheme)
if(NOT doxygentheme_POPULATED)
  FetchContent_Populate(doxygentheme)
endif()