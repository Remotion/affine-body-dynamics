cmake_minimum_required(VERSION 3.8)

###############################################################################
# Download pybind11
function(abd_download_pybind11)
  abd_download_project(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.6.2
  )
endfunction()

if(NOT TARGET pybind11::module)
  abd_download_pybind11()
  add_subdirectory(
    ${ABD_EXTERNAL}/pybind11
    ${CMAKE_CURRENT_BINARY_DIR}/pybind11)
endif()

###############################################################################
# Create Python module for ABD

pybind11_add_module(abd src/bindings.cpp)
target_link_libraries(abd PRIVATE ${PROJECT_NAME})
# set_target_properties(abd PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
