
################################################################################
# Options
################################################################################
set(WITH_MATPLOTLIB FALSE)

set(WITH_MATPLOTLIB FALSE CACHE BOOL "Enable MATPLOTLIB for visualization")

################################################################################
# Tests
################################################################################

set(TEST_SOURCE_FILES
  main.cpp

  # barrier/test_barriers.cpp

  # Test CCD
  # ccd/collision_generator.cpp
  # ccd/test_edge_vertex_ccd.cpp
  # ccd/test_time_of_impact.cpp
  # ccd/test_additive_ccd.cpp
  # ccd/test_hash_grid.cpp

  # interval/test_interval.cpp
  # interval/test_interval_root_finder.cpp
  # ccd/test_rigid_body_time_of_impact.cpp
  # ccd/test_rigid_body_hash_grid.cpp

  # solvers/test_newton_solver.cpp
  # solvers/test_barrier_newton_solver.cpp
  # solvers/test_barrier_displacements_opt.cpp

  # opt/test_distance_barrier_constraint.cpp

  # physics/test_mass.cpp
  # physics/test_pose.cpp
  physics/test_rigid_body.cpp
  # physics/test_rigid_body_system.cpp
  # physics/test_orthogonality_constraint.cpp
  # physics/test_rigid_body_problem.cpp

  problems/test_linear_constraints.cpp

  # io/test_serialize_json.cpp
  # io/test_read_rb_scene.cpp

  # geometry/test_distance.cpp
  # geometry/test_intersection.cpp

  # utils/test_sinc.cpp
)

if(WITH_MATPLOTLIB)
    list(APPEND TEST_SOURCE_FILES validation/test_validation.cpp)
endif()

add_executable(abd_tests ${TEST_SOURCE_FILES})
set_property(TARGET abd_tests PROPERTY CUDA_RESOLVE_DEVICE_SYMBOLS ON)
################################################################################
# Required Libraries
################################################################################

target_link_libraries(abd_tests PUBLIC abd::abd)

include(abd_warnings)
target_link_libraries(abd_tests PRIVATE abd::abd::warnings)

include(catch2)
target_link_libraries(abd_tests PUBLIC Catch2::Catch2)

include(finite_diff)
target_link_libraries(abd_tests PUBLIC finitediff::finitediff)

if(WITH_MATPLOTLIB)
    # find python libraries
    # FIND_ABI setting debug to OFF (means that we will link with the release version of the library even when building C++ target in debug)
    # this is helpful because most machines won't have python debug binaries
    set(Python_FIND_ABI "OFF;ANY;ANY")
    find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
    find_package(PythonLibs 3.0 REQUIRED)
    message("Python Executable Path: ${Python3_EXECUTABLE}")
    message("NumPy Version: ${Python3_NumPy_VERSION}")

    include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

    # populate matplotlib repository
    include(FetchContent)
    FetchContent_Declare(
        matplotlib
        GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
        GIT_TAG        f23347fca25219d1c42cbb91608b5556814bf572
    )
    FetchContent_GetProperties(matplotlib)
    if(NOT matplotlib_POPULATED)
        FetchContent_Populate(matplotlib)
    endif()
    include_directories(SYSTEM ${matplotlib_SOURCE_DIR})

    # link python and numpy
    target_link_libraries(abd_tests
        PRIVATE
            ${PYTHON_LIBRARIES}
            Python3::NumPy

    )
endif()

################################################################################
# Compiler options
################################################################################

target_compile_definitions(abd_tests PUBLIC CATCH_CONFIG_ENABLE_BENCHMARKING)

################################################################################
# Register tests
################################################################################

# Register tests
set(PARSE_CATCH_TESTS_ADD_TO_CONFIGURE_DEPENDS ON)
catch_discover_tests(abd_tests)

################################################################################

add_executable(test_rounding rounding_test.cpp)
target_link_libraries(test_rounding PUBLIC Boost::boost)
target_compile_options(test_rounding PUBLIC -msse2)
