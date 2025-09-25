cmake_minimum_required(VERSION 3.8)
project(horiokart_obstacle_detector_3d)

# Prevent ament_uncrustify from running by clearing its executable path early.
# This is a defensive measure to ensure the older uncrustify-based test is not
# executed when ament_lint_auto auto-adds it.
set(ENV{AMENT_UNCRUSTIFY_EXECUTABLE} "")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Prevent ament_lint_auto from auto-adding the 'uncrustify' test.
# Set these before ament_lint_auto is found so the exclusion is respected.
# This is defensive: the environment variable was cleared earlier, but set the
# CMake variables too so ament_lint_auto doesn't create the uncrustify target.
set(AMENT_LINT_AUTO_EXCLUDE
  "build;install;ament_cmake_uncrustify;uncrustify;clang_format;"
  "ament_clang_format;ament_cmake_clang_format")
# Also mark ament_uncrustify as not found to be defensive in case the find_package
# logic looks for the wrapper; keeping both approaches increases the chance the
# uncrustify test won't be auto-added.
set(ament_uncrustify_FOUND FALSE)

add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
target_include_directories(obstacle_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(obstacle_detector_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  obstacle_detector_node
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "pcl_conversions"
  "pcl_msgs"
  "tf2_ros"
  "tf2_sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "diagnostic_msgs"
  "visualization_msgs"
)

# Core library (headless algorithms)
add_library(horiokart_obstacle_detector_3d_core
  src/core/grid_height_map.cpp
  src/core/cluster_detector.cpp
  src/core/ground_separator.cpp
  src/core/scan_projector.cpp
  src/core/pca_slope_estimator.cpp
  src/core/intensity_utils.cpp
)
target_include_directories(horiokart_obstacle_detector_3d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(horiokart_obstacle_detector_3d_core PUBLIC cxx_std_17)
target_include_directories(horiokart_obstacle_detector_3d_core PUBLIC ${EIGEN3_INCLUDE_DIR})

ament_target_dependencies(
  horiokart_obstacle_detector_3d_core
  "rclcpp"
)

if(PCL_FOUND)
  target_include_directories(horiokart_obstacle_detector_3d_core PUBLIC ${PCL_INCLUDE_DIRS})
  target_link_libraries(horiokart_obstacle_detector_3d_core ${PCL_LIBRARIES})
  add_definitions(${PCL_DEFINITIONS})
  # ensure node target also sees PCL includes and links
  target_include_directories(obstacle_detector_node PUBLIC ${PCL_INCLUDE_DIRS})
  target_link_libraries(obstacle_detector_node ${PCL_LIBRARIES})
endif()

if(Eigen3_FOUND)
  target_include_directories(obstacle_detector_node PUBLIC ${EIGEN3_INCLUDE_DIR})
  target_include_directories(horiokart_obstacle_detector_3d_core PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

target_link_libraries(obstacle_detector_node horiokart_obstacle_detector_3d_core)

install(TARGETS obstacle_detector_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # Prevent ament_lint_auto from auto-adding the 'uncrustify' test: mark it as not found.
  set(ament_uncrustify_FOUND FALSE)
  # prefer clang-format for C/C++ formatting checks; keep find_package QUIET so
  # configuration does not fail in environments where the CMake package config
  # for ament_clang_format is not installed (the ROS wrapper executable may
  # still exist). This keeps builds working while allowing CI to enable the
  # stricter REQUIRED check when the environment provides the package.
  # The package installed via apt is named ament_cmake_clang_format which
  # provides the ament_clang_format macro. Find that package instead.
  find_package(ament_cmake_clang_format REQUIRED)
  # Exclude generated build/install directories from lint checks to avoid
  # reporting style issues on CMake-generated files under the workspace build dir.
  # This prevents lint_cmake from scanning the build/ and install/ folders.
  # Exclude build/install, uncrustify and clang-format related auto-added tests
  # from automatic execution by ament_lint_auto. This prevents duplicate
  # 'clang_format' and 'uncrustify' test/targets from being created.
  set(AMENT_LINT_AUTO_EXCLUDE
    "build;install;ament_cmake_uncrustify;uncrustify;clang_format;"
    "ament_clang_format;ament_cmake_clang_format" CACHE STRING "Paths to exclude from ament_lint_auto")
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # ament_lint_auto may add a 'clang_format' test; remove it so the
  # clang-format package hook can register the test (or vice-versa) without
  # causing a duplicate add_test NAME error.
  if(COMMAND remove_test)
    remove_test(clang_format)
    remove_test(clang-format)
  endif()
  if(TARGET clang_format)
    message(STATUS "Removing auto-generated 'clang_format' target (clang-format managed separately)")
    set_target_properties(clang_format PROPERTIES EXCLUDE_FROM_ALL TRUE)
  endif()
  # ament_lint_auto may add an 'uncrustify' test; we prefer clang-format and remove it here
  if(COMMAND remove_test)
    # remove the CTest entry named 'uncrustify' if present
    remove_test(uncrustify)
  endif()
  # If an 'uncrustify' custom target was created, remove it to avoid running the formatter
  if(TARGET uncrustify)
    message(STATUS "Removing auto-generated 'uncrustify' target (clang-format in use)")
    # CMake does not provide a direct remove target command; try to mark it as EXCLUDE_FROM_ALL
    set_target_properties(uncrustify PROPERTIES EXCLUDE_FROM_ALL TRUE)
  endif()
endif()

if(BUILD_TESTING)
  # Run clang-format checks on the C/C++ sources if the ament_clang_format
  # CMake command is available. We used find_package(... QUIET) above, so the
  # package may not be present in some environments; guard the macro call to
  # avoid an unknown-command error during configure.
  if(COMMAND ament_clang_format)
    ament_clang_format()
  else()
    message(WARNING "ament_clang_format not found; skipping clang-format checks")
  endif()
endif()

if(BUILD_TESTING)
  ament_add_gtest(test_grid_height_map test/test_grid_height_map.cpp)
  if(TARGET test_grid_height_map)
    target_include_directories(test_grid_height_map PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(test_grid_height_map horiokart_obstacle_detector_3d_core)
  endif()
  ament_add_gtest(test_ground_separator test/test_ground_separator.cpp)
  if(TARGET test_ground_separator)
    target_include_directories(test_ground_separator PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(test_ground_separator horiokart_obstacle_detector_3d_core)
  endif()
  ament_add_gtest(test_cluster_detector test/test_cluster_detector.cpp)
  if(TARGET test_cluster_detector)
    target_include_directories(test_cluster_detector PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(test_cluster_detector horiokart_obstacle_detector_3d_core)
  endif()
  ament_add_gtest(test_cluster_detector_expand test/test_cluster_detector_expand.cpp)
  if(TARGET test_cluster_detector_expand)
    target_include_directories(test_cluster_detector_expand PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(test_cluster_detector_expand horiokart_obstacle_detector_3d_core)
  endif()
  ament_add_gtest(test_scan_projector test/test_scan_projector.cpp)
  if(TARGET test_scan_projector)
    target_include_directories(test_scan_projector PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(test_scan_projector horiokart_obstacle_detector_3d_core)
  endif()
  # Add gtests for PCA and intensity utils
  ament_add_gtest(test_pca_slope_estimator test/test_pca_slope_estimator.cpp)
  if(TARGET test_pca_slope_estimator)
    target_include_directories(test_pca_slope_estimator PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(test_pca_slope_estimator horiokart_obstacle_detector_3d_core)
    if(PCL_FOUND)
      target_include_directories(test_pca_slope_estimator PUBLIC ${PCL_INCLUDE_DIRS})
      target_link_libraries(test_pca_slope_estimator ${PCL_LIBRARIES})
    endif()
    if(Eigen3_FOUND)
      target_include_directories(test_pca_slope_estimator PUBLIC ${EIGEN3_INCLUDE_DIR})
    endif()
  endif()
  ament_add_gtest(test_intensity_utils test/test_intensity_utils.cpp)
  if(TARGET test_intensity_utils)
    target_include_directories(test_intensity_utils PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(test_intensity_utils horiokart_obstacle_detector_3d_core)
    if(Eigen3_FOUND)
      target_include_directories(test_intensity_utils PUBLIC ${EIGEN3_INCLUDE_DIR})
    endif()
  endif()
  ament_add_gtest(test_ground_score_extra test/test_ground_score_extra.cpp)
  if(TARGET test_ground_score_extra)
    target_include_directories(test_ground_score_extra PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(test_ground_score_extra horiokart_obstacle_detector_3d_core)
  endif()
  ament_add_gtest(test_grid_height_map_hole test/test_grid_height_map_hole.cpp)
  if(TARGET test_grid_height_map_hole)
    target_include_directories(test_grid_height_map_hole PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(test_grid_height_map_hole horiokart_obstacle_detector_3d_core)
  endif()
  ament_add_gtest(test_vegetation_logic test/test_vegetation_logic.cpp)
  if(TARGET test_vegetation_logic)
    target_include_directories(test_vegetation_logic PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  endif()
  ament_add_gtest(test_publish_empty_scan test/test_publish_empty_scan_unit.cpp)
  if(TARGET test_publish_empty_scan)
    target_include_directories(test_publish_empty_scan PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(test_publish_empty_scan horiokart_obstacle_detector_3d_core)
    ament_target_dependencies(test_publish_empty_scan "sensor_msgs")
  endif()
endif()

if(BUILD_TESTING)
  # Ensure any auto-added uncrustify test/target is removed before running CTest.
  if(COMMAND remove_test)
    remove_test(uncrustify)
  endif()
  if(TARGET uncrustify)
    set_target_properties(uncrustify PROPERTIES EXCLUDE_FROM_ALL TRUE)
  endif()
endif()

ament_package()
