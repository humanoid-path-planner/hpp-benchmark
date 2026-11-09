#
# Copyright (c) 2014 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of hpp_benchmark
# hpp_benchmark is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp_benchmark is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp_benchmark  If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 3.22)

SET(PROJECT_NAME hpp_benchmark)
SET(PROJECT_DESCRIPTION "Benchmarks for humanoid path planner platform.")
SET(INSTALL_DOCUMENTATION OFF)

# Check if the submodule cmake have been initialized
find_package(jrl-cmakemodules QUIET CONFIG)
if(jrl-cmakemodules_FOUND)
  get_property(
    JRL_CMAKE_MODULES
    TARGET jrl-cmakemodules::jrl-cmakemodules
    PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
else()
  message(STATUS "JRL cmakemodules not found. Let's fetch it.")
  include(FetchContent)
  FetchContent_Declare(
    "jrl-cmakemodules"
    GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
  FetchContent_MakeAvailable("jrl-cmakemodules")
  FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
endif()

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE("${JRL_CMAKE_MODULES}/hpp.cmake")
INCLUDE("${JRL_CMAKE_MODULES}/python.cmake")

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

FINDPYTHON()

ADD_PROJECT_DEPENDENCY(hpp-core)
ADD_PROJECT_DEPENDENCY(hpp-gepetto-viewer REQUIRED)
ADD_PROJECT_DEPENDENCY(hpp-manipulation-urdf REQUIRED)

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(FILES
  urdf/lydia.urdf
  urdf/obstacle.urdf
  urdf/obstacle_medium.urdf
  urdf/obstacle_easy.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES
  urdf/pr2-manipulation-kitchen/box.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/pr2-manipulation-kitchen
  )
install(FILES srdf/lydia.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )
install(FILES
  srdf/pr2-manipulation-kitchen/box.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf/pr2-manipulation-kitchen
  )
install(FILES
  future/baxter-manipulation-boxes/script.py
  future/baxter-manipulation-boxes/problem.toml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/baxter-manipulation-boxes)

install(FILES
  future/pyrene-on-the-ground/script.py
  future/pyrene-on-the-ground/problem.toml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pyrene-on-the-ground)

install(FILES
  future/pr2-in-iai-kitchen/script.py
  future/pr2-in-iai-kitchen/problem.toml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pr2-in-iai-kitchen)

install(FILES
  future/pr2-manipulation-kitchen/script.py
  future/pr2-manipulation-kitchen/problem.toml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pr2-manipulation-kitchen)

install(FILES
  future/pr2-manipulation-two-hand/script.py
  future/pr2-manipulation-two-hand/problem.toml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pr2-manipulation-two-hand)

install(FILES
  future/romeo-placard/script.py
  future/romeo-placard/problem.toml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/romeo-placard)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/images/cpp)
if(false)
add_custom_target(images
  ${PYTHON_EXECUTABLE} script/parse_and_generate_python_benchmark_plots.py ${CMAKE_BINARY_DIR}/images
  COMMAND ${PYTHON_EXECUTABLE} script/parse_and_generate_cpp_benchmark_plots.py ${CMAKE_BINARY_DIR}/images/cpp
  COMMENT "Generating benchmark images"
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
add_dependencies(doc images)
else()
set(ALL_GENERATED_IMAGES
  "${CMAKE_BINARY_DIR}/images/baxter-manipulation-boxes.svg"
  "${CMAKE_BINARY_DIR}/images/baxter-manipulation-boxes-easy.svg"
  "${CMAKE_BINARY_DIR}/images/baxter-two-arms-three-boxes.svg"
  "${CMAKE_BINARY_DIR}/images/baxter-two-arms-two-boxes.svg"
  "${CMAKE_BINARY_DIR}/images/construction-set.svg"
  "${CMAKE_BINARY_DIR}/images/pyrene-on-the-ground.svg"
  "${CMAKE_BINARY_DIR}/images/pr2-in-iai-kitchen.svg"
  "${CMAKE_BINARY_DIR}/images/pr2-manipulation-kitchen.svg"
  "${CMAKE_BINARY_DIR}/images/pr2-manipulation-two-hand.svg"
  "${CMAKE_BINARY_DIR}/images/romeo-placard.svg"
  "${CMAKE_BINARY_DIR}/images/ur3-spheres.svg"
  "${CMAKE_BINARY_DIR}/images/ur5-spline-optimization.svg"
  "${CMAKE_BINARY_DIR}/images/cpp/pr2_in_iai_maps.svg"
  "${CMAKE_BINARY_DIR}/images/cpp/pyrene_on_the_ground.svg"
  "${CMAKE_BINARY_DIR}/images/cpp/implicit_versus_explicit_Explicit left hand.svg"
  "${CMAKE_BINARY_DIR}/images/cpp/implicit_versus_explicit_Explicit right hand.svg"
  "${CMAKE_BINARY_DIR}/images/cpp/implicit_versus_explicit_Implicit both hands.svg"
  "${CMAKE_BINARY_DIR}/images/cpp/implicit_versus_explicit_Implicit right hand.svg"
  "${CMAKE_BINARY_DIR}/images/cpp/implicit_versus_explicit_Explicit both hands.svg"
  "${CMAKE_BINARY_DIR}/images/cpp/implicit_versus_explicit_Implicit left hand.svg"
  )
add_custom_command(OUTPUT ${ALL_GENERATED_IMAGES}
  COMMAND ${PYTHON_EXECUTABLE} script/parse_and_generate_python_benchmark_plots.py ${CMAKE_BINARY_DIR}/images
  COMMAND ${PYTHON_EXECUTABLE} script/parse_and_generate_cpp_benchmark_plots.py ${CMAKE_BINARY_DIR}/images/cpp
  COMMENT "Generating benchmark images"
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
add_custom_target(images DEPENDS ${ALL_GENERATED_IMAGES})
endif()
ADD_SUBDIRECTORY(python)
