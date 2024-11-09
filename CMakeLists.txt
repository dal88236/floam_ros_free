cmake_minimum_required(VERSION 2.8)
project(floam_ros_free)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -O3")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED) 
find_package(Sophus REQUIRED)
find_package(yaml-cpp REQUIRED)

set(library_name floam_core)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(G2O_LIBS
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_CLI_LIBRARY}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_DENSE}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_SLAM2D_LINEAR}
  ${G2O_SOLVER_STRUCTURE_ONLY}
  ${G2O_SOLVER_EIGEN}
  ${G2O_TYPES_DATA}
  ${G2O_TYPES_ICP}
  ${G2O_TYPES_SBA}
  ${G2O_TYPES_SCLAM2D}
  ${G2O_TYPES_SIM3}
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_EXPMAP}
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${G2O_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(${library_name} SHARED
  src/config_parser.cpp
  src/g2o_type.cpp
  src/laser_mapper.cpp
  src/laser_processor.cpp
  src/lidar.cpp
  src/odom_estimator.cpp
  src/system.cpp
)

target_link_libraries(${library_name}
  ${EIGEN3_LIBS}
  ${G2O_LIBS} 
  ${Sophus_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
  ${YAML_CPP_LIBRARIES}
)

add_executable(vlp_exec 
  src/vlp_exec.cpp)
target_link_libraries(vlp_exec ${library_name} ${PCL_LIBRARIES})