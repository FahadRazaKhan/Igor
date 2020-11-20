cmake_minimum_required(VERSION 2.8)

project(persobalance2)
set(CMAKE_BUILD_TYPE Release)

find_package(PkgConfig)
pkg_search_module(EIGEN3 REQUIRED eigen3)

find_package(Boost REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_library(persobalance2 SHARED mlcp.cpp graph.cpp)
target_compile_features(persobalance2 PRIVATE cxx_lambdas)


install(TARGETS persobalance2
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
