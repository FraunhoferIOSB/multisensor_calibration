cmake_minimum_required(VERSION 3.14)
project(small_gicp_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

find_package(small_gicp QUIET)

# Define the URL and branch
set(GITREPO "https://github.com/koide3/small_gicp.git")
set(GITVERSION "master")

ament_vendor(small_gicp_vendor
  SATISFIED ${small_gicp_FOUND}
  VCS_URL ${GITREPO}
  VCS_VERSION ${GITVERSION}
  CMAKE_ARGS
    "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}"
)


ament_package()
