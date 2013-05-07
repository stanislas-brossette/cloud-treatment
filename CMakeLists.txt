cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/eigen.cmake)
include(cmake/pkg-config.cmake)

set(PROJECT_NAME cloud-treatment)
set(PROJECT_DESCRIPTION cloud-treatment)
set(PROJECT_URL https://github.com/stanislas-brossette/cloud-treatment)

set(CUSTOM_HEADER_DIR cloud-treatment)
set(HEADERS )

setup_project()

# Add main library to pkg-config file.
PKG_CONFIG_APPEND_LIBS(cloud-treatment)

ADD_REQUIRED_DEPENDENCY("yaml-cpp")

# Look for PCL.
find_package(PCL 1.7 REQUIRED 
	COMPONENTS apps common io surface filters segmentation visualization)
search_for_boost()
search_for_eigen()

# Use PCL
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Use Boost
include_directories(${BOOST_INCLUDEDIR})

add_subdirectory(src)
add_subdirectory(share)
add_subdirectory(tests)
add_subdirectory(bin)

setup_project_finalize()
