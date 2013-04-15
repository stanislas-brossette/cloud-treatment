cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include(cmake/base.cmake)
include(cmake/eigen.cmake)

set(PROJECT_NAME cloud-treatment)
set(PROJECT_DESCRIPTION cloud-treatment)
set(PROJECT_URL https://github.com/stanislas-brossette/cloud-treatment)

set(HEADERS )

setup_project()

# Look for PCL.
find_package(PCL 1.3 REQUIRED 
	COMPONENTS common io surface filters segmentation visualization)
search_for_eigen()

include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cloud-treatment 
    src/main.cpp
    src/application.cpp
    src/plancloud.cpp
    src/cell.cpp
    src/filecell.cpp
    src/filtercell.cpp
    src/xyzswitchcell.cpp
    src/planextractioncell.cpp
    src/planprojectioncell.cpp
    src/filewritingcell.cpp
    src/orientationcell.cpp
    src/hullconvexcell.cpp
    src/typedefs.h
    src/custompclvisualizor.cpp
    src/visualizer.cpp
    src/regiongrowingsegmentationcell.cpp
    src/verbose.h
)
target_link_libraries(cloud-treatment ${PCL_LIBRARIES})

setup_project_finalize()

