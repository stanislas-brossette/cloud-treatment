cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include(cmake/base.cmake)

set(PROJECT_NAME cloud-treatment)
set(PROJECT_DESCRIPTION cloud-treatment)
set(PROJECT_URL https://github.com/stanislas-brossette/cloud-treatment)

set(HEADERS )

setup_project()

# Look for PCL.
find_package(PCL 1.3 REQUIRED)

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
    src/hullconvexcell.cpp
    src/typedefs.h
    src/custompclvisualizor.cpp
)
target_link_libraries(cloud-treatment ${PCL_LIBRARIES})

setup_project_finalize()

