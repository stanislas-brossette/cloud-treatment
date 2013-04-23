cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/eigen.cmake)
include(cmake/pkg-config.cmake)

set(PROJECT_NAME cloud-treatment)
set(PROJECT_DESCRIPTION cloud-treatment)
set(PROJECT_URL https://github.com/stanislas-brossette/cloud-treatment)

set(HEADERS )

setup_project()
ADD_REQUIRED_DEPENDENCY("yaml-cpp")
# Look for PCL.
find_package(PCL 1.7 REQUIRED 
	COMPONENTS common io surface filters segmentation visualization)
search_for_boost()
search_for_eigen()
#add_required_dependency(eigen)

include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(cloud_treatment_lib SHARED
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
    src/passthroughfiltercell.cpp
    src/verbose.h
)

target_link_libraries(cloud_treatment_lib ${PCL_LIBRARIES})
PKG_CONFIG_USE_DEPENDENCY(cloud_treatment_lib yaml_cpp)

add_executable(model_calibration src/modelcalibration.cpp)
add_dependencies(model_calibration  cloud_treatment_lib)
target_link_libraries(model_calibration cloud_treatment_lib)

add_executable(cloud_treatment src/main.cpp)
add_dependencies(cloud_treatment  cloud_treatment_lib)
target_link_libraries(cloud_treatment cloud_treatment_lib
${Boost_LIBRARIES})

setup_project_finalize()

