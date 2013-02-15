cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(CloudTreatment)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(CloudTreatment 
    src/main.cpp
    src/application.cpp
    src/plancloud.cpp
    src/cell.cpp
    src/filecell.cpp
    src/filtercell.cpp
    src/xyzswitchcell.cpp
)
target_link_libraries(CloudTreatment ${PCL_LIBRARIES})

