# generate headers.
config_files(dirs.hh)

# look for headers in build directory.
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# cloud treatment main library
add_library(cloud-treatment SHARED
    application.cpp
    cell.cpp
    displayconvexcloudcell.cpp
    displaykeypointcloudcell.cpp
    displaynormalcloudcell.cpp
    displayxyzcloudcell.cpp
    dominantplanesegmentationcell.cpp
    euclidianclustersextractioncell.cpp
    filecell.cpp
    filewritingcell.cpp
    filtercell.cpp
    hullconvexcell.cpp
    modelcalibrationcell.cpp
    normalestimationcell.cpp
    orientationcell.cpp
    passthroughfiltercell.cpp
    plancloud.cpp
    planextractioncell.cpp
    planprojectioncell.cpp
    regiongrowingsegmentationcell.cpp
    typedefs.h
    verbose.h
    visualizer.cpp
    xyzswitchcell.cpp
)

set_target_properties(cloud-treatment PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 0.1
)

target_link_libraries(cloud-treatment ${PCL_LIBRARIES})
PKG_CONFIG_USE_DEPENDENCY(cloud-treatment yaml-cpp)
install(TARGETS cloud-treatment
  DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})

# main executable
add_executable(cloud-processing main.cpp)
add_dependencies(cloud-processing cloud-treatment)
target_link_libraries(cloud-processing cloud-treatment
${Boost_LIBRARIES})
install(TARGETS cloud-processing
  DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
