SOURCES += \
    src/main.cpp \
    src/application.cpp \
    src/cell.cpp \
    src/custompclvisualizor.cpp \
    src/filecell.cpp \
    src/filewritingcell.cpp \
    src/filtercell.cpp \
    src/hullconvexcell.cpp \
    src/orientationcell.cpp \
    src/passthroughfiltercell.cpp \
    src/plancloud.cpp \
    src/planextractioncell.cpp \
    src/planprojectioncell.cpp \
    src/regiongrowingsegmentationcell.cpp \
    src/visualizer.cpp \
    src/xyzswitchcell.cpp \
    src/displayxyzcloudcell.cpp \
    src/displayconvexcloudcell.cpp \
    src/dominantplanesegmentationcell.cpp \
    src/euclidianclustersextractioncell.cpp

HEADERS += \
    src/application.h \
    src/cell.h \
    src/custompclvisualizor.h \
    src/filecell.h \
    src/filewritingcell.h \
    src/filtercell.h \
    src/hullconvexcell.h \
    src/orientationcell.h \
    src/passthroughfiltercell.h \
    src/plancloud.h \
    src/planextractioncell.h \
    src/planprojectioncell.h \
    src/regiongrowingsegmentationcell.h \
    src/typedefs.h \
    src/verbose.h \
    src/visualizer.h \
    src/xyzswitchcell.h \
    src/factory.h \
    src/displayxyzcloudcell.h \
    src/displayconvexcloudcell.h \
    src/dominantplanesegmentationcell.h \
    src/euclidianclustersextractioncell.h

OTHER_FILES += \
    CMakeLists.txt \
    share/yaml/modelcalibration.yaml \
    share/yaml/cloudtreatment.yaml \
    src/dirs.hh.in \
    src/CMakeLists.txt \
    share/cloud-treatment/pipeline/cloudtreatment.yaml \
    share/cloud-treatment/pipeline/modelcalibration.yaml \
    share/cloud-treatment/pipeline/pipelinetest.yaml \
    share/cloud-treatment/pipeline/calibrationtest.yaml
