# Install pipeline files.
 install(FILES
  cloud-treatment/pipeline/cadmodeltreatment.yaml
  cloud-treatment/pipeline/cloudtreatment.yaml
  cloud-treatment/pipeline/cloudviewer.yaml
  cloud-treatment/pipeline/crapahut.yaml
  cloud-treatment/pipeline/modelcalibration.yaml
  cloud-treatment/pipeline/table-climbing.yaml
  DESTINATION
  ${CMAKE_INSTALL_FULL_DATADIR}/cloud-treatment/pipeline)

# Install point cloud example files.
 install(FILES
  cloud-treatment/point-cloud/table-climbing.pcd
  cloud-treatment/point-cloud/crapahut.pcd
  cloud-treatment/point-cloud/empty-cloud.pcd
  #FIXME: to be continued!
  DESTINATION
  ${CMAKE_INSTALL_FULL_DATADIR}/cloud-treatment/point-cloud)

# Install CAD models example files.
 install(FILES
  cloud-treatment/cad-model/bunny.ply
  cloud-treatment/cad-model/chair.ply
  #FIXME: to be continued!
  DESTINATION
  ${CMAKE_INSTALL_FULL_DATADIR}/cloud-treatment/cad-model)

# Install man page.
FIND_PROGRAM(GZIP gzip)
MAKE_DIRECTORY("${CMAKE_CURRENT_BINARY_DIR}/man/man1/")
EXECUTE_PROCESS(
  COMMAND "${GZIP}" "-c"
  "${CMAKE_CURRENT_SOURCE_DIR}/man/man1/cloud-processing.1"
  OUTPUT_FILE
  ${CMAKE_CURRENT_BINARY_DIR}/man/man1/cloud-processing.1.gz)

INSTALL(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/man/man1/cloud-processing.1.gz"
  DESTINATION ${CMAKE_INSTALL_MANDIR}/man1)
