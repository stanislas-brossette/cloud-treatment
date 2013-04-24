# Install pipeline files.
 install(FILES
  cloud-treatment/pipeline/cloudtreatment.yaml
  cloud-treatment/pipeline/modelcalibration.yaml
  DESTINATION
  ${CMAKE_INSTALL_FULL_DATADIR}/cloud-treatment/pipeline)

# Install point cloud example files.
 install(FILES
  cloud-treatment/point-cloud/Abder1.pcd
  cloud-treatment/point-cloud/Abder2.pcd
  cloud-treatment/point-cloud/Abder3.pcd
  cloud-treatment/point-cloud/crapahut.pcd
  #FIXME: to be continued!
  DESTINATION
  ${CMAKE_INSTALL_FULL_DATADIR}/cloud-treatment/point-cloud)

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
