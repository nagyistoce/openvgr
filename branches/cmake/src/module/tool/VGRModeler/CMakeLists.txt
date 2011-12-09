include_directories (${OpenVGR_SOURCE_DIR}/src/lib/libopenvgr)
link_directories (${OpenVGR_BINARY_DIR}/src/lib/libopenvgr)

add_executable (VGRModeler
  VGRModeler.cpp
  ModelInformation.cpp
  trans.cpp
  VRMLWriter.cpp)
target_link_libraries (VGRModeler openvgr_misc)
