set (IDL_DIR "${OpenVGR_SOURCE_DIR}/src/idl")

set (OBJS
  quaternion.c)

set (COMOBJS
  calib.cpp calibUtil.cpp recogParameter.cpp
  recogImage.cpp imageUtil.cpp
  extractEdge.cpp extractFeature_old.cpp
  vertex.cpp circle.cpp conic.cpp stereo.cpp
  pairedcircle.cpp match3Dfeature.cpp
  modelpoints.cpp score2d.cpp
  modelFileio.cpp modelListFileIO.cpp
  rtvcm.cpp debugutil.cpp vectorutil.cpp quaternion.c
  geometry.cpp extractFeature.cpp correspondence.cpp
  drawing.cpp mathmisc.cpp
  searchEllipse_IW.cpp RecognitionKernel.cpp
  merge_ellipse.cpp)

include_directories (${OpenCV_INCLUDE_DIRS})

add_library (openvgr_misc ${OBJS})

if (OpenRTM_FOUND)
  include_directories (${CMAKE_CURRENT_BINARY_DIR} ${OPENRTM_INCLUDE_DIRS})
  link_directories (${OPENRTM_LIBRARY_DIRS})

  genheader_from_idl(${IDL_DIR}/Img.idl)
  set_source_files_properties (
    calibUtil.cpp imageUtil.cpp
    PROPERTIES OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/Img.hh")

  add_library (openvgr ${COMOBJS})
endif (OpenRTM_FOUND)
