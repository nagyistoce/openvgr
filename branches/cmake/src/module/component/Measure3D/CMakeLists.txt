set (IDL_DIR "${OpenVGR_SOURCE_DIR}/src/idl")

set (RTC_NAME "Measure3D")

set (IDL_FILES
  Img.idl
  Vision.idl
)

set (SKEL_SRCS
  ${CMAKE_CURRENT_BINARY_DIR}/VisionSkel.cpp)

set (STUB_SRCS
  ${CMAKE_CURRENT_BINARY_DIR}/ImgStub.cpp)

set (IMPL_SRCS
  VisionSVC_impl.cpp
)

set (IMPL_INCS
  VisionSVC_impl.h
)

set (LIBS_SRCS Measure3D.cpp measure3d_scene.c read_measure3d_config.c)
set (LIBS_INCS Measure3D.h)
set (EXEC_SRCS Measure3DComp.cpp)
set (OTHER_SRCS
  CMakeLists.txt Doxyfile.in
  RTC.xml Measure3D.conf rtc.conf)

add_definitions (-I${CMAKE_CURRENT_BINARY_DIR})
include_directories (${OpenCV_INCLUDE_DIRS})

genheader_from_idl (${IDL_FILES})

foreach (_lib_src ${LIBS_SRCS})
  foreach (_dep ${SKEL_SRCS} ${STUB_SRCS})
    set_source_files_properties (${_lib_src} PROPERTIES OBJECT_DEPENDS ${_dep})
  endforeach (_dep ${SKEL_SRCS} ${STUB_SRCS})
endforeach (_lib_src ${LIBS_SRCS})

set_source_files_properties (measure3d_scene.c PROPERTIES LANGUAGE CXX)
set_source_files_properties (read_measure3d_config.c PROPERTIES LANGUAGE CXX)

add_library (${RTC_NAME} SHARED
  ${LIBS_SRCS} ${IMPL_SRCS} ${SKEL_SRCS} ${STUB_SRCS})
target_link_libraries (${RTC_NAME} ${LIBRARIES})
target_link_libraries (${RTC_NAME} ${OpenCV_LIBS})
set_target_properties (${RTC_NAME} PROPERTIES PREFIX "")
set_target_properties (${RTC_NAME} PROPERTIES LINK_FLAGS "${LDFLAGS}")

add_library (${RTC_NAME}-static STATIC
  ${LIBS_SRCS} ${IMPL_SRCS} ${SKEL_SRCS} ${STUB_SRCS})
target_link_libraries (${RTC_NAME}-static ${LIBRARIES})
set_target_properties (${RTC_NAME}-static PROPERTIES PREFIX "")
set_target_properties (${RTC_NAME}-static PROPERTIES LINK_FLAGS "${LDFLAGS}")


add_executable (${RTC_NAME}Comp ${EXEC_SRCS})
add_dependencies (${RTC_NAME}Comp ${RTC_NAME}-static)
target_link_libraries (${RTC_NAME}Comp ${RTC_NAME}-static ${OpenCV_LIBS})
target_link_libraries (${RTC_NAME}Comp ${OpenCV_LIBS})
target_link_libraries (${RTC_NAME}Comp ${LIBRARIES})
