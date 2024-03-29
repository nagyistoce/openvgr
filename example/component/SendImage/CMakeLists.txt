set (IDL_DIR "${OpenVGR_SOURCE_DIR}/src/idl")

set (RTC_NAME "SendImage")

set (IDL_FILES
  ${IDL_DIR}/Img.idl
  ${IDL_DIR}/Vision.idl
)

set (SKEL_SRCS
  ${CMAKE_CURRENT_BINARY_DIR}/VisionSkel.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/ImgSkel.cpp)

set (STUB_SRCS
)

set (IMPL_SRCS
  VisionSVC_impl.cpp
)

set (IMPL_INCS
  VisionSVC_impl.h
)

set (LIBS_SRCS SendImage.cpp)
set (LIBS_INCS SendImage.h)
set (EXEC_SRCS SendImageComp.cpp)
set (OTHER_SRCS
  CMakeLists.txt Doxyfile.in
  RTC.xml SendImage.conf rtc.conf)

add_definitions (-I${CMAKE_CURRENT_BINARY_DIR})
include_directories (${OpenCV_INCLUDE_DIRS})

genheader_from_idl (${IDL_FILES})

foreach (_lib_src ${LIBS_SRCS})
  foreach (_dep ${SKEL_SRCS} ${STUB_SRCS})
    set_source_files_properties (${_lib_src} PROPERTIES OBJECT_DEPENDS ${_dep})
  endforeach (_dep ${SKEL_SRCS} ${STUB_SRCS})
endforeach (_lib_src ${LIBS_SRCS})

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
target_link_libraries (${RTC_NAME}Comp ${RTC_NAME}-static)
target_link_libraries (${RTC_NAME}Comp ${OpenCV_LIBS})
