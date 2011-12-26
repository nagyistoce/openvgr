set (IDL_DIR "${OpenVGR_SOURCE_DIR}/src/idl")

set (RTC_NAME "MultiCameraTrigger")

set (IDL_FILES
  ${IDL_DIR}/Img.idl
)

set (SKEL_SRCS
)

set (STUB_SRCS
  ${CMAKE_CURRENT_BINARY_DIR}/ImgStub.cpp)

set (IMPL_SRCS
)

set (IMPL_INCS
)

set (LIBS_SRCS MultiCameraTrigger.cpp)
set (LIBS_INCS MultiCameraTrigger.h)
set (EXEC_SRCS MultiCameraTriggerComp.cpp)
set (OTHER_SRCS
  CMakeLists.txt Doxyfile.in
  RTC.xml MultiCameraTrigger.conf rtc.conf)

add_definitions (-I${CMAKE_CURRENT_BINARY_DIR})

genheader_from_idl (${IDL_FILES})

foreach (_lib_src ${LIBS_SRCS})
  foreach (_dep ${SKEL_SRCS} ${STUB_SRCS})
    set_source_files_properties (${_lib_src} PROPERTIES OBJECT_DEPENDS ${_dep})
  endforeach (_dep ${SKEL_SRCS} ${STUB_SRCS})
endforeach (_lib_src ${LIBS_SRCS})

add_library (${RTC_NAME} SHARED
  ${LIBS_SRCS} ${IMPL_SRCS} ${SKEL_SRCS} ${STUB_SRCS})
target_link_libraries (${RTC_NAME} ${LIBRARIES})
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
