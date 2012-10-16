include_directories (${OpenCV_INCLUDE_DIRS})

if (LIBDC1394_FOUND)
  include_directories (${LIBDC1394_INCLUDE_DIRS})
  link_directories (${LIBDC1394_LIBRARY_DIRS})

  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${LIBDC1394_CFLAGS}")
  set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} ${LIBDC1394_LDFLAGS}")

  # calibtest
  include_directories (${OpenVGR_SOURCE_DIR}/src/lib/libcapture ${OpenVGR_SOURCE_DIR}/src/lib/libopenvgr)
  include_directories (${GLUT_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS})
  add_executable (calibtest calibtest.cpp glwindow.cpp cv3d.cpp globj.cpp capture.cpp)
  target_link_libraries (calibtest capture openvgr_misc ${OpenCV_LIBS} ${LIBDC1394_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

  install (TARGETS calibtest RUNTIME DESTINATION bin)
endif (LIBDC1394_FOUND)
