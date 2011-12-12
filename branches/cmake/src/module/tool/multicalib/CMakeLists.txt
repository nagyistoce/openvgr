include_directories (${OpenCV_INCLUDE_DIRS})

if (LIBDC1394_FOUND)
  include_directories (${LIBDC1394_INCLUDE_DIRS})
  link_directories (${LIBDC1394_LIBRARY_DIRS})

  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${LIBDC1394_CFLAGS}")
  set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} ${LIBDC1394_LDFLAGS}")

  # ichimatsu
  include_directories (${OpenVGR_SOURCE_DIR}/src/lib/libcapture)
  add_executable (ichimatsu ichimatsu.c checker_data.c detect_checker.c)
  target_link_libraries (ichimatsu capture ${OpenCV_LIBS} ${LIBDC1394_LIBRARIES})
  set_target_properties (ichimatsu PROPERTIES
    COMPILE_FLAGS "-I ${OpenVGR_SOURCE_DIR}/src/lib/libcapture")

  file (COPY chessboard_8x8_20mm.svg ieee1394board.0 test.xcdata DESTINATION .)

  # genconf
  add_executable (genconf genconf.c)
  target_link_libraries (genconf ${LIBDC1394_LIBRARIES})

  # reset_bus
  add_executable (reset_bus reset_bus.c)
  target_link_libraries (reset_bus ${LIBDC1394_LIBRARIES})
endif (LIBDC1394_FOUND)

# multicalib
add_executable (multicalib multicalib.c calib_proc.c calib_data.c camera_param.c quaternion.c optimizer.c)
target_link_libraries (multicalib ${OpenCV_LIBS})
