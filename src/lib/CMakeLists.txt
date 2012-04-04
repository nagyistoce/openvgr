add_definitions (-fpic)

if (LIBDC1394_FOUND)
  add_subdirectory (libcapture)
endif (LIBDC1394_FOUND)
add_subdirectory (libopenvgr)
