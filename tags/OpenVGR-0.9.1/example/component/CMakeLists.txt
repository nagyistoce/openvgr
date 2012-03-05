list (APPEND CFLAGS ${ONIORB_CFLAGS} ${OPENRTM_CFLAGS})
list (APPEND INCLUDE_DIRS ${OMNIORB_INCLUDE_DIRS} ${OPENRTM_INCLUDE_DIRS})
list (APPEND LDFLAGS ${OMNIORB_LDFLAGS} ${OPENRTM_LDFLAGS})
list (APPEND LIBRARY_DIRS ${OMNIORB_LIBRARY_DIRS} ${OPENRTM_LIBRARY_DIRS})
list (APPEND LIBRARIES ${OMNIORB_LIBRARIES} ${OPENRTM_LIBRARIES})

include_directories (${OpenVGR_SOURCE_DIR}/src/lib/libopenvgr ${INCLUDE_DIRS})
link_directories (${OpenVGR_SOURCE_DIR}/src/lib/libopenvgr ${LIBRARY_DIRS})

add_subdirectory(MultiCameraTrigger)
add_subdirectory(SendImage)
add_subdirectory(SetModelID)
