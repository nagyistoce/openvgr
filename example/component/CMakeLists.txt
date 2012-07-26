list (APPEND INCLUDE_DIRS ${OMNIORB_INCLUDE_DIRS} ${OPENRTM_INCLUDE_DIRS})
list (APPEND LIBRARY_DIRS ${OMNIORB_LIBRARY_DIRS} ${OPENRTM_LIBRARY_DIRS})
list (APPEND LIBRARIES ${OMNIORB_LIBRARIES} ${OPENRTM_LIBRARIES})

string (REPLACE ";" " " CFLAGS_ADD "${OMNIORB_CFLAGS} ${OPENRTM_CFLAGS}")
set (CFLAGS "${CFLAGS} ${CFLAGS_ADD}")

string (REPLACE ";" " " LDFLAGS_ADD "${OMNIORB_LDFLAGS} ${OPENRTM_LDFLAGS}")
set (LDFLAGS "${LDFLAGS} ${LDFLAGS_ADD}")

include_directories (${OpenVGR_SOURCE_DIR}/src/lib/libopenvgr ${INCLUDE_DIRS})
link_directories (${OpenVGR_SOURCE_DIR}/src/lib/libopenvgr ${LIBRARY_DIRS})

add_subdirectory(MultiCameraTrigger)
add_subdirectory(SendImage)
add_subdirectory(SetModelID)
