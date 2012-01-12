cmake_minimum_required (VERSION 2.8)
project (OpenVGR)

set (OpenVGR_VERSION_MAJOR 0)
set (OpenVGR_VERSION_MINOR 9)
set (OpenVGR_VERSION_PATCH 0)
set (OpenVGR_VERSION "${OpenVGR_VERSION_MAJOR}.${OpenVGR_VERSION_MINOR}.${OpenVGR_VERSION_PATCH}")

message (STATUS "Package: OpenVGR-${OpenVGR_VERSION}")

# set a default build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release
    CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
  message (STATUS "set a default build type (${CMAKE_BUILD_TYPE})")
endif (NOT CMAKE_BUILD_TYPE)

# set default options for C/C++
set (CMAKE_C_FLAGS_RELEASE "-O3 -Wall -DNDEBUG -march=native"
  CACHE STRING "Flags used by the compiler during release builds" FORCE)
set (CMAKE_CXX_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE}
  CACHE STRING "Flags used by the compiler during release builds" FORCE)

# options
option (USE_OpenMP "enable OpenMP feature." OFF)

include (FindPkgConfig)

macro (show_var_info)
  foreach (_varname ${ARGN})
    message (STATUS "  ${_varname}=${${_varname}}")
  endforeach (_varname)
endmacro (show_var_info)

# OpenRTM
find_package (OpenRTM QUIET)
if (NOT OpenRTM_FOUND)
  message (STATUS "Fall back to rtm-config")
  find_program(RTM_CONFIG rtm-config)
  if (RTM_CONFIG)
    message (STATUS "rtm-config found.")
    mark_as_advanced (RTM_CONFIG)
    unset (OpenRTM_DIR CACHE)

    macro (_rtm_config_execute _varname _regex)
      set (_rtm_config_result)
      execute_process (
        COMMAND ${RTM_CONFIG} ${ARGN}
        RESULT_VARIABLE _rtm_config_failed
        OUTPUT_VARIABLE _rtm_config_result)

      if (_rtm_config_failed)
        set (${_varname})

      else (_rtm_config_failed)
        separate_arguments (_rtm_config_result UNIX_COMMAND ${_rtm_config_result})

        # filter by _regex
        if (NOT ${_regex} STREQUAL "")
          set (_rtm_config_result_temp)

          foreach (_item ${_rtm_config_result})
            string (REGEX MATCH ${_regex} _match_result ${_item})

            if (NOT ${_match_result} STREQUAL "")
              list (APPEND _rtm_config_result_temp ${_item})
            endif (NOT ${_match_result} STREQUAL "")
          endforeach (_item ${_rtm_config_result})

          set (_rtm_config_result ${_rtm_config_result_temp})
        endif (NOT ${_regex} STREQUAL "")

        set (${_varname} ${_rtm_config_result})
      endif (_rtm_config_failed)
    endmacro (_rtm_config_execute)

    _rtm_config_execute (OPENRTM_DIR "" "--prefix")
    _rtm_config_execute (OPENRTM_INCLUDE_DIRS "^-I.*" "--cflags")
    string (REGEX REPLACE "-I([^ ;-]*)" "\\1;" OPENRTM_INCLUDE_DIRS ${OPENRTM_INCLUDE_DIRS})

    _rtm_config_execute (OPENRTM_CFLAGS "^-[^I].*" "--cflags")
    set (OPENRTM_LDFLAGS "")
    _rtm_config_execute (OPENRTM_LIBRARIES "" "--libs")
    _rtm_config_execute (OPENRTM_LIBRARY_DIRS "" "--libdir")

    _rtm_config_execute (OPENRTM_ORB "" "--orb")
    _rtm_config_execute (OPENRTM_IDLC "" "--idlc")
    _rtm_config_execute (OPENRTM_IDLFLAGS "" "--idlflags")
    set (OPENRTM_IDLFLAGS ${OPENRTM_IDLFLAGS} "-I${OPENRTM_DIR}/include/rtm/idl")

    find_program (OPENRTM_IDL_WRAPPER NAMES rtm-skelwrapper)
    set (OPENRTM_IDL_WRAPPER_FLAGS "--include-dir=\"\";--skel-suffix=Skel;--stub-suffix=Stub")

    show_var_info (OPENRTM_INCLUDE_DIRS OPENRTM_CFLAGS OPENRTM_LDFLAGS OPENRTM_LIBRARIES OPENRTM_LIBRARY_DIRS)
    show_var_info (OPENRTM_ORB OPENRTM_IDLC OPENRTM_IDLFLAGS)
    show_var_info (OPENRTM_IDL_WRAPPER OPENRTM_IDL_WRAPPER_FLAGS)

    # define a macro to generate *.hh/*Skel.h/*Stub.h from *.idl
    macro (genheader_from_idl)
      foreach (_idl_file ${ARGN})
        get_filename_component (_filename ${_idl_file} NAME_WE)
        
        add_custom_command (
          OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_filename}.hh
          COMMAND ${OPENRTM_IDLC} -I${OpenVGR_SOURCE_DIR}/src/idl ${OPENRTM_IDLFLAGS} ${_idl_file}
          DEPENDS ${_idl_file})

        add_custom_command (
          OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_filename}Skel.h
                 ${CMAKE_CURRENT_BINARY_DIR}/${_filename}Skel.cpp
                 ${CMAKE_CURRENT_BINARY_DIR}/${_filename}Stub.h
                 ${CMAKE_CURRENT_BINARY_DIR}/${_filename}Stub.cpp
          COMMAND ${CMAKE_COMMAND} -E create_symlink ${_idl_file} ${CMAKE_CURRENT_BINARY_DIR}/${_filename}.idl
          COMMAND ${OPENRTM_IDLC} -I${OpenVGR_SOURCE_DIR}/src/idl ${OPENRTM_IDLFLAGS} ${_idl_file}
          COMMAND ${OPENRTM_IDL_WRAPPER} ${OPENRTM_IDL_WRAPPER_FLAGS} --idl-file=${_filename}.idl > /dev/null 2>&1
          DEPENDS ${_idl_file})
      endforeach (_idl_file ${ARGN})
    endmacro (genheader_from_idl)

    set (OpenRTM_FOUND 1)
  else (RTM_CONFIG)
    message (WARNING "Neither OpenRTMConfig.cmake nor rtm-config found. RT components will NOT be built.")
  endif (RTM_CONFIG)
endif (NOT OpenRTM_FOUND)

# OpenCV
find_package (OpenCV QUIET)
if (OpenCV_FOUND)
  find_path(OpenCV_INCLUDE_DIRS cv.h PATH_SUFFIXES opencv opencv2)
else (OpenCV_FOUND)
  message (STATUS "Fall back to pkg-config.")
  unset (OpenCV_DIR CACHE)
  pkg_check_modules (OpenCV opencv REQUIRED)
  
  set (OpenCV_LIBS ${OpenCV_LIBRARIES})
endif (OpenCV_FOUND)

# libdc1394
pkg_check_modules (LIBDC1394 libdc1394-2)

set (CMAKE_CXX_FLAGS "${CMAKE_CFLAGS}")

# OpenMP
if (USE_OpenMP)
  find_package (OpenMP)

  set (CMAKE_CFLAGS "${CMAKE_CFLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXXFLAGS "${CMAKE_CXXFLAGS} ${OpenMP_CXX_FLAGS}")
endif (USE_OpenMP)

#set (CMAKE_VERBOSE_MAKEFILE ON)

add_subdirectory (src)
add_subdirectory (example)