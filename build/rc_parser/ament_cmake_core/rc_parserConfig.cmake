# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rc_parser_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rc_parser_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rc_parser_FOUND FALSE)
  elseif(NOT rc_parser_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rc_parser_FOUND FALSE)
  endif()
  return()
endif()
set(_rc_parser_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rc_parser_FIND_QUIETLY)
  message(STATUS "Found rc_parser: 0.1.0 (${rc_parser_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rc_parser' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT rc_parser_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rc_parser_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${rc_parser_DIR}/${_extra}")
endforeach()
