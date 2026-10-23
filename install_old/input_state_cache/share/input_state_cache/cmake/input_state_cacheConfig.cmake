# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_input_state_cache_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED input_state_cache_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(input_state_cache_FOUND FALSE)
  elseif(NOT input_state_cache_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(input_state_cache_FOUND FALSE)
  endif()
  return()
endif()
set(_input_state_cache_CONFIG_INCLUDED TRUE)

# output package information
if(NOT input_state_cache_FIND_QUIETLY)
  message(STATUS "Found input_state_cache: 0.1.0 (${input_state_cache_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'input_state_cache' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT input_state_cache_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(input_state_cache_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${input_state_cache_DIR}/${_extra}")
endforeach()
