# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sensors_viewer_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sensors_viewer_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sensors_viewer_FOUND FALSE)
  elseif(NOT sensors_viewer_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sensors_viewer_FOUND FALSE)
  endif()
  return()
endif()
set(_sensors_viewer_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sensors_viewer_FIND_QUIETLY)
  message(STATUS "Found sensors_viewer: 0.0.0 (${sensors_viewer_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sensors_viewer' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${sensors_viewer_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sensors_viewer_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sensors_viewer_DIR}/${_extra}")
endforeach()
