# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_otv_bf7_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED otv_bf7_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(otv_bf7_FOUND FALSE)
  elseif(NOT otv_bf7_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(otv_bf7_FOUND FALSE)
  endif()
  return()
endif()
set(_otv_bf7_CONFIG_INCLUDED TRUE)

# output package information
if(NOT otv_bf7_FIND_QUIETLY)
  message(STATUS "Found otv_bf7: 1.0.3 (${otv_bf7_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'otv_bf7' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${otv_bf7_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(otv_bf7_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${otv_bf7_DIR}/${_extra}")
endforeach()
