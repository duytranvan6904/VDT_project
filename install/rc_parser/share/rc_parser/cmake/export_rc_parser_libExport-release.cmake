#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rc_parser::rc_parser_lib" for configuration "Release"
set_property(TARGET rc_parser::rc_parser_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rc_parser::rc_parser_lib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librc_parser_lib.a"
  )

list(APPEND _cmake_import_check_targets rc_parser::rc_parser_lib )
list(APPEND _cmake_import_check_files_for_rc_parser::rc_parser_lib "${_IMPORT_PREFIX}/lib/librc_parser_lib.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
