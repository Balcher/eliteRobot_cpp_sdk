#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "elite_robot_SDK" for configuration ""
set_property(TARGET elite_robot_SDK APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(elite_robot_SDK PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libelite_robot_SDK.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS elite_robot_SDK )
list(APPEND _IMPORT_CHECK_FILES_FOR_elite_robot_SDK "${_IMPORT_PREFIX}/lib/libelite_robot_SDK.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
