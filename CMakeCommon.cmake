cmake_minimum_required(VERSION 3.7)


# Install version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-version.cmake"
    VERSION ${${PROJECT_NAME}_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Install library
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Export library
install(
    EXPORT ${PROJECT_NAME}-targets
    FILE ${PROJECT_NAME}-config.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

install(
  FILES 
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-release.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-version.cmake"
  DESTINATION ./cmake
  CONFIGURATIONS Release
)

install(
  FILES 
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-debug.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-version.cmake"
  DESTINATION ./cmake
  CONFIGURATIONS Debug
)

install(
    EXPORT_ANDROID_MK ${PROJECT_NAME}-targets 
    DESTINATION share/ndk-modules/${PROJECT_NAME}
)

export(
    TARGETS ${PROJECT_NAME}
    FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    NAMESPACE ${PROJECT_NAME}::
    EXPORT_LINK_INTERFACE_LIBRARIES
)

export(
    PACKAGE ${PROJECT_NAME}
)


install(DIRECTORY "${PROJECT_BINARY_DIR}/../bin/targets/syscfg/generated/include" DESTINATION include/
        FILES_MATCHING PATTERN "*.h"
)
