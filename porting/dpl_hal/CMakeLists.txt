cmake_minimum_required(VERSION 3.12)
project(dpl_hal VERSION ${VERSION} LANGUAGES C)

file(GLOB ${PROJECT_NAME}_SOURCES ./src/*.c)
file(GLOB ${PROJECT_NAME}_HEADERS ./include/*.h)

include_directories(
    include
    "${PROJECT_SOURCE_DIR}/../../bin/targets/syscfg/generated/include/"
)

add_library(${PROJECT_NAME} 
    STATIC
    ${${PROJECT_NAME}_SOURCES} 
    ${${PROJECT_NAME}_HEADERS}
)

add_library(libdpl_linux ALIAS dpl_linux)
get_target_property(libdpl_linux_INCLUDE_DIRECTORIES libdpl_linux INCLUDE_DIRECTORIES)
add_library(libdpl_os ALIAS dpl_os)
get_target_property(libdpl_os_INCLUDE_DIRECTORIES libdpl_os INCLUDE_DIRECTORIES)

#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../porting/dpl_os")
#include(dpl_os-config)
#find_package(dpl_os CONFIG QUIET)
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}/../../porting/dpl/linux")
#include(dpl_linux-config)
#find_package(dpl_linux CONFIG QUIET)

#add_dependencies(${PROJECT_NAME} dpl_linux dpl_os)

include(GNUInstallDirs)
target_include_directories(
    ${PROJECT_NAME} 
    PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/>
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    PRIVATE ${libdpl_linux_INCLUDE_DIRECTORIES}
    PRIVATE ${libdpl_os_INCLUDE_DIRECTORIES}
)

if(dpl_os_FOUND)
target_include_directories(
    ${PROJECT_NAME} 
    PRIVATE
      $<TARGET_PROPERTY:dpl_os,INTERFACE_INCLUDE_DIRECTORIES>
)
endif() 

if(dpl_linux_FOUND)
target_include_directories(
    ${PROJECT_NAME} 
   PRIVATE
      $<TARGET_PROPERTY:dpl_linux,INTERFACE_INCLUDE_DIRECTORIES>
)
endif() 

#target_link_libraries(
#    ${PROJECT_NAME} 
#  #dpl_linux
#  #dpl_os
#)

# Install version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-version.cmake"
    VERSION ${timescale_VERSION}
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
  DESTINATION ./
  CONFIGURATIONS Release
)

install(
  FILES 
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-debug.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-version.cmake"
  DESTINATION ./
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

#configure_file(${PROJECT_NAME}-config.cmake
#    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-release.cmake"
#    COPYONLY
#)

install(DIRECTORY include/ DESTINATION include/
        FILES_MATCHING PATTERN "*.h"
)