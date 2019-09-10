cmake_minimum_required(VERSION 3.2)
project(dsp VERSION ${VERSION} LANGUAGES C)

file(GLOB ${PROJECT_NAME}_SOURCES 
    src/*.c
)
file(GLOB ${PROJECT_NAME}_HEADERS 
    include/*.h
)

include_directories(
    include
    "${PROJECT_SOURCE_DIR}/../../bin/targets/syscfg/generated/include/"
    "${PROJECT_SOURCE_DIR}/../../../porting/dpl_hal/include"
)

source_group("include" FILES ${${PROJECT_NAME}_HEADERS})
source_group("lib" FILES ${${PROJECT_NAME}_SOURCES})

add_library(${PROJECT_NAME} 
    STATIC
    ${${PROJECT_NAME}_SOURCES} 
    ${${PROJECT_NAME}_HEADERS}
)

include(GNUInstallDirs)
target_include_directories(${PROJECT_NAME} 
    PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/>
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
)

install(DIRECTORY include/ DESTINATION include/
        FILES_MATCHING PATTERN "*.h"
)

include(../../CMakeCommon.cmake)

