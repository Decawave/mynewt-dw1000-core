cmake_minimum_required(VERSION 3.12)
project(nrng VERSION ${VERSION} LANGUAGES C)

file(GLOB ${PROJECT_NAME}_SOURCES 
    src/*.c
)
file(GLOB ${PROJECT_NAME}_HEADERS 
    include/*.h
)

#configure_file(${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)

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

add_library(libdpl_os ALIAS dpl_os)
get_target_property(libdpl_os_INCLUDE_DIRECTORIES libdpl_os INCLUDE_DIRECTORIES)
add_library(libuwb_dw1000 ALIAS uwb_dw1000)
get_target_property(libuwb_dw1000_INCLUDE_DIRECTORIES libuwb_dw1000 INCLUDE_DIRECTORIES)
add_library(libdsp ALIAS dsp)
get_target_property(libdsp_INCLUDE_DIRECTORIES libdsp INCLUDE_DIRECTORIES)
add_library(libeuclid ALIAS euclid)
get_target_property(libeuclid_INCLUDE_DIRECTORIES libeuclid INCLUDE_DIRECTORIES)
add_library(librng ALIAS rng)
get_target_property(librng_INCLUDE_DIRECTORIES librng INCLUDE_DIRECTORIES)
add_library(libcir ALIAS cir)
get_target_property(libcir_INCLUDE_DIRECTORIES libcir INCLUDE_DIRECTORIES)


include(GNUInstallDirs)
target_include_directories(${PROJECT_NAME} 
    PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/>
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      PRIVATE ${libdpl_os_INCLUDE_DIRECTORIES}
      PRIVATE ${libuwb_dw1000_INCLUDE_DIRECTORIES}
      PRIVATE ${libdsp_INCLUDE_DIRECTORIES}
      PRIVATE ${libeuclid_INCLUDE_DIRECTORIES}
      PRIVATE ${librng_INCLUDE_DIRECTORIES}
      PRIVATE ${libcir_INCLUDE_DIRECTORIES}
)

# Install library
install(DIRECTORY include/ DESTINATION include/
        FILES_MATCHING PATTERN "*.h"
)

include(../../CMakeCommon.cmake)

#[[

target_include_directories(
    ${PROJECT_NAME} 
    PUBLIC 
    include
)

target_link_libraries(
    ${PROJECT_NAME} 
    dw1000
    dpl_os
    dsp
    euclid
    cir
    rng
    twr_ss_nrng
)

# Install library
install(
    TARGETS ${PROJECT_NAME} ARCHIVE 
    DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include/
        FILES_MATCHING PATTERN "*.h"
)
]]
