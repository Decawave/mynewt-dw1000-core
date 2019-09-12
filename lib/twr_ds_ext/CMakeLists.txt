project(twr_ds_ext VERSION ${VERSION} LANGUAGES C)

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

add_library(libdpl_linux ALIAS dpl_linux)
get_target_property(libdpl_linux_INCLUDE_DIRECTORIES libdpl_linux INCLUDE_DIRECTORIES)
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

include(GNUInstallDirs)
target_include_directories(${PROJECT_NAME} 
    PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/>
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      PRIVATE ${libdpl_linux_INCLUDE_DIRECTORIES}
      PRIVATE ${libdpl_os_INCLUDE_DIRECTORIES}
      PRIVATE ${libuwb_dw1000_INCLUDE_DIRECTORIES}
      PRIVATE ${libdsp_INCLUDE_DIRECTORIES}
      PRIVATE ${libeuclid_INCLUDE_DIRECTORIES}
      PRIVATE ${librng_INCLUDE_DIRECTORIES}
)

# Install library
install(DIRECTORY include/ DESTINATION include/
        FILES_MATCHING PATTERN "*.h"
)

include(../../CMakeCommon.cmake)


