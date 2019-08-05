cmake_minimum_required(VERSION 3.2)
project(twr_ss_ext)

file(GLOB ${PROJECT_NAME}_SOURCES 
    src/*.c
)
file(GLOB ${PROJECT_NAME}_HEADERS 
    include/*.h
)

#configure_file(${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)

include_directories(
    include
    "${PROJECT_SOURCE_DIR}/../../../../bin/targets/syscfg/generated/include/"
    "${PROJECT_SOURCE_DIR}/../../../porting/dpl_hal/include"
)

source_group("include" FILES ${${PROJECT_NAME}_HEADERS})
source_group("lib" FILES ${${PROJECT_NAME}_SOURCES})

add_library(${PROJECT_NAME} 
    STATIC
    ${${PROJECT_NAME}_SOURCES} 
    ${${PROJECT_NAME}_HEADERS}
)

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
    rng
)

# Install library
install(
    TARGETS ${PROJECT_NAME} ARCHIVE 
    DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include/
        FILES_MATCHING PATTERN "*.h"
)

