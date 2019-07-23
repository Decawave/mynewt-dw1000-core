cmake_minimum_required(VERSION 3.2)
project(dw1000)

file(GLOB ${PROJECT_NAME}_SOURCES ./src/*.c)
file(GLOB ${PROJECT_NAME}_HEADERS 
    ./include/dw1000/*.h 
    ./include/syscfg/*.h 
    ./include/sysflash/*.h 
)

configure_file(dw1000.pc.in dw1000.pc @ONLY)
message(WARNING "${PROJECT_SOURCE_DIR}/../../../porting/dpl_hal")
include_directories(
    include
    "${PROJECT_SOURCE_DIR}/../../../../../bin/targets/syscfg/generated/include/"
    "${PROJECT_SOURCE_DIR}/../../../porting/dpl_hal/include"
)

source_group("include" FILES ${${PROJECT_NAME}_HEADERS})
source_group("lib" FILES ${${PROJECT_NAME}_SOURCES})

add_library(${PROJECT_NAME} 
    STATIC
    ${${PROJECT_NAME}_SOURCES} 
    ${${PROJECT_NAME}_HEADERS}
)

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(
    ${PROJECT_NAME} 
    dpl_os
    dpl_hal
#    $<TARGET_OBJECTS:dpl_hal>
)

# Install library
install(
    TARGETS ${PROJECT_NAME} ARCHIVE 
    DESTINATION lib/${PROJECT_NAME}
)
install(
    FILES 
    ${${PROJECT_NAME}_HEADERS} 
    DESTINATION include/${PROJECT_NAME}
)
#install(
#    TARGETS ${PROJECT_NAME}
#    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
#)

