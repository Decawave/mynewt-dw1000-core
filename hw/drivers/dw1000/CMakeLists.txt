cmake_minimum_required(VERSION 3.2)
project(dw1000_driver)

file(GLOB ${PROJECT_NAME}_SOURCES ./src/*.c)
file(GLOB ${PROJECT_NAME}_HEADERS ./include/*.h)

include_directories(include)
add_subdirectory(src)
add_subdirectory(include)

source_group("include" FILES ${${PROJECT_NAME}_HEADERS})
source_group("lib" FILES ${${PROJECT_NAME}_SOURCES})

add_library(${PROJECT_NAME} 
    STATIC
    ${${PROJECT_NAME}_SOURCES} 
    ${${PROJECT_NAME}_HEADERS}
)

target_link_libraries(
    ${PROJECT_NAME} 
    dpl_os
#    $<TARGET_OBJECTS:dpl_hal>
)
install(
    TARGETS 
    ${PROJECT_NAME} ARCHIVE 
    DESTINATION lib
)
install(
    FILES 
    ${${PROJECT_NAME}_HEADERS} 
    DESTINATION include/dw1000
)
