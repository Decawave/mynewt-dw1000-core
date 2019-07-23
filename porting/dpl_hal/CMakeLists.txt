project(dpl_hal)

file(GLOB ${PROJECT_NAME}_SOURCES ./src/*.c)
file(GLOB ${PROJECT_NAME}_HEADERS ./include/*.h)

add_library(${PROJECT_NAME} 
    STATIC
    ${${PROJECT_NAME}_SOURCES} 
    ${${PROJECT_NAME}_HEADERS}
)
target_include_directories(${PROJECT_NAME} 
    PRIVATE INTERFACE    
    ./include
)
target_link_libraries(
    ${PROJECT_NAME} 
    dpl_os
)
install(
    TARGETS 
    ${PROJECT_NAME} ARCHIVE 
    DESTINATION lib
)
install(
    FILES 
    ${${PROJECT_NAME}_HEADERS} 
    DESTINATION include/hal
)
