project(dpl_hal)

file(GLOB ${PROJECT_NAME}_SOURCES ./src/*.c)
file(GLOB ${PROJECT_NAME}_HEADERS ./include/*.h)

add_library(${PROJECT_NAME} 
    STATIC
    ${${PROJECT_NAME}_SOURCES} 
    ${${PROJECT_NAME}_HEADERS}
)

include_directories(
    include
    "${PROJECT_SOURCE_DIR}/../../../../bin/targets/syscfg/generated/include/"
)

target_include_directories(${PROJECT_NAME} 
    PRIVATE INTERFACE    
    ./include
)
target_link_libraries(
    ${PROJECT_NAME} 
    dpl_os
)

# Install library
install(
    TARGETS ${PROJECT_NAME} ARCHIVE 
    DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include/
        FILES_MATCHING PATTERN "*.h"
)