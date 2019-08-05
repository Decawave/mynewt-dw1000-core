project(dpl_os)

file(GLOB ${PROJECT_NAME}_SOURCES 
    src/*.c
)
file(GLOB ${PROJECT_NAME}_HEADERS 
    include/log/*.h
    include/mem/*.h
    include/modlog/*.h
    include/os/*.h
    include/stat/*.h
)

include_directories(
    include
    "${PROJECT_SOURCE_DIR}/../../../../bin/targets/syscfg/generated/include/"
)


source_group("include" FILES ${HEADERS})
source_group("lib" FILES ${SOURCES})

add_library(${PROJECT_NAME} 
    STATIC
    ${${PROJECT_NAME}_SOURCES} 
    ${${PROJECT_NAME}_HEADERS}
)
target_include_directories(${PROJECT_NAME} 
    PUBLIC  
    include
    dw1000_driver
)
target_link_libraries(
    ${PROJECT_NAME} 
    dpl_linux
    dpl_hal
)

# Install library
install(
    TARGETS ${PROJECT_NAME} ARCHIVE 
    DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include/
        FILES_MATCHING PATTERN "*.h"
)

