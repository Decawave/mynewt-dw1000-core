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
    include/syscfg/*.h
    include/sysinit/*.h
)

list(REMOVE_ITEM ${PROJECT_NAME}_HEADERS os_cputime.h)
list(REMOVE_ITEM ${PROJECT_NAME}_SOURCES os_cputime.c os_cputime_pwr2.c)

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
)
target_link_libraries(
    ${PROJECT_NAME} 
    dpl_linux
)
install(
    TARGETS 
    ${PROJECT_NAME} ARCHIVE 
    DESTINATION lib
)
install(
    FILES 
    ${${PROJECT_NAME}_HEADERS} 
    DESTINATION include
)
