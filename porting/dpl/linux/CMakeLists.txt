project(dpl_linux)

set(CMAKE_VERBOSE_MAKEFILE ON)
file(GLOB ${PROJECT_NAME}_SOURCES src/*.c src/*.cc)
file(GLOB ${PROJECT_NAME}_HEADERS include/*.h)

add_library(${PROJECT_NAME} 
    STATIC
    ${${PROJECT_NAME}_SOURCES} 
    ${${PROJECT_NAME}_HEADERS}
)


include_directories(
    include
    "${PROJECT_SOURCE_DIR}/../../../../../bin/targets/syscfg/generated/include/"
)

target_include_directories(
    ${PROJECT_NAME} 
    PUBLIC 
    include
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

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
    DESTINATION include
)

#[[
# API Unit test
add_executable(dpl_sem test/test_dpl_sem.c)

target_link_libraries(
    dpl_sem 
    dpl_linux
    Threads::Threads
)

add_executable(dpl_task test/test_dpl_task.c)
target_link_libraries(
    dpl_task 
    dpl_linux
    Threads::Threads
)

add_executable(dpl_eventq test/test_dpl_eventq.c)
target_link_libraries(
    dpl_eventq
    dpl_linux
    Threads::Threads
)

add_executable(dpl_callout test/test_dpl_callout.c)
target_link_libraries(
    dpl_callout
    dpl_linux
    Threads::Threads
    -lrt
)

#add_executable(dpl_mempool test/test_dpl_mempool.c)
#target_link_libraries(
#    dpl_mempool
#    dpl_linux
#    Threads::Threads
#)

]]
