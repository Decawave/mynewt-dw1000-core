project(dpl_linux VERSION ${VERSION} LANGUAGES C)

set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)

file(GLOB ${PROJECT_NAME}_SOURCES 
    src/*.c
)
file(GLOB ${PROJECT_NAME}_HEADERS 
    include/*.h
)

include_directories(
    include
    "${PROJECT_SOURCE_DIR}/../../../bin/targets/syscfg/generated/include/"
)

source_group("include" FILES ${${PROJECT_NAME}_HEADERS})
source_group("lib" FILES ${${PROJECT_NAME}_SOURCES})

add_library(${PROJECT_NAME} 
    STATIC
    ${${PROJECT_NAME}_SOURCES} 
    ${${PROJECT_NAME}_HEADERS}
)

include(GNUInstallDirs)
target_include_directories(${PROJECT_NAME} 
    PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/>
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
)

include(../../../CMakeCommon.cmake)

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
    -lpthread
    -lrt
)

#add_executable(dpl_mempool test/test_dpl_mempool.c)
#target_link_libraries(
#    dpl_mempool
#    dpl_linux
#    Threads::Threads
#)

]]
