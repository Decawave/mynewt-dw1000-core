cmake_minimum_required(VERSION 3.2)
project(syscfg)

set(CMAKE_VERBOSE_MAKEFILE ON)

if(NOT EXISTS "${PROJECT_BINARY_DIR}/../../../bin/targets/${PROJECT_NAME}/generated/include/syscfg/syscfg.h")
    message(WARNING "Building newt dependencies")
    execute_process(COMMAND newt target create ${PROJECT_NAME})
    execute_process(COMMAND newt target set ${PROJECT_NAME} app=apps/${PROJECT_NAME})
    execute_process(COMMAND newt target set ${PROJECT_NAME} bsp=@mynewt-dw1000-core/hw/bsp/dwm1001)
    execute_process(COMMAND newt build ${PROJECT_NAME})

if(NOT EXISTS "${PROJECT_BINARY_DIR}/../../../bin/targets/${PROJECT_NAME}/generated/include/syscfg/syscfg.h")
    message( FATAL_ERROR "Building newt dependencies failed." )
endif() 
endif()

file(GLOB ${PROJECT_NAME}_SOURCES 
    src/*.c 
#    "${PROJECT_BINARY_DIR}/../../../bin/targets/${PROJECT_NAME}/generated/src/syscfg-sysinit-app.c"
)
file(GLOB ${PROJECT_NAME}_HEADERS 
    include/*.h
    "${PROJECT_BINARY_DIR}/../../../bin/targets/${PROJECT_NAME}/generated/include/syscfg/*.h"
    "${PROJECT_BINARY_DIR}/../../../bin/targets/${PROJECT_NAME}/generated/include/sysflash/*.h"
)

#add_executable(
#    ${PROJECT_NAME} 
#    ${${PROJECT_NAME}_SOURCES}
#)


