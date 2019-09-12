project(syscfg VERSION ${VERSION} LANGUAGES C)

#set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)

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
    "${PROJECT_BINARY_DIR}/../../../bin/targets/${PROJECT_NAME}/generated/src/syscfg-sysinit-app.c"
)
file(GLOB ${PROJECT_NAME}_HEADERS 
    include/*.h
    "${PROJECT_BINARY_DIR}/../../../bin/targets/${PROJECT_NAME}/generated/include/syscfg/*.h"
    "${PROJECT_BINARY_DIR}/../../../bin/targets/${PROJECT_NAME}/generated/include/sysflash/*.h"
)

install(
  FILES 
  "${PROJECT_BINARY_DIR}/../../../bin/targets/${PROJECT_NAME}/generated/src/syscfg-sysinit-app.c"
  DESTINATION syscfg/src
)
install(
  FILES 
  ${${PROJECT_NAME}_HEADERS}
  DESTINATION syscfg/include
)

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

# Install library
install(DIRECTORY include/ DESTINATION include/
        FILES_MATCHING PATTERN "*.h"
)

include(../../CMakeCommon.cmake)




