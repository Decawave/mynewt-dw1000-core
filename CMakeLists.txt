cmake_minimum_required(VERSION 3.2)
project(dw1000)
set (${PROJECT_NAME}_VERSION_MAJOR ${VERSION_MAJOR})
set (${PROJECT_NAME}_VERSION_MINOR ${VERSION_MINOR})
set (${PROJECT_NAME}_VERSION_PATCH ${VERSION_PATCH})
configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/../include/config.h"
  )
set(CMAKE_BUILD_TYPE ${BUILD_TYPE})

add_subdirectory(hw/drivers/dw1000)
add_subdirectory(lib)
add_subdirectory(porting/dpl)
add_subdirectory(porting/dpl_os)
add_subdirectory(porting/dpl_hal)

option(BUILD_DOC "Build documentation" ON)

if (CMAKE_BUILD_TYPE MATCHES "^[Rr]elease")
find_package(Doxygen)
if(DOXYGEN_FOUND)
	option(BUILD_DOC "Build documentation" ON)
	set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doxy/Doxyfile-dw1000.conf)
    	set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
	configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    	message("Doxygen build started")

	add_custom_target(doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
endif()
endif()

# build a CPack driven installer package
include (InstallRequiredSystemLibraries)
set (CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set (CPACK_PACKAGE_VERSION_MAJOR "${${PROJECT_NAME}_VERSION_MAJOR}")
set (CPACK_PACKAGE_VERSION_MINOR "${${PROJECT_NAME}_VERSION_MINOR}")
set (CPACK_PACKAGE_VERSION_PATCH "${${PROJECT_NAME}_VERSION_PATCH}")

set (CPACK_COMPONENTS_ALL applications libraries headers)
set (CPACK_COMPONENT_APPLICATIONS_DESCRIPTION "Decawave Driver")
set (CPACK_COMPONENT_LIBRARIES_DESCRIPTION "Static libraries used to build programs with example")
set (CPACK_COMPONENT_HEADERS_DESCRIPTION "C/C++ header files for use with template")
set (CPACK_COMPONENT_DOCS_DISPLAY_NAME "C++ API Documentation")
set (CPACK_COMPONENT_DOCS_DESCRIPTION "C++ API HTML files describing the API for Interface")

find_package(Doxygen)
if(DOXYGEN_FOUND)
install (DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs/html DESTINATION ./)
endif()

include(CPack)
include(CTest)

# does the application run
add_test (TutorialRuns Tutorial 25)

