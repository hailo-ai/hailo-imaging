cmake_minimum_required(VERSION 3.1.0)
# define module name & interface version
set (module HAILO_IMX334)
# define interface version
set (${module}_INTERFACE_CURRENT  1)
set (${module}_INTERFACE_REVISION 0)
set (${module}_INTERFACE_AGE      0)
#set(CMAKE_INSTALL_PREFIX
add_definitions(-DLINUX)
add_definitions(-DHAL_ALTERA)
# we want to compile all .c files as default
file(GLOB libsources source/IMX334.c )
# set public headers, these get installed
file(GLOB pub_headers include/*.h)
# define include paths
include_directories(
    include
    include_priv
    ${LIB_ROOT}
    ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/include
    ${CMAKE_SYSROOT}/usr/include/imaging
    )
# module specific defines
###add_definitions(-Wno-error=unused-function)
# add lib to build env
if(${DUMMY_BUILD} STREQUAL 0)
    add_library(${module}_shared SHARED ${libsources})
    # install the sensor driver as well, but to 'bin' location!
    install(FILES       ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/lib/lib${module}.so.${${module}_INTERFACE_CURRENT}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    RENAME      ${module}.drv
    )
else(${DUMMY_BUILD} STREQUAL 0)
    add_library(${module}_shared SHARED)
endif(${DUMMY_BUILD} STREQUAL 0)
SET_TARGET_PROPERTIES(${module}_shared PROPERTIES OUTPUT_NAME     ${module})
SET_TARGET_PROPERTIES(${module}_shared PROPERTIES LINK_FLAGS      -shared)
SET_TARGET_PROPERTIES(${module}_shared PROPERTIES SOVERSION       ${${module}_INTERFACE_CURRENT})
SET_TARGET_PROPERTIES(${module}_shared PROPERTIES VERSION         ${${module}_INTERFACE_CURRENT}.${${module}_INTERFACE_REVISION}.${${module}_INTERFACE_AGE})
SET_TARGET_PROPERTIES(${module}_shared PROPERTIES FRAMEWORK       TRUE PUBLIC_HEADER "${pub_headers}")

install(TARGETS ${module}_shared
        PUBLIC_HEADER   DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${module}
        ARCHIVE         DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/${module}
        LIBRARY         DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
        )
# create common targets for this module
#include(${UNITS_TOP_DIRECTORY}/targets.cmake)
# create calib data targets
#add_subdirectory(calib)
