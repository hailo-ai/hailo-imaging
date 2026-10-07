cmake_minimum_required(VERSION 3.1.0)

# define module name & interface version
set (module HAILO_SV4E)

if(DEFINED LOCAL)
add_definitions("-DLINUX")
add_definitions("-DHAL_ALTERA")
endif()

# define interface version
set (${module}_INTERFACE_CURRENT  1)
set (${module}_INTERFACE_REVISION 0)
set (${module}_INTERFACE_AGE      0)

# we want to compile all .c files as default
file(GLOB libsources source/SV4E.c source/sv4e_mode_loader.cpp )

# set public headers, these get installed
file(GLOB pub_headers include/*.h)

# define include paths
include_directories(
    include
    include_priv
    ${LIB_ROOT}
    ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/include
    ${CMAKE_SYSROOT}/usr/include/imaging
    ${UNITS_TOP_DIRECTORY}/../utils3rd/3rd/jsoncpp/include
    )

# module specific defines
###add_definitions(-Wno-error=unused-function)

# add lib to build env
if(${DUMMY_BUILD} STREQUAL 0)
    add_library(${module}_shared SHARED ${libsources})
    if(TARGET jsoncpp_lib)
        target_link_libraries(${module}_shared PRIVATE jsoncpp_lib)
    else()
        target_link_libraries(${module}_shared PRIVATE jsoncpp)
    endif()

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

if( NOT DEFINED ISS_BUILD )
    # add convenience target: put sensor driver into the 'bin' output dir as well
    if ( NOT ANDROID )
    add_custom_target(${module}.drv
                    ALL
                    COMMAND ${CMAKE_COMMAND} -E copy ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/lib/lib${module}.so.${${module}_INTERFACE_CURRENT} ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/bin/${module}.drv
                    DEPENDS ${module}_shared
                    COMMENT "Copying ${module} driver module"
                    )
    endif()
endif()

install(TARGETS ${module}_shared
        PUBLIC_HEADER   DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${module}
        ARCHIVE         DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/${module}
        LIBRARY         DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
        )

# Ship the JSON mode-table example next to the driver. Operators copy
# this to /etc/hailo/sv4e_modes.json (or point HAILO_SV4E_MODES_JSON at
# their own copy) to override psv4e_mode_info_default at load time.
# Not picked up by calib/CMakeLists.txt because that one only globs *.xml.
install(FILES   ${CMAKE_CURRENT_SOURCE_DIR}/calib/HAILO_SV4E/sv4e_modes.json.example
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
        )

if(NOT DEFINED LOCAL)
# create common targets for this module
include(${UNITS_TOP_DIRECTORY}/targets.cmake)

# create calib data targets
add_subdirectory(calib)
endif()
