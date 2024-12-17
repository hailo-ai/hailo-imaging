cmake_minimum_required(VERSION 2.6)

set(APP hdr_manager_example)

# include directories
include_directories(
    ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/include
    )

file(GLOB TEST_SRC src/*.cpp)

if(${DUMMY_BUILD} STREQUAL 0)
    # add build target
    add_executable(${APP}
        ${TEST_SRC}
    )

    set_property(TARGET ${APP} PROPERTY CXX_STANDARD 20)
    target_link_libraries(
        ${APP}
        ${drv_libs}
        ${aaa_libs}
        ${ctrl_libs}
        ${base_libs}
        ${platform_libs}
        ${LIB_PTHREAD_NAME}
	-lhailort
    )

    link_directories(
        ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/lib
    )

    install(TARGETS ${APP} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

    else(${DUMMY_BUILD} STREQUAL 0)
        add_library(${APP}_shared SHARED)
        target_link_libraries(${APP}_shared )
endif(${DUMMY_BUILD} STREQUAL 0)

