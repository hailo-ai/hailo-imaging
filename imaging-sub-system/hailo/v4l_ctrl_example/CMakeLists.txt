cmake_minimum_required(VERSION 2.6)

set(APP v4l_ctrl_example)

# include directories
include_directories(
    include
    ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/include
    )

file(GLOB TEST_SRC src/*.c)

# add build target
add_executable(${APP}
    ${TEST_SRC}
)

target_link_libraries(
    ${APP}
    ${drv_libs}
    ${aaa_libs}
    ${ctrl_libs}
    ${base_libs}
    ${platform_libs}
    ${LIB_PTHREAD_NAME}
)

link_directories(
    ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/lib
)

install(TARGETS ${APP} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
