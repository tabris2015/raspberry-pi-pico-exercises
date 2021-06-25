cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_FETCH_FROM_GIT on)
include(pico_sdk_import.cmake)

project(blink)
pico_sdk_init()
add_executable(blink
        blink.c
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(blink pico_stdlib)
pico_add_extra_outputs(blink)
