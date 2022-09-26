cmake_minimum_required(VERSION 3.16)
project("sdpf")

#set(CMAKE_CXX_FLAGS "-std=c++20 -ggdb")
set(CMAKE_CXX_FLAGS "-std=c++20 -ggdb -DLUA_USE_DLOPEN")
set(CMAKE_C_FLAGS "-DLUA_USE_DLOPEN")

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-E -fopenmp")

find_package(SDL2 2.0.17 REQUIRED)
find_path(sdl2_INCLUDE_DIR SDL.h)
find_library(sdl2_LIBRARY SDL2)
find_library(sdl2main_LIBRARY SDL2main)
find_path(sdl2_ttf_INCLUDE_DIR SDL_ttf.h)
find_library(sdl2_ttf_LIBRARY SDL2_ttf)


include_directories(
    ./
    ./renderer
    ./sdpf
    ./dep/
    ./dep/imgui
    ./dep/imgui/backends
    /usr/local/include/SDL2 #imgui的头文件路径有问题，必须加这一行。可根据实际情况修改
)

add_executable(sdpf
    ./renderer/main.cpp

    ./dep/imgui/imgui.cpp
    ./dep/imgui/imgui_draw.cpp
    ./dep/imgui/imgui_tables.cpp
    ./dep/imgui/imgui_widgets.cpp
    ./dep/imgui/imgui_demo.cpp
    ./dep/imgui/backends/imgui_impl_sdl.cpp
    ./dep/imgui/backends/imgui_impl_sdlrenderer.cpp

)

target_link_libraries(sdpf
    ${SDL2_LIBRARIES}
    ${sdl2_ttf_LIBRARY}
)

set_target_properties(sdpf PROPERTIES OUT_NAME "sdpf")

file(COPY res DESTINATION ./)