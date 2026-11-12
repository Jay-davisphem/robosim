# Minimal CMake build for the vendored Dear ImGui sources.
# Upstream ImGui does not ship a CMakeLists.txt, so we provide the three
# targets that the top-level project expects: imgui, imgui_impl_glfw and
# imgui_impl_opengl3.

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_library(imgui STATIC
    imgui.cpp
    imgui_draw.cpp
    imgui_tables.cpp
    imgui_widgets.cpp
    imgui_demo.cpp
)

target_include_directories(imgui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/backends
)

target_compile_features(imgui PUBLIC cxx_std_17)

add_library(imgui_impl_glfw STATIC
    backends/imgui_impl_glfw.cpp
)

target_link_libraries(imgui_impl_glfw PUBLIC
    imgui
    glfw
)

add_library(imgui_impl_opengl3 STATIC
    backends/imgui_impl_opengl3.cpp
)

target_link_libraries(imgui_impl_opengl3 PUBLIC
    imgui
    OpenGL::GL
)
