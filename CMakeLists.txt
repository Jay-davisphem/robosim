cmake_minimum_required(VERSION 3.26)
project(
    robosim
    VERSION 1.0
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

## Find dependencies
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)

# Build ImGui as subdirectory
add_subdirectory(lib/imgui)

set(SOURCES
    src/core/Application.cpp
    src/core/DhParam.cpp
    src/core/InverseKinematics.cpp
    src/core/JointedLink.cpp
    src/core/PidController.cpp
    src/core/Robot.cpp
    src/Main.cpp
    src/meshes/BoxMesh.cpp
    src/meshes/CylinderMesh.cpp
    src/meshes/JointedLinkMesh.cpp
    src/meshes/Mesh.cpp
    src/meshes/SphereMesh.cpp
    src/rendering/Camera.cpp 
)


include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/lib
    ${PROJECT_SOURCE_DIR}/lib/imgui
    ${PROJECT_SOURCE_DIR}/lib/ImGuizmo
)

add_library(ImGuizmo lib/ImGuizmo/ImGuizmo.cpp)
target_link_libraries(
    ImGuizmo 
    PRIVATE
        imgui
        imgui_impl_glfw
        imgui_impl_opengl3
)

add_executable(robosim ${SOURCES})

target_link_libraries(
    robosim
    PRIVATE
        OpenGL::GL
        glfw
        GLEW::GLEW
        imgui
        imgui_impl_glfw
        imgui_impl_opengl3
        ImGuizmo
)


set_target_properties(robosim PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if (MSVC)
    # M_PI and friends are not exposed by <cmath> on MSVC without this flag.
    target_compile_definitions(robosim PRIVATE _USE_MATH_DEFINES NOMINMAX)
endif()
  