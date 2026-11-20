cmake_minimum_required(VERSION 3.16)
project(RobotArm C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(TP ${CMAKE_SOURCE_DIR}/third_party)

# --- GLFW: system package if present, otherwise the copy staged by scripts/setup_deps.sh
find_package(glfw3 3.3 QUIET)
if(glfw3_FOUND)
    set(GLFW_LIB glfw)
else()
    if(NOT EXISTS ${TP}/lib/libglfw.so)
        message(FATAL_ERROR "GLFW not found. Install libglfw3-dev or run scripts/setup_deps.sh")
    endif()
    add_library(glfw_local SHARED IMPORTED)
    set_target_properties(glfw_local PROPERTIES
        IMPORTED_LOCATION ${TP}/lib/libglfw.so
        INTERFACE_INCLUDE_DIRECTORIES ${TP}/glfw/include)
    set(GLFW_LIB glfw_local)
endif()

# --- glm: header only
find_package(glm QUIET)
if(NOT glm_FOUND AND NOT EXISTS /usr/include/glm/glm.hpp)
    if(NOT EXISTS ${TP}/glm/glm/glm.hpp)
        message(FATAL_ERROR "glm not found. Install libglm-dev or run scripts/setup_deps.sh")
    endif()
    set(GLM_INCLUDE ${TP}/glm)
endif()

find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

add_executable(robot_arm
    main.cpp
    RobotArm.cpp
    ObjMesh.cpp
    stb_image.cpp
    src/glad.c)

target_include_directories(robot_arm PRIVATE ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/include ${GLM_INCLUDE})
target_link_libraries(robot_arm PRIVATE ${GLFW_LIB} OpenGL::GL Threads::Threads ${CMAKE_DL_LIBS})
target_compile_definitions(robot_arm PRIVATE ROBOT_ASSET_DIR="${CMAKE_SOURCE_DIR}")
set_target_properties(robot_arm PROPERTIES BUILD_RPATH ${TP}/lib)
