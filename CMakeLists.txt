cmake_minimum_required(VERSION 3.16)
project(RobotArm C CXX)

option(ROBOT_BUILD_APP   "Build the OpenGL simulator" ON)
option(ROBOT_BUILD_TESTS "Build headless unit tests (GoogleTest)" OFF)

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

find_package(Threads REQUIRED)

# --- Kinematics, world and robot language (no OpenGL), shared by the app and the tests
add_library(robot_core STATIC RobotArm.cpp World.cpp RobotLanguage.cpp CommandServer.cpp)
target_include_directories(robot_core PUBLIC ${CMAKE_SOURCE_DIR} ${GLM_INCLUDE})
# RobotArm.cpp only uses GLFW key constants (header), no GLFW functions
get_target_property(_glfw_inc ${GLFW_LIB} INTERFACE_INCLUDE_DIRECTORIES)
if(_glfw_inc)
    target_include_directories(robot_core PRIVATE ${_glfw_inc})
endif()
target_link_libraries(robot_core PUBLIC Threads::Threads)

if(ROBOT_BUILD_APP)
    find_package(OpenGL REQUIRED)
    add_executable(robot_arm
        main.cpp
        ObjMesh.cpp
        stb_image.cpp
        src/glad.c)

    target_include_directories(robot_arm PRIVATE ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/include ${GLM_INCLUDE})
    target_link_libraries(robot_arm PRIVATE robot_core ${GLFW_LIB} OpenGL::GL Threads::Threads ${CMAKE_DL_LIBS})
    target_compile_definitions(robot_arm PRIVATE ROBOT_ASSET_DIR="${CMAKE_SOURCE_DIR}")
    set_target_properties(robot_arm PROPERTIES BUILD_RPATH ${TP}/lib)
endif()

# --- Unit tests: cmake -DROBOT_BUILD_TESTS=ON, then ctest
if(ROBOT_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
        GIT_SHALLOW TRUE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    add_executable(robot_tests tests/test_robot.cpp)
    target_link_libraries(robot_tests PRIVATE robot_core GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(robot_tests PROPERTIES TIMEOUT 300)
endif()
