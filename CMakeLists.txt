# tiny_mpc_project/CMakeLists.txt (重构后)

cmake_minimum_required(VERSION 3.14)
project(GimbalControllerPackage CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED)

# --- 1. 定义 tinympc 库 ---
# 注意：我们不再创建 SHARED 库，让父项目决定库的类型。
# 如果你确实需要 SHARED，可以保留，但通常由顶层项目控制。
add_library(tinympc src/tiny_mpc/TinyMpcSolver.cpp)
target_include_directories(tinympc
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tiny_mpc
    PRIVATE
        ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(tinympc PUBLIC Eigen3::Eigen)
set_property(TARGET tinympc PROPERTY POSITION_INDEPENDENT_CODE ON)


# --- 2. 定义 gimbalcontroller 库 ---
add_library(gimbalcontroller src/gimbal_control/GimbalController.cpp)
target_include_directories(gimbalcontroller
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gimbal_control
)
# gimbalcontroller 需要 tinympc 和 Eigen
target_link_libraries(gimbalcontroller PUBLIC tinympc Eigen3::Eigen)
set_property(TARGET gimbalcontroller PROPERTY POSITION_INDEPENDENT_CODE ON)


# --- 3. 仅在独立编译时，创建测试程序 ---
# 当这个 CMakeLists.txt 是顶层项目时，PROJECT_SOURCE_DIR 和 CMAKE_SOURCE_DIR 相等。
if(PROJECT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(STATUS "GimbalController: Building in STANDALONE mode.")

    # 设置输出目录
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

    # 添加测试可执行文件
    add_executable(gimbal_standalone_test src/gimbal_test.cpp)

    # 链接测试程序所需的库
    target_link_libraries(gimbal_standalone_test PRIVATE gimbalcontroller)

    # 编译选项
    if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(gimbal_standalone_test PRIVATE -g -O0 -Wall)
        message(STATUS "Standalone mode: Building in DEBUG mode")
    else()
        target_compile_options(gimbal_standalone_test PRIVATE -O3)
        message(STATUS "Standalone mode: Building in RELEASE mode")
    endif()

else()
    message(STATUS "GimbalController: Integrated into a larger project.")
endif()