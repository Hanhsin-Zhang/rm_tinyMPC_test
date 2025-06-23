# tiny_mpc_project/CMakeLists.txt

cmake_minimum_required(VERSION 3.14)
project(TinyMpcTestProject CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 根据构建类型设置编译选项
if(false)
    add_definitions(-DDEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall")
    message(STATUS "Building in DEBUG mode")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    message(STATUS "Building in RELEASE mode")
endif()
# 设置构建输出目录，符合 /bin 和 /lib 的要求
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
# 可执行文件将放在 /bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
# 静态/动态库将放在 /lib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
# 寻找 Eigen3 库
# 确保你已经安装了 Eigen，或者将其路径添加到 CMAKE_PREFIX_PATH
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 添加 tiny_mpc 模块子目录
add_subdirectory(src/tiny_mpc)

# --- 构建测试用例 ---
# 添加可执行文件
add_executable(mpc_test test/main.cpp)
add_executable(gimbal_test test/gimbal.cpp)
# 为测试用例添加头文件搜索路径，以便能找到 #include "tiny_mpc/TinyMpcSolver.h"
target_include_directories(mpc_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(gimbal_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
# 链接 tinympc_static 库和 Eigen3 库到测试用例
target_link_libraries(mpc_test PRIVATE tinympc_static Eigen3::Eigen)
target_link_libraries(gimbal_test PRIVATE tinympc_static Eigen3::Eigen)
message(STATUS "Project configured. Executable will be in '${EXECUTABLE_OUTPUT_PATH}'")