# src/tiny_mpc/CMakeLists.txt (修正后)

add_library(tinympc_static STATIC
    TinyMpcSolver.cpp
)

set_property(TARGET tinympc_static PROPERTY POSITION_INDEPENDENT_CODE ON)

# 链接到 Eigen3。这会自动处理头文件包含路径。
# 使用 PUBLIC 是因为 tinympc_static 的公共头文件（*.h）中包含了 Eigen 的头文件。
# 这意味着任何链接到 tinympc_static 的目标（比如我们的 mpc_test）也需要知道 Eigen 的路径。
target_link_libraries(tinympc_static PUBLIC Eigen3::Eigen)