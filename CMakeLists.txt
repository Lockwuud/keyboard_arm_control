cmake_minimum_required(VERSION 3.8)
project(keyboard_arm_control)

# 设置编译器警告选项[4](@ref)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包[3,5](@ref)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(piper_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 包含头文件目录
include_directories(include)

# 创建键盘阅读器静态库[6](@ref)
add_library(keyboard_reader src/keyboard_reader.cpp)
# 为库目标设置依赖项
ament_target_dependencies(keyboard_reader rclcpp)

# 创建主控制节点可执行文件[5](@ref)
add_executable(keyboard_arm_control_node src/keyboard_arm_control_node.cpp)
# 为主节点设置依赖项[3](@ref)
ament_target_dependencies(keyboard_arm_control_node 
  rclcpp geometry_msgs control_msgs std_msgs piper_msgs tf2)
# 链接静态库
target_link_libraries(keyboard_arm_control_node keyboard_reader)

# 安装目标文件[3,6](@ref)
install(TARGETS
  keyboard_arm_control_node
  keyboard_reader
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装启动文件[6](@ref)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

# 可选：测试配置[4](@ref)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 生成ament包[5](@ref)
ament_package()
