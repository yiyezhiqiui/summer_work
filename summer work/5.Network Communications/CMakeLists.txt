cmake_minimum_required(VERSION 3.2)
project(cmake_test)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)

# 包含源文件
file(GLOB SRC    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
                 "${CMAKE_CURRENT_SOURCE_DIR}/src/armor_visual/*.cpp"
                 "${CMAKE_CURRENT_SOURCE_DIR}/src/quater_pos/*.cpp"
                  )


# 指定头文件路径
include_directories(${PROJECT_SOURCE_DIR}/include)

# 查找并包含 OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# 添加可执行文件并链接源文件及 OpenCV 库
add_executable(test ${SRC})
target_link_libraries(test PRIVATE ${OpenCV_LIBS})  # 使用关键字方式链接
