# 声明要求的cmake最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个cmake工程
project( helloworld )

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(GTSAM REQUIRED QUIET)

# include directories
include_directories(
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
  ${GTSAM_LIBRARY_DIRS}
)
# 添加一个可执行程序
# 语法为： add_executable( 程序名  源代码文件 ）
add_executable(gtsam_demo1 GTSAM_demo1.cpp)
target_link_libraries(gtsam_demo1 gtsam)

add_executable(gtsam_demo2 GTSAM_demo2.cpp)
target_link_libraries(gtsam_demo2 gtsam)

add_executable(gtsam_demo3 GTSAM_demo3.cpp)
target_link_libraries(gtsam_demo3 gtsam)

add_executable(gtsam_demo4 GTSAM_demo4.cpp)
target_link_libraries(gtsam_demo4 gtsam)




