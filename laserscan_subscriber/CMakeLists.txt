cmake_minimum_required(VERSION 3.8)
project(laserscan_subscriber)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# new stuff
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PYTHON_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS} /usr/lib/python3.10/site-packages/numpy/core/include)
# external libs
# find_package(FastAD CONFIG REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(Ceres REQUIRED)
find_package(autodiff REQUIRED)
find_package(Curses REQUIRED)
# matplot
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
find_package(OpenMP REQUIRED)
#

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#add_executable(laserscan_subscriber src/laserscan_subscriber.cpp src/matplotlibcpp.cpp src/visualizer.hpp src/functions.hpp)
add_executable(laserscan_subscriber src/laserscan_subscriber.cpp)
ament_target_dependencies(laserscan_subscriber rclcpp std_msgs geometry_msgs sensor_msgs)

target_include_directories(laserscan_subscriber PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(laserscan_subscriber PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS laserscan_subscriber
        DESTINATION lib/${PROJECT_NAME})

#########
if (Python3_NumPy_FOUND AND OpenMP_CXX_FOUND)
    target_link_libraries(laserscan_subscriber #INTERFACE
            Python3::NumPy OpenMP::OpenMP_CXX autodiff::autodiff
            )
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} ")
else ()
    target_compile_definitions(laserscan_subscriber INTERFACE WITHOUT_NUMPY)
endif ()
install(
        TARGETS laserscan_subscriber
        EXPORT install_targets
)

target_include_directories(laserscan_subscriber
        PRIVATE
        ${CURSES_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}
        $<INSTALL_INTERFACE:include>
        )

target_link_libraries(laserscan_subscriber
        #PRIVATE
        ${CURSES_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        ${CERES_LIBRARIES}
        Python3::Python
        Python3::Module
        )
#######

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
