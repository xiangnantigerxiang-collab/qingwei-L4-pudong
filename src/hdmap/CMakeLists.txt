cmake_minimum_required(VERSION 3.10)
project(hdmap VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Boost.Polygon 仅在加载阶段做车道区域并集，头文件依赖不暴露到 SDK 消费者。
find_package(Boost REQUIRED)

set(HDMAP_CATKIN_DEFAULT OFF)
if(DEFINED CATKIN_DEVEL_PREFIX)
    set(HDMAP_CATKIN_DEFAULT ON)
endif()
option(HDMAP_WITH_CATKIN "Export the SDK as a ROS1 catkin package" ${HDMAP_CATKIN_DEFAULT})
if(HDMAP_WITH_CATKIN)
    find_package(catkin REQUIRED)
    catkin_package(INCLUDE_DIRS include LIBRARIES hdmap_server)
endif()

add_library(hdmap_server SHARED
    src/hdmap_server.cpp
    src/coordinate.cpp
    src/trajectory_processor.cpp
    src/cubic_spline.cpp
    src/trajectory_csv.cpp
    src/lane_geometry.cpp
    src/lane_map_server.cpp
)
add_library(hdmap::hdmap_server ALIAS hdmap_server)
target_include_directories(hdmap_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(hdmap_server SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_compile_options(hdmap_server PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_compile_features(hdmap_server PUBLIC cxx_std_11)
set_target_properties(hdmap_server PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1
    CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN YES)

add_executable(hdmap_process tools/hdmap_process.cpp)
target_link_libraries(hdmap_process PRIVATE hdmap_server)
target_compile_options(hdmap_process PRIVATE -Wall -Wextra -Wpedantic -Werror)

include(GNUInstallDirs)
set_target_properties(hdmap_process PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
install(TARGETS hdmap_server hdmap_process EXPORT hdmapTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES README.md DESTINATION ${CMAKE_INSTALL_DATADIR}/hdmap)
install(DIRECTORY examples/ DESTINATION ${CMAKE_INSTALL_DATADIR}/hdmap/examples)
install(DIRECTORY docs/ DESTINATION ${CMAKE_INSTALL_DATADIR}/hdmap/docs)
if(HDMAP_WITH_CATKIN)
    # ros::package::getPath("hdmap") 在 install 空间也能定位默认车道目录。
    install(DIRECTORY map_processed/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/map_processed
        FILES_MATCHING PATTERN "*.csv")
endif()
install(EXPORT hdmapTargets FILE hdmapTargets.cmake NAMESPACE hdmap::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hdmap)

# catkin 自己生成 hdmapConfig.cmake；独立构建才安装下列 find_package 配置。
if(NOT HDMAP_WITH_CATKIN)
    include(CMakePackageConfigHelpers)
    configure_package_config_file(cmake/hdmapConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/hdmapConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hdmap)
    write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/hdmapConfigVersion.cmake
        VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/hdmapConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/hdmapConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hdmap)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(hdmap_tests tests/hdmap_tests.cpp)
    target_link_libraries(hdmap_tests PRIVATE hdmap_server)
    target_compile_options(hdmap_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME hdmap_core COMMAND hdmap_tests)
    add_executable(hdmap_lane_tests tests/lane_tests.cpp)
    target_link_libraries(hdmap_lane_tests PRIVATE hdmap_server)
    target_compile_options(hdmap_lane_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME hdmap_lanes COMMAND hdmap_lane_tests)
    add_executable(hdmap_lane_map_tests tests/lane_map_tests.cpp)
    target_link_libraries(hdmap_lane_map_tests PRIVATE hdmap_server)
    target_compile_options(hdmap_lane_map_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/map_processed")
        add_test(NAME hdmap_lane_maps COMMAND hdmap_lane_map_tests ${CMAKE_CURRENT_SOURCE_DIR}/map_processed)
    endif()
    add_executable(hdmap_lane_probe tests/lane_probe.cpp)
    target_link_libraries(hdmap_lane_probe PRIVATE hdmap_server)
    target_compile_options(hdmap_lane_probe PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_executable(hdmap_lane_benchmark tests/lane_benchmark.cpp)
    target_link_libraries(hdmap_lane_benchmark PRIVATE hdmap_server)
    target_compile_options(hdmap_lane_benchmark PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

# 可传真实 catkin 生成头目录及 ROS include；本机验证使用由真实 .msg 生成的测试桩。
set(HDMAP_ROBOT_INCLUDE_DIRS "" CACHE STRING "Include directories for optional robot message examples")
set(HDMAP_ROBOT_LIBRARIES "" CACHE STRING "Optional ROS libraries, e.g. rostime, for real message tests")
if(HDMAP_ROBOT_INCLUDE_DIRS)
    add_executable(hdmap_perception_example examples/pnc_classify_perception.cpp)
    target_include_directories(hdmap_perception_example PRIVATE ${HDMAP_ROBOT_INCLUDE_DIRS})
    target_link_libraries(hdmap_perception_example PRIVATE hdmap_server ${HDMAP_ROBOT_LIBRARIES})
    target_compile_options(hdmap_perception_example PRIVATE -Wall -Wextra -Wpedantic -Werror)
    if(BUILD_TESTING)
        add_executable(hdmap_perception_tests tests/perception_adapter_tests.cpp)
        target_include_directories(hdmap_perception_tests PRIVATE ${HDMAP_ROBOT_INCLUDE_DIRS})
        target_link_libraries(hdmap_perception_tests PRIVATE hdmap_server ${HDMAP_ROBOT_LIBRARIES})
        target_compile_options(hdmap_perception_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        add_test(NAME hdmap_perception_adapter COMMAND hdmap_perception_tests)
        add_executable(hdmap_perception_benchmark tests/lane_benchmark.cpp)
        target_compile_definitions(hdmap_perception_benchmark PRIVATE HDMAP_BENCH_ROBOT)
        target_include_directories(hdmap_perception_benchmark PRIVATE ${HDMAP_ROBOT_INCLUDE_DIRS})
        target_link_libraries(hdmap_perception_benchmark PRIVATE hdmap_server ${HDMAP_ROBOT_LIBRARIES})
        target_compile_options(hdmap_perception_benchmark PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
endif()

set(HDMAP_PNC_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pnc/include" CACHE PATH
    "Optional PNC include directory for the XYZ_COOR_S integration example")
if(EXISTS "${HDMAP_PNC_INCLUDE_DIR}/common/struct_type.h")
    add_executable(hdmap_pnc_example examples/pnc_load_path.cpp)
    target_include_directories(hdmap_pnc_example PRIVATE ${HDMAP_PNC_INCLUDE_DIR})
    target_link_libraries(hdmap_pnc_example PRIVATE hdmap_server)
    target_compile_options(hdmap_pnc_example PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
