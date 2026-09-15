#!/usr/bin/env python3
"""编译真实感知节点，验证 HDMap → 两秒历史滤波 → 0.25 置信度筛选 → publish 的实际回调链。"""

import argparse
import json
from pathlib import Path
import sys

from verify import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--hdmap-sdk", type=Path)
    args = parser.parse_args()
    pnc = Path(__file__).resolve().parents[3]
    sdk = (args.hdmap_sdk or pnc.parent / "hdmap/sdk").resolve()
    work = args.output.resolve()
    work.mkdir(parents=True, exist_ok=True)
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(pnc / "tests/planning"))
    from generate_stubs import generate
    generate(pnc, work / "stubs")
    # 只在本次输出目录的 ROS 时钟桩中增加一次性故障注入；消息结构仍由真实 .msg 生成。
    clock_header = work / "stubs/ros/ros.h"
    clock_text = clock_header.read_text()
    # 允许回归实际 main 循环；默认仍不运行，避免测试依赖真实 ROS 线程和睡眠。
    original_loop = "inline bool ok() { return false; }\ninline void spinOnce() {}"
    assert clock_text.count(original_loop) == 1
    clock_text = clock_text.replace(original_loop, '''
inline int &testLoopCount() { static int count = 0; return count; }
inline std::function<void()> &testSpinOnce() { static std::function<void()> hook; return hook; }
inline bool ok() { return testLoopCount() > 0 && --testLoopCount() >= 0; }
inline void spinOnce() { if (testSpinOnce()) testSpinOnce()(); }
''')
    original_clock = "static Time now() { return Time(testTime()); }"
    assert clock_text.count(original_clock) == 1
    clock_header.write_text(clock_text.replace(original_clock, '''
    static int &testFailAfter() { static int remaining = -1; return remaining; }
    static int &testChangeAfter() { static int remaining = -1; return remaining; }
    static double &testChangedTime() { static double timestamp = 0; return timestamp; }
    static Time now() {
        int &remaining = testFailAfter();
        if (remaining == 0) { remaining = -1; return Time(std::nan("")); }
        if (remaining > 0) { --remaining; }
        int &change = testChangeAfter();
        if (change == 0) { change = -1; return Time(testChangedTime()); }
        if (change > 0) { --change; }
        return Time(testTime());
    }''') + '\n#define ROS_WARN_THROTTLE(...) ((void)0)\n')
    fake = work / "catkin"
    fake.mkdir(exist_ok=True)
    (fake / "catkinConfig.cmake").write_text(f'''
set(catkin_FOUND TRUE)
set(catkin_INCLUDE_DIRS "{work / 'stubs'}")
set(catkin_LIBRARIES "")
set(CATKIN_PACKAGE_BIN_DESTINATION "lib/robot")
set(CATKIN_PACKAGE_SHARE_DESTINATION "share/robot")
macro(add_message_files)
endmacro()
macro(generate_messages)
    add_custom_target(robot_generate_messages_cpp)
endmacro()
macro(catkin_package)
endmacro()
''')
    # 消息/通信使用桩；编译源文件、目标配置和 SDK 动态库均为实际交付内容。
    run(["cmake", "-S", str(pnc), "-B", str(work / "build"), "-Dcatkin_DIR=" + str(fake),
         "-Dhdmap_DIR=" + str(sdk / "lib/cmake/hdmap"), "-DCMAKE_BUILD_TYPE=Release",
         "-DCMAKE_CXX_FLAGS=-include cstdint -include array"], work / "configure.log")
    run(["cmake", "--build", str(work / "build"), "--target", "perception_msg_convert", "-j2"], work / "build.log")
    links = run(["ldd", str(work / "build/perception_msg_convert")], work / "dynamic_links.log")
    assert str(sdk / "lib/libhdmap_server.so") in links, links
    source = Path(__file__).with_name("publish_integration.cpp")
    flags = ["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-include", "cstdint", "-include", "array"]
    flags += ["-I" + str(path) for path in (work / "stubs", pnc / "include", sdk / "include")]
    library = [str(sdk / "lib/libhdmap_server.so"), "-Wl,-rpath," + str(sdk / "lib")]
    binary = work / "publish_integration"
    run([*flags, "-O2", str(source), *library, "-o", str(binary)], work / "integration_build.log")
    result = run([str(binary)], work / "integration.log").splitlines()[-1]
    print(result, flush=True)
    sanitized = work / "publish_sanitize"
    run([*flags, "-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
         str(source), *library, "-o", str(sanitized)], work / "sanitize_build.log")
    sanitized_result = run([str(sanitized)], work / "sanitize.log").splitlines()[-1]
    print("ASan/UBSan:", sanitized_result, flush=True)
    summary = {"language": "C++11", "transport": "generated ROS message stubs", "sdk": str(sdk),
               "cmake_target": "perception_msg_convert", "dynamic_link": "PASS", "integration": result,
               "sanitizers": sanitized_result}
    (work / "result.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print("PASS: actual PNC CMake target and SDK dynamic link")


if __name__ == "__main__":
    main()
