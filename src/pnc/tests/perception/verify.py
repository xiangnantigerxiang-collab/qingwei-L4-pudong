#!/usr/bin/env python3
"""用真实 robot .msg 生成桩，编译真实 PNC CMake 感知目标并验证发布前分类。"""

import argparse
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "planning"))
from generate_stubs import generate


def run(command, log):
    completed = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    log.write_text(completed.stdout)
    if completed.returncode:
        raise RuntimeError(f"失败：{log}\n{completed.stdout[-5000:]}")
    return completed.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--hdmap-sdk", type=Path)
    args = parser.parse_args()
    pnc = Path(__file__).resolve().parents[2]
    sdk = (args.hdmap_sdk or pnc.parent / "hdmap/sdk").resolve()
    work = args.output.resolve()
    work.mkdir(parents=True, exist_ok=True)
    generate(pnc, work / "stubs")
    # 仅模拟 catkin 的消息生成/目录接口；源码、目标列表、include 和链接来自真实 CMakeLists。
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
    run(["cmake", "-S", str(pnc), "-B", str(work / "build"), "-Dcatkin_DIR=" + str(fake),
         "-Dhdmap_DIR=" + str(sdk / "lib/cmake/hdmap"), "-DCMAKE_BUILD_TYPE=Release",
         "-DCMAKE_CXX_FLAGS=-include cstdint -include array"], work / "configure.log")
    run(["cmake", "--build", str(work / "build"), "--target", "perception_msg_convert", "-j2"], work / "build.log")
    source = Path(__file__).with_name("integration.cpp")
    binary = work / "integration"
    includes = [work / "stubs", pnc / "include", sdk / "include"]
    flags = ["g++", "-std=c++11", "-O1", "-Wall", "-Wextra", "-Werror", "-include", "cstdint", "-include", "array"]
    flags += ["-I" + str(path) for path in includes]
    run([*flags, str(source), str(sdk / "lib/libhdmap_server.so"), "-Wl,-rpath," + str(sdk / "lib"),
         "-o", str(binary)], work / "integration_build.log")
    result = run([str(binary)], work / "integration.log")
    print(result.splitlines()[-1])
    package = ET.parse(pnc / "package.xml").getroot()
    for name in ("hdmap", "roslib"):
        for tag in ("build_depend", "exec_depend"):
            assert name in [element.text for element in package.findall(tag)], (name, tag)
    rebuild = (pnc.parents[1] / "rebuild_all.sh").read_text()
    assert rebuild.index('--pkg hdmap') < rebuild.index('--pkg robot')
    run(["bash", "-n", str(pnc.parents[1] / "rebuild_all.sh")], work / "rebuild_syntax.log")
    links = run(["ldd", str(work / "build/perception_msg_convert")], work / "dynamic_links.log")
    assert str(sdk / "lib/libhdmap_server.so") in links, links
    # catkin 隔离构建只导出传统变量，单独检查没有命名空间目标时的 include/link 分支。
    legacy = work / "hdmap_catkin"
    legacy.mkdir(exist_ok=True)
    (legacy / "hdmapConfig.cmake").write_text(f'''
set(hdmap_FOUND TRUE)
set(hdmap_INCLUDE_DIRS "{sdk / 'include'}")
set(hdmap_LIBRARIES "{sdk / 'lib/libhdmap_server.so'}")
''')
    run(["cmake", "-S", str(pnc), "-B", str(work / "build_catkin_export"), "-Dcatkin_DIR=" + str(fake),
         "-Dhdmap_DIR=" + str(legacy), "-DCMAKE_BUILD_TYPE=Release",
         "-DCMAKE_CXX_FLAGS=-include cstdint -include array"], work / "catkin_export_configure.log")
    run(["cmake", "--build", str(work / "build_catkin_export"), "--target", "perception_msg_convert", "-j2"],
        work / "catkin_export_build.log")
    links = run(["ldd", str(work / "build_catkin_export/perception_msg_convert")], work / "catkin_export_links.log")
    assert str(sdk / "lib/libhdmap_server.so") in links, links
    summary = {"cmake_target": "perception_msg_convert", "language": "C++11", "transport": "ROS stubs",
               "integration": result.splitlines()[-1], "sdk": str(sdk), "dynamic_link": "PASS",
               "catkin_export_variables": "PASS", "package_dependencies": "PASS", "rebuild_order": "PASS"}
    (work / "result.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print("PASS: PNC CMake target, SDK/legacy export dynamic link, package dependencies and rebuild order")


if __name__ == "__main__":
    main()
