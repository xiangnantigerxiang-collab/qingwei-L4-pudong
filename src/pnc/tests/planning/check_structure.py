#!/usr/bin/env python3
"""Audit retained methods, object layout declarations and the single-TU include tree."""

import re
from pathlib import Path


def tokens(source):
    pattern = r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[A-Za-z_]\w*|\d+(?:\.\d*)?(?:[eE][+-]?\d+)?|::|->|\+\+|--|&&|\|\||<<|>>|<=|>=|==|!=|\+=|-=|\*=|/=|\S'
    return [token for token in re.findall(pattern, source) if not token.startswith(("//", "/*"))]


def methods(root):
    result = {}
    for path in list(root.rglob("*.inc")) + [root / "path_plan_comply.cpp"]:
        for match in re.finditer(r'^[^\n]*PathPlanComply::(\w+)\([^;]*?\)\s*\{.*?^\}', path.read_text(), re.M | re.S):
            result.setdefault(match[1], []).append(match[0])
    return result


def check(before, after):
    old_root = before / "src/robot_path_plan"
    new_root = after / "src/robot_path_plan"
    old = methods(old_root)
    new = methods(new_root)
    assert not old.keys() - new.keys(), "An existing method was removed"
    extracted = {"calcuGlobalPath", "SetTaskPlanData", "LimitSpeedByDistanceToStop", "PublishReferPath", "PublishPlanPath"}
    unchanged = []
    for name in old:
        if name not in extracted:
            assert [tokens(code) for code in old[name]] == [tokens(code) for code in new[name]], "Moved method changed: " + name
            unchanged.extend([name] * len(old[name]))

    old_header = (old_root / "path_plan_comply.h").read_text()
    new_header = (new_root / "path_plan_comply.h").read_text()
    for name in new.keys() - old.keys():
        pattern = r'^\s*(?:void|float|bool|std::vector<robot::object>)\s+' + name + r'\([^;]*?\);\n'
        new_header, count = re.subn(pattern, "\n", new_header, flags=re.M)
        assert count == 1, "Missing/duplicate helper declaration: " + name
    assert tokens(old_header) == tokens(new_header), "Existing declarations or member layout changed"

    old_cpp = (old_root / "path_plan_comply.cpp").read_text()
    geometry = old_cpp[old_cpp.index("using Dot"):old_cpp.index("// 下列职责分片")]
    new_geometry = (new_root / "path_plan_geometry.inc").read_text()
    assert tokens(geometry) == tokens(new_geometry[:new_geometry.index("bool PathPlanComply::")]), "Rectangle helpers changed"
    callback = r'void CommandMsgCallBack\([^;]*?\)\s*\{.*?^\}'
    old_node = re.sub(callback, '', (old_root / "path_plan_node.cpp").read_text(), flags=re.M | re.S)
    new_node = re.sub(callback, '', (new_root / "path_plan_node.cpp").read_text(), flags=re.M | re.S)
    assert tokens(old_node) == tokens(new_node), "Node wiring, timing or other callbacks changed"

    visited = set()

    def visit(path):
        for include in re.findall(r'^#include "([^"]+\.inc)"', path.read_text(), re.M):
            child = path.parent / include
            assert child.is_file(), "Missing include: " + str(child)
            assert child not in visited, "Repeated include: " + str(child)
            visited.add(child)
            visit(child)

    visit(new_root / "path_plan_comply.cpp")
    assert visited == set(new_root.rglob("*.inc")), "An implementation fragment is not compiled"
    assert '.inc\n' not in re.sub(r'#[^\n]*', '', (after / "CMakeLists.txt").read_text()), "Fragment listed as a translation unit"
    for path in (before / "msg").glob("*.msg"):
        assert path.read_bytes() == (after / "msg" / path.name).read_bytes(), "Message contract changed"
    print("PASS:", len(unchanged), "retained methods have identical tokens;",
          len(visited), "fragments included exactly once; existing declarations and messages unchanged")


def check_format_only(before, after):
    # 与本次格式化前快照对照，覆盖所有自有源码及预处理指令，独立于重构阶段的函数白名单。
    import sys
    sys.path.insert(0, str(after / "tools"))
    from format_cpp import source_files, directives
    old_files = {path.relative_to(before) for path in source_files(before)}
    new_files = {path.relative_to(after) for path in source_files(after)}
    assert old_files == new_files, "A C/C++ source was added or removed"
    for relative in sorted(old_files):
        old = (before / relative).read_text()
        new = (after / relative).read_text()
        assert tokens(old) == tokens(new), "Code tokens changed: " + str(relative)
        assert directives(old) == directives(new), "Preprocessor directives changed: " + str(relative)
    for path in (before / "msg").glob("*.msg"):
        assert path.read_bytes() == (after / "msg" / path.name).read_bytes(), "Message changed"
    print("PASS:", len(old_files), "C/C++ files have identical code tokens and directives; messages unchanged")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    parser.add_argument("--format-only", action="store_true")
    args = parser.parse_args()
    if args.format_only:
        check_format_only(args.before, args.after)
    else:
        check(args.before, args.after)
