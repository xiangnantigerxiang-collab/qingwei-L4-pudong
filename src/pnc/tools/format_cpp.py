#!/usr/bin/env python3
"""统一 PNC 自有 C/C++ 的大括号风格，并保持有效 token 和预处理指令不变。"""

import argparse
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


# 注释和字面量整体识别，避免把其中的 {}、//、转义换行当作代码。
LEXER = re.compile(
    r'//[^\n]*|/\*[\s\S]*?\*/|(?:u8|u|U|L)?R"([^\s()\\]{0,16})\([\s\S]*?\)\1"'
    r'|(?:u8|u|U|L)?"(?:\\[\s\S]|[^"\\])*"'
    r"|(?:u8|u|U|L)?'(?:\\[\s\S]|[^'\\])*'"
    r'|[A-Za-z_]\w*|\d+(?:\.\d*)?(?:[eE][+-]?\d+)?'
    r'|::|->\*|->|\.\*|\.\.\.|\+\+|--|&&|\|\||<<=|>>=|<<|>>'
    r'|<=|>=|==|!=|\+=|-=|\*=|/=|%=|&=|\|=|\^=|##|\S'
)


def code_matches(source):
    return [match for match in LEXER.finditer(source)
            if not match.group().startswith(("//", "/*"))]


def tokens(source):
    # C/C++ 在词法分析前先处理反斜杠续行；不能将续行作为业务 token。
    return [match.group() for match in code_matches(re.sub(r"\\\r?\n", "", source))]


def directives(source):
    joined = re.sub(r"\\\r?\n", "", source)
    # 逐条比较指令，防止 #define/#if 的换行边界或 include 顺序发生变化。
    return [tokens(line) for line in joined.splitlines() if line.lstrip().startswith("#")]


def source_files(root):
    extensions = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".inc", ".ipp", ".tpp"}
    return sorted(path for folder in ("src", "include/common", "tests")
                  for path in (root / folder).rglob("*")
                  if path.is_file() and path.suffix in extensions)


def attach_braces(source):
    matches = code_matches(source)
    edits = []
    for index, match in enumerate(matches):
        if match.group() != "{" or index == 0:
            continue
        start = source.rfind("\n", 0, match.start()) + 1
        if source[start:match.start()].strip():
            continue
        previous = matches[index - 1]
        # 分号、case 标签或完整语句后面的 {} 是独立局部作用域，应保留独立行。
        if previous.group() in (";", "{", "}", ":", "(", ","):
            continue
        previous_start = source.rfind("\n", 0, previous.start()) + 1
        between = source[previous.end():match.start()]
        # 不能跨预处理分支挪动括号，也不改多行宏的结构。
        if "#" in source[previous_start:match.start()] or "\\" in between:
            continue
        # if(cond) // 说明 + 下一行 {：将 { 放到注释前，保留注释内容。
        edits.append((previous.end(), previous.end(), " {"))
        end = source.find("\n", match.end())
        if end < 0:
            end = len(source)
        if not source[match.end():end].strip():
            edits.append((start, min(end + 1, len(source)), ""))
        else:
            edits.append((match.start(), match.end(), ""))
    for start, end, replacement in sorted(edits, reverse=True):
        source = source[:start] + replacement + source[end:]
    return source


def format_source(path, formatter):
    original = path.read_text()
    prepared = attach_braces(original)
    source = prepared.encode("utf-8")
    assumed = path.with_suffix(".cpp") if path.suffix == ".inc" else path
    result = subprocess.run(
        [formatter, "--style=file", "--assume-filename=" + str(assumed),
         "--output-replacements-xml"], input=source, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, check=True)
    tree = ET.fromstring(result.stdout)
    if tree.attrib.get("incomplete_format") == "true":
        raise RuntimeError("clang-format 未完整解析: " + str(path))
    replacements = []
    for item in tree.findall("replacement"):
        start, length = int(item.attrib["offset"]), int(item.attrib["length"])
        old = source[start:start + length]
        new = (item.text or "").encode("utf-8")
        # 保留旧代码的续行符，不让格式化顺带清理它们。
        if old.strip() != new.strip():
            if old.strip() == b"\\" and not new.strip():
                continue
            raise RuntimeError("格式化试图改写非空白内容: " + str(path))
        # C++11 允许 > > 合并为 >>；本次仍保留原始 token 边界，便于严格比较。
        if old and not new and source[start - 1:start] == b">" and source[start + length:start + length + 1] == b">":
            new = b" "
        replacements.append((start, start + length, new))
    for start, end, replacement in sorted(replacements, reverse=True):
        source = source[:start] + replacement + source[end:]
    formatted = source.decode("utf-8")
    if tokens(original) != tokens(formatted) or directives(original) != directives(formatted):
        raise RuntimeError("格式化前后代码或预处理指令不等价: " + str(path))
    return formatted


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--clang-format", default="clang-format")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    formatter = shutil.which(args.clang_format)
    if not formatter:
        parser.error("需要 clang-format 18 或兼容版本，请通过 --clang-format 指定路径")
    changed = []
    files = source_files(args.root.resolve())
    for path in files:
        formatted = format_source(path, formatter)
        if formatted != path.read_text():
            changed.append(path)
            if not args.check:
                path.write_text(formatted)
    print("检查", len(files), "个源码文件；", "待格式化" if args.check else "已格式化", len(changed), "个")
    if args.check and changed:
        for path in changed:
            print(path)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
