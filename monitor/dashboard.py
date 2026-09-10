# -*- coding: utf-8 -*-
"""
dashboard - 仪表板子服务(默认 8082,只读)
=====================================================

由 monitor_server.main() 在独立端口拉起:与主服务同进程、同一 rospy 节点,
复用 RosVisualizer 的订阅与 stash。展示 /can_msg 与 /ehb_msg 全部字段:

  GET /                static/dashboard.html(前端 2Hz 轮询)
  GET /api/dashboard   JSON 快照(字段值+中文名称+注释+枚举译码)

中文名称对齐策略(用户需求:与 ehb_msg 中文注释对齐):
  /ehb_msg  启动时运行时解析 ../src/canbus/msg/ehb_msg.msg,中文名称与
            注释直接取自该文件——构造性对齐,后续改 .msg 重启即自动跟上
  /can_msg  can_msg.msg 本身无中文注释 -> 采用本文件 CAN_LABELS 的
            monitor 侧命名(语义出处 src/canbus/README.md;改字段需同步)

展示瘦身(2026-09-07 用户需求,payload 即瘦身后的形态):
  - 隐藏字段:CAN_HIDDEN/EHB_HIDDEN(原始帧 hex/恒 0 死字段/RC/CS 机制字段)
  - 故障位折叠:35 个 *_FAU_* bool 聚合为分节首行汇总(全部正常/N 项点名;
    无数据时 value=None,不得假'全部正常')
  - 注释只留换算/单位(ehb=_conv_note,can=_can_unit_note);
    枚举译码仍用 .msg 原注释——先译码后裁剪,顺序不可倒
  - can 枚举译码=CAN_ENUMS 显式表;VHL_VehicleGear 按 _decode_gear 出字符
"""

import json
import os
import re
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MON_DIR = os.path.dirname(os.path.abspath(__file__))
MSG_DIR = os.path.normpath(os.path.join(MON_DIR, "..", "src", "canbus", "msg"))

VERSION = "1.1.0"

# ---------------------------------------------------------------------------
# .msg 解析:字段行 -> (type, name, label(中文名称), note(注释原文))
# ---------------------------------------------------------------------------

# 注释头部可剥离的前缀词:0x 报文 ID、"1.1-1.2" 位号
_HEX_RE = re.compile(r"^0[xX][0-9A-Fa-f]+$")
_POS_RE = re.compile(r"^\d+\.\d+(-\d+\.\d+)?$")
_ENUM_RE = re.compile(r"^\d+[:：]")          # "00:" 形态的枚举项
# 枚举键形态(ehb_msg.msg 实存三种):hex "0x0:"、二进制等宽 "00:"、纯十进制 "0:"
_ENUM_PAIR_RE = re.compile(
    r"^((?:0[xX][0-9A-Fa-f]{1,2})|\d{1,8})[:：](.*)$")


def _has_cjk(text):
    return any("一" <= ch <= "鿿" for ch in text)


def _label_from_tokens(tokens):
    """注释词序列 -> 中文名称。

    规则(按 ehb_msg.msg 现行注释形态逐一验算):
      - 命中枚举项(00:xx)/0x 开头/raw 开头/× 开头/无汉字 -> 终止
      - 词内含 "(" 时取 "(" 前部分(如 "液压传感器1原始压力值(raw×0.1")
      - 汉字词拼接("车辆当前档位" 这类多词名称仅一例,单词即止)
    """
    out = []
    for tok in tokens:
        if _ENUM_RE.match(tok):
            break
        head = tok.split("(", 1)[0]
        if head.startswith("0x") or head.startswith("raw") \
                or head.startswith("×"):
            break
        if not _has_cjk(head):
            break
        out.append(head)
        if "(" in tok:
            break
    return "".join(out)


def _read_text(path):
    """读 .msg 文本:utf-8 优先,GBK 兜底(工程内确有 GBK 文件,如 auto_couple)。

    两者都失败(errors="replace" 只会置换坏字节)不会抛出——彻底读不了的
    场景由调用方按 OSError/异常降级到 msg_load_errors。
    """
    try:
        with open(path, "r", encoding="utf-8") as f:
            return f.read()
    except UnicodeDecodeError:
        with open(path, "r", encoding="gbk", errors="replace") as f:
            return f.read()


def parse_msg_spec(path):
    """解析 .msg -> {"header": [注释行], "sections": [
        {"title": None|str, "fields": [{"type","name","label","note"}]}]}

    "# ---- 帧1 ... ----" 分节注释成为小节标题;字段行取 "类型 名称 # 注释"。
    """
    sections = [{"title": None, "fields": []}]
    header = []
    for line in _read_text(path).splitlines():
        code = line.split("#", 1)[0].strip()
        cmt = line.split("#", 1)[1].strip() if "#" in line else ""
        if code:
            parts = code.split()
            if len(parts) == 2:
                field = {"type": parts[0], "name": parts[1],
                         "label": None, "note": cmt}
                toks = cmt.split()
                while toks and (_HEX_RE.match(toks[0])
                                or _POS_RE.match(toks[0])):
                    toks.pop(0)
                field["label"] = (_label_from_tokens(toks)
                                  or " ".join(toks) or None)   # 英文注释(如 CAN BUS OFF)回退取注释原文
                sections[-1]["fields"].append(field)
            continue
        if not cmt:
            continue
        if cmt.startswith("----"):
            title = cmt.strip("- ").strip()
            sections.append({"title": title, "fields": []})
        elif not sections[-1]["fields"] and sections[-1]["title"] is None:
            header.append(cmt)
    return {"header": header, "sections": sections}


# ---------------------------------------------------------------------------
# can_msg 中文名称(monitor 侧命名;can_msg.msg 无中文注释)
# ---------------------------------------------------------------------------

CAN_LABELS = {
    "throttlePercent":    "油门指令(%)",
    "brakePercent":       "刹车指令(%)",
    "wheelAngle":         "转向轮角度(°)",
    "vehicleSpeed":       "车速(m/s)",
    "curGear":            "当前挡位(2=N,3=R,4=D)",
    "hookState":          "钩销挂接状态",
    "linkPallet":         "鞍座连托状态",
    "controlPanelState":  "控制面板(0=手动,1=自动)",
    "eabPanelState":      "急停面板状态",
    "emergencyStop":      "急停(0=正常,1=急停)",
    "batteryPower":       "电池电量(%)",
    "hookButton":         "钩销按钮(0=停,1=升,2=降)",
    "linkButton":         "鞍座按钮(0=停,1=升,2=降)",
    "faultCode":          "故障码",
    "epsCMD":             "EPS转向指令",
    "wheelAngleCMD":      "EPS转向指令角(°)",
    "epsMode":            "EPS转向模式",
    "epsCurrent":         "EPS转向电流(A)",
    "epsCentring":        "EPS对中状态",
    "epsERR1":            "EPS故障码1(现恒0)",
    "epsERR2":            "EPS故障码2(现恒0)",
    "hookPos":            "钩销位置码",
    "palletPos":          "鞍座位置码",
    "hookStatus":         "钩销状态机(0=移动,1=堵,3=底,4=顶)",
    "palletStatus":       "鞍座状态机(0=移动,3=底,4=顶)",
    "rawcommand":         "0x185原始反馈帧(hex)",
    "rawfeedback":        "0x0C02A0A2原始反馈帧(hex)",
}


def _apply_can_labels(spec):
    """CAN_LABELS 写回 spec:label=括号前名称,note=括号段(与 ehb 展示一致)。"""
    for sec in spec["sections"]:
        for f in sec["fields"]:
            text = CAN_LABELS.get(f["name"])
            if text:
                name, sep, detail = text.partition("(")
                f["label"] = name
                f["note"] = ("(" + detail) if sep else ""
    return spec


# ---------------------------------------------------------------------------
# 展示瘦身(2026-09-07 用户需求:去垃圾信息/故障位折叠/注释只留换算)
# ---------------------------------------------------------------------------

# 不进 payload 的字段:原始帧 hex / 恒 0 死字段 / 协议机制字段
CAN_HIDDEN = {"rawcommand", "rawfeedback",   # 0x185/0x0C02A0A2 原始帧 hex
              "epsERR1", "epsERR2",          # 悬空恒 0
              "faultCode"}                   # 写路径 09-03 已下线,恒 0
EHB_HIDDEN = {"VHL_ATB_RollingCounter", "VHL_ATB_CheckSum",
              "VHL_EPB_RollingCounter", "VHL_EPB_CheckSum"}

# can_msg 枚举译码:CAN_LABELS 括注是 "0=停" 形态,_decode_enum 只认冒号键,
# 显式映射零误译(改 can_msg 字段语义时同步本表)
CAN_ENUMS = {
    "curGear":           {2: "N", 3: "R", 4: "D"},
    "controlPanelState": {0: "手动", 1: "自动"},
    "emergencyStop":     {0: "正常", 1: "急停"},
    "hookButton":        {0: "停", 1: "升", 2: "降"},
    "linkButton":        {0: "停", 1: "升", 2: "降"},
    "hookStatus":        {0: "移动", 1: "堵", 3: "底", 4: "顶"},
    "palletStatus":      {0: "移动", 3: "底", 4: "顶"},
}

# ehb 注释换算提取:"raw×0.1 MPa" / "raw×0.04=0~8 MPa"(=号后带范围+单位)
_CONV_RE = re.compile(
    r"raw×([0-9.]+)(?:=[0-9.~]+\s*([A-Za-z/%]+)|\s+([A-Za-z/%]+))")
_PCT_RE = re.compile(r"\((\d+-\d+%)")


def _conv_note(note):
    """ehb 注释 -> 只留换算说明('×0.1 MPa'/'×0.04 MPa'/'×0.1 km/h'/'0-100%')。

    位号/枚举表/工程注记(I02~I04/0xFE 无效等)一律丢弃;无换算返回 ""。
    枚举译码用 spec 原注释(先于裁剪),不受本函数影响。
    """
    if not note:
        return ""
    m = _CONV_RE.search(note)
    if m:
        unit = (m.group(2) or m.group(3) or "").strip()
        return "×%s %s" % (m.group(1), unit)
    m = _PCT_RE.search(note)
    return m.group(1) if m else ""


def _can_unit_note(note):
    """can 括注 -> 单位('m/s'/'%'/'°'/'A');含 '=' 的枚举说明丢弃(已译码)。"""
    if not note or "=" in note or "0x" in note:
        return ""
    return note.strip("()") or ""


def _decode_gear(v):
    """VHL_VehicleGear(uint16 小端原始拼接,摆放未定义 I09) -> 档位字符。

    双字节各取可打印 A~Z,恰一个字母则译出;两字母并存(摆放歧义)不译。
    """
    if not isinstance(v, int) or isinstance(v, bool):
        return None
    letters = {chr(v & 0xFF), chr((v >> 8) & 0xFF)}
    letters = set(c for c in letters if "A" <= c <= "Z")
    return letters.pop() if len(letters) == 1 else None


def _short_title(title):
    """分节标题精简:去 0x ID/英文名/括注,只留含汉字的词(如'帧4 制动请求')。"""
    if not title:
        return title
    head = title.split("(", 1)[0]
    toks = [t for t in head.split() if _has_cjk(t)]
    if not toks:
        return head.strip() or title
    if toks[0].startswith("接收方向"):
        toks[0] = toks[0][len("接收方向"):]
    return " ".join(t for t in toks if t).strip() or title


# ---------------------------------------------------------------------------
# 值格式化与枚举译码
# ---------------------------------------------------------------------------

def _fmt_value(v):
    """消息字段值 -> JSON 安全标量(数组转 hex 串,NaN/Inf 归 0)。

    uint8[] 在 rospy(Python3) 下反序列化为 bytes(非 list),bytes/
    bytearray/memoryview 一律按字节转 hex。
    """
    if v is None:
        return None
    if isinstance(v, bool):
        return 1 if v else 0
    if isinstance(v, memoryview):
        v = v.tobytes()
    if isinstance(v, (bytes, bytearray)):
        return " ".join("%02X" % (b & 0xFF) for b in v[:64])
    if isinstance(v, (list, tuple)):
        items = list(v)[:64]
        return " ".join("%02X" % (x & 0xFF) for x in items)
    if isinstance(v, float):
        if v != v or v in (float("inf"), float("-inf")):
            return 0.0
        return round(v, 3)
    if isinstance(v, int):
        return v
    return str(v)


def _decode_enum(value, note):
    """按注释中的枚举表把原始值译成中文(best-effort,失败返回 None)。

    支持三种键形态(ehb_msg 注释实存):
      十六进制键 "0x0:已夹紧 0x1:已释放 ..." —— 按 int(hex) 匹配
      二进制等宽键 "00:加压关 01:加压开 ..." —— 全部等宽 w,按 %0<w>b 匹配
        (必须先于十进制,否则 "10" 会被当作十进制 10)
      纯十进制键 "0:无请求 1:请求驻车 2:请求释放 3:无效" —— 按 int 匹配
    冒号后带空格的写法("0x0: 正常"/"0x2:  夹紧中")先归一为紧贴;
    译码文本截去 "(" 与 ";""," 后缀(如 "无效(AC 列..."/"忽略;64768-03");
    "0x4~0x6:未知" 这类区间写法不匹配键正则,自动跳过。
    """
    if not note or not isinstance(value, int) or isinstance(value, bool):
        return None
    work = re.sub(r"[:：]\s+", ":", note)
    pairs = []
    for tok in work.split():
        m = _ENUM_PAIR_RE.match(tok)
        if not m:
            if pairs:
                break               # 枚举表被普通词打断,后续不再收
            continue
        text = m.group(2)
        for cut in ("(", ";", "；", ",", "，"):
            text = text.split(cut, 1)[0]
        text = text.strip()
        if text:
            pairs.append((m.group(1), text))
    if len(pairs) < 2:
        return None
    keys = [k for k, _ in pairs]
    if all(k[:2].lower() == "0x" for k in keys):
        for k, text in pairs:
            try:
                if int(k, 16) == value:
                    return text
            except ValueError:
                return None
        return None
    widths = {len(k) for k in keys}
    if len(widths) == 1 and all(set(k) <= {"0", "1"} for k in keys):
        w = widths.pop()
        if value < (1 << w):
            want = format(value, "0%db" % w)
            for k, text in pairs:
                if k == want:
                    return text
        return None
    if all(k.isdigit() for k in keys):     # 纯十进制键(VHL_EPB_ParkingRequest 形态)
        for k, text in pairs:
            if int(k, 10) == value:
                return text
    return None


# ---------------------------------------------------------------------------
# 应用聚合与 HTTP
# ---------------------------------------------------------------------------

class DashboardApp(object):

    def __init__(self, viz, msg_dir=None):
        self._viz = viz
        self._msg_dir = msg_dir or MSG_DIR
        self._specs = {}          # topic -> spec
        self._load_errors = []
        for topic, fname in (("/can_msg", "can_msg.msg"),
                             ("/ehb_msg", "ehb_msg.msg")):
            path = os.path.join(self._msg_dir, fname)
            try:
                spec = parse_msg_spec(path)
                if topic == "/can_msg":
                    _apply_can_labels(spec)
                self._specs[topic] = spec
            except Exception as exc:      # 缺文件/坏编码/解析异常一律降级,勿炸主服务
                self._load_errors.append("%s: %s" % (fname, exc))
        if self._load_errors:
            try:                          # stderr 断开时容忍(对标 ros_visualizer._safe_print)
                sys.stderr.write("[MONITOR] dashboard .msg 解析失败(降级为无字段): %s\n"
                                 % "; ".join(self._load_errors))
            except Exception:
                pass

    def _sections_payload(self, topic, msg):
        spec = self._specs.get(topic)
        if spec is None:
            return None
        hidden = CAN_HIDDEN if topic == "/can_msg" else EHB_HIDDEN
        is_can = topic == "/can_msg"
        sections = []
        for idx, sec in enumerate(spec["sections"]):
            if not sec["fields"]:
                continue
            rows = []
            faults = []          # [(label, raw)] —— bool 故障位折叠(raw 三态)
            for f in sec["fields"]:
                if f["name"] in hidden:
                    continue
                if f["type"] == "bool" and "_FAU_" in f["name"]:
                    # 属性缺失=None(位不可读),≠False(未激活)——
                    # 与真实 catkin 类"全字段必有"对齐,缺失只出现在
                    # .msg 领先于编译的窗口,该态不得计入"全部正常"
                    raw = (getattr(msg, f["name"], None)
                           if msg is not None else None)
                    faults.append((f["label"] or f["name"], raw))
                    continue
                row = {"name": f["name"], "label": f["label"] or f["name"],
                       "note": (_can_unit_note(f["note"]) if is_can
                                else _conv_note(f["note"])),
                       "value": None, "text": None}
                if msg is not None:
                    try:
                        v = _fmt_value(getattr(msg, f["name"], None))
                    except Exception:
                        # 单坏字段降级字符串,勿打死整个 /api/dashboard
                        v = str(getattr(msg, f["name"], None))[:256]
                    row["value"] = v
                    if is_can:
                        emap = CAN_ENUMS.get(f["name"])
                        if emap is not None and isinstance(v, int) \
                                and not isinstance(v, bool):
                            row["text"] = emap.get(v)
                    else:
                        if f["type"] == "uint8":
                            row["text"] = _decode_enum(v, f["note"])
                        elif f["name"] == "VHL_VehicleGear":
                            row["text"] = _decode_gear(v)
                rows.append(row)
            if faults:
                # 无数据(msg 缺或任一位属性不可读)时 value/text=None:
                # 不得在不可读的位上显示"全部正常"
                if any(v is None for _, v in faults):
                    sumrow = {"name": "__faults_%d" % idx, "label": "故障位",
                              "note": "", "value": None, "text": None}
                else:
                    names = [lab for lab, v in faults if v]
                    sumrow = {"name": "__faults_%d" % idx, "label": "故障位",
                              "note": "", "value": len(names),
                              "text": "、".join(names) if names else "全部正常"}
                rows.insert(0, sumrow)
            sections.append({"title": _short_title(sec["title"]),
                             "fields": rows})
        return sections

    def payload(self):
        now = time.monotonic()
        topics = {}
        for topic in ("/can_msg", "/ehb_msg"):
            ent = self._viz.latest(topic)
            msg, age = ent if ent else (None, None)
            rec = {
                "age": round(now - age, 1) if age is not None else None,
                "sections": self._sections_payload(topic, msg),
            }
            topics[topic] = rec
        return {
            "ok": True,
            "version": VERSION,
            "server_time": round(time.time(), 3),
            "ros_available": self._viz.ros_available(),
            "master_ok": self._viz.master_ok(),
            "msg_load_errors": self._load_errors,
            "topics": topics,
        }


def make_dashboard_handler(app):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "qingweiDASH/" + VERSION
        timeout = 30

        def log_message(self, fmt, *args):
            pass

        def _json(self, code, obj):
            body = json.dumps(obj, ensure_ascii=False,
                              allow_nan=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type",
                             "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _bytes(self, body, ctype, cache):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", cache)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            try:
                self._do_get()
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _do_get(self):
            from urllib.parse import urlparse
            p = urlparse(self.path).path

            if p in ("/", "/index.html", "/dashboard"):
                page = os.path.join(MON_DIR, "static", "dashboard.html")
                try:
                    with open(page, "rb") as f:
                        body = f.read()
                except OSError:
                    self._json(404, {"ok": False,
                                     "error": "缺少 static/dashboard.html"})
                    return
                self._bytes(body, "text/html; charset=utf-8", "no-store")
                return

            if p == "/api/dashboard":
                self._json(200, app.payload())
                return

            if p == "/favicon.ico":
                self.send_response(204)
                self.end_headers()
                return

            self._json(404, {"ok": False, "error": "unknown path " + p})

    return Handler
