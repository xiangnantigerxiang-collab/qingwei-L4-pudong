// -*- coding: utf-8 -*-
// 前端无头渲染测试:DOM 桩 + fetch 桩,驱动真实 index.html 的脚本,
// 断言渲染产物(DOM 文本/类名/横幅/按钮态/交互回调 URL)。
// 用法:node tests/frontend_test.js

"use strict";
const fs = require("fs");
const path = require("path");

const HMI_DIR = path.join(__dirname, "..");
const html = fs.readFileSync(path.join(HMI_DIR, "static", "index.html"), "utf8");
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

let PASS = 0;
const FAILS = [];
function check(name, cond, detail) {
  if (cond) { PASS++; console.log("  ✓ " + name); }
  else { FAILS.push(name); console.log("  ✗ " + name + "  " + (detail || "")); }
}

// ---------------- DOM 桩 ----------------
class El {
  constructor(key) {
    this.key = key;
    this._text = ""; this._html = "";
    this.className = ""; this.style = {}; this.disabled = false;
    this.checked = true; this.scrollTop = 0;
    this.scrollHeight = 100; this.clientHeight = 100;
    this.href = ""; this._ls = {}; this._attrs = {}; this._first = null;
  }
  get classList() {
    const self = this;
    const arr = () => self.className.split(/\s+/).filter(Boolean);
    return {
      add(...cs) { const s = new Set(arr()); cs.forEach(c => s.add(c)); self.className = [...s].join(" "); },
      remove(...cs) { const s = new Set(arr()); cs.forEach(c => s.delete(c)); self.className = [...s].join(" "); },
      toggle(c, f) { const s = new Set(arr()); if (f === undefined) f = !s.has(c); f ? s.add(c) : s.delete(c); self.className = [...s].join(" "); return f; },
      contains(c) { return arr().includes(c); },
    };
  }
  set textContent(v) { this._text = String(v); }
  get textContent() { return this._text; }
  set innerHTML(v) { this._html = String(v); }
  get innerHTML() { return this._html; }
  get firstElementChild() { if (!this._first) this._first = new El(this.key + ":first"); return this._first; }
  addEventListener(t, f) { (this._ls[t] = this._ls[t] || []).push(f); }
  fire(t, ev) { (this._ls[t] || []).forEach(f => f(ev || {})); }
  setAttribute(k, v) { this._attrs[k] = v; }
  getAttribute(k) { return this._attrs[k]; }
  closest() { return null; }
}
const registry = new Map();
function el(key) { if (!registry.has(key)) registry.set(key, new El(key)); return registry.get(key); }

global.document = {
  querySelector: s => el(s),
  getElementById: id => el("#" + id),
  addEventListener() {},
  hidden: false,
};
global.window = { confirm: () => window._confirmResult !== false };

// ---------------- fetch 桩 ----------------
let currentState = null;
let logResponse = { lines: ["L1", "L2", "L3"] };
let calibResponse = null;   // /api/calibrate 的应答桩(null=回落 currentState)
let rejectFetch = false;
const fetchCalls = [];
global.fetch = function (url, opts) {
  fetchCalls.push({ url, opts });
  if (rejectFetch) return Promise.reject(new Error("offline"));
  let payload = String(url).indexOf("/api/logs") >= 0 ? logResponse : currentState;
  if (String(url).indexOf("/api/calibrate") >= 0 && calibResponse)
    payload = calibResponse;
  return Promise.resolve({ ok: true, json: async () => payload });
};

// ---------------- 状态构造 ----------------
function comp(o) {
  return Object.assign({
    name: "x", title: "组件", group: 0, group_title: "组", state: "STOPPED",
    uptime_s: null, pid: null, exit_code: null, restarts: 0, log_size: 0,
    log_file: null, health: null, health_detail: "", optional: false,
    enabled: true, foreign: false, stop_failed: false, last_error: null,
  }, o);
}
function mkst(o) {
  return Object.assign({
    server: { version: "1.0.0", started_at: 1756000000, ros_available: true,
              master_ok: true, msg_import_errors: {} },
    sequence: { active: false, action: null, current_group: null,
                failed: [], started_at: null, note: null },
    components: [],
    recording_config: { enabled: false, ok: true, error: null },
    vehicle: { ros_available: false, reason: "无 ROS 环境" },
    system: { cpu_pct: 10, mem_pct: 30, mem_total_gb: 16, cpu_temp_c: 45,
              disk_pct: 40, disk_free_gb: 50, disk_warn: false, load1: 1, sys_uptime_s: 100 },
  }, o);
}
function veh(o) {
  return Object.assign({
    ros_available: true, master_ok: true,
    speed_kmh: 3.5, gear: "D", gear_raw: 4, mode: "自动", emergency_stop: false,
    driving_state: 3, driving_state_text: "巡航",
    task: { status: 1, status_text: "执行中", task_id: 88, type: 1,
            stop_xy_m: [3.5, -2], procedure: 1, fail_text: null },
    battery_pct: 77,
    localization: { x_m: 12.3, y_m: -5.6, heading_deg: 91.2, rtk_raw: 0,
                    rtk_text: "RTK固定" },
    obstacle: { raw: 120, text: "无风险", level: "ok" },
    distance_to_stop_m: 25, lateral_dev_m: 0.2, lateral_dev_warn: false,
    hook: { text: "已挂钩", state: 4, v2n: 4, center_distance: 1.8, beta: 3.1 },
    sensors: { sensorstate: 0, fault: false, lidar: true, camera: true,
               gnss: true, alive: true, vehicle_ok: true },
    can: { hz: 50, fault_codes: [], fault: false },
    net: { internet_ok: true, alarm: 0 },
    status_str: "RUNNING",
    ages: { "/can_msg": 0.2, "/navigation_msg": 0.3, "/path_plan_status": 0.5,
            "/control_msg": 0.2, "/task_plan_msg": 1, "/cloud/task/task_status": 1,
            "/v2nHeartBeat": 0.4, "/hook_position": 0.6 },
  }, o);
}

const sleep = ms => new Promise(r => setTimeout(r, ms));
async function apply(st) { currentState = st; await sleep(1150); }

// ---------------- 载入被测脚本 ----------------
eval(script);
(async function main() {
  console.log("=" .repeat(60));
  console.log("前端无头渲染测试(Node DOM 桩)");
  console.log("=".repeat(60));

  // S1 无 ROS 初始态
  await apply(mkst({ components: [comp({ name: "roscore", title: "ROS 核心" })],
                     server: Object.assign(mkst({}).server,
                                           { ros_available: false }) }));
  check("S1 无ROS横幅", el("#banners").innerHTML.indexOf("无 ROS 环境") >= 0);
  check("S1 总状态=未运行", el("#overall").textContent === "未运行");
  check("S1 车速占位 --", el("#v-speed").textContent === "--");
  check("S1 模式pill无数据灰删", el("#modePill").className.indexOf("m-none") >= 0
        && el("#modePill").textContent.indexOf("无数据") >= 0);

  // S2 全部运行
  await apply(mkst({ components: [
    comp({ name: "roscore", state: "RUNNING", uptime_s: 125 }),
    comp({ name: "pnc", state: "RUNNING", uptime_s: 62 }),
  ] }));
  check("S2 总状态=全部运行(绿)", el("#overall").textContent === "全部运行"
        && el("#overall").className.indexOf("p-good") >= 0);
  check("S2 卡片类名 st-RUNNING", el("#components").innerHTML.indexOf("st-RUNNING") >= 0);
  check("S2 运行卡启动钮禁用", el("#components").innerHTML.indexOf("disabled") >= 0);
  check("S2 uptime 显示(分)", el("#components").innerHTML.indexOf("分钟") >= 0);

  // S3 故障卡
  await apply(mkst({ components: [
    comp({ name: "roscore", state: "RUNNING" }),
    comp({ name: "camera", state: "CRASHED", last_error: "异常退出:返回码 1",
           exit_code: 1 }),
  ] }));
  check("S3 总状态含故障(红)", el("#overall").textContent.indexOf("故障") >= 0
        && el("#overall").className.indexOf("p-crit") >= 0);
  check("S3 卡片 st-CRASHED 与错误文案",
        el("#components").innerHTML.indexOf("st-CRASHED") >= 0
        && el("#components").innerHTML.indexOf("异常退出") >= 0);

  // S4 车辆面板渲染
  await apply(mkst({ components: [comp({ state: "RUNNING" })],
                     vehicle: veh({ emergency_stop: true, battery_pct: 15,
                                    lateral_dev_m: 6.0, lateral_dev_warn: true,
                                    obstacle: { raw: 5, text: "5.0 m", level: "danger" },
                                    task: { status: 1, status_text: "执行中",
                                            task_id: 88, type: 1, stop_xy_m: [3, -2],
                                            procedure: 1,
                                            fail_text: "路径异常(5)" } }) }));
  check("S4 车速 3.5", el("#v-speed").textContent === "3.5");
  check("S4 挡位 D", el("#v-gear").textContent === "D");
  check("S4 急停红块激活", el("#v-estopbar").className.indexOf("on") >= 0
        && el("#v-estop").textContent === "按下");
  const meter = el("#v-battmeter");
  check("S4 低电量 meter=crit 宽 15%", meter.className.indexOf("crit") >= 0
        && meter.firstElementChild.style.width === "15%");
  check("S4 横向偏差告警红", el("#v-latdev").textContent === "6 m"
        && el("#v-latdev").className.indexOf("crit") >= 0);
  check("S4 障碍物 5.0 m", el("#v-obstacle").textContent === "5.0 m");
  check("S4 任务失败文案", el("#v-taskfail").textContent.indexOf("路径异常") >= 0);
  check("S4 RTK 正常绿", el("#v-rtk").textContent === "RTK固定"
        && el("#v-rtk").className.indexOf("good") >= 0);
  check("S4 传感器全绿", el("#s-lidar").className.indexOf("ok") >= 0);

  // S5 数据过期灰化
  await apply(mkst({ components: [comp({ state: "RUNNING" })],
                     vehicle: veh({ ages: Object.assign(veh({}).ages,
                                   { "/can_msg": 9 }) }) }));
  check("S5 挡位 stale 灰化", el("#v-gear").className.indexOf("stale") >= 0);

  // S6 一键编排中:按钮禁用 + 横幅
  await apply(mkst({ sequence: { active: true, action: "start", current_group: 2,
                                 failed: [], started_at: 1, note: null },
                     components: [comp({ state: "STARTING" })] }));
  check("S6 编排横幅", el("#banners").innerHTML.indexOf("进行中") >= 0);
  check("S6 一键按钮禁用", el("#btnStart").disabled === true
        && el("#btnStop").disabled === true);

  // S7 master 失联横幅
  await apply(mkst({ components: [], server: Object.assign(
      mkst({}).server, { ros_available: true, master_ok: false }) }));
  check("S7 master 失联横幅", el("#banners").innerHTML.indexOf("master 失联") >= 0);

  // S8 磁盘告警横幅 + foreign 角标
  await apply(mkst({
    system: Object.assign(mkst({}).system, { disk_warn: true }),
    components: [comp({ name: "pnc", state: "STOPPED", foreign: true })],
  }));
  check("S8 磁盘告警横幅", el("#banners").innerHTML.indexOf("磁盘") >= 0);
  check("S8 外部进程角标", el("#components").innerHTML.indexOf("外部") >= 0
        && el("#banners").innerHTML.indexOf("外部进程") >= 0);
  check("S8 外部进程禁止重复启动/重启",
        el("#components").innerHTML.indexOf(
          'data-act="start" data-name="pnc" class="op-start" disabled') >= 0
        && el("#components").innerHTML.indexOf(
          'data-act="restart" data-name="pnc" disabled') >= 0);
  check("S8 外部进程允许单卡停止清理",
        el("#components").innerHTML.indexOf(
          'data-act="stop" data-name="pnc" class="op-stop"') >= 0
        && el("#components").innerHTML.indexOf(
          'data-act="stop" data-name="pnc" class="op-stop" disabled') < 0);

  await apply(mkst({ components: [comp({
    name: "pnc", state: "STOPPING", foreign: true,
  })] }));
  check("S8 外部进程清理中禁止重复停止",
        el("#components").innerHTML.indexOf(
          'data-act="stop" data-name="pnc" class="op-stop" disabled') >= 0);

  await apply(mkst({
    components: [],
    recording_config: {
      enabled: true, ok: false, error: "两个录制分组 <topic> 清单不一致",
    },
  }));
  check("S8 录制 topic 配置错误显示顶部告警",
        el("#banners").innerHTML.indexOf("录制 topic 配置错误") >= 0
        && el("#banners").innerHTML.indexOf("&lt;topic&gt;") >= 0
        && el("#banners").innerHTML.indexOf("<topic>") < 0);

  // S8b 手动组件停止时不影响总状态；启动失败后必须进入总告警
  await apply(mkst({ components: [comp({
    name: "perception_bags", title: "感知数据录制",
    state: "CRASHED", enabled: false,
  })] }));
  check("S8b 手动组件显示手动启动标记",
        el("#components").innerHTML.indexOf("手动启动") >= 0);
  check("S8b 手动组件故障进入顶部总状态",
        el("#overall").textContent.indexOf("1 个组件故障") >= 0,
        el("#overall").textContent);

  // S9 交互回调 URL
  const n0 = fetchCalls.length;
  el("#btnStart").fire("click");
  check("S9 一键启动 POST /api/start",
        fetchCalls[n0] && fetchCalls[n0].url === "/api/start");
  window._confirmResult = true;
  el("#btnStop").fire("click");
  const stopCall = fetchCalls[fetchCalls.length - 1];
  check("S9 全部停止带确认体", stopCall.url === "/api/stop"
        && JSON.parse(stopCall.opts.body).confirm === "STOP");
  window._confirmResult = false;
  fetchCalls.length = 0;
  el("#btnStop").fire("click");
  check("S9 confirm 取消时不发请求", fetchCalls.length === 0);
  // 组件按钮(事件委托)
  const fakeBtn = { getAttribute: k => ({ "data-act": "restart",
                                           "data-name": "pnc" }[k]) };
  el("#components").fire("click", { target: { closest: () => fakeBtn } });
  check("S9 组件操作 URL", fetchCalls.length > 0
        && fetchCalls[fetchCalls.length - 1].url === "/api/components/pnc/restart");

  // S10 日志弹窗
  const logBtn = { getAttribute: k => ({ "data-act": "log",
                                         "data-name": "pnc" }[k]) };
  el("#components").fire("click", { target: { closest: () => logBtn } });
  await sleep(50);
  check("S10 弹窗标题/下载链接", el("#logTitle").textContent.indexOf("pnc") >= 0
        && el("#logDl").href.indexOf("download=1") >= 0);
  check("S10 日志内容渲染", el("#logBody").textContent.indexOf("L3") >= 0);
  el("#logClose").fire("click");
  check("S10 关闭弹窗", el("#logModal").classList.contains("open") === false);

  // S11 掉线横幅
  rejectFetch = true;
  await sleep(1300);
  check("S11 掉线横幅", el("#banners").innerHTML.indexOf("无法连接") >= 0
        && el("#overall").textContent.indexOf("无法连接") >= 0);
  rejectFetch = false;

  // S12 页脚
  await apply(mkst({ components: [] }));
  check("S12 页脚含版本与急停提示", el("#ftInfo").textContent.indexOf("v1.0.0") >= 0
        && el("#ftInfo").textContent.indexOf("急停") >= 0);

  // S13 页头驾驶模式 pill 高亮 + 运行状态行
  // (注:S4→S5 已留有少量历史事件,故不断言"暂无异常事件";S12 后 LAST_SNAP 已置
  //  null,此处首帧重建基线,后续跳变断言不受影响)
  await apply(mkst({ components: [comp({ state: "RUNNING" })], vehicle: veh() }));
  check("S13 自动模式绿实心", el("#modePill").className.indexOf("m-auto") >= 0
        && el("#modePill").textContent === "自动");
  check("S13 运行行三段", el("#v-runstate").textContent === "执行中 · 巡航 · 自动"
        && el("#v-runstate").className.indexOf("good") >= 0);
  check("S13 CAN 正常绿", el("#f-can").textContent === "正常"
        && el("#f-can").className.indexOf("good") >= 0);
  check("S13 规划心跳格 ok", el("#s-alive").className.indexOf("ok") >= 0);
  check("S13 云端格无组件 na", el("#s-cloud").className.indexOf("na") >= 0);
  await apply(mkst({ components: [comp({ state: "RUNNING" })],
                     vehicle: veh({ mode: "手动" }) }));
  check("S13 手动黄警示", el("#modePill").className.indexOf("m-manual") >= 0
        && el("#modePill").textContent === "手动"
        && el("#v-runstate").className.indexOf("warn") >= 0);
  await apply(mkst({ components: [comp({ state: "RUNNING" })],
                     vehicle: veh({ ages: Object.assign(veh({}).ages,
                                   { "/can_msg": 9 }) }) }));
  check("S13 数据超时灰删", el("#modePill").className.indexOf("m-none") >= 0
        && el("#modePill").textContent === "模式 自动");

  // S14 故障矩阵(横向偏差负向超限验证 Math.abs)
  await apply(mkst({ components: [comp({ state: "RUNNING" })], vehicle: veh({
    emergency_stop: true,
    can: { hz: 50, fault_codes: [0, 0, 1], fault: true },
    sensors: { sensorstate: 2, fault: false, lidar: false, camera: true,
               gnss: true, alive: 0, vehicle_ok: true },
    net: { internet_ok: false, alarm: 1 },
    lateral_dev_m: -6.2, lateral_dev_warn: true,
  }) }));
  check("S14 CAN 断流红", el("#f-can").textContent.indexOf("断流") >= 0
        && el("#f-can").className.indexOf("crit") >= 0);
  check("S14 急停按下红", el("#f-estop").textContent === "按下"
        && el("#f-estop").className.indexOf("crit") >= 0);
  check("S14 雷达故障点名", el("#f-sensor").textContent === "雷达 故障"
        && el("#f-sensor").className.indexOf("crit") >= 0);
  check("S14 规划心跳丢失", el("#f-alive").textContent === "丢失"
        && el("#s-alive").className.indexOf("bad") >= 0);
  check("S14 围栏告警红", el("#f-alarm").textContent.indexOf("围栏告警") >= 0
        && el("#f-alarm").className.indexOf("crit") >= 0);
  check("S14 网络断开黄", el("#f-net").textContent === "断开"
        && el("#f-net").className.indexOf("warn") >= 0);
  check("S14 负向偏差取绝对值红", el("#f-latdev").textContent.indexOf("6.20") >= 0
        && el("#f-latdev").className.indexOf("crit") >= 0);

  // S15 异常事件时间线(前端 diff;事件需连续 2 帧一致才确认——去抖机制)
  await apply(mkst({ components: [comp({ state: "RUNNING" })], vehicle: veh({
    emergency_stop: true,
    can: { hz: 50, fault_codes: [0, 0, 1], fault: true },
    sensors: { sensorstate: 2, fault: false, lidar: false, camera: true,
               gnss: true, alive: 0, vehicle_ok: true },
    net: { internet_ok: false, alarm: 1 },
    lateral_dev_m: -6.2, lateral_dev_warn: true,
  }) }));                                   // 故障态第 2 帧:确认帧
  check("S15 记录跳变事件", el("#evtList").innerHTML.indexOf("激光雷达") >= 0
        && el("#evtList").innerHTML.indexOf("急停") >= 0);
  await apply(mkst({ components: [comp({ state: "RUNNING" })], vehicle: veh() }));
  await apply(mkst({ components: [comp({ state: "RUNNING" })], vehicle: veh() }));
  check("S15 恢复事件", el("#evtList").innerHTML.indexOf("激光雷达 恢复") >= 0);

  // S16 脱挂钩标定:左侧"业务与辅助"组件卡(同外网监测形态)/
  //    进行中/完成弹窗(精确文案)/失败原因/前置提醒/日志入口
  await apply(mkst({ components: [comp({ state: "RUNNING" })], vehicle: veh(),
    calibration: { phase: "idle", current: { hook: [185, 240], pallet: [130, 240] } } }));
  var ch = el("#components").innerHTML;
  check("S16 卡片在业务与辅助栏(同组件卡形态)",
        ch.indexOf("业务与辅助") >= 0 && ch.indexOf("脱挂钩标定") >= 0
        && ch.indexOf('data-act="calibrate"') >= 0, ch.slice(0, 120));
  check("S16 当前行程显示", ch.indexOf("销子 185-240 · 托盘 130-240") >= 0,
        ch.slice(ch.indexOf("当前行程"), ch.indexOf("当前行程") + 40));
  check("S16 未标定态+按钮可用", ch.indexOf("未标定") >= 0
        && ch.indexOf('data-act="calibrate" data-name="calib" class="op-start">') >= 0, "");
  await apply(mkst({ components: [comp({ state: "RUNNING" })], vehicle: veh(),
    calibration: { phase: "running", elapsed_s: 5,
                   current: { hook: [185, 240], pallet: [130, 240] } } }));
  ch = el("#components").innerHTML;
  check("S16 进行中:按钮禁用+计时", ch.indexOf("进行中 5s") >= 0
        && ch.indexOf('data-act="calibrate" data-name="calib" class="op-start" disabled') >= 0,
        "");
  check("S16 进行中不弹窗", el("#calibModal").classList.contains("open") === false);
  await apply(mkst({ components: [comp({ state: "RUNNING" })], vehicle: veh(),
    calibration: { phase: "done", hook_range: [186, 249], pallet_range: [127, 249],
                   current: { hook: [186, 249], pallet: [127, 249] } } }));
  check("S16 完成弹窗精确文案",
        el("#calibModal").classList.contains("open") === true
        && el("#calibMsg").textContent ===
          "标定已完成，当前行程为销子：186(min)-249(max)  托盘：127(min)-249(max)",
        el("#calibMsg").textContent);
  ch = el("#components").innerHTML;
  check("S16 完成态绿(st-RUNNING 卡)", ch.indexOf("已完成") >= 0
        && ch.indexOf("st-RUNNING") >= 0, "");
  el("#calibClose").fire("click");
  check("S16 关闭弹窗", el("#calibModal").classList.contains("open") === false);
  await apply(mkst({ components: [comp({ state: "RUNNING" })], vehicle: veh(),
    calibration: { phase: "running", elapsed_s: 8 } }));
  await apply(mkst({ components: [comp({ state: "RUNNING" })], vehicle: veh(),
    calibration: { phase: "failed", message: "refused:manual mode active" } }));
  check("S16 失败弹窗带原因",
        el("#calibModal").classList.contains("open") === true
        && el("#calibMsg").textContent.indexOf("标定未完成：refused") === 0
        && el("#calibMsg").className.indexOf("crit") >= 0,
        el("#calibMsg").textContent);
  ch = el("#components").innerHTML;
  check("S16 失败态红闪+原因常显卡片",
        ch.indexOf("st-CRASHED") >= 0 && ch.indexOf("refused:manual mode active") >= 0, "");
  el("#calibClose").fire("click");
  // 真配置注入路径:已有"业务与辅助"组时卡片进组且全页唯一(此前 S16
  // 只测过 fallback 追加路径——review 指出的覆盖缺口)
  await apply(mkst({ components: [
    comp({ name: "pnc", group: 3, group_title: "规划控制", state: "RUNNING" }),
    comp({ name: "fms", group: 4, group_title: "业务与辅助", state: "RUNNING" }),
    comp({ name: "netcheck", title: "外网监测", group: 4,
           group_title: "业务与辅助", optional: true, enabled: false }),
  ], vehicle: veh(),
     calibration: { phase: "idle", current: { hook: [185, 240], pallet: [130, 240] } } }));
  ch = el("#components").innerHTML;
  var nCalBtn = (ch.match(/data-act="calibrate"/g) || []).length;
  var iGrp = ch.lastIndexOf("业务与辅助");
  var iCal = ch.indexOf("脱挂钩标定");
  var iNext = ch.indexOf("外网监测");
  check("S16 真配置:恰一张标定卡且注入该组内",
        nCalBtn === 1 && iGrp >= 0 && iCal > iGrp && iCal < iNext,
        "btns=" + nCalBtn + " grp=" + iGrp + " cal=" + iCal + " net=" + iNext);
  // 畸形 current(形状防御):不得抛错、行程回落 "--"
  await apply(mkst({ components: [comp({ state: "RUNNING" })], vehicle: veh(),
     calibration: { phase: "idle", current: { hook: null } } }));
  ch = el("#components").innerHTML;
  check("S16 畸形 current 不炸页面且回落 --",
        ch.indexOf("当前行程 --") >= 0 && ch.indexOf("脱挂钩标定") >= 0,
        "");
  check("S16 畸形 current 后页面仍在线(非假离线)",
        el("#overall").textContent.indexOf("无法连接") < 0,
        el("#overall").textContent);

  // 前置检查未过:经 #components 委托点击 → POST /api/calibrate 返回提醒 → 弹提醒窗
  calibResponse = { ok: false,
                    reminder: "一键标定前请停车挂N档，切换到自动驾驶模式",
                    detail: "当前驾驶模式为手动" };
  var calibBtn = { disabled: false, getAttribute: function(k){
    return k === "data-act" ? "calibrate" : "calib"; } };
  el("#components").fire("click", { target: { closest: function(){ return calibBtn; } } });
  await sleep(80);
  const nCal = fetchCalls.filter(c => c.url === "/api/calibrate").length;
  check("S16 提醒请求已发", nCal >= 1, String(nCal));
  check("S16 提醒弹窗(前置未过)",
        el("#calibModal").classList.contains("open") === true
        && el("#calibMsg").textContent.indexOf(
              "一键标定前请停车挂N档，切换到自动驾驶模式") >= 0
        && el("#calibMsg").className.indexOf("warn") >= 0,
        el("#calibMsg").textContent);
  el("#calibClose").fire("click");
  // 日志入口:卡片上的"日志"按钮打开 canbus 组件日志
  var logBtn2 = { disabled: false, getAttribute: function(k){
    return k === "data-act" ? "log" : "canbus"; } };
  el("#components").fire("click", { target: { closest: function(){ return logBtn2; } } });
  check("S16 日志按钮打开 canbus 日志",
        el("#logModal").classList.contains("open") === true
        && el("#logTitle").textContent.indexOf("canbus") >= 0,
        el("#logTitle").textContent);
  el("#logClose").fire("click");
  calibResponse = null;

  console.log("-".repeat(60));
  if (FAILS.length) {
    console.log("结果:%d 通过,%d 失败 → %s", PASS, FAILS.length, FAILS.join(", "));
    process.exit(1);
  }
  console.log("结果:%d 通过,0 失败", PASS);
  process.exit(0);
})();
