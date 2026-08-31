// -*- coding: utf-8 -*-
// monitor 前端无头测试:DOM 桩 + THREE 桩 + fetch 桩,驱动真实 index.html 的
// 内联脚本;MonitorMath 纯函数直测;bin fixture 与 test_full.py t05 同源。
// 用法:node tests/frontend_test.js

"use strict";
const fs = require("fs");
const path = require("path");

const MON = path.join(__dirname, "..");
const html = fs.readFileSync(path.join(MON, "static", "index.html"), "utf8");
const script = html.match(/<script>\s*([\s\S]*?)<\/script>/)[1];

let PASS = 0;
const FAILS = [];
function check(name, cond, detail) {
  if (cond) { PASS++; console.log("  ok " + name); }
  else { FAILS.push(name); console.log("  FAIL " + name + "  " + (detail || "")); }
}
function near(a, b, tol) { return Math.abs(a - b) <= (tol || 1e-6); }
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// ---------------- DOM 桩(对标 hmi/tests/frontend_test.js) ----------------
class El {
  constructor(key) {
    this.key = key;
    this._text = ""; this._html = ""; this.className = "";
    this.style = {}; this.checked = true; this.type = "";
    this._ls = {}; this._children = []; this.id = "";
  }
  get classList() {
    const self = this;
    const arr = () => self.className.split(/\s+/).filter(Boolean);
    return {
      add(...cs) { const s = new Set(arr()); cs.forEach((c) => s.add(c)); self.className = [...s].join(" "); },
      remove(...cs) { const s = new Set(arr()); cs.forEach((c) => s.delete(c)); self.className = [...s].join(" "); },
      contains(c) { return arr().includes(c); },
    };
  }
  set textContent(v) { this._text = String(v); }
  get textContent() { return this._text; }
  set innerHTML(v) { this._html = String(v); if (v === "") this._children = []; }
  get innerHTML() { return this._html; }
  appendChild(c) { this._children.push(c); return c; }
  addEventListener(t, f) { (this._ls[t] = this._ls[t] || []).push(f); }
  fire(t, ev) { (this._ls[t] || []).forEach((f) => f(ev || {})); }
  getContext() {          // canvas 2d 桩(文字 sprite + 2D 降级渲染)
    const self2 = this;
    if (!self2._ctx2d) {
      const calls = { fillRect: 0, lineTo: 0, strokeRect: 0, arc: 0,
                      strokeRects: [] };
      self2._ctx2d = {
        __calls: calls,
        clearRect() {}, fillText() {},
        fillRect() { calls.fillRect++; },
        beginPath() {}, moveTo() {}, closePath() {},
        lineTo() { calls.lineTo++; },
        stroke() {}, fill() {}, save() {}, restore() {},
        translate() {}, rotate() {},
        strokeRect(...args) { calls.strokeRect++; calls.strokeRects.push(args); },
        arc() { calls.arc++; },
        set font(v) { self2._font = v; },
        get font() { return self2._font || ""; },
        strokeStyle: "", fillStyle: "", lineWidth: 1,
        textAlign: "", textBaseline: "",
      };
    }
    return self2._ctx2d;
  }
}
const registry = new Map();
const created = [];            // createElement 出来的真实元素(带 id 的)
function el(sel) {
  if (!registry.has(sel)) registry.set(sel, new El(sel));
  return registry.get(sel);
}
const viewEl = el("#view");
viewEl.clientWidth = 800; viewEl.clientHeight = 600;

global.document = {
  // 带 id 的动态元素(如图层 checkbox)优先返回 createElement 创建的那个,
  // 否则 fire/change 会打到 registry 里另一个没有 listener 的对象上
  getElementById: (idv) =>
    created.find((e) => e.id === idv) || el("#" + idv),
  createElement: (tag) => { const e = new El("<" + tag + ">");
                            created.push(e); return e; },
  createTextNode: (s) => ({ text: String(s) }),
  addEventListener: (t, f) => {},
};
global.window = {
  _ls: {},
  addEventListener(t, f) { (this._ls[t] = this._ls[t] || []).push(f); },
  fire(t, ev) { (this._ls[t] || []).forEach((f) => f(ev || {})); },
  devicePixelRatio: 1,
};
global.requestAnimationFrame = (fn) => { rafCb = fn; return 0; };
let rafCb = null;      // 测试手动 flush(车辆位姿在 tick 里更新)
global.localStorage = {
  _d: {},
  getItem(k) { return (k in this._d) ? this._d[k] : null; },
  setItem(k, v) { this._d[k] = String(v); },
};

// ---------------- fetch 桩(可编程响应) ----------------
const fetchLog = [];
let fetchRoutes = {};
let failSnapshotOnce = false;
global.fetch = function (url) {
  fetchLog.push(url);
  if (url.indexOf("/api/snapshot") === 0 && failSnapshotOnce) {
    failSnapshotOnce = false;
    return Promise.reject(new Error("first snapshot down"));
  }
  const hit = Object.keys(fetchRoutes).find((k) => url.indexOf(k) === 0);
  if (!hit) { return Promise.reject(new Error("no route " + url)); }
  return Promise.resolve({ ok: true, json: () => Promise.resolve(fetchRoutes[hit].json),
                           arrayBuffer: () => Promise.resolve(fetchRoutes[hit].buf) });
};

// ---------------- THREE 桩(只桩被触到的面) ----------------
const OBJ_SEQ = [];
function rec(o, kind) { o.__kind = kind; OBJ_SEQ.push(o); return o; }
function makeGeom() {
  return rec({
    _attrs: {}, _pts: null,
    setAttribute(n, a) { this._attrs[n] = a; },
    setFromPoints(pts) { this._pts = pts; return this; },
    dispose() {},
  }, "geom");
}
function mat(opts) { return rec(Object.assign({ dispose() {} }, opts), "mat"); }
function obj3(kind, extra) {
  return rec(Object.assign({
    visible: true, children: [], parent: null,
    position: { set(x, y, z) { this.x = x; this.y = y; this.z = z; },
                copy(o) { this.x = o.x; this.y = o.y; this.z = o.z; },
                x: 0, y: 0, z: 0 },
    rotation: { y: 0, x: 0 },
    scale: { set(x, y, z) { this.x = x; this.y = y; this.z = z; }, x: 1, y: 1, z: 1 },
    add(c) { c.parent = this; this.children.push(c); },
    geometry: makeGeom(), material: mat({}),
  }, extra), kind);
}
const THREE = {
  WebGLRenderer: function () {
    return rec({
      setPixelRatio() {}, setSize() {},
      domElement: new El("<canvas>"),
      render() { this.__renders = (this.__renders || 0) + 1; },
      __renders: 0,
    }, "renderer");
  },
  Scene: function () { return obj3("scene", { background: {} }); },
  Color: function (c) { this.c = c; },
  PerspectiveCamera: function () {
    return obj3("camera", { lookAt() {}, aspect: 0, updateProjectionMatrix() {} });
  },
  OrbitControls: function (cam, dom) {
    return rec({
      enableDamping: false, dampingFactor: 0, maxPolarAngle: 0,
      enableRotate: true, update() {},
      target: { set(x, y, z) { this.x = x; this.y = y; this.z = z; }, x: 0, y: 0, z: 0 },
    }, "controls");
  },
  GridHelper: function () { return obj3("grid"); },
  Group: function () { return obj3("group"); },
  Line: function (g, m) { return obj3("line", { geometry: g, material: m, frustumCulled: false }); },
  BufferGeometry: makeGeom,
  BufferAttribute: function (arr, n) { this.array = arr; this.itemSize = n; },
  Vector3: function (x, y, z) { this.x = x; this.y = y; this.z = z; },
  LineBasicMaterial: mat, MeshBasicMaterial: mat, PointsMaterial: mat,
  SpriteMaterial: mat,
  Mesh: function (g, m) { return obj3("mesh", { geometry: g, material: m, userData: {} }); },
  Points: function (g, m) { return obj3("points", { geometry: g, material: m }); },
  Sprite: function (m) { return obj3("sprite", { material: m, userData: {} }); },
  SphereGeometry: makeGeom, BoxGeometry: makeGeom, ConeGeometry: makeGeom,
  TorusGeometry: makeGeom,
  CanvasTexture: function (cv) { this.minFilter = 0; this.needsUpdate = false; this.cv = cv; },
  LinearFilter: 1006,
};
global.THREE = THREE;

// ---------------- bin fixture(与 test_full.py t05 同源) ----------------
// 单点 (x=1500mm, y=-2250mm, z=500mm, i=200),头+单通道
function fixtureBin() {
  const b = Buffer.alloc(40);
  b.write("QWMC", 0, "ascii");
  b.writeUInt16LE(1, 4);          // version
  b.writeUInt16LE(1, 6);          // 通道数
  b.writeUInt32LE(1, 8);          // 总点数
  b.writeBigUInt64LE(1700000000000n, 12);
  b.writeUInt8(1, 20); b.writeUInt8(0, 21);   // 通道子头
  b.writeUInt16LE(0, 22);
  b.writeUInt32LE(1, 24);
  b.writeInt32LE(1500, 28); b.writeInt32LE(-2250, 32);
  b.writeInt16LE(500, 36); b.writeUInt8(200, 38); b.writeUInt8(0, 39);
  return b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength);
}

// ---------------- 直测 MonitorMath(先于脚本 eval 亦可) ----------------
// (脚本 eval 后 window.MM 生效;此处用同一对象)

async function main() {
  // 快照与地图路由必须在 eval 前就位:init() 会立即 poll()+loadMapOnce()
  const SNAP = {
    ros_available: true, master_ok: true, typed_ok: true,
    msg_import_errors: [], origin: [49.23, 0.56],
    ages: { "/navigation_msg": 0.2, "/perception": 0.1, "/back_left_scan": 8.0 },
    vehicle: { x: 6.7, y: -13.1, yaw: -3.2198, speed: 2.5 },
    obstacles: [
      { x: 8.8, y: 2.4, l: 4.0, w: 1.0, h: 1.8, yaw: -0.1745, id: 1, vx: 0, vy: 0 },
      { x: 9.9, y: 3.4, l: 2.0, w: 0.9, h: 1.2, yaw: 0.1, id: 2, vx: 0, vy: 0 }],
    paths: { plan: [[0, 0], [1, 1], [2, 2]], refer: [[0, 0], [3, 3]] },
    stop: { x: 10.7, y: -1.5, yaw: 1.57 }, pallet: { x: 9.8, y: -2.4 },
    task: { id: 88, type: 1, work_mode: 1, exec: 1, cloud_proc: 1,
            fail_code: 0, fail_reason: "" },
    plan: { desire_speed: 2.5, planspeed: 2.4, safety: true },
    control: { steer: -11.0, brake: 0, throttle: 18, bia: 0.21 },
    can: { gear: 4, mode: 1, estop: 0, battery: 77, hook: 1,
           steer_fb: -220.0, fault: [0], speed: 1.2, brake_fb: 30,
           link_pallet: 1, eab: 1, hook_btn: 1, link_btn: 2,
           eps_mode: 3, eps_current: -4.75, pin_pos: 185, seat_pos: 130 },
  };
  const MAP = { n: 2,
                maps: [
                  { name: "a.csv", center: [[0, 0], [1, 1]],
                    left: [[0, 1]], right: [[0, -1]] },
                  { name: "b.csv", center: [[5, 5], [6, 6]],
                    left: [[5, 6]], right: [[5, 4]] },
                ],
                bbox: [0, 0, 6, 6],
                layers: { vehicle: true, lidar: true, routing: true,
                          planning: false, loadpos: true, stoppose: true,
                          map: false, scan: true, cloud: true, grid: false } };
  fetchRoutes = { "/api/snapshot": { json: SNAP }, "/api/map": { json: MAP } };
  failSnapshotOnce = true;

  // eval 真实页面脚本("use strict" direct eval:脚本变量不可直接引用,
  // 全部断言走桩的产物;页面自身的 setInterval(500ms) 驱动 poll)
  eval(script);
  const MM2 = window.MM || MM;
  check("页面脚本 eval 无异常", true);
  /* 第一次 snapshot 就失败也必须立即自报,不能等曾经 online 之后才
   * 把 master/ROS 初始灰点改红。 */
  await sleep(50);
  check("首次连接失败立即显示断连横幅",
        el("#banners")._html.indexOf("断开") >= 0, el("#banners")._html);
  check("首次连接失败 master/ROS 立即变红",
        el("#dotRos").className.indexOf("bad") >= 0 &&
        el("#dotMaster").className.indexOf("bad") >= 0,
        el("#dotRos").className + "/" + el("#dotMaster").className);

  // F1 坐标映射
  let t = MM2.rosToThree(1, 2, 3);
  check("rosToThree (x,z,-y)", t[0] === 1 && t[1] === 3 && t[2] === -2, JSON.stringify(t));

  // F2 外推
  let pose = { x: 0, y: 0, yaw: 0, speed: 2 };
  let e = MM2.extrapolate(pose, 0.25);
  check("外推 0.25s 沿 yaw 前进 0.5m", near(e.x, 0.5) && near(e.y, 0), JSON.stringify(e));
  e = MM2.extrapolate(pose, 5);
  check("外推钳 0.6s(1.2m)", near(e.x, 1.2), JSON.stringify(e));
  e = MM2.extrapolate(null, 1);
  check("外推空位姿 null", e === null);

  // F3/F4 解码
  let d = MM2.decodeBin(fixtureBin());
  check("decodeBin 头", d && d.total === 1 && d.channels.length === 1 &&
        d.channels[0].id === 1, JSON.stringify(d && d.total));
  let p = d.channels[0].pts;
  check("decodeBin fixture 点(与 test_full t05 同源)",
        near(p[0], 1.5) && near(p[1], 0.5) && near(p[2], 2.25) && d.channels[0].count === 1,
        JSON.stringify([p[0], p[1], p[2]]));
  check("decodeBin 坏 magic -> null", MM2.decodeBin(new ArrayBuffer(30)) === null);
  check("decodeBin 短包 -> null", MM2.decodeBin(new ArrayBuffer(8)) === null);

  // F4b 双通道帧(scan 两路):回归 08-28 排障发现的双重推进 bug
  // (通道内修了 off+=12 但外层 cnt*12 没删 -> 第二通道读错位)
  {
    const b = Buffer.alloc(20 + 8 + 2 * 12 + 8 + 1 * 12);
    b.write("QWMC", 0, "ascii");
    b.writeUInt16LE(1, 4);
    b.writeUInt16LE(2, 6);              // 两个通道
    b.writeUInt32LE(3, 8);              // 总点数 2+1
    b.writeUInt32LE(1, 12);             // ts lo
    // 通道1: id=1, 2 点 (1000,2000,300,i1) (-4000,-5000,-600,i2)
    b.writeUInt8(1, 20); b.writeUInt32LE(2, 24);
    b.writeInt32LE(1000, 28); b.writeInt32LE(2000, 32);
    b.writeInt16LE(300, 36); b.writeUInt8(77, 38);
    b.writeInt32LE(-4000, 40); b.writeInt32LE(-5000, 44);
    b.writeInt16LE(-600, 48); b.writeUInt8(88, 50);
    // 通道2 子头起于 52: id=7, 1 点 (5000,6000,700,i3)
    b.writeUInt8(7, 52); b.writeUInt32LE(1, 56);
    b.writeInt32LE(5000, 60); b.writeInt32LE(6000, 64);
    b.writeInt16LE(700, 68); b.writeUInt8(99, 70);
    const d2ch = MM2.decodeBin(b.buffer.slice(b.byteOffset,
                                             b.byteOffset + b.byteLength));
    check("双通道: 头与计数", d2ch && d2ch.channels.length === 2 &&
          d2ch.total === 3 && d2ch.channels[0].count === 2 &&
          d2ch.channels[1].count === 1,
          JSON.stringify(d2ch && [d2ch.total, d2ch.channels.map(c => c.count)]));
    if (d2ch && d2ch.channels.length === 2) {
      const f1 = d2ch.channels[0].pts, f2 = d2ch.channels[1].pts;
      check("双通道: 通道1数值", near(f1[0], 1) && near(f1[1], 0.3) &&
            near(f1[2], -2) && near(f1[3], -4) && near(f1[5], 5),
            JSON.stringify(Array.from(f1)));
      check("双通道: 通道2从正确偏移读(曾错位)", d2ch.channels[1].id === 7 &&
            near(f2[0], 5) && near(f2[1], 0.7) && near(f2[2], -6),
            JSON.stringify([d2ch.channels[1].id, Array.from(f2)]));
    }
  }

  // F5 场景装配断言(eval 时 init 已跑)
  const sceneObj = OBJ_SEQ.find((o) => o.__kind === "scene");
  check("场景已装配", !!sceneObj && sceneObj.children.length >= 8,
        String(sceneObj && sceneObj.children.length));
  const gridObj = OBJ_SEQ.filter((o) => o.__kind === "grid")[0];
  check("GridHelper 默认关", gridObj && gridObj.visible === false);
  const ctrlObj = OBJ_SEQ.find((o) => o.__kind === "controls");
  const vehGroup = OBJ_SEQ.filter((o) => o.__kind === "group")[0];
  check("车辆组已建", !!vehGroup);
  const vehLine = vehGroup && vehGroup.children[0];
  check("车身轮廓 5 点", vehLine && vehLine.geometry._pts &&
        vehLine.geometry._pts.length === 5,
        String(vehLine && vehLine.geometry._pts && vehLine.geometry._pts.length));

  // F6/F7 快照应用 + 图层默认态(等页面自身 poll 周期)
  await sleep(800);
  if (rafCb) { rafCb(); }      // 手动跑一帧(车辆位姿在 tick 里更新)
  check("poll 请求 /api/snapshot", fetchLog.indexOf("/api/snapshot") >= 0);
  // 快照->帧间隔内有速度外推(x 沿 yaw 前进<=0.6*speed=1.5m),用区间断言
  check("车辆组位姿/朝向更新(含外推)",
        vehGroup.position.x > 5.0 && vehGroup.position.x <= 6.71 &&
        vehGroup.position.z >= 12.9 && vehGroup.position.z < 13.3 &&
        near(vehGroup.rotation.y, -3.2198, 1e-3),
        JSON.stringify([vehGroup.position.x, vehGroup.position.z, vehGroup.rotation.y]));
  const obstacleMeshes = OBJ_SEQ.filter(
    (o) => o.__kind === "mesh" && o.parent === sceneObj &&
    o.scale.y !== 1);       // 障碍池化项(scale 被设过)
  check("障碍池化 2 个", obstacleMeshes.length === 2,
        String(obstacleMeshes.length));
  if (obstacleMeshes.length === 2) {
    const m0 = obstacleMeshes[0];
    check("障碍 scale=l,h,w(dx/dy 互换)", near(m0.scale.x, 4.0) &&
          near(m0.scale.y, 1.8) && near(m0.scale.z, 1.0),
          JSON.stringify([m0.scale.x, m0.scale.y, m0.scale.z]));
    check("障碍 z 抬高 h/2", near(m0.position.y, 0.9), String(m0.position.y));
  }
  check("HUD 速度文本", el("#hSpd")._text.indexOf("2.5") >= 0,
        el("#hSpd")._text);
  // 数据龄列表段已按需求移除:灰化能力回归改走数值行(见 F2b)
  check("HUD 任务段: #88 执行中",
        el("#hTaskId")._text.indexOf("88") >= 0 &&
        el("#hTaskExec")._text.indexOf("执行中") >= 0,
        el("#hTaskId")._text + "/" + el("#hTaskExec")._text);
  check("HUD 规划段: 期望速度/安全",
        el("#hDesire")._text.indexOf("2.5") >= 0 &&
        el("#hSafety")._text.indexOf("安全") >= 0,
        el("#hDesire")._text + "/" + el("#hSafety")._text);
  check("HUD 控制段: 前轮转角/踏板",
        el("#hSteer")._text.indexOf("-11.0") >= 0 &&
        el("#hPedal")._text.indexOf("T18") >= 0,
        el("#hSteer")._text + "/" + el("#hPedal")._text);
  check("HUD CAN 段: 挡位模式/方向盘转角",
        el("#hGearMode")._text.indexOf("D") >= 0 &&
        el("#hGearMode")._text.indexOf("自动") >= 0 &&
        el("#hCanFb")._text.indexOf("-220.0") >= 0 &&
        el("#hCanFb")._text.indexOf("无") >= 0,   /* [0] 滤零 */
        el("#hGearMode")._text + "/" + el("#hCanFb")._text);
  check("HUD 安全段: ✗不安全+红色强调(极性锁定)",
        el("#hSafety")._text.indexOf("✗不安全") >= 0 &&
        el("#hSafety").className.indexOf("bad") >= 0 &&
        el("#hSafety").className.indexOf("stale") < 0,
        el("#hSafety")._text + "/" + el("#hSafety").className);
  check("HUD 挂接: 0/1 域文案",
        el("#hEbat")._text.indexOf("已挂") >= 0,
        el("#hEbat")._text);
  check("HUD CAN 扩展: 车速/制动反馈+限位/EAB(精确串)",
        el("#hCanVeh")._text === "1.2 m/s / B30%" &&
        el("#hCanLink")._text === "已连 / 1",
        el("#hCanVeh")._text + "/" + el("#hCanLink")._text);
  /* 精确串锁列序(挂钩列在前):无序 indexOf 曾放过 hook/link 两列互换
   * (对抗校验轮实测,列一换断言照绿);eps_mode 也一并锁定 */
  check("HUD CAN 扩展: 按钮/EPS/位置码(精确串,锁列序)",
        el("#hCanBtn")._text === "升 / 降" &&
        el("#hCanEps")._text === "3 / -4.75" &&
        el("#hCanPos")._text === "185 / 130",
        el("#hCanBtn")._text + "/" + el("#hCanEps")._text + "/" +
        el("#hCanPos")._text);
  // F2d HTML 结构锁定:DOM 桩对任意 id 按需造元素,行节点被删/标签错/
  // 数据龄段回加都不会让上面的断言变红 -> 直接对页面源文本断言
  check("F2d CAN 行节点与标签在页面源中存在",
        ["hCanVeh", "hCanLink", "hCanBtn", "hCanEps", "hCanPos",
         "hGearMode", "hEbat", "hCanFb"].every(
          (id) => html.indexOf('id="' + id + '"') >= 0) &&
        ["车速/制动(反馈)", "托盘限位/EAB面板", "挂钩/链接按钮",
         "EPS模式/电流", "挂钩/托盘位置(码)"].every(
          (t) => html.indexOf(t) >= 0),
        "行节点/标签缺失");
  check("F2d 数据龄展示未回加(R1)",
        html.indexOf('id="ageList"') < 0 && html.indexOf("AGE_TOPICS") < 0,
        "ageList/AGE_TOPICS 仍存在");

  // F2b 数据龄列表已移除,>5s 灰化回归改走数值行:定位龄推高 -> hSpd stale
  SNAP.ages["/navigation_msg"] = 8.0;
  await sleep(1300);
  check("F2b 数值行 stale 灰化(数据龄列表已移除)",
        el("#hSpd").className.indexOf("stale") >= 0,
        el("#hSpd").className);
  SNAP.ages["/navigation_msg"] = 0.2;
  await sleep(1300);
  check("F2b 新鲜数据不灰(灰化极性反向锁定)",
        el("#hSpd").className.indexOf("stale") < 0,
        el("#hSpd").className);

  // F2c 旧服务端快照(无 CAN 扩展字段):不抛异常伪装断连,五行全部 "--"
  const CAN_KEYS = ["speed", "brake_fb", "link_pallet", "eab", "hook_btn",
                    "link_btn", "eps_mode", "eps_current", "pin_pos",
                    "seat_pos"];
  const savedCan = {};
  CAN_KEYS.forEach((k) => { savedCan[k] = SNAP.can[k]; delete SNAP.can[k]; });
  await sleep(1300);
  check("F2c 旧服务端形态:五行扩展行全 -- 且不误报断连",
        el("#hCanVeh")._text === "--" && el("#hCanLink")._text === "--" &&
        el("#hCanBtn")._text === "--" && el("#hCanEps")._text === "--" &&
        el("#hCanPos")._text === "--" &&
        el("#banners")._html.indexOf("断开") < 0,
        el("#hCanVeh")._text + el("#hCanLink")._text + el("#hCanBtn")._text +
        el("#hCanEps")._text + el("#hCanPos")._text + "/" +
        el("#banners")._html);
  Object.assign(SNAP.can, savedCan);

  // F2e 位置码 0 哨兵(0x285 从未到达):显示 "--" 而非可读作"最高位"的 0
  const savedPos = [SNAP.can.pin_pos, SNAP.can.seat_pos];
  SNAP.can.pin_pos = 0; SNAP.can.seat_pos = 0;
  await sleep(1300);
  check("F2e 位置码 0 哨兵显示 --",
        el("#hCanPos")._text === "-- / --",
        el("#hCanPos")._text);
  [SNAP.can.pin_pos, SNAP.can.seat_pos] = savedPos;
  check("HUD 任务: fail_reason 透传(桩无失败不拼)",
        el("#hTaskCloud")._text.indexOf("执行中") >= 0,
        el("#hTaskCloud")._text);

  // 3D 路径必须与车辆/停车点使用同一 ROS -> Three z=-y 映射
  const planLine = OBJ_SEQ.find((o) => o.__kind === "line" &&
    o.parent === sceneObj && o.material && o.material.color === 0x00ff00);
  const planPos = planLine && planLine.geometry._attrs.position &&
    planLine.geometry._attrs.position.array;
  check("3D 规划路径 y 映射为 Three -z",
        planPos && near(planPos[3], 1) && near(planPos[5], -1),
        JSON.stringify(planPos && Array.from(planPos)));
  check("首帧车辆到来后 3D 相机自动对准车辆",
        ctrlObj && near(ctrlObj.target.x, SNAP.vehicle.x) &&
        near(ctrlObj.target.z, -SNAP.vehicle.y),
        JSON.stringify(ctrlObj && ctrlObj.target));

  // F6b 多地图 3D 锁定:mapGroup 6 条线(2 张×3) + 首点数值
  // (突变证明:buildMapLines slice(0,1) 时旧断言全绿)
  {
    const groups = OBJ_SEQ.filter((o) => o.__kind === "group");
    const mg = groups.filter(
      (g) => g.children.length === 6 &&
             g.children.every((c) => c.__kind === "line"))[0] || null;
    check("F6b 多地图: mapGroup 6 条线(2张x3)",
          !!mg, JSON.stringify(groups.map((g) => g.children.length)));
    if (mg) {
      const a0 = mg.children[0].geometry._attrs.position.array;
      const b0 = mg.children[3].geometry._attrs.position.array;
      check("F6b 首图中心线首点 [0,0,0](z=-y)",
            a0.length === 6 && a0[0] === 0 && a0[1] === 0 && a0[2] === 0,
            JSON.stringify(Array.from(a0)));
      check("F6b 次图中心线首点 [5,0,-5]",
            b0[0] === 5 && b0[1] === 0 && b0[2] === -5,
            JSON.stringify(Array.from(b0)));
    }
  }
  check("图层 10 项", el("#layerList")._children.length === 10,
        String(el("#layerList")._children.length));
  check("map 默认关", document.getElementById("ly_map").checked === false);
  check("vehicle 默认开", document.getElementById("ly_vehicle").checked === true);

  // F8 图层开关联动
  const cbLidar = document.getElementById("ly_lidar");
  cbLidar.checked = false;
  cbLidar.fire("change", { target: cbLidar });
  check("关 lidar 图层 -> 障碍隐藏",
        obstacleMeshes.every((m) => m.visible === false));
  cbLidar.checked = true;
  cbLidar.fire("change", { target: cbLidar });
  check("开 lidar 图层 -> 障碍恢复", obstacleMeshes.every((m) => m.visible === true));

  // F9 断网横幅(等下一个 poll 周期的 catch 分支)
  fetchRoutes = {};
  global.fetch = function () { return Promise.reject(new Error("down")); };
  await sleep(1300);
  check("断网横幅", el("#banners")._html.indexOf("断开") >= 0,
        el("#banners")._html);
  // F11 断连时两个状态点必须显式置 bad(实车 08-28 "灰点"症状回归:
  // 此前 renderHud(null) 提前 return 漏设, 点永远保持初始灰)
  check("断连点变红(非灰)", el("#dotRos").className.indexOf("bad") >= 0 &&
        el("#dotMaster").className.indexOf("bad") >= 0,
        el("#dotRos").className + "/" + el("#dotMaster").className);

  // F12 WebGL 初始化失败降级:轮询/HUD 不得死(实车 08-28 症状回归:
  // buildScene 抛异常曾使 init 中断 -> 轮询从未启动 -> 灰点无数据)
  {
    // 清掉第一次页面 eval 创建的动态 checkbox/canvas,构造干净的降级页面。
    created.length = 0;
    viewEl._children = [];
    el("#layerList")._children = [];
    localStorage._d = {};
    global.fetch = function (url) {
      fetchLog.push(url);
      return Promise.resolve({ ok: true,
        json: () => Promise.resolve(url.indexOf("/api/map") === 0 ? MAP : SNAP),
        arrayBuffer: () => Promise.resolve(fixtureBin()) });
    };
    const goodRenderer = THREE.WebGLRenderer;
    THREE.WebGLRenderer = function () { throw new Error("no webgl"); };
    fetchLog.length = 0;
    try {
      eval(script);         // 重新跑整页脚本(场景失败路径)
    } catch (e) {
      check("F12 场景失败不逃逸", false, String(e));
    }
    THREE.WebGLRenderer = goodRenderer;
    await sleep(300);
    check("F12 场景失败仍启动轮询",
          fetchLog.indexOf("/api/snapshot") >= 0,
          JSON.stringify(fetchLog.slice(0, 3)));
    check("F12 2D 降级仍拉取 scan/cloud",
          fetchLog.indexOf("/api/scan.bin") >= 0 &&
          fetchLog.indexOf("/api/cloud.bin") >= 0,
          JSON.stringify(fetchLog));
    check("F12 自报 3D 初始化失败并切换 2D",
          el("#err")._text.indexOf("3D 初始化失败") >= 0 &&
          el("#err")._text.indexOf("2D") >= 0,
          el("#err")._text);
    check("F12 2D 图层默认态按配置应用",
          document.getElementById("ly_map").checked === false &&
          document.getElementById("ly_scan").checked === true &&
          document.getElementById("ly_cloud").checked === true);
    let mapToggleErr = null;
    const cbMap2d = document.getElementById("ly_map");
    cbMap2d.checked = true;
    try { cbMap2d.fire("change", { target: cbMap2d }); }
    catch (toggleErr) { mapToggleErr = toggleErr; }
    check("F12 2D 点击地图图层不访问未创建的 THREE 线对象",
          mapToggleErr === null, String(mapToggleErr || ""));

    // F13 2D 降级渲染:必须实际画出路径/车辆/点,不能只画背景假通过
    if (rafCb) { rafCb(1000); }
    const cvEl = created.filter((e) => e.key === "<canvas>").pop() || null;
    const ctx2 = cvEl && cvEl.getContext();
    check("F13 2D 实际绘制路径/车辆/点(非仅背景)",
          ctx2 && ctx2.__calls.lineTo > 0 && ctx2.__calls.strokeRect > 0 &&
          ctx2.__calls.fillRect > 3,
          JSON.stringify(ctx2 && ctx2.__calls));
    const vehicleRectOk = ctx2 && ctx2.__calls.strokeRects.some((a) =>
      near(a[0], -0.8 * 6) && near(a[2], 3.1 * 6));
    check("F13 2D 车身矩形从后缘 -0.8m 起画", vehicleRectOk,
          JSON.stringify(ctx2 && ctx2.__calls.strokeRects));
    const drawCount = ctx2 && ctx2.__calls.fillRect;
    if (rafCb) { rafCb(1050); }
    check("F13 2D 重绘限制为最多 10FPS",
          ctx2 && ctx2.__calls.fillRect === drawCount,
          JSON.stringify(ctx2 && ctx2.__calls));
    viewEl.clientWidth = 900; viewEl.clientHeight = 500;
    window.fire("resize");
    check("F13 2D Canvas 跟随窗口尺寸",
          cvEl && cvEl.width === 900 && cvEl.height === 500,
          JSON.stringify(cvEl && [cvEl.width, cvEl.height]));
  }

  // F10 相机预设切换
  el("#cam3d").fire("click");
  check("3D 预设可旋转+高亮", ctrlObj.enableRotate === true &&
        el("#cam3d").className.indexOf("on") >= 0);
  el("#camTop").fire("click");
  check("俯视预设锁旋转", ctrlObj.enableRotate === false);

  console.log("\n==== %d passed, %d failed ====", PASS, FAILS.length);
  FAILS.forEach((f) => console.log("  FAILED:", f));
  process.exit(FAILS.length ? 1 : 0);
}

main().catch((e) => { console.error("HARNESS ERROR", e); process.exit(2); });
