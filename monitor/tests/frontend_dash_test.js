// -*- coding: utf-8 -*-
// dashboard 前端无头测试:DOM 桩 + fetch 桩,驱动 static/dashboard.html 内联脚本。
// 重点:瓦片渲染(值/译码/单位)、故障汇总三态、数据新鲜度三态(ok/stale/off)、
//       停滞卡片标记、esc/valHtml/fmtAge、总断连指示全灭。
// 用法:node tests/frontend_dash_test.js

"use strict";
const fs = require("fs");
const path = require("path");

const MON = path.join(__dirname, "..");
const html = fs.readFileSync(path.join(MON, "static", "dashboard.html"),
                             "utf8");
const script = html.match(/<script>\s*([\s\S]*?)<\/script>/)[1];

let PASS = 0;
const FAILS = [];
function check(name, cond, detail) {
  if (cond) { PASS++; console.log("  ok " + name); }
  else { FAILS.push(name); console.log("  FAIL " + name + "  " + (detail || "")); }
}

// ---------------- DOM 桩 ----------------
class El {
  constructor(key) {
    this.key = key;
    this._text = ""; this._html = ""; this.className = "";
    this.style = {};
  }
  set textContent(v) { this._text = String(v); }
  get textContent() { return this._text; }
  set innerHTML(v) { this._html = String(v); }
  get innerHTML() { return this._html; }
}
const reg = new Map();
function el(id) {
  if (!reg.has(id)) reg.set(id, new El(id));
  return reg.get(id);
}
global.document = { getElementById: (id) => el(id) };
global.setInterval = () => 0;
// poll() 首拍挂起(不 resolve),避免异步副作用污染同步断言
global.fetch = () => new Promise(() => {});

// 载入页面脚本;追加导出行把内部函数抬到可测位置(同一作用域内闭包);
// resetPoll 清防重入标志(页面首拍用永不 resolve 的 fetch 桩挂起,不清则
// 后续 T.poll() 全被挡)
eval(script + ";globalThis.__t={render,setDot,liveState,fmtAge,esc,"
    + "valHtml,tileHtml,fsumHtml,poll,"
    + "resetPoll:function(){inflight=false;}};");
const T = globalThis.__t;

// ---------------- 纯函数 ----------------
check("脚本可载入并导出", !!T && typeof T.render === "function");
check("fmtAge null", T.fmtAge(null) === "—");
check("fmtAge undefined", T.fmtAge(undefined) === "—");
check("fmtAge 大值∞", T.fmtAge(100000) === "∞");
check("fmtAge 数值透传", T.fmtAge(3.2) === 3.2);
check("esc 转义", T.esc('<a>&"') === "&lt;a&gt;&amp;&quot;");
check("liveState ok(阈值内)", T.liveState({age: 4.9}) === "ok");
check("liveState ok(恰阈值)", T.liveState({age: 5.0}) === "ok");
check("liveState stale(超阈值)", T.liveState({age: 5.1}) === "stale");
check("liveState off(null)", T.liveState({age: null}) === "off");
check("liveState off(缺 rec)", T.liveState(undefined) === "off");
check("valHtml None 值 -> —", T.valHtml({value: null}) === "—");
check("valHtml 译码绿字", T.valHtml({value: 1, text: "开"})
      === '1<span class="tx">开</span>');
check("valHtml 无译码裸值", T.valHtml({value: 1.2, text: null}) === "1.2");

// ---------------- render 三态(瓦片契约) ----------------
function fakePayload(canAge, ehbAge) {
  return {
    ok: true, ros_available: true, master_ok: true,
    topics: {
      "/can_msg": {
        age: canAge,
        sections: [{title: null, fields: [
          {name: "curGear", label: "当前挡位", note: "",
           value: 4, text: "D"},
          {name: "vehicleSpeed", label: "车速", note: "m/s",
           value: 1.2, text: null}]}]},
      "/ehb_msg": {
        age: ehbAge,
        sections: [{title: "帧1 测试", fields: [
          {name: "__faults_0", label: "故障位", note: "",
           value: 0, text: "全部正常"},
          {name: "X", label: "测试字段", note: "×0.1 MPa",
           value: 1, text: "开"}]}]},
    }};
}

T.render(fakePayload(0.3, 0.4));
check("新鲜:话题绿点", el("d-can").className === "dot ok"
      && el("d-ehb").className === "dot ok", el("d-can").className);
check("新鲜:卡片无 stale", el("card-can").className === "card can"
      && el("card-ehb").className === "card ehb");
check("新鲜:ROS/master 绿", el("d-ros").className === "dot ok"
      && el("d-master").className === "dot ok");
check("瓦片渲染:名称", el("body-can").innerHTML.indexOf("当前挡位") >= 0
      && el("body-can").innerHTML.indexOf("车速") >= 0);
check("瓦片渲染:裸值", el("body-can").innerHTML.indexOf(">1.2<") >= 0);
check("瓦片渲染:译码绿字", el("body-can").innerHTML
      .indexOf('<span class="tx">D</span>') >= 0);
check("瓦片渲染:单位小字", el("body-can").innerHTML.indexOf("m/s") >= 0
      && el("body-ehb").innerHTML.indexOf("×0.1 MPa") >= 0);
check("故障汇总:正常态", el("body-ehb").innerHTML
      .indexOf('tile fsum ok') >= 0
      && el("body-ehb").innerHTML.indexOf("全部正常") >= 0);
check("无英文字段名副行", el("body-can").innerHTML.indexOf("curGear") < 0
      && el("body-can").innerHTML.indexOf("vehicleSpeed") < 0);
check("分节标题渲染", el("body-ehb").innerHTML.indexOf("帧1 测试") >= 0);
check("枚举译码渲染", el("body-ehb").innerHTML.indexOf("开") >= 0);

// 故障汇总异常/无数据两态
T.render({ok: true, ros_available: true, master_ok: true,
  topics: {"/can_msg": {age: 0.3, sections: []},
           "/ehb_msg": {age: 0.3, sections: [{title: null, fields: [
             {name: "__faults_0", label: "故障位", note: "",
              value: 2, text: "电源电压过高、电源电压过低"},
             {name: "Y", label: "字段", note: "", value: 0, text: null}]}]}}});
check("故障汇总:异常态点名", el("body-ehb").innerHTML
      .indexOf("tile fsum bad") >= 0
      && el("body-ehb").innerHTML.indexOf("2 项") >= 0
      && el("body-ehb").innerHTML.indexOf("电源电压过高") >= 0);
T.render({ok: true, ros_available: true, master_ok: true,
  topics: {"/can_msg": {age: 0.3, sections: []},
           "/ehb_msg": {age: 0.3, sections: [{title: null, fields: [
             {name: "__faults_0", label: "故障位", note: "",
              value: null, text: null}]}]}}});
check("故障汇总:无数据不假绿", el("body-ehb").innerHTML
      .indexOf("tile fsum off") >= 0
      && el("body-ehb").innerHTML.indexOf("无数据") >= 0
      && el("body-ehb").innerHTML.indexOf("全部正常") < 0);

// 数值变化闪烁(同字段两拍不同值 -> tile flash)
T.render(fakePayload(0.3, 0.4));          // 值未变:不闪
check("值不变不闪烁", el("body-can").innerHTML.indexOf("tile flash") < 0);
const changed = fakePayload(0.3, 0.4);
changed.topics["/can_msg"].sections[0].fields[0].value = 3;
changed.topics["/can_msg"].sections[0].fields[0].text = "R";
T.render(changed);
check("值变化闪烁", el("body-can").innerHTML.indexOf("tile flash") >= 0);

T.render(fakePayload(0.3, 999));
check("停滞:话题黄点", el("d-ehb").className === "dot warn",
      el("d-ehb").className);
check("停滞:卡片标 stale", el("card-ehb").className === "card ehb stale");
check("停滞:新鲜侧不受累", el("d-can").className === "dot ok"
      && el("card-can").className === "card can");
check("停滞:冻结值仍渲染(半透明)", el("body-ehb").innerHTML
      .indexOf("测试字段") >= 0);
check("停滞:年龄数字显示", el("a-ehb").textContent === "999");

T.render(fakePayload(null, null));
check("无数据:灰点回落", el("d-can").className === "dot"
      && el("d-ehb").className === "dot");
check("无数据:提示渲染", el("body-can").innerHTML
      .indexOf("话题无数据") >= 0);

T.render({ok: false});
check("异常 payload:错误横幅", el("err").style.display === "block");

// ---- 降级 payload:sections=null(服务端 .msg 解析失败的真实形态) ----
T.render({ok: true, ros_available: true, master_ok: true,
  topics: {"/can_msg": {age: 0.2, sections: null},
           "/ehb_msg": {age: 0.2, sections: null}}});
check("降级 sections=null:无字段定义文案", el("body-can").innerHTML
      .indexOf("无字段定义") >= 0);
check("降级 sections=null:不误报总断连", el("d-can").className === "dot ok"
      && el("err").style.display !== "block");

// ---- ros/master off 态(8082 活着但 ROS 不可用):灰点,不得绿 ----
T.render({ok: true, ros_available: false, master_ok: false,
  topics: {"/can_msg": {age: 0.2, sections: []},
           "/ehb_msg": {age: 0.2, sections: []}}});
check("ros/master off:灰点", el("d-ros").className === "dot"
      && el("d-master").className === "dot");
check("ros/master off:话题点不受累", el("d-can").className === "dot ok");

// ---- 故障计数变化闪烁(新故障发生的关键提示时刻) ----
function fsumPayload(v, txt) {
  return {ok: true, ros_available: true, master_ok: true,
          topics: {"/can_msg": {age: 0.3, sections: []},
                   "/ehb_msg": {age: 0.3, sections: [{title: null, fields: [
                     {name: "__faults_0", label: "故障位", note: "",
                      value: v, text: txt}]}]}}};
}
T.render(fsumPayload(0, "全部正常"));
check("故障计数不变不闪烁", el("body-ehb").innerHTML
      .indexOf("fsum ok flash") < 0);
T.render(fsumPayload(1, "电源电压过高"));
check("故障计数 0→1 闪烁", el("body-ehb").innerHTML
      .indexOf("tile fsum bad flash") >= 0);
check("故障计数闪烁后态名点名", el("body-ehb").innerHTML
      .indexOf("电源电压过高") >= 0);

// ---------------- 总断连:指示全灭(不得残留最后一拍全绿) ----------------
(async function () {
  T.render(fakePayload(0.3, 0.4));          // 先恢复全绿基线
  check("断连前基线绿点", el("d-can").className === "dot ok"
        && el("a-can").textContent === "0.3");

  T.render({ok: false});
  check("API 异常:四点全红",
        el("d-ros").className === "dot bad"
        && el("d-master").className === "dot bad"
        && el("d-can").className === "dot bad"
        && el("d-ehb").className === "dot bad");
  check("API 异常:龄清空", el("a-can").textContent === "—"
        && el("a-ehb").textContent === "—");
  check("API 异常:卡片停滞", el("card-can").className === "card can stale"
        && el("card-ehb").className === "card ehb stale");

  // fetch 拒绝路径(poll.catch)
  T.render(fakePayload(0.3, 0.4));          // 再次恢复绿
  T.resetPoll();                            // 清掉首拍挂起留下的 inflight
  global.fetch = () => Promise.reject(new Error("down"));
  T.poll();
  await new Promise((r) => setImmediate(r));
  check("fetch 拒绝:点变红", el("d-can").className === "dot bad"
        && el("d-ehb").className === "dot bad");
  check("fetch 拒绝:横幅+文案", el("err").style.display === "block"
        && el("err").textContent.indexOf("请求失败") >= 0);
  check("fetch 拒绝:卡片停滞", el("card-can").className === "card can stale");

  console.log("\n==== %d passed, %d failed ====", PASS, FAILS.length);
  if (FAILS.length) {
    FAILS.forEach((n) => console.log("  FAIL:", n));
    process.exit(1);
  }
})().catch((e) => { console.error(e); process.exit(1); });
