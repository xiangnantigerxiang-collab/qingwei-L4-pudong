#!/usr/bin/env python3
"""Compile the actual canbus node with ROS/Comply stubs; no CAN hardware I/O.

Message field stubs are generated from this package's real .msg files. The node
source is copied verbatim so only ROS wiring and Comply are replaced, not its
configuration loader, startup parameter publication or T1 state machine.
"""

import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile


PACKAGE = Path(__file__).resolve().parents[1]

ROS_STUB = r'''
#pragma once
#include <map>
#include <string>
#include <vector>
namespace ros {
static std::map<std::string, int> params;
static std::vector<double> periods;
static std::string package_path;
static int loop_hz = 0;
struct TimerEvent {};
struct Subscriber {};
struct Timer {};
struct Duration { double seconds; explicit Duration(double s): seconds(s) {} };
struct Publisher { template<class T> void publish(const T &) {} };
struct TransportHints { TransportHints &tcpNoDelay() { return *this; } };
namespace param {
inline bool get(const std::string &key, int &value) {
    if(params.count(key) == 0) return false;
    value = params[key]; return true;
}
inline void set(const std::string &key, int value) { params[key] = value; }
}
namespace package { inline std::string getPath(const std::string &) { return package_path; } }
struct NodeHandle {
    template<class F> Subscriber subscribe(const char *, int, F, const TransportHints &) {
        return Subscriber();
    }
    template<class T> Publisher advertise(const char *, int) { return Publisher(); }
    template<class F> Timer createTimer(Duration period, F) {
        periods.push_back(period.seconds); return Timer();
    }
};
inline void init(int, char **, const char *) {}
inline bool ok() { return false; }
inline void spinOnce() {}
struct Rate { explicit Rate(int hz) { loop_hz = hz; } void sleep() {} };
struct Time { static Time now() { return Time(); } double toSec() const { return 0; } };
}
'''

COMPLY_STUB = r'''
#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include "ros/package.h"
#include "messages.h"
namespace can_msgs { struct Frame {}; }
struct CanbusComply {
    canbus::can_msg mCanMsg{};
    canbus::ehb_msg mEHBMsg{};
    canbus::can_comm_msg can_comm_cmd{};
    struct { double now = 0; double msgCanComm = 0; } sysTime;
    int EmergencyStop = 0;
    void RecvCanData(const can_msgs::Frame &) {}
    void VehicleComm() {}
};
'''

DRIVER = r'''
#define main CanbusNodeMain
#include "canbus_node.cpp"
#undef main
#include <iostream>
#include <stdexcept>

static int checks = 0;
static void Check(bool condition, const char *description)
{
    ++checks;
    if(!condition) throw std::runtime_error(description);
}

static int Tick(int palletpos, int hookpos = 178)
{
    canbusComply.mCanMsg.palletPos = palletpos;
    canbusComply.mCanMsg.hookPos = hookpos;
    T1Callback(ros::TimerEvent());
    return canbusComply.mCanMsg.palletStatus;
}

static void CheckPalletBands(int upper, int lower, int up_lo, int up_hi, int down_lo, int down_hi)
{
    pallet_position_min = upper;
    pallet_position_max = lower;
    for(int pos = 0; pos <= 255; ++pos) {
        Check(Tick(60) == 0, "leaving endpoint clears status");
        for(int tick = 0; tick < 10; ++tick)
            Check(Tick(pos) == 0, "endpoint must wait for 11 consecutive ticks");
        int expected = 0;
        if(pos >= up_lo && pos <= up_hi) expected = 4;
        if(pos >= down_lo && pos <= down_hi) expected = 3;
        Check(Tick(pos) == expected, "endpoint band or direction mismatch");
    }
}

int main(int argc, char **argv)
{
    if(argc != 3) return 2;
    std::string mode = argv[1];
    if(mode == "load") {
        int rejected = LoadPositionConfig(argv[2]);
        std::cout << "RESULT " << rejected << ' ' << hook_position_min << ' '
                  << hook_position_max << ' ' << pallet_position_min << ' '
                  << pallet_position_max << '\n';
    } else if(mode == "boot") {
        ros::package_path = argv[2];
        CanbusNodeMain(argc, argv);
        Check(ros::periods.size() == 2 && ros::periods[0] == 0.1 && ros::periods[1] == 0.05,
              "T1/T2 periods changed");
        Check(ros::loop_hz == 100, "main loop frequency changed");
        std::cout << "RESULT " << ros::params.at("/canbus/hookposition/min") << ' '
                  << ros::params.at("/canbus/hookposition/max") << ' '
                  << ros::params.at("/canbus/palletposition/min") << ' '
                  << ros::params.at("/canbus/palletposition/max") << '\n';
    } else if(mode == "states") {
        Check(LoadPositionConfig(argv[2]) == 0, "reverse configuration rejected");
        Check(pallet_position_min == 212 && pallet_position_max == 115, "endpoints reordered");
        ros::param::set("/planning/alive", 1);
        for(int i = 0; i < 120; ++i) Check(Tick(122) == 0, "122 must not mean down end at 115");
        CheckPalletBands(212, 115, 208, 216, 111, 119);
        CheckPalletBands(213, 254, 209, 217, 250, 255);
        pallet_position_min = 212;
        pallet_position_max = 115;
        Tick(60);
        for(int i = 0; i < 10; ++i) Check(Tick(115) == 0, "early down end");
        Check(Tick(122) == 0, "outside band must reset debounce");
        for(int i = 0; i < 10; ++i) Check(Tick(115) == 0, "debounce must restart");
        Check(Tick(115) == 3, "down end after renewed 11 ticks");
        for(int i = 0; i < 120; ++i) Check(Tick(115) == 3, "stable endpoint lost after counter cap");
        for(int i = 0; i < 10; ++i) Check(Tick(212) == 0, "opposite end confirmed too early");
        Check(Tick(212) == 4, "upper endpoint direction incorrect");
        for(int i = 0; i < 12; ++i) Tick(60, 178);
        Check(canbusComply.mCanMsg.hookStatus == 4, "hook upper endpoint changed");
        for(int i = 0; i < 11; ++i) Tick(60, 252);
        Check(canbusComply.mCanMsg.hookStatus == 3, "hook lower endpoint changed");
        for(int i = 0; i < 12; ++i) Tick(60, 215);
        Check(canbusComply.mCanMsg.hookStatus == 1, "hook midrange block changed");
        std::cout << "RESULT " << checks << '\n';
    } else return 2;
}
'''


def message_stubs():
    types = {"uint8": "uint8_t", "uint16": "uint16_t", "int16": "int16_t",
             "float32": "float", "bool": "bool"}
    lines = ["#pragma once", "namespace canbus {"]
    for name in ("can_msg", "can_comm_msg", "ehb_msg"):
        lines.append("struct " + name + " {")
        for raw in (PACKAGE / "msg" / (name + ".msg")).read_text().splitlines():
            fields = raw.split("#", 1)[0].split()
            if not fields:
                continue
            msg_type, field = fields
            cpp_type = types[msg_type[:-2] if msg_type.endswith("[]") else msg_type]
            if msg_type.endswith("[]"):
                cpp_type = "std::vector<" + cpp_type + ">"
            lines.append("    " + cpp_type + " " + field + "{};")
        lines.append("};")
    return "\n".join(lines + ["}"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=PACKAGE / "src/canbus_node.cpp")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="canbus_position_verify_") as directory:
        build = Path(directory)
        (build / "ros").mkdir()
        (build / "ros/package.h").write_text(ROS_STUB)
        (build / "canbus_comply.h").write_text(COMPLY_STUB)
        (build / "messages.h").write_text(message_stubs())
        (build / "driver.cpp").write_text(DRIVER)
        shutil.copyfile(args.source, build / "canbus_node.cpp")
        exe = build / "position_verify"
        subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                        "-Wno-unused-parameter", "-Wno-unused-variable",
                        "-Wno-unused-but-set-variable", "-I", str(build),
                        str(build / "driver.cpp"), "-o", str(exe)], check=True)

        def run(mode, path):
            output = subprocess.check_output([str(exe), mode, str(path)], text=True)
            return tuple(map(int, next(line for line in output.splitlines()
                                       if line.startswith("RESULT ")).split()[1:]))

        cases = 0
        fixture = build / "config.cfg"
        hook = "hook_position_min = 178\nhook_position_max = 252\n"

        def check(body, expected, prefix=hook):
            nonlocal cases
            fixture.write_text(prefix + body)
            actual = run("load", fixture)
            assert actual == expected, (body, actual, expected)
            cases += 1

        for upper, lower in ((212, 115), (213, 254), (0, 255), (255, 0),
                             (100, 120), (120, 100)):
            check(f"pallet_position_min = {upper}\npallet_position_max = {lower}\n",
                  (0, 178, 252, upper, lower))
        for upper, lower in ((212, 231), (231, 212), (115, 115), (-1, 115),
                             (256, 115), (212, -1), (212, 256)):
            check(f"pallet_position_min = {upper}\npallet_position_max = {lower}\n",
                  (1, 178, 252, 213, 254))
        for key, other in (("min", "max"), ("max", "min")):
            for bad in ("", "abc", "115x", "115.5", "999999999999999999999999999999"):
                check(f"pallet_position_{other} = 212\npallet_position_{key} = {bad}\n",
                      (1, 178, 252, 213, 254))
        for body in ("pallet_position_min = 212\n", "pallet_position_max = 115\n",
                     "pallet_position_min = 212\npallet_position_max_extra = 115\n",
                     "pallet_position_min = 212\npallet_position_max = 115\npallet_position_max = abc\n"):
            check(body, (1, 178, 252, 213, 254))
        check("\tpallet_position_min\t= +212 # upper\r\n  pallet_position_max = 115\t# lower\r\n",
              (0, 178, 252, 212, 115))
        reverse = "pallet_position_min = 212\npallet_position_max = 115\n"
        check(reverse, (1, 182, 254, 212, 115),
              "hook_position_min = 252\nhook_position_max = 178\n")
        check(reverse, (1, 182, 254, 212, 115), "hook_position_min = 178\n")
        assert run("load", build / "missing.cfg") == (-1, 182, 254, 213, 254)
        assert run("load", PACKAGE / "config.cfg") == (0, 178, 252, 212, 115)
        fixture.write_text(hook + reverse)
        assert run("boot", build) == (178, 252, 212, 115)
        fixture.write_text(hook + "pallet_position_min = 212\npallet_position_max = abc\n")
        assert run("boot", build) == (178, 252, 213, 254)
        fixture.write_text(hook + reverse)
        state_checks, = run("states", fixture)
        print(f"PASS: C++11 node compile; {cases + 4} config/startup cases; "
              f"{state_checks} state assertions; T1/T2/main periods unchanged.")


if __name__ == "__main__":
    main()
