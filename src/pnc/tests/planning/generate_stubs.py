#!/usr/bin/env python3
"""Generate ROS transport stubs; robot messages always come from the real .msg files."""

import re
import json
from pathlib import Path


def generate(pnc, output):
    def write(name, text):
        target = output / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text)

    write("ros/ros.h", r'''#pragma once
#include <array>
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>
namespace ros {
inline double &testTime() { static double time = 100.0; return time; }
struct Time {
    double value;
    Time(double v = 0) : value(v) {}
    double toSec() const { return value; }
    static Time now() { return Time(testTime()); }
};
struct Duration { explicit Duration(double) {} };
struct WallTime {
    static WallTime now() { return WallTime(); }
    boost::posix_time::ptime toBoost() const {
        return boost::posix_time::ptime(boost::gregorian::date(2026, 9, 14));
    }
};
struct TimerEvent {};
struct Timer {};
struct Subscriber {};
struct TransportHints { TransportHints &tcpNoDelay() { return *this; } };
namespace param {
inline std::map<std::string, std::string> &values() {
    static std::map<std::string, std::string> data; return data;
}
inline std::vector<std::string> &events() {
    static std::vector<std::string> data; return data;
}
template<class T> bool get(const std::string &key, T &value) {
    events().push_back("get:" + key);
    auto it = values().find(key);
    if (it == values().end()) return false;
    std::istringstream stream(it->second); stream >> value; return !stream.fail();
}
inline bool get(const std::string &key, std::string &value) {
    events().push_back("get:" + key);
    auto it = values().find(key);
    if (it == values().end()) return false;
    value = it->second; return true;
}
template<class T> bool getCached(const std::string &key, T &value) { return get(key, value); }
template<class T> void set(const std::string &key, const T &value) {
    std::ostringstream stream; stream.precision(17); stream << value;
    values()[key] = stream.str(); events().push_back("set:" + key + "=" + stream.str());
}
}
struct Publisher {
    std::string name;
    std::function<void(const std::type_info &, const void *)> capture;
    template<class T> void publish(const T &msg) const {
        param::events().push_back("publish:" + name);
        if (capture) capture(typeid(T), &msg);
    }
};
struct NodeHandle {
    template<class... T> Subscriber subscribe(T&&...) { return Subscriber(); }
    template<class T> Publisher advertise(const std::string &name, int, bool = false) {
        Publisher pub; pub.name = name; return pub;
    }
    template<class... T> Timer createTimer(T&&...) { return Timer(); }
    template<class T> bool getParam(const std::string &key, T &value) { return param::get(key, value); }
    template<class T> void setParam(const std::string &key, const T &value) { param::set(key, value); }
    template<class T> void param(const std::string &key, T &value, const T &fallback) {
        if (!param::get(key, value)) value = fallback;
    }
};
struct Rate { explicit Rate(double) {} void sleep() {} };
inline bool ok() { return false; }
inline void spinOnce() {}
inline void init(int, char **, const char *) {}
namespace message_traits {
struct TrueType {}; struct FalseType {};
template<class T> struct IsMessage; template<class T> struct IsFixedSize;
template<class T> struct HasHeader; template<class T> struct MD5Sum;
template<class T> struct DataType; template<class T> struct Definition;
}
namespace serialization { template<class T> struct Serializer; }
namespace message_operations {
template<class T> struct Printer {
    template<class S> static void stream(S &, const std::string &, const T &) {}
};
}
}
#define ROS_DECLARE_ALLINONE_SERIALIZER
#define ROS_INFO(...) ((void)0)
#define ROS_WARN(...) ((void)0)
#define ROS_ERROR(...) ((void)0)
#define ROS_DEBUG(...) ((void)0)
''')
    for name in ("types", "serialization", "builtin_message_traits", "message_operations"):
        write("ros/" + name + ".h", '#pragma once\n#include "ros/ros.h"\n')

    write("ros/package.h", '''#pragma once
#include "ros/ros.h"
namespace ros { namespace package {
inline std::map<std::string, std::string> &paths() {
    static std::map<std::string, std::string> data = {{"robot", @PNC@}, {"hdmap", @HDMAP@}};
    return data;
}
inline std::string getPath(const std::string &name) {
    auto it = paths().find(name);
    return it == paths().end() ? std::string() : it->second;
}
}}
'''.replace("@PNC@", json.dumps(str(pnc.resolve()))).replace(
        "@HDMAP@", json.dumps(str((pnc.parent / "hdmap").resolve()))))

    specs = {
        "std_msgs/Header": "uint32 seq\ntime stamp\nstring frame_id",
        "std_msgs/Bool": "bool data",
        "std_msgs/String": "string data",
        "std_msgs/ColorRGBA": "float32 r\nfloat32 g\nfloat32 b\nfloat32 a",
        "geometry_msgs/Point": "float64 x\nfloat64 y\nfloat64 z",
        "geometry_msgs/Vector3": "float64 x\nfloat64 y\nfloat64 z",
        "geometry_msgs/Quaternion": "float64 x\nfloat64 y\nfloat64 z\nfloat64 w",
        "geometry_msgs/Pose": "Point position\nQuaternion orientation",
        "geometry_msgs/PoseStamped": "std_msgs/Header header\nPose pose",
        "sensor_msgs/CompressedImage": "std_msgs/Header header\nstring format\nuint8[] data",
        "jsk_recognition_msgs/BoundingBox": "std_msgs/Header header\ngeometry_msgs/Pose pose\ngeometry_msgs/Vector3 dimensions\nfloat32 value\nuint32 label",
        "jsk_recognition_msgs/BoundingBoxArray": "std_msgs/Header header\nBoundingBox[] boxes",
        "nav_msgs/MapMetaData": "float32 resolution\nuint32 width\nuint32 height\ngeometry_msgs/Pose origin",
        "nav_msgs/OccupancyGrid": "std_msgs/Header header\nMapMetaData info\nint8[] data",
        "nav_msgs/Path": "std_msgs/Header header\ngeometry_msgs/PoseStamped[] poses",
        "visualization_msgs/Marker": "int32 ARROW=0\nint32 CUBE=1\nint32 LINE_STRIP=4\nint32 POINTS=8\nint32 ADD=0\nstd_msgs/Header header\nstring ns\nint32 id\nint32 type\nint32 action\ngeometry_msgs/Pose pose\ngeometry_msgs/Vector3 scale\nstd_msgs/ColorRGBA color\ngeometry_msgs/Point[] points",
        "visualization_msgs/MarkerArray": "Marker[] markers",
    }
    for path in sorted((pnc / "msg").glob("*.msg")):
        specs["robot/" + path.stem] = path.read_text()
    # 同工作空间包或根目录模块均从真实消息生成，不手写闸机消息字段。
    for folder in (pnc.parent / "gantry_detect", pnc.parent.parent / "gantry_detect"):
        if (folder / "msg/gantry_state.msg").is_file():
            for path in sorted((folder / "msg").glob("*.msg")):
                specs["gantry_detect/" + path.stem] = path.read_text()
            break

    scalar = {name: name + "_t" for name in (
        "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64")}
    scalar.update({"float32": "float", "float64": "double", "string": "std::string",
                   "bool": "bool", "time": "ros::Time", "byte": "int8_t", "char": "uint8_t"})
    for full_name, spec in specs.items():
        namespace, name = full_name.split("/")
        includes, fields, constants = [], [], []
        for raw in spec.splitlines():
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            field_type, field_name = line.split(None, 1)
            match = re.fullmatch(r"(\w+(?:/\w+)?)(?:\[(\d*)\])?", field_type)
            base, count = match.groups()
            cpp_type = scalar.get(base)
            if cpp_type is None:
                dependency = base if "/" in base else namespace + "/" + base
                includes.append('#include "' + dependency + '.h"')
                cpp_type = dependency.replace("/", "::")
            if "=" in field_name:
                key, value = field_name.split("=", 1)
                constants.append("    static constexpr " + cpp_type + " " + key.strip() + " = " + value.strip() + ";")
                continue
            if count is not None:
                cpp_type = "std::vector<" + cpp_type + ">" if count == "" else "boost::array<" + cpp_type + ", " + count + ">"
            fields.append("    " + cpp_type + " " + field_name.strip() + "{};")
        write(full_name + ".h", '\n'.join([
            '#pragma once', '#include "ros/ros.h"', *dict.fromkeys(includes),
            "namespace " + namespace + " {", "struct " + name + " {",
            "    typedef boost::shared_ptr<const " + name + "> ConstPtr;",
            *constants, *fields, "};", "typedef " + name + "::ConstPtr " + name + "ConstPtr;",
            "template<class Allocator> using " + name + "_ = " + name + ";", "}", ""]))

    write("tf/tf.h", '''#pragma once
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
namespace tf {
inline geometry_msgs::Quaternion createQuaternionMsgFromYaw(double yaw) {
    geometry_msgs::Quaternion q; q.z = std::sin(yaw / 2); q.w = std::cos(yaw / 2); return q;
}
inline double getYaw(const geometry_msgs::Quaternion &q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
}
}
''')
    for name in ("tf2/LinearMath/Transform.h", "tf2/LinearMath/Quaternion.h",
                 "tf2_geometry_msgs/tf2_geometry_msgs.h", "tf/LinearMath/Transform.h",
                 "tf/LinearMath/Matrix3x3.h", "tf/transform_broadcaster.h", "tf/transform_datatypes.h"):
        write(name, '#pragma once\n#include "tf/tf.h"\n')


if __name__ == "__main__":
    import sys
    generate(Path(sys.argv[1]), Path(sys.argv[2]))
