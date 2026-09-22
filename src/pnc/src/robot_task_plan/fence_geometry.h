#ifndef ROBOT_TASK_FENCE_GEOMETRY_H
#define ROBOT_TASK_FENCE_GEOMETRY_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <vector>
#include "common/path_csv.h"

// task_plan 的围栏几何；planning 只复用此校验器保护最终输出，不复制判据。
namespace task_fence {

struct Point {
    double x, y;
    Point(double px = 0.0, double py = 0.0) : x(px), y(py) {}
};

inline uint64_t HashBytes(uint64_t hash, const void *data, size_t size) {
    const auto *bytes = static_cast<const unsigned char *>(data);
    for(size_t i = 0; i < size; ++i) hash = (hash ^ bytes[i]) * 1099511628211ULL;
    return hash;
}

inline uint64_t HashStart() { return 14695981039346656037ULL; }
inline uint64_t HashWord(uint64_t hash, uint64_t word) {
    // 明确字节序，节点架构变化也不改变指纹。
    for(int i = 0; i < 8; ++i) { const unsigned char b = word & 255; hash = HashBytes(hash, &b, 1); word >>= 8; }
    return hash;
}
inline uint64_t HashFloat(uint64_t hash, float value) {
    if(value == 0.0f) value = 0.0f;
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return HashWord(hash, bits);
}
inline uint64_t HashPose(uint64_t hash, float x, float y, float heading) {
    return HashFloat(HashFloat(HashFloat(hash, x), y), heading);
}

template<class Task> uint64_t TaskKey(const Task &task) {
    uint64_t hash = HashWord(HashStart(), task.task_id);
    hash = HashWord(hash, task.taskType);
    hash = HashWord(hash, task.workMode);
    hash = HashWord(hash, task.desireGear);
    hash = HashWord(hash, task.hookCmd);
    hash = HashPose(hash, task.stopX, task.stopY, task.stopAngle);
    hash = HashFloat(hash, task.desireSpeed);
    for(const auto &name : task.pathList) {
        hash = HashWord(hash, name.size());
        hash = HashBytes(hash, name.data(), name.size());
    }
    return hash;
}

struct FileStamp {
    uint64_t device = 0, inode = 0, size = 0;
    int64_t modified = 0, modified_ns = 0, changed = 0, changed_ns = 0;
    bool valid = false;
    static FileStamp Read(const std::string &file) {
        struct stat s;
        FileStamp result;
        if(::stat(file.c_str(), &s) != 0 || !S_ISREG(s.st_mode)) return result;
        result.device = s.st_dev; result.inode = s.st_ino; result.size = s.st_size;
        result.modified = s.st_mtim.tv_sec; result.modified_ns = s.st_mtim.tv_nsec;
        result.changed = s.st_ctim.tv_sec; result.changed_ns = s.st_ctim.tv_nsec;
        result.valid = true;
        return result;
    }
    bool operator==(const FileStamp &b) const {
        return valid && b.valid && device == b.device && inode == b.inode && size == b.size &&
               modified == b.modified && modified_ns == b.modified_ns && changed == b.changed && changed_ns == b.changed_ns;
    }
};

class Geometry {
public:
    void Clear() { mPoints.clear(); mEdges.clear(); mStamp = FileStamp(); mVersion = 0; }
    bool Valid() const { return mPoints.size() >= 3; }
    uint64_t Version() const { return mVersion; }
    const std::vector<Point> &Points() const { return mPoints; }
    const std::string &File() const { return mFile; }

    bool Load(const std::string &file, std::string &error) {
        const FileStamp stamp = FileStamp::Read(file);
        if(file == mFile && stamp == mStamp && Valid()) return true;
        Clear(); mFile = file;
        if(!stamp.valid || stamp.size > 4 * 1024 * 1024) { error = "fence file unavailable"; return false; }
        std::ifstream input(file.c_str());
        std::vector<Point> points;
        std::string line;
        while(std::getline(input, line)) {
            path_csv::ROW_S row;
            const auto status = path_csv::ParseRow(line, row);
            if(status == path_csv::SKIP_ROW) continue;
            if(status != path_csv::VALID_ROW) { error = "invalid fence CSV row"; return false; }
            Point p(static_cast<float>(row.values[0]), static_cast<float>(row.values[1]));
            if(!std::isfinite(p.x) || !std::isfinite(p.y)) { error = "non-finite fence coordinate"; return false; }
            if(!points.empty() && Equal(points.back(), p)) continue;
            points.push_back(p);
            if(points.size() > 4096) { error = "fence vertex limit exceeded"; return false; }
        }
        if(input.bad() || !(stamp == FileStamp::Read(file))) { error = "fence changed while reading"; return false; }
        if(points.size() > 1 && Equal(points.front(), points.back())) points.pop_back();
        if(points.size() < 3) { error = "fence needs at least three vertices"; return false; }
        double area = 0.0;
        for(size_t i = 0; i < points.size(); ++i) {
            const Point &a = points[i], &b = points[(i + 1) % points.size()];
            area += a.x * b.y - a.y * b.x;
            for(size_t j = i + 2; j < points.size(); ++j) {
                if(i == 0 && j + 1 == points.size()) continue;
                if(Touch(a, b, points[j], points[(j + 1) % points.size()])) {
                    error = "self-intersecting fence"; return false;
                }
            }
        }
        if(std::abs(area) < 1e-6) { error = "zero-area fence"; return false; }
        mPoints.swap(points); mStamp = stamp; mVersion = HashStart();
        mMinX = mMaxX = mPoints[0].x; mMinY = mMaxY = mPoints[0].y;
        for(size_t i = 0; i < mPoints.size(); ++i) {
            const auto &p = mPoints[i];
            mMinX = std::min(mMinX, p.x); mMaxX = std::max(mMaxX, p.x);
            mMinY = std::min(mMinY, p.y); mMaxY = std::max(mMaxY, p.y);
            mVersion = HashFloat(HashFloat(mVersion, p.x), p.y);
            mEdges.emplace_back(p, mPoints[(i + 1) % mPoints.size()]);
        }
        return true;
    }

    bool Contains(const Point &p) const {
        if(!Valid() || !std::isfinite(p.x) || !std::isfinite(p.y) ||
           p.x < mMinX || p.x > mMaxX || p.y < mMinY || p.y > mMaxY) return false;
        bool inside = false;
        for(const auto &edge : mEdges) {
            if(OnEdge(edge.a, edge.b, p)) return false; // 原 control：压线也不属于栏内。
            if((edge.a.y > p.y) != (edge.b.y > p.y) &&
               p.x < edge.a.x + (p.y - edge.a.y) * (edge.b.x - edge.a.x) / (edge.b.y - edge.a.y)) inside = !inside;
        }
        return inside;
    }

    bool SegmentInside(const Point &a, const Point &b, bool endpoints_checked = false) const {
        if(!endpoints_checked && (!Contains(a) || !Contains(b))) return false;
        for(const auto &edge : mEdges) if(Touch(a, b, edge.a, edge.b)) return false;
        return true;
    }

    static Point Front(double x, double y, double heading, double side) {
        const double angle = (90.0 - heading) * 0.017453292519943295;
        return Point(x + 2.3 * std::cos(angle) - side * std::sin(angle),
                     y + 2.3 * std::sin(angle) + side * std::cos(angle));
    }
    bool PoseInside(double x, double y, double heading) const {
        if(!std::isfinite(heading) || !Contains(Point(x, y))) return false;
        return SegmentInside(Front(x, y, heading, 0.73), Front(x, y, heading, -0.73));
    }

private:
    struct Edge { Point a, b; Edge(Point p, Point q) : a(p), b(q) {} };
    static bool Equal(const Point &a, const Point &b) { return a.x == b.x && a.y == b.y; }
    static double Cross(const Point &a, const Point &b, const Point &p) {
        return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    }
    static bool OnEdge(const Point &a, const Point &b, const Point &p) {
        return std::abs(Cross(a, b, p)) <= 1e-9 && p.x >= std::min(a.x, b.x) &&
               p.x <= std::max(a.x, b.x) && p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y);
    }
    static bool Touch(const Point &a, const Point &b, const Point &c, const Point &d) {
        if(std::max(a.x, b.x) < std::min(c.x, d.x) || std::max(c.x, d.x) < std::min(a.x, b.x) ||
           std::max(a.y, b.y) < std::min(c.y, d.y) || std::max(c.y, d.y) < std::min(a.y, b.y)) return false;
        const double ab_c = Cross(a, b, c), ab_d = Cross(a, b, d), cd_a = Cross(c, d, a), cd_b = Cross(c, d, b);
        return ((ab_c > 0) != (ab_d > 0) && (cd_a > 0) != (cd_b > 0)) ||
               OnEdge(a, b, c) || OnEdge(a, b, d) || OnEdge(c, d, a) || OnEdge(c, d, b);
    }
    std::string mFile;
    FileStamp mStamp;
    std::vector<Point> mPoints;
    std::vector<Edge> mEdges;
    uint64_t mVersion = 0;
    double mMinX = 0, mMinY = 0, mMaxX = 0, mMaxY = 0;
};
}
#endif
