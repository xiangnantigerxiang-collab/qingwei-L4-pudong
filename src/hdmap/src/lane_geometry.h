#ifndef HDMAP_LANE_GEOMETRY_H
#define HDMAP_LANE_GEOMETRY_H

#include "hdmap/lane_types.h"
#include <array>
#include <limits>

namespace hdmap {
    namespace detail {

        struct Vec2 {
            Vec2(double tX = 0, double tY = 0)
                : x(tX), y(tY) {
            }
            double x, y;
            Vec2 operator+(const Vec2& tOther) const {
                return Vec2(x + tOther.x, y + tOther.y);
            }
            Vec2 operator-(const Vec2& tOther) const {
                return Vec2(x - tOther.x, y - tOther.y);
            }
            Vec2 operator*(double tScale) const {
                return Vec2(x * tScale, y * tScale);
            }
        };
        inline double Dot(const Vec2& a, const Vec2& b) {
            return a.x * b.x + a.y * b.y;
        }
        inline double Cross(const Vec2& a, const Vec2& b) {
            return a.x * b.y - a.y * b.x;
        }
        inline Vec2 Left(const Vec2& a) {
            return Vec2(-a.y, a.x);
        }

        struct Aabb {
            Aabb()
                : min(1e100, 1e100), max(-1e100, -1e100) {
            }
            Aabb(const Vec2& a, const Vec2& b);
            void Add(const Vec2& p);
            void Add(const Aabb& b);
            bool Intersects(const Aabb& b) const;
            double DistanceSquared(const Vec2& p) const;
            Vec2 min, max;
        };

        typedef std::vector<Vec2> Polygon;
        struct Ring {
            Polygon points;
            int sign;  // 外环 +1、内洞 -1；求面积时保留内洞，不能把整块孔洞填满。
        };
        struct Segment {
            Segment(const Vec2& a, const Vec2& b, int tLane);
            Vec2 a, b, tangent;
            double length;
            int lane;
        };
        struct Projection {
            Projection()
                : distance_squared(std::numeric_limits<double>::infinity()), segment(0) {
            }
            Vec2 point, tangent;
            double distance_squared;
            std::size_t segment;
        };

        // 索引不改变几何：包围盒下界剪枝，最近点最终投到线段上，而非取最近采样点。
        class AabbIndex {
        public:
            void Build(const std::vector<Aabb>& tBounds);
            void Query(const Aabb& tBounds, std::vector<std::size_t>& tIndices) const;
            Projection Nearest(const Vec2& tPoint, const std::vector<Segment>& tSegments) const;

        private:
            struct Node {
                Aabb bounds;
                std::size_t begin, end;
                int left, right;
            };
            int BuildNode(std::size_t tBegin, std::size_t tEnd);
            void QueryNode(int tNode, const Aabb& tBounds, std::vector<std::size_t>& tIndices) const;
            void NearestNode(int tNode, const Vec2& tPoint,
                             const std::vector<Segment>& tSegments, Projection& tBest) const;
            std::vector<Aabb> mBounds;
            std::vector<std::size_t> mOrder;
            std::vector<Node> mNodes;
        };

        std::array<Vec2, 4> BoxCorners(const OBSTACLE_BOX_S& tBox);
        Polygon ClipPolygon(const Polygon& tPolygon, const std::array<Vec2, 4>& tClip);
        double PolygonArea(const Polygon& tPolygon);
        bool Contains(const std::vector<Ring>& tRings, const Vec2& tPoint);
        double OverlapArea(const std::vector<Ring>& tRings, const std::array<Vec2, 4>& tBox);
        STATUS_S BuildLaneRings(const std::vector<Segment>& tSegments, double tHalfWidth,
                                std::vector<Ring>& tRings);

    }
}
#endif
