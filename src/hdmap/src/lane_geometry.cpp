#include "lane_geometry.h"
#include <boost/polygon/polygon.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>

namespace hdmap {
    namespace detail {

        Aabb::Aabb(const Vec2& a, const Vec2& b)
            : Aabb() {
            Add(a);
            Add(b);
        }
        void Aabb::Add(const Vec2& p) {
            min.x = std::min(min.x, p.x);
            min.y = std::min(min.y, p.y);
            max.x = std::max(max.x, p.x);
            max.y = std::max(max.y, p.y);
        }
        void Aabb::Add(const Aabb& b) {
            Add(b.min);
            Add(b.max);
        }
        bool Aabb::Intersects(const Aabb& b) const {
            return min.x <= b.max.x && max.x >= b.min.x && min.y <= b.max.y && max.y >= b.min.y;
        }
        double Aabb::DistanceSquared(const Vec2& p) const {
            double dx = std::max(0.0, std::max(min.x - p.x, p.x - max.x));
            double dy = std::max(0.0, std::max(min.y - p.y, p.y - max.y));
            return dx * dx + dy * dy;
        }
        Segment::Segment(const Vec2& tA, const Vec2& tB, int tLane)
            : a(tA), b(tB), length(std::hypot(b.x - a.x, b.y - a.y)), lane(tLane) {
            tangent = (b - a) * (1.0 / length);
        }

        void AabbIndex::Build(const std::vector<Aabb>& tBounds) {
            mBounds = tBounds;
            mOrder.resize(mBounds.size());
            std::iota(mOrder.begin(), mOrder.end(), 0);
            mNodes.clear();
            mNodes.reserve(mBounds.size() * 2);
            if(!mBounds.empty()) {
                BuildNode(0, mBounds.size());
            }
        }
        int AabbIndex::BuildNode(std::size_t tBegin, std::size_t tEnd) {
            Node node;
            node.begin = tBegin;
            node.end = tEnd;
            node.left = -1;
            node.right = -1;
            for(std::size_t i = tBegin; i < tEnd; ++i) {
                node.bounds.Add(mBounds[mOrder[i]]);
            }
            int index = static_cast<int>(mNodes.size());
            mNodes.push_back(node);
            if(tEnd - tBegin > 8) {
                bool x_axis = node.bounds.max.x - node.bounds.min.x >= node.bounds.max.y - node.bounds.min.y;
                std::size_t mid = (tBegin + tEnd) / 2;
                std::nth_element(mOrder.begin() + tBegin, mOrder.begin() + mid, mOrder.begin() + tEnd,
                                 [this, x_axis](std::size_t a, std::size_t b) {
                                     return x_axis ? mBounds[a].min.x + mBounds[a].max.x < mBounds[b].min.x + mBounds[b].max.x
                                                   : mBounds[a].min.y + mBounds[a].max.y < mBounds[b].min.y + mBounds[b].max.y;
                                 });
                int left = BuildNode(tBegin, mid);
                int right = BuildNode(mid, tEnd);
                mNodes[index].left = left;
                mNodes[index].right = right;
            }
            return index;
        }
        void AabbIndex::Query(const Aabb& tBounds, std::vector<std::size_t>& tIndices) const {
            tIndices.clear();
            if(!mNodes.empty()) {
                QueryNode(0, tBounds, tIndices);
            }
        }
        void AabbIndex::QueryNode(int tNode, const Aabb& tBounds, std::vector<std::size_t>& tIndices) const {
            const Node& node = mNodes[tNode];
            if(!node.bounds.Intersects(tBounds)) {
                return;
            }
            if(node.left < 0) {
                for(std::size_t i = node.begin; i < node.end; ++i) {
                    if(mBounds[mOrder[i]].Intersects(tBounds)) {
                        tIndices.push_back(mOrder[i]);
                    }
                }
                return;
            }
            QueryNode(node.left, tBounds, tIndices);
            QueryNode(node.right, tBounds, tIndices);
        }
        Projection AabbIndex::Nearest(const Vec2& tPoint, const std::vector<Segment>& tSegments) const {
            Projection best;
            if(!mNodes.empty()) {
                NearestNode(0, tPoint, tSegments, best);
            }
            return best;
        }
        void AabbIndex::NearestNode(int tNode, const Vec2& tPoint,
                                    const std::vector<Segment>& tSegments, Projection& tBest) const {
            const Node& node = mNodes[tNode];
            if(node.bounds.DistanceSquared(tPoint) > tBest.distance_squared) {
                return;
            }
            if(node.left < 0) {
                for(std::size_t i = node.begin; i < node.end; ++i) {
                    std::size_t index = mOrder[i];
                    const Segment& segment = tSegments[index];
                    double along = std::max(0.0, std::min(segment.length, Dot(tPoint - segment.a, segment.tangent)));
                    Vec2 point = segment.a + segment.tangent * along;
                    double distance = Dot(tPoint - point, tPoint - point);
                    if(distance < tBest.distance_squared ||
                       (distance == tBest.distance_squared && index < tBest.segment)) {
                        tBest.point = point;
                        tBest.tangent = segment.tangent;
                        tBest.distance_squared = distance;
                        tBest.segment = index;
                    }
                }
                return;
            }
            int first = node.left, second = node.right;
            if(mNodes[first].bounds.DistanceSquared(tPoint) > mNodes[second].bounds.DistanceSquared(tPoint)) {
                std::swap(first, second);
            }
            NearestNode(first, tPoint, tSegments, tBest);
            NearestNode(second, tPoint, tSegments, tBest);
        }

        std::array<Vec2, 4> BoxCorners(const OBSTACLE_BOX_S& tBox) {
            Vec2 center(tBox.x, tBox.y);
            Vec2 forward(std::cos(tBox.yaw), std::sin(tBox.yaw));
            Vec2 a = forward * (tBox.dx * 0.5), b = Left(forward) * (tBox.dy * 0.5);
            return {{center - a - b, center + a - b, center + a + b, center - a + b}};
        }

        // 用凸框逐边裁剪有向环。凹环裁剪后的分离部分由往返边相连，面积相互抵消，
        // 因此可直接计算总交面积；内洞按负面积扣除，不能逐段矩形累加重复覆盖。
        Polygon ClipPolygon(const Polygon& tPolygon, const std::array<Vec2, 4>& tClip) {
            Polygon input = tPolygon, output;
            output.reserve(input.size() + 8);
            for(std::size_t edge = 0; edge < 4 && !input.empty(); ++edge) {
                output.clear();
                Vec2 a = tClip[edge], direction = tClip[(edge + 1) % 4] - a;
                Vec2 previous = input.back();
                double previous_side = Cross(direction, previous - a);
                for(std::size_t i = 0; i < input.size(); ++i) {
                    Vec2 current = input[i];
                    double current_side = Cross(direction, current - a);
                    if((current_side >= 0) != (previous_side >= 0)) {
                        double ratio = previous_side / (previous_side - current_side);
                        output.push_back(previous + (current - previous) * ratio);
                    }
                    if(current_side >= 0) {
                        output.push_back(current);
                    }
                    previous = current;
                    previous_side = current_side;
                }
                input.swap(output);
            }
            return input;
        }
        double PolygonArea(const Polygon& tPolygon) {
            if(tPolygon.size() < 3) {
                return 0;
            }
            double area = 0;
            // 平移后算叉积，避免大地图坐标的平方项相减损失精度。
            for(std::size_t i = 1; i + 1 < tPolygon.size(); ++i) {
                area += Cross(tPolygon[i] - tPolygon[0], tPolygon[i + 1] - tPolygon[0]);
            }
            return std::abs(area) * 0.5;
        }
        bool Contains(const std::vector<Ring>& tRings, const Vec2& tPoint) {
            int coverage = 0;
            for(std::size_t r = 0; r < tRings.size(); ++r) {
                const Polygon& points = tRings[r].points;
                bool inside = false;
                for(std::size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
                    Vec2 a = points[j], b = points[i], d = b - a;
                    double length = std::hypot(d.x, d.y);
                    if(length > 1e-12 && std::abs(Cross(d, tPoint - a)) <= 1e-8 * length &&
                       Dot(tPoint - a, tPoint - b) <= 1e-16) {
                        return true;
                    }
                    if((a.y > tPoint.y) != (b.y > tPoint.y) &&
                       tPoint.x < a.x + (tPoint.y - a.y) * (b.x - a.x) / (b.y - a.y)) {
                        inside = !inside;
                    }
                }
                if(inside) {
                    coverage += tRings[r].sign;
                }
            }
            return coverage > 0;
        }
        double OverlapArea(const std::vector<Ring>& tRings, const std::array<Vec2, 4>& tBox) {
            double area = 0;
            for(std::size_t i = 0; i < tRings.size(); ++i) {
                area += tRings[i].sign * PolygonArea(ClipPolygon(tRings[i].points, tBox));
            }
            return std::max(0.0, area);
        }

        namespace {
            namespace bp = boost::polygon;
            typedef std::int64_t Integer;
            const double SCALE = 100000.0;  // 初始化并集使用 0.01 毫米定点网格，避免拓扑浮点歧义。

            Polygon ConvexHull(Polygon points) {
                std::sort(points.begin(), points.end(), [](const Vec2& a, const Vec2& b) {
                    return a.x < b.x || (a.x == b.x && a.y < b.y);
                });
                points.erase(std::unique(points.begin(), points.end(), [](const Vec2& a, const Vec2& b) {
                                 return a.x == b.x && a.y == b.y;
                             }),
                             points.end());
                if(points.size() < 3) {
                    return points;
                }
                Polygon hull;
                for(std::size_t i = 0; i < points.size(); ++i) {
                    while(hull.size() >= 2 && Cross(hull.back() - hull[hull.size() - 2], points[i] - hull.back()) <= 0) {
                        hull.pop_back();
                    }
                    hull.push_back(points[i]);
                }
                std::size_t lower = hull.size();
                for(std::size_t i = points.size() - 1; i-- > 0;) {
                    while(hull.size() > lower && Cross(hull.back() - hull[hull.size() - 2], points[i] - hull.back()) <= 0) {
                        hull.pop_back();
                    }
                    hull.push_back(points[i]);
                }
                hull.pop_back();
                return hull;
            }

            void InsertPolygon(bp::polygon_set_data<Integer>& tSet, const Polygon& tPoints, const Vec2& tOrigin) {
                std::vector<bp::point_data<Integer>> points;
                for(std::size_t i = 0; i < tPoints.size(); ++i) {
                    points.push_back(bp::point_data<Integer>(
                        static_cast<Integer>(std::llround((tPoints[i].x - tOrigin.x) * SCALE)),
                        static_cast<Integer>(std::llround((tPoints[i].y - tOrigin.y) * SCALE))));
                }
                bp::polygon_data<Integer> polygon;
                bp::set_points(polygon, points.begin(), points.end());
                tSet.insert(polygon);
            }
            template <typename T>
            Ring ReadRing(const T& tPolygon, int tSign, const Vec2& tOrigin) {
                Ring ring;
                ring.sign = tSign;
                for(auto it = bp::begin_points(tPolygon); it != bp::end_points(tPolygon); ++it) {
                    ring.points.push_back(Vec2(bp::x(*it) / SCALE + tOrigin.x, bp::y(*it) / SCALE + tOrigin.y));
                }
                return ring;
            }
        }

        STATUS_S BuildLaneRings(const std::vector<Segment>& tSegments, double tHalfWidth,
                                std::vector<Ring>& tRings) {
            bp::polygon_set_data<Integer> occupied;
            Vec2 origin = tSegments.front().a;
            bool closed = std::hypot(tSegments.back().b.x - origin.x, tSegments.back().b.y - origin.y) <= 1e-6;
            std::vector<Vec2> miters(tSegments.size() + 1);
            std::vector<bool> bevels(tSegments.size() + 1, false);
            miters.front() = Left(tSegments.front().tangent) * tHalfWidth;
            miters.back() = Left(tSegments.back().tangent) * tHalfWidth;
            for(std::size_t i = 0; i < tSegments.size(); ++i) {
                if(i == 0 && !closed) {
                    continue;
                }
                const Segment& previous = i == 0 ? tSegments.back() : tSegments[i - 1];
                double denominator = 1.0 + Dot(previous.tangent, tSegments[i].tangent);
                Vec2 miter = denominator > 1e-8 ? Left(previous.tangent + tSegments[i].tangent) *
                                                      (tHalfWidth / denominator)
                                                : Vec2();
                bevels[i] = denominator <= 1e-8 || Dot(miter, miter) > 4 * tHalfWidth * tHalfWidth;
                miters[i] = miter;
            }
            if(closed) {
                miters.back() = miters.front();
                bevels.back() = bevels.front();
            }
            for(std::size_t i = 0; i < tSegments.size(); ++i) {
                const Segment& segment = tSegments[i];
                Vec2 normal = Left(segment.tangent) * tHalfWidth;
                Polygon strip{segment.a - normal, segment.b - normal, segment.b + normal, segment.a + normal};
                // 相邻条带共用同一对斜接端点，避免各自量化端面后在中心线节点处产生微小裂缝。
                if(!bevels[i]) {
                    strip.push_back(segment.a - miters[i]);
                    strip.push_back(segment.a + miters[i]);
                }
                if(!bevels[i + 1]) {
                    strip.push_back(segment.b - miters[i + 1]);
                    strip.push_back(segment.b + miters[i + 1]);
                }
                InsertPolygon(occupied, ConvexHull(strip), origin);
                if(bevels[i]) {
                    Vec2 previous = Left(i == 0 ? tSegments.back().tangent : tSegments[i - 1].tangent) * tHalfWidth;
                    // 尖锐掉头改斜切接头，同时覆盖两条端面，限制外侧尖角长度。
                    InsertPolygon(occupied, ConvexHull({segment.a + previous, segment.a - previous, segment.a + normal, segment.a - normal}), origin);
                }
            }
            // 同一车道的所有线段先求并集：弯道、回环和交叉处的面积只能计一次。
            std::vector<bp::polygon_with_holes_data<Integer>> polygons;
            occupied.get(polygons);
            tRings.clear();
            for(std::size_t i = 0; i < polygons.size(); ++i) {
                tRings.push_back(ReadRing(polygons[i], 1, origin));
                for(auto hole = bp::begin_holes(polygons[i]); hole != bp::end_holes(polygons[i]); ++hole) {
                    tRings.push_back(ReadRing(*hole, -1, origin));
                }
            }
            if(tRings.empty()) {
                return STATUS_S(INVALID_DATA, "车道缓冲区域为空");
            }
            return STATUS_S();
        }

    }
}
