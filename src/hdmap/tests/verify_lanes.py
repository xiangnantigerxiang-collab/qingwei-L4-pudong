#!/usr/bin/env python3
"""用 GEOS 完整车道 buffer/intersection 和 NumPy 全扫描校验 .so 的索引及分块结果。

仅测试依赖 NumPy、libgeos_c；不参与 SDK 编译或运行，不需要 Shapely。
"""

import argparse
import ctypes as ct
import ctypes.util
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

import numpy as np


class Geometry:
    def __init__(self):
        self.lib = ct.CDLL(ctypes.util.find_library("geos_c"))
        signatures = {
            "initGEOS": (None, [ct.c_void_p, ct.c_void_p]),
            "GEOSGeomFromWKT": (ct.c_void_p, [ct.c_char_p]),
            "GEOSBufferWithStyle": (ct.c_void_p, [ct.c_void_p, ct.c_double, ct.c_int,
                                                   ct.c_int, ct.c_int, ct.c_double]),
            "GEOSIntersection": (ct.c_void_p, [ct.c_void_p, ct.c_void_p]),
            "GEOSArea": (ct.c_int, [ct.c_void_p, ct.POINTER(ct.c_double)]),
            "GEOSCovers": (ct.c_char, [ct.c_void_p, ct.c_void_p]),
            "GEOSGeom_destroy": (None, [ct.c_void_p]),
        }
        for name, (result, arguments) in signatures.items():
            function = getattr(self.lib, name)
            function.restype, function.argtypes = result, arguments
        self.lib.initGEOS(None, None)

    def read(self, kind, points):
        coordinates = ",".join(f"{x:.17g} {y:.17g}" for x, y in points)
        text = f"POLYGON(({coordinates}))" if kind == "POLYGON" else f"{kind}({coordinates})"
        result = self.lib.GEOSGeomFromWKT(text.encode())
        assert result, text
        return result

    def destroy(self, geometry):
        self.lib.GEOSGeom_destroy(geometry)

    def buffer(self, points):
        line = self.read("LINESTRING", points)
        result = self.lib.GEOSBufferWithStyle(line, 2.0, 8, 2, 2, 2.0)
        self.destroy(line)
        assert result
        return result

    def covers(self, region, point):
        shape = self.read("POINT", [point])
        result = self.lib.GEOSCovers(region, shape)
        self.destroy(shape)
        assert result in (b"\x00", b"\x01")
        return result == b"\x01"

    def overlap(self, region, points):
        rectangle = self.read("POLYGON", [*points, points[0]])
        intersection = self.lib.GEOSIntersection(region, rectangle)
        assert intersection
        result = ct.c_double()
        assert self.lib.GEOSArea(intersection, ct.byref(result)) == 1
        self.destroy(intersection)
        self.destroy(rectangle)
        return result.value


def read_map(directory):
    paths, arrays = [], []
    for path in sorted(directory.glob("*.csv")):
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        paths.append(path)
        arrays.append(np.array([[float(row["x"]), float(row["y"])] for row in rows]))
    return paths, arrays


def closest(points, query):
    a, vectors = points[:-1], np.diff(points, axis=0)
    squared = np.einsum("ij,ij->i", vectors, vectors)
    along = np.clip(np.einsum("ij,ij->i", query - a, vectors) / squared, 0, 1)
    feet = a + along[:, None] * vectors
    distances = np.sum((feet - query)**2, axis=1)
    index = int(np.argmin(distances))
    return feet[index], vectors[index] / math.sqrt(squared[index]), distances[index]


def expected(geometry, arrays, regions, pose, boxes):
    point = np.array(pose[:2])
    yaw = math.radians(90 - pose[3])
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    left = np.array([-forward[1], forward[0]])
    membership = []
    for lane, (points, region) in enumerate(zip(arrays, regions)):
        if geometry.covers(region, point):
            projection, tangent, distance = closest(points, point)
            membership.append((distance, -abs(tangent @ forward), lane, tangent))
    ego = min(membership, key=lambda item: item[:3]) if membership else None
    results = []
    for x, y, dx, dy, angle in boxes:
        center = np.array([x, y])
        outside = 3 if (center - point) @ left >= -1e-10 else 4
        if ego is None:
            results.append((outside, -1, 0.0))
            continue
        ego_lane = ego[2]
        foot, tangent, _ = closest(arrays[ego_lane], center)
        tangent *= -1 if ego[3] @ forward < 0 else 1
        normal = np.array([-tangent[1], tangent[0]])
        neighbors = []
        # 全扫描所有线段与横断面的相交点，不使用 SDK 的 BVH、缓存或车道排序结果。
        for lane, points in enumerate(arrays):
            if lane == ego_lane:
                continue
            vectors = np.diff(points, axis=0)
            lengths = np.linalg.norm(vectors, axis=1)
            start = (points[:-1] - foot) @ tangent
            denominator = vectors @ tangent
            valid = np.abs(denominator / lengths) >= math.cos(math.pi / 4)
            ratio = np.divide(-start, denominator, out=np.full(len(start), np.inf), where=valid)
            valid &= (ratio >= -1e-8) & (ratio <= 1 + 1e-8)
            if not valid.any():
                continue
            crossings = points[:-1][valid] + np.clip(ratio[valid], 0, 1)[:, None] * vectors[valid]
            offsets = (crossings - foot) @ normal
            offsets = offsets[(offsets > 1e-6) & (offsets <= 12 + 1e-8)]
            if len(offsets):
                neighbors.append((offsets.min(), lane))
        relatives = [ego_lane] + [item[1] for item in sorted(neighbors)[:2]]
        u = np.array([math.cos(angle), math.sin(angle)]) * dx / 2
        v = np.array([-math.sin(angle), math.cos(angle)]) * dy / 2
        corners = [center - u - v, center + u - v, center + u + v, center - u + v]
        best_type, best_lane, best_area, best_distance = outside, -1, 0.0, math.inf
        for type_value, lane in enumerate(relatives):
            area = geometry.overlap(regions[lane], corners)
            distance = closest(arrays[lane], center)[2]
            tie = max(1e-9, dx * dy * 1e-8)
            if area > 1e-10 and (best_lane < 0 or area > best_area + tie or
                (abs(area - best_area) <= tie and distance < best_distance - 1e-10)):
                best_type, best_lane, best_area, best_distance = type_value, lane, area, distance
        results.append((best_type, best_lane, best_area))
    return results


def run(probe, directory, frames, grid=8):
    lines = []
    for pose, boxes in frames:
        lines.append(" ".join(map(str, [*pose, len(boxes)])))
        lines.extend(" ".join(map(str, box)) for box in boxes)
    completed = subprocess.run([str(probe), str(directory), str(grid)], input="\n".join(lines) + "\n",
                               text=True, capture_output=True, check=True)
    return [[float(value) for value in line.split(",")] for line in completed.stdout.splitlines()]


def verify(probe, directory, frames, geometry, check_grid=False):
    paths, arrays = read_map(directory)
    regions = [geometry.buffer(points) for points in arrays]
    actual = run(probe, directory, frames)
    index, max_error = 0, 0
    try:
        for frame, (pose, boxes) in enumerate(frames):
            oracle = expected(geometry, arrays, regions, pose, boxes)
            for obstacle, (type_value, lane, area) in enumerate(oracle):
                row = actual[index]
                assert row[:2] == [frame, obstacle]
                assert row[2:4] == [type_value, lane], (directory.name, frame, obstacle, row, oracle[obstacle], pose, boxes[obstacle])
                error = abs(row[4] - area)
                # 生产并集有 0.01 毫米定点量化，GEOS 使用浮点；阈值只用于面积，不放宽 type。
                assert error < max(0.003, area * 1e-5), (directory.name, frame, obstacle, row[4], area)
                max_error = max(max_error, error)
                index += 1
        assert index == len(actual)
        if check_grid:
            for grid in (4, 16):
                alternative = run(probe, directory, frames, grid)
                assert len(alternative) == len(actual)
                for a, b in zip(actual, alternative):
                    assert a[:4] == b[:4] and abs(a[4] - b[4]) < 1e-6, ("grid-dependent result", a, b)
    finally:
        for region in regions:
            geometry.destroy(region)
    print(f"PASS {directory.name}: {index} boxes, max area difference {max_error:.6g} m²", flush=True)
    return {"dataset": directory.name, "frames": len(frames), "boxes": index,
            "max_area_difference_m2": max_error, "grid_independence": check_grid,
            "sources": {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}}


def write_map(directory, name, points):
    delta = np.diff(points, axis=0)
    lengths = np.r_[0, np.linalg.norm(delta, axis=1)]
    headings = (90 - np.rad2deg(np.arctan2(delta[:, 1], delta[:, 0]))) % 360
    headings = np.r_[headings, headings[-1]]
    with (directory / (name + ".csv")).open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(["x", "y", "heading", "curvature", "signed_curvature", "dist_origin", "p2p_distance", "direction"])
        for (x, y), heading, distance, chord in zip(points, headings, np.cumsum(lengths), lengths):
            writer.writerow([x, y, heading, 0, 0, distance, chord, 1])


def frames_for(arrays, rng, count=55):
    frames = []
    for points in arrays:
        for fraction in (0.1, 0.35, 0.6, 0.85):
            index = int((len(points) - 2) * fraction)
            tangent = points[index + 1] - points[index]
            tangent /= np.linalg.norm(tangent)
            normal = np.array([-tangent[1], tangent[0]])
            heading = (90 - math.degrees(math.atan2(tangent[1], tangent[0]))) % 360
            for offset in (0, 25):
                position = points[index] + normal * offset
                boxes = []
                for _ in range(count):
                    center = position + tangent * rng.uniform(-20, 65) + normal * rng.uniform(-15, 16)
                    boxes.append([*center, rng.uniform(0.5, 14), rng.uniform(0.3, 9), rng.uniform(-math.pi, math.pi)])
                frames.append(([*position, 0, heading], boxes))
    return frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probe", type=Path)
    parser.add_argument("maps", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    geometry = Geometry()
    rng = np.random.default_rng(20260915)
    results = []
    with tempfile.TemporaryDirectory(prefix="hdmap_oracle_") as temporary:
        root = Path(temporary)
        straight = root / "straight_crossing"
        straight.mkdir()
        for i in range(3):
            points = np.column_stack((np.linspace(-100, 100, 401), np.full(401, i * 4)))
            write_map(straight, f"lane{i}", points[::-1] if i == 2 else points)
        write_map(straight, "crossing", np.column_stack((np.full(161, 50), np.linspace(-40, 40, 161))))
        _, arrays = read_map(straight)
        results.append(verify(args.probe, straight, frames_for(arrays, rng, 35), geometry, True))
        curves = root / "curves"
        curves.mkdir()
        for i in range(3):
            angles = np.linspace(-1, 3.8, 321)
            write_map(curves, f"lane{i}", (30 - 4 * i) * np.column_stack((np.cos(angles), np.sin(angles))))
        _, arrays = read_map(curves)
        results.append(verify(args.probe, curves, frames_for(arrays, rng, 35), geometry, True))
        loop = root / "self_intersection"
        loop.mkdir()
        angles = np.linspace(0, 2 * math.pi, 601)
        write_map(loop, "figure_eight", np.column_stack((30 * np.sin(angles), 20 * np.sin(2 * angles))))
        _, arrays = read_map(loop)
        frames = frames_for(arrays, rng, 40)
        frames.append(([5, 6, 0, 45], [[0, 0, 40, 40, 0], [0, 0, 70, 60, 0.4]]))
        results.append(verify(args.probe, loop, frames, geometry, True))
    _, arrays = read_map(args.maps)
    results.append(verify(args.probe, args.maps, frames_for(arrays, rng), geometry))
    report = {"seed": 20260915, "boxes": sum(item["boxes"] for item in results),
              "max_area_difference_m2": max(item["max_area_difference_m2"] for item in results),
              "results": results}
    if args.report:
        args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(f"PASS: {report['boxes']} independent geometry cases")


if __name__ == "__main__":
    main()
