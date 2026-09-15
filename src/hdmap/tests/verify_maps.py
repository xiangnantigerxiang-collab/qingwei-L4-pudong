#!/usr/bin/env python3
"""用独立 SciPy 实现校验真实地图；SDK 运行时不依赖 Python/SciPy。"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.integrate import quad
from scipy.interpolate import CubicSpline
from scipy.optimize import brentq


def verify_file(source, output):
    raw = np.loadtxt(source, delimiter=",", ndmin=2)
    with output.open() as stream:
        rows = list(csv.DictReader(stream))
    data = np.array([[float(row[key]) for key in (
        "x", "y", "heading", "curvature", "signed_curvature",
        "dist_origin", "p2p_distance", "direction")] for row in rows])
    assert np.isfinite(data).all() and len(data) >= 2
    assert ((data[:, 2] >= 0) & (data[:, 2] < 360)).all()
    assert np.all(data[:, 7] == data[0, 7]) and data[0, 7] in (-1, 1)
    assert np.array_equal(data[:, 5], np.arange(len(data)) * 0.5)
    assert np.max(np.abs(data[:, 3] - np.abs(data[:, 4]))) < 1e-12
    chord = np.linalg.norm(np.diff(data[:, :2], axis=0), axis=1)
    assert data[0, 6] == 0 and np.max(np.abs(chord - data[1:, 6])) < 1e-10
    assert chord.max() <= 0.5 + 1e-6

    # 重建要求中的 >= 1 米节点，以 SciPy 的线性求解器拟合夹持三次样条。
    indices = [0]
    for index in range(1, len(raw) - 1):
        delta = raw[index, :2] - raw[indices[-1], :2]
        if math.hypot(*delta) >= 1.0:
            indices.append(index)
    while len(indices) > 1 and math.hypot(*(raw[-1, :2] - raw[indices[-1], :2])) < 1:
        indices.pop()
    indices.append(len(raw) - 1)
    anchors = raw[indices]
    spans = np.array([math.hypot(*delta) for delta in np.diff(anchors[:, :2], axis=0)])
    assert spans.min() >= 1.0
    u = np.r_[0.0, np.cumsum(spans)]
    direction = int(data[0, 7])
    yaw = np.deg2rad(90 - anchors[[0, -1], 2])
    tangent = direction * np.column_stack((np.cos(yaw), np.sin(yaw)))
    spline = CubicSpline(u, anchors[:, :2], bc_type=((1, tangent[0]), (1, tangent[1])))

    def speed(value):
        return math.hypot(*spline(value, 1))

    def integrate(start, end):
        return quad(speed, start, end, epsabs=2e-10, epsrel=2e-12)[0]

    segment_lengths = [integrate(u[i], u[i + 1]) for i in range(len(u) - 1)]
    lengths = np.r_[0.0, np.cumsum(segment_lengths)]
    assert len(data) == math.floor(lengths[-1] / 0.5) + 1
    parameters = []
    for target in data[:, 5]:
        index = min(np.searchsorted(lengths, target, side="right") - 1, len(u) - 2)
        local = target - lengths[index]
        if abs(local) < 1e-11:
            parameter = u[index]
        else:
            parameter = brentq(lambda value: integrate(u[index], value) - local,
                               u[index], u[index + 1], xtol=5e-13)
        parameters.append(parameter)
    parameters = np.array(parameters)
    expected_xy = spline(parameters)
    first, second = spline(parameters, 1), spline(parameters, 2)
    heading = (90 - np.rad2deg(np.arctan2(direction * first[:, 1], direction * first[:, 0]))) % 360
    curvature = (first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0]) / np.linalg.norm(first, axis=1)**3
    position_error = np.linalg.norm(data[:, :2] - expected_xy, axis=1).max()
    heading_error = np.abs((data[:, 2] - heading + 180) % 360 - 180).max()
    curvature_error = np.abs(data[:, 4] - curvature).max()
    assert position_error < 2e-7, (source.name, "position", position_error)
    assert heading_error < 2e-6, (source.name, "heading", heading_error)
    assert curvature_error < 2e-7, (source.name, "curvature", curvature_error)

    # 将实际 CSV 点投回独立样条，再逐段积分；此检查不信任 CSV 自带的 s 列。
    projected = parameters.copy()
    for _ in range(4):
        residual = spline(projected) - data[:, :2]
        first, second = spline(projected, 1), spline(projected, 2)
        projected -= np.sum(residual * first, axis=1) / (
            np.sum(first * first, axis=1) + np.sum(residual * second, axis=1))
        projected = np.clip(projected, 0, u[-1])
    actual_lengths = []
    for value in projected:
        index = min(np.searchsorted(u, value, side="right") - 1, len(u) - 2)
        actual_lengths.append(lengths[index] + integrate(u[index], value))
    arc_error = np.max(np.abs(np.diff(actual_lengths) - 0.5))
    assert arc_error < 2e-7, (source.name, "arc spacing", arc_error)
    return {
        "file": source.name, "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "input_count": len(raw), "anchor_count": len(anchors), "output_count": len(data),
        "direction": direction, "spline_length_m": lengths[-1],
        "remaining_length_m": lengths[-1] - data[-1, 5],
        "max_position_error_m": position_error, "max_arc_spacing_error_m": arc_error,
        "max_heading_error_deg": heading_error, "max_curvature_error_inv_m": curvature_error,
        "max_curvature_inv_m": data[:, 3].max(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    results = []
    for source in sorted(args.source.glob("*.csv")):
        result = verify_file(source, args.output / (source.stem + "_spline.csv"))
        results.append(result)
        print(f"PASS {source.name}: {result['output_count']} points, "
              f"arc spacing error {result['max_arc_spacing_error_m']:.3g} m", flush=True)
    assert results, "no input CSVs"
    summary = {
        "files": len(results), "input_points": sum(row["input_count"] for row in results),
        "output_points": sum(row["output_count"] for row in results),
        "max_arc_spacing_error_m": max(row["max_arc_spacing_error_m"] for row in results),
        "max_position_error_m": max(row["max_position_error_m"] for row in results),
        "max_heading_error_deg": max(row["max_heading_error_deg"] for row in results),
        "max_curvature_error_inv_m": max(row["max_curvature_error_inv_m"] for row in results),
        "results": results,
    }
    if args.report:
        args.report.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    print(f"PASS: {summary['files']} files, {summary['output_points']} points")


if __name__ == "__main__":
    main()
