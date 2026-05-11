#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np


DEFAULT_DATASET_ROOT = Path("/home/wjxu22/Datasets/outputs/rtab/extracted")


def stamp_to_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ns_to_seconds(stamp_ns: int) -> float:
    return float(stamp_ns) / 1_000_000_000.0


def normalize_quaternion_xyzw(quaternion_xyzw: np.ndarray) -> np.ndarray:
    quaternion_xyzw = np.asarray(quaternion_xyzw, dtype=np.float64)
    norm = np.linalg.norm(quaternion_xyzw)
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quaternion_xyzw / norm


def matrix_from_translation_quaternion(
    translation_xyz: np.ndarray | list[float],
    quaternion_xyzw: np.ndarray | list[float],
) -> np.ndarray:
    tx, ty, tz = np.asarray(translation_xyz, dtype=np.float64)
    qx, qy, qz, qw = normalize_quaternion_xyzw(quaternion_xyzw)

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    matrix = np.eye(4, dtype=np.float64)
    matrix[0, 0] = 1.0 - 2.0 * (yy + zz)
    matrix[0, 1] = 2.0 * (xy - wz)
    matrix[0, 2] = 2.0 * (xz + wy)
    matrix[1, 0] = 2.0 * (xy + wz)
    matrix[1, 1] = 1.0 - 2.0 * (xx + zz)
    matrix[1, 2] = 2.0 * (yz - wx)
    matrix[2, 0] = 2.0 * (xz - wy)
    matrix[2, 1] = 2.0 * (yz + wx)
    matrix[2, 2] = 1.0 - 2.0 * (xx + yy)
    matrix[:3, 3] = [tx, ty, tz]
    return matrix


def quaternion_from_matrix(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=np.float64)
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (matrix[2, 1] - matrix[1, 2]) * s
        qy = (matrix[0, 2] - matrix[2, 0]) * s
        qz = (matrix[1, 0] - matrix[0, 1]) * s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
        qw = (matrix[2, 1] - matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (matrix[0, 1] + matrix[1, 0]) / s
        qz = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
        qw = (matrix[0, 2] - matrix[2, 0]) / s
        qx = (matrix[0, 1] + matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
        qw = (matrix[1, 0] - matrix[0, 1]) / s
        qx = (matrix[0, 2] + matrix[2, 0]) / s
        qy = (matrix[1, 2] + matrix[2, 1]) / s
        qz = 0.25 * s

    return normalize_quaternion_xyzw(np.array([qx, qy, qz, qw], dtype=np.float64))


def invert_transform(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=np.float64)
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def make_pose_entry(
    matrix: np.ndarray,
    reference_frame: str,
    target_frame: str,
    source: str,
    age_ns: int | None = None,
    dt_ns: int | None = None,
) -> dict[str, Any]:
    quaternion_xyzw = quaternion_from_matrix(matrix[:3, :3])
    return {
        "reference_frame": reference_frame,
        "target_frame": target_frame,
        "source": source,
        "translation_xyz": matrix[:3, 3].astype(float).tolist(),
        "quaternion_xyzw": quaternion_xyzw.astype(float).tolist(),
        "age_ns": age_ns,
        "dt_ns": dt_ns,
    }


def pose_matrix_from_entry(pose_entry: dict[str, Any]) -> np.ndarray:
    return matrix_from_translation_quaternion(
        pose_entry["translation_xyz"],
        pose_entry["quaternion_xyzw"],
    )


def write_json(path: Path, payload: dict[str, Any] | list[Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        json.dump(payload, file, ensure_ascii=True, indent=2)

