#!/usr/bin/env python3
"""Publish raw and optimized Folded City maps as latched ROS 2 messages."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def read_colored_ply(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        vertex_count = None
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PLY header ended unexpectedly")
            decoded = line.decode("ascii").strip()
            if decoded.startswith("format ") and decoded != "format binary_little_endian 1.0":
                raise ValueError(f"Unsupported PLY format: {decoded}")
            if decoded.startswith("element vertex "):
                vertex_count = int(decoded.split()[-1])
            if decoded == "end_header":
                break
        if vertex_count is None:
            raise ValueError("PLY vertex count is missing")
        dtype = np.dtype(
            {
                "names": ["x", "y", "z", "red", "green", "blue"],
                "formats": ["<f4", "<f4", "<f4", "u1", "u1", "u1"],
                "offsets": [0, 4, 8, 12, 13, 14],
                "itemsize": 15,
            }
        )
        cloud = np.fromfile(stream, dtype=dtype, count=vertex_count)
    return cloud


def point_cloud_message(cloud: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    packed = np.empty(
        (len(cloud),),
        dtype=np.dtype(
            {
                "names": ["x", "y", "z", "rgb"],
                "formats": ["<f4", "<f4", "<f4", "<u4"],
                "offsets": [0, 4, 8, 12],
                "itemsize": 16,
            }
        ),
    )
    packed["x"] = cloud["x"]
    packed["y"] = cloud["y"]
    packed["z"] = cloud["z"]
    packed["rgb"] = (
        cloud["red"].astype(np.uint32) << 16
        | cloud["green"].astype(np.uint32) << 8
        | cloud["blue"].astype(np.uint32)
    )
    message = PointCloud2()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.height = 1
    message.width = len(packed)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = packed.tobytes()
    return message


def trajectory_message(path: Path, frame_id: str, stamp) -> PathMessage:
    message = PathMessage()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(row["x"])
            pose.pose.position.y = float(row["y"])
            pose.pose.position.z = float(row["z"])
            pose.pose.orientation.x = float(row["qx"])
            pose.pose.orientation.y = float(row["qy"])
            pose.pose.orientation.z = float(row["qz"])
            pose.pose.orientation.w = float(row["qw"])
            message.poses.append(pose)
    return message


class FoldedCityPublisher(Node):
    def __init__(
        self,
        raw_cloud_path: Path,
        optimized_cloud_path: Path,
        raw_trajectory_path: Path,
        optimized_trajectory_path: Path,
        frame_id: str,
        optimized_non_ground_path: Path | None = None,
        optimized_ground_path: Path | None = None,
        reference_cloud_path: Path | None = None,
        reference_trajectory_path: Path | None = None,
        topic_prefix: str = "/folded_city",
    ) -> None:
        super().__init__("folded_city_map_publisher_" + frame_id.replace("/", "_"))
        prefix = topic_prefix.rstrip("/")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.raw_cloud_publisher = self.create_publisher(
            PointCloud2, prefix + "/cloud_raw", qos
        )
        self.optimized_cloud_publisher = self.create_publisher(
            PointCloud2, prefix + "/cloud_optimized", qos
        )
        self.optimized_non_ground_publisher = (
            self.create_publisher(
                PointCloud2, prefix + "/cloud_optimized_no_ground", qos
            )
            if optimized_non_ground_path
            else None
        )
        self.optimized_ground_publisher = (
            self.create_publisher(PointCloud2, prefix + "/ground_optimized", qos)
            if optimized_ground_path
            else None
        )
        self.raw_trajectory_publisher = self.create_publisher(
            PathMessage, prefix + "/trajectory_raw", qos
        )
        self.optimized_trajectory_publisher = self.create_publisher(
            PathMessage, prefix + "/trajectory_optimized", qos
        )
        self.reference_cloud_publisher = self.create_publisher(PointCloud2, prefix + "/cloud_app", qos) if reference_cloud_path else None
        self.reference_trajectory_publisher = self.create_publisher(PathMessage, prefix + "/trajectory_app", qos) if reference_trajectory_path else None
        self.reference_cloud = read_colored_ply(reference_cloud_path) if reference_cloud_path else None
        self.reference_trajectory_path = reference_trajectory_path
        self.raw_cloud = read_colored_ply(raw_cloud_path)
        self.optimized_cloud = read_colored_ply(optimized_cloud_path)
        self.optimized_non_ground_cloud = (
            read_colored_ply(optimized_non_ground_path)
            if optimized_non_ground_path
            else None
        )
        self.optimized_ground_cloud = (
            read_colored_ply(optimized_ground_path) if optimized_ground_path else None
        )
        self.raw_trajectory_path = raw_trajectory_path
        self.optimized_trajectory_path = optimized_trajectory_path
        self.frame_id = frame_id
        self.timer = self.create_timer(2.0, self.publish_map)
        self.publish_map()
        self.get_logger().info(
            f"Loaded {len(self.raw_cloud):,} raw points from {raw_cloud_path}"
        )
        self.get_logger().info(
            f"Loaded {len(self.optimized_cloud):,} optimized points from {optimized_cloud_path}"
        )

    def publish_map(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.raw_cloud_publisher.publish(
            point_cloud_message(self.raw_cloud, self.frame_id, stamp)
        )
        self.optimized_cloud_publisher.publish(
            point_cloud_message(self.optimized_cloud, self.frame_id, stamp)
        )
        if (
            self.optimized_non_ground_publisher is not None
            and self.optimized_non_ground_cloud is not None
        ):
            self.optimized_non_ground_publisher.publish(
                point_cloud_message(
                    self.optimized_non_ground_cloud, self.frame_id, stamp
                )
            )
        if (
            self.optimized_ground_publisher is not None
            and self.optimized_ground_cloud is not None
        ):
            self.optimized_ground_publisher.publish(
                point_cloud_message(self.optimized_ground_cloud, self.frame_id, stamp)
            )
        self.raw_trajectory_publisher.publish(
            trajectory_message(self.raw_trajectory_path, self.frame_id, stamp)
        )
        self.optimized_trajectory_publisher.publish(
            trajectory_message(self.optimized_trajectory_path, self.frame_id, stamp)
        )
        if self.reference_cloud_publisher is not None:
            self.reference_cloud_publisher.publish(point_cloud_message(self.reference_cloud, self.frame_id, stamp))
        if self.reference_trajectory_publisher is not None:
            self.reference_trajectory_publisher.publish(trajectory_message(self.reference_trajectory_path, self.frame_id, stamp))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-cloud", type=Path, required=True)
    parser.add_argument("--optimized-cloud", type=Path, required=True)
    parser.add_argument("--optimized-non-ground", type=Path)
    parser.add_argument("--optimized-ground", type=Path)
    parser.add_argument("--raw-trajectory", type=Path, required=True)
    parser.add_argument("--optimized-trajectory", type=Path, required=True)
    parser.add_argument("--frame", default="map")
    parser.add_argument("--reference-cloud", type=Path)
    parser.add_argument("--reference-trajectory", type=Path)
    parser.add_argument("--topic-prefix", default="/folded_city")
    args = parser.parse_args()
    rclpy.init()
    node = FoldedCityPublisher(
        args.raw_cloud,
        args.optimized_cloud,
        args.raw_trajectory,
        args.optimized_trajectory,
        args.frame,
        optimized_non_ground_path=args.optimized_non_ground,
        optimized_ground_path=args.optimized_ground,
        reference_cloud_path=args.reference_cloud,
        reference_trajectory_path=args.reference_trajectory,
        topic_prefix=args.topic_prefix,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
