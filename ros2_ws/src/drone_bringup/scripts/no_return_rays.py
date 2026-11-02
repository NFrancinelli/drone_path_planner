#!/usr/bin/env python3
"""Turns the depth camera's no-return pixels into points just past OctoMap's max range.

Gazebo's depth camera reports a pixel with nothing within its far clip as (inf, inf, inf),
and OctoMap drops non-finite points, so the space along those rays stays unknown. In a
hall with no roof, that's every ray at or above the horizon down a long aisle: the space
straight ahead at flight altitude only gets mapped once a wall comes within range, and the
planner, which won't fly into unknown space close by (known_radius), stops short of it.
A point past max_range along the pixel's ray makes OctoMap clear the ray up to max_range
without marking an obstacle at its end.

The ray of pixel (row, col) comes from the camera's intrinsics (square pixels, principal
point at the image centre), in the camera's frame (x forward, y left, z up). Pixels closer
than the near clip (-inf) are left alone: they may be anything.

    in:  cloud_in (sensor_msgs/PointCloud2, organized, from ros_gz_bridge)
    out: cloud_out
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class NoReturnRays(Node):
    def __init__(self):
        super().__init__("no_return_rays")
        self.hfov = self.declare_parameter("horizontal_fov", 1.51).value  # [rad], as the SDF
        # Past OctoMap's sensor_model.max_range, so it clears the ray but marks nothing.
        self.range = self.declare_parameter("range", 10.5).value  # [m]
        self.rays = None  # (height, width, 3) unit rays, per image size
        self.pub = self.create_publisher(PointCloud2, "cloud_out", 1)
        self.create_subscription(PointCloud2, "cloud_in", self.on_cloud, 1)

    def ray_table(self, height, width):
        f = (width / 2.0) / math.tan(self.hfov / 2.0)
        cols, rows = np.meshgrid(np.arange(width), np.arange(height))
        rays = np.stack([np.full(cols.shape, f), (width - 1) / 2.0 - cols,
                         (height - 1) / 2.0 - rows], axis=-1)
        return (rays / np.linalg.norm(rays, axis=-1, keepdims=True)).astype(np.float32)

    def on_cloud(self, msg):
        if msg.height <= 1:
            self.get_logger().error("Point cloud is not organized; passing it through",
                                    throttle_duration_sec=10.0)
            self.pub.publish(msg)
            return
        if self.rays is None or self.rays.shape[:2] != (msg.height, msg.width):
            self.rays = self.ray_table(msg.height, msg.width)
        offsets = {f.name: f.offset for f in msg.fields}
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
            msg.height, msg.row_step)[:, :msg.width * msg.point_step].reshape(
            msg.height, msg.width, msg.point_step).copy()
        xyz = np.stack([data[..., offsets[a]:offsets[a] + 4].copy().view(np.float32)[..., 0]
                        for a in ("x", "y", "z")], axis=-1)
        none = np.isposinf(xyz).all(axis=-1)
        xyz[none] = self.rays[none] * self.range
        for i, a in enumerate(("x", "y", "z")):
            data[..., offsets[a]:offsets[a] + 4] = (
                np.ascontiguousarray(xyz[..., i]).view(np.uint8).reshape(
                    msg.height, msg.width, 4))
        msg.row_step = msg.width * msg.point_step
        msg.data = data.tobytes()
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(NoReturnRays())


if __name__ == "__main__":
    main()
