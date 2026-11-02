#!/usr/bin/env python3
"""Publishes a world SDF's boxes as faint markers (latched), for the video: ground truth
next to the map. world_markers.py <world.sdf>"""
import sys
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


def pose(e):
    return [0.0] * 3 if e is None else [float(v) for v in e.text.split()[:3]]


def main():
    rclpy.init()
    node = Node("world_markers")
    pub = node.create_publisher(MarkerArray, "world_layout",
                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    out = MarkerArray()
    for model in ET.parse(sys.argv[1]).getroot().iter("model"):
        origin = pose(model.find("pose"))
        for col in model.iter("collision"):
            size = col.find("geometry/box/size")
            if size is None:
                continue
            c = [o + d for o, d in zip(origin, pose(col.find("pose")))]
            m = Marker()
            m.header.frame_id = "odom"
            m.ns, m.id, m.type = "world", len(out.markers), Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = c
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in size.text.split())
            m.color.r = m.color.g = m.color.b = 0.75
            m.color.a = 0.12
            out.markers.append(m)
    pub.publish(out)
    # A halo following the drone, so it stands out in the overhead view.
    halo_pub = node.create_publisher(Marker, "drone_halo",
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    halo = Marker()
    halo.header.frame_id = "base_link"
    halo.frame_locked = True
    halo.ns, halo.id, halo.type = "halo", 0, Marker.CYLINDER
    halo.pose.orientation.w = 1.0
    halo.scale.x = halo.scale.y = 1.4
    halo.scale.z = 0.05
    halo.color.r, halo.color.g, halo.color.b, halo.color.a = 1.0, 0.85, 0.1, 0.45
    halo_pub.publish(halo)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
