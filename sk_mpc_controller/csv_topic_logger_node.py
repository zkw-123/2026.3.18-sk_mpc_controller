#!/usr/bin/env python3
import os
import csv
import json
import time
import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter

from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict


def _has_header_stamp(msg) -> bool:
    return (
        hasattr(msg, "header")
        and hasattr(msg.header, "stamp")
        and hasattr(msg.header.stamp, "sec")
        and hasattr(msg.header.stamp, "nanosec")
    )


def _header_stamp_to_ns(msg) -> Optional[int]:
    if not _has_header_stamp(msg):
        return None
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def _get_frame_id(msg) -> str:
    if hasattr(msg, "header") and hasattr(msg.header, "frame_id"):
        return str(msg.header.frame_id)
    return ""


class CsvTopicLogger(Node):
    """
    Log multiple ROS2 topics into a single CSV file.

    CSV columns:
      wall_time_ns, ros_recv_time_ns, msg_header_time_ns, topic, type, frame_id, json

    - wall_time_ns: time.time_ns() (wall clock)
    - ros_recv_time_ns: node.get_clock().now().nanoseconds (ROS clock)
    - msg_header_time_ns: msg.header.stamp if present, else empty
    """

    def __init__(self):
        super().__init__("csv_topic_logger")

        # IMPORTANT:
        # Declare topics as STRING_ARRAY explicitly to avoid rclpy inferring BYTE_ARRAY from [].
        self.declare_parameter("topics", Parameter.Type.STRING_ARRAY)

        # These are safe with default values (type inferred correctly)
        self.declare_parameter("output_dir", "/tmp")
        self.declare_parameter("filename", "topic_log.csv")
        self.declare_parameter("discover_period_sec", 0.2)

        self.topics: List[str] = list(
            self.get_parameter("topics").get_parameter_value().string_array_value
        )
        self.output_dir: str = str(self.get_parameter("output_dir").value)
        self.filename: str = str(self.get_parameter("filename").value)
        self.discover_period_sec: float = float(self.get_parameter("discover_period_sec").value)

        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, self.filename)

        # Best-effort QoS for compatibility in mixed networks
        self.qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self._lock = threading.Lock()
        self._subs: Dict[str, object] = {}

        self._f = open(self.csv_path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(
            [
                "wall_time_ns",
                "ros_recv_time_ns",
                "msg_header_time_ns",
                "topic",
                "type",
                "frame_id",
                "json",
            ]
        )
        self._f.flush()

        self.get_logger().info(f"[csv_topic_logger] output={self.csv_path}")
        self.get_logger().info(f"[csv_topic_logger] waiting for topics: {self.topics}")

        self._timer = self.create_timer(self.discover_period_sec, self._discover_and_subscribe)

    def destroy_node(self):
        try:
            self._f.flush()
            self._f.close()
        except Exception:
            pass
        super().destroy_node()

    def _topic_type_map(self) -> Dict[str, str]:
        m: Dict[str, str] = {}
        for name, types in self.get_topic_names_and_types():
            if types:
                m[name] = types[0]
        return m

    def _discover_and_subscribe(self):
        if not self.topics:
            return

        tmap = self._topic_type_map()

        for t in self.topics:
            if t in self._subs:
                continue
            if t not in tmap:
                continue

            type_str = tmap[t]
            try:
                msg_cls = get_message(type_str)
            except Exception as e:
                self.get_logger().error(f"[csv_topic_logger] cannot import {type_str} for {t}: {e}")
                continue

            sub = self.create_subscription(
                msg_cls,
                t,
                lambda msg, topic=t, type_s=type_str: self._on_msg(topic, type_s, msg),
                self.qos,
            )
            self._subs[t] = sub
            self.get_logger().info(f"[csv_topic_logger] subscribed: {t} [{type_str}]")

        if all(t in self._subs for t in self.topics):
            self.get_logger().info("[csv_topic_logger] all topics subscribed. logging...")
            self._timer.cancel()

    def _on_msg(self, topic: str, type_str: str, msg):
        wall_ns = time.time_ns()
        ros_ns = int(self.get_clock().now().nanoseconds)
        hdr_ns = _header_stamp_to_ns(msg)
        frame_id = _get_frame_id(msg)

        try:
            payload = json.dumps(message_to_ordereddict(msg), ensure_ascii=False)
        except Exception as e:
            payload = json.dumps({"_error": f"serialize_failed: {str(e)}"}, ensure_ascii=False)

        with self._lock:
            self._w.writerow(
                [
                    str(wall_ns),
                    str(ros_ns),
                    "" if hdr_ns is None else str(hdr_ns),
                    topic,
                    type_str,
                    frame_id,
                    payload,
                ]
            )
            self._f.flush()


def main():
    rclpy.init()
    node = CsvTopicLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

