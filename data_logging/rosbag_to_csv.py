#!/usr/bin/env python3
"""Export Float32MultiArray logging topics from a ROS 2 bag to CSV files."""

import argparse
import csv
import re
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def topic_filename(topic: str) -> str:
    stem = re.sub(r"[^A-Za-z0-9]+", "_", topic).strip("_")
    return f"{stem or 'topic'}.csv"


def export_topics(bag: Path, output: Path, topics: list[str], storage_id: str) -> None:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = sorted(set(topics) - set(type_map))
    if missing:
        raise RuntimeError(f"topics absent from bag: {', '.join(missing)}")

    output.mkdir(parents=True, exist_ok=True)
    rows: dict[str, list[tuple[int, list[float]]]] = {topic: [] for topic in topics}
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in rows:
            continue
        message = deserialize_message(serialized, get_message(type_map[topic]))
        if not hasattr(message, "data"):
            raise TypeError(f"{topic} ({type_map[topic]}) has no data field")
        rows[topic].append((timestamp_ns, list(message.data)))

    for topic, samples in rows.items():
        if not samples:
            print(f"warning: no samples for {topic}")
            continue
        width = max(len(data) for _, data in samples)
        destination = output / topic_filename(topic)
        with destination.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["timestamp_ns"] + [f"data[{index}]" for index in range(width)])
            for timestamp_ns, data in samples:
                writer.writerow([timestamp_ns] + data + [""] * (width - len(data)))
        print(f"wrote {destination} ({len(samples)} rows, {width} values)")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory")
    parser.add_argument("--output", type=Path, default=Path("csv"))
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument(
        "--topics",
        nargs="+",
        default=["/data_logging_msg", "/chr/data_logging_msg"],
    )
    arguments = parser.parse_args()
    export_topics(arguments.bag, arguments.output, arguments.topics, arguments.storage_id)


if __name__ == "__main__":
    main()
