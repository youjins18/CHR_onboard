#!/usr/bin/env python3
"""Export every message in a ROS 2 bag to one long-form CSV file."""

from __future__ import annotations

import argparse
import csv
from collections.abc import Sequence
from pathlib import Path
import sys
from typing import Any, Iterator

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BASE_COLUMNS = ("time_ns", "time_s", "topic", "message_type")


def flatten_message(value: Any, prefix: str = "", output=None) -> dict[str, Any]:
    """Flatten nested ROS messages and arrays into stable CSV columns."""
    output = {} if output is None else output
    fields = getattr(value, "get_fields_and_field_types", None)
    if callable(fields):
        for field_name in fields():
            child_prefix = f"{prefix}.{field_name}" if prefix else field_name
            flatten_message(getattr(value, field_name), child_prefix, output)
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        for index, item in enumerate(value):
            flatten_message(item, f"{prefix}[{index}]", output)
    elif hasattr(value, "tolist"):
        flatten_message(value.tolist(), prefix, output)
    else:
        output[prefix] = value
    return output


def open_reader(bag_path: Path, storage_id: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def rows(bag_path: Path, storage_id: str) -> Iterator[dict[str, Any]]:
    reader = open_reader(bag_path, storage_id)
    topic_types = {}
    for topic in reader.get_all_topics_and_types():
        try:
            message_class = get_message(topic.type)
        except (AttributeError, ImportError, ModuleNotFoundError) as error:
            package = topic.type.split("/", maxsplit=1)[0]
            raise RuntimeError(
                f"cannot load {topic.type}; source the workspace that provides {package}"
            ) from error
        topic_types[topic.name] = (topic.type, message_class)

    failed_topics: set[str] = set()
    first_timestamp = None
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if first_timestamp is None:
            first_timestamp = timestamp
        type_name, message_class = topic_types[topic]
        row = {
            "time_ns": timestamp,
            "time_s": (timestamp - first_timestamp) * 1e-9,
            "topic": topic,
            "message_type": type_name,
        }
        try:
            message = deserialize_message(serialized, message_class)
        except Exception as error:
            if topic not in failed_topics:
                print(
                    f"WARNING: skipping messages that cannot be deserialized: "
                    f"{topic} ({type_name})\n  {error}",
                    file=sys.stderr,
                )
                failed_topics.add(topic)
            continue

        row.update(flatten_message(message))
        yield row


def export_bag(bag_path: Path, output_path: Path, storage_id: str) -> int:
    if not bag_path.is_dir():
        raise FileNotFoundError(f"bag directory does not exist: {bag_path}")

    dynamic_columns: set[str] = set()
    message_count = 0
    for row in rows(bag_path, storage_id):
        dynamic_columns.update(row.keys())
        message_count += 1

    columns = [*BASE_COLUMNS, *sorted(dynamic_columns.difference(BASE_COLUMNS))]
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows(bag_path, storage_id))
    return message_count


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert one ROS 2 bag to one CSV file.")
    parser.add_argument("bag", help="bag directory containing metadata.yaml")
    parser.add_argument("--storage-id", default="sqlite3")
    arguments = parser.parse_args()

    bag_path = Path(arguments.bag).expanduser().resolve()
    output_path = bag_path / f"{bag_path.name}.csv"
    try:
        count = export_bag(bag_path, output_path, arguments.storage_id)
    except (FileNotFoundError, RuntimeError) as error:
        print(f"rosbag_to_csv: error: {error}", file=sys.stderr)
        return 1
    print(f"Exported {count} messages to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
