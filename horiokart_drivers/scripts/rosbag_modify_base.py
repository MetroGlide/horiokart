#!/usr/bin/env python3

import os
import argparse

from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from nav_msgs.msg import Odometry


def add_covariance_to_odom(msg: Odometry):
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    msg.pose.covariance = [
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ]
    msg.twist.covariance = [
        0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.5,
    ]
    return msg


MODIFY_FUNC_DICT = {
    "/odom": [add_covariance_to_odom],
}


def get_reader(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def get_writer(output_bag: str, topic_meta_list: list):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    for topic_meta in topic_meta_list:
        writer.create_topic(topic_meta)
    return writer


def get_topic_type(topic: str, topic_meta_list: list):
    for topic_meta in topic_meta_list:
        if topic_meta.name == topic:
            return topic_meta.type
    raise ValueError(f"topic {topic} not found in topic_meta_list")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-i",
        "--input",
        type=str,
        help="input bag file path",
        default="",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="output bag file path",
        default="",
    )

    args = parser.parse_args()
    if args.input == "":
        raise ValueError("input file is not specified")
    if not os.path.exists(args.input):
        raise FileNotFoundError(f"input file {args.input} not found")

    if args.output == "":
        input_path = os.path.dirname(args.input) + "/.."
        input_path = os.path.abspath(input_path)

        input_file_name = os.path.basename(args.input)
        input_file_name = os.path.splitext(input_file_name)[0]
        input_file_ext = os.path.splitext(input_file_name)[1]

        args.output = input_path + "/" + input_file_name + "_modified" + input_file_ext

    if os.path.exists(args.output):
        raise FileExistsError(f"output file {args.output} already exists")

    reader = get_reader(args.input)
    topic_meta_list = reader.get_all_topics_and_types()

    writer = get_writer(args.output, topic_meta_list)
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(get_topic_type(topic, topic_meta_list))
        msg = deserialize_message(data, msg_type)
        if topic in MODIFY_FUNC_DICT:
            for func in MODIFY_FUNC_DICT[topic]:
                msg = func(msg)
        writer.write(topic, serialize_message(msg), timestamp)

    del reader
    del writer

    print("DONE")
    print(f"output file: {args.output}")


if __name__ == "__main__":
    main()