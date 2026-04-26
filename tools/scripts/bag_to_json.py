#!/usr/bin/env python3
import argparse
import os
import json
import sys
from collections import defaultdict


def parse_args():
    parser = argparse.ArgumentParser(description="rosbag2の内容をjsonに変換")
    parser.add_argument("--bag_path",
                        default="/root/ros2_data/rosbag/TC2025/20251004/rosbag2_2025_10_04-02_19_02",
                        help="rosbag2のディレクトリパス")
    parser.add_argument("--output_dir",
                        default="/root/ros2_data/rosbag/TC2025/20251004/rosbag2_2025_10_04-02_19_02",
                        help="出力ディレクトリパス")
    parser.add_argument("--topics", nargs='+',
                        default=['/gps/fix', '/navstatus'],
                        help="抽出するトピック名のリスト（スペース区切りで指定）")
    return parser.parse_args()


def read_rosbag2(bag_path, filter_topics=None):
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    from rclpy.serialization import deserialize_message
    import importlib

    topic_msgs = defaultdict(list)
    all_msgs = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if filter_topics is not None and topic not in filter_topics:
            continue
        msg_type = type_map[topic]
        pkg, msg_name = msg_type.split('/')[0], msg_type.split('/')[-1]
        mod = importlib.import_module(f"{pkg}.msg")
        msg_class = getattr(mod, msg_name)
        msg = deserialize_message(data, msg_class)
        msg_dict = message_to_dict(msg)
        entry = {
            'topic': topic,
            'timestamp': t,
            'msg': msg_dict
        }
        topic_msgs[topic].append(entry)
        all_msgs.append(entry)
    return topic_msgs, all_msgs


def message_to_dict(msg):
    # ROS2 messageをdictに再帰変換し、numpy配列等もlist化
    import numpy as np
    if hasattr(msg, '__slots__'):
        result = {}
        for slot in msg.__slots__:
            val = getattr(msg, slot)
            if isinstance(val, (list, tuple)):
                result[slot] = [message_to_dict(v) if hasattr(
                    v, '__slots__') else message_to_dict(v) for v in val]
            elif hasattr(val, '__slots__'):
                result[slot] = message_to_dict(val)
            elif isinstance(val, np.ndarray):
                result[slot] = val.tolist()
            else:
                result[slot] = val
        return result
    elif isinstance(msg, (list, tuple)):
        return [message_to_dict(v) for v in msg]
    elif 'numpy' in str(type(msg)):
        try:
            return msg.tolist()
        except Exception:
            return str(msg)
    else:
        return msg


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    topic_msgs, all_msgs = read_rosbag2(
        args.bag_path, filter_topics=args.topics)

    # トピックごと
    topics_json = {}
    for topic, msgs in topic_msgs.items():
        topics_json[topic] = [
            {'timestamp': m['timestamp'], 'msg': m['msg']} for m in msgs
        ]

    # 時系列
    all_msgs_sorted = sorted(all_msgs, key=lambda x: x['timestamp'])
    timeline_json = [
        {'topic': m['topic'], 'timestamp': m['timestamp'], 'msg': m['msg']} for m in all_msgs_sorted
    ]

    # まとめて1つのjsonに
    result = {
        'by_topic': topics_json,
        'timeline': timeline_json
    }
    out_path = os.path.join(args.output_dir, 'rosbag2_summary.json')
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2, default=str)
    print(f"書き出し完了: {out_path}")


if __name__ == '__main__':
    main()
