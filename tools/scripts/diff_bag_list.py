#!/usr/bin/env python3

"""
This script compares the record topic list file with current topic list.
The record topic list file is get from argument of this script.
The record topic list file format is as follows:
/topic1
/topic2
/topic3

current topic list is get from rostopic list command.
"""

import os
import sys
import subprocess
import argparse


def get_topic_list():
    topic_list = []
    try:
        topic_list = subprocess.check_output(
            "ros2 topic list", shell=True).decode().split("\n")
    except:
        pass
    return topic_list


def get_record_topic_list(record_topic_list_file):
    record_topic_list = []
    try:
        with open(record_topic_list_file, "r") as f:
            record_topic_list = f.read().split("\n")
    except:
        pass
    return record_topic_list


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("record_topic_list_file",
                        help="record topic list file")
    args = parser.parse_args()

    record_topic_list = get_record_topic_list(args.record_topic_list_file)
    current_topic_list = get_topic_list()

    # check record topic list.
    # format:
    # /topic (only RECORD LIST)
    # /topic (only CURRENT TOPIC)
    # /topic
    #
    # sorted list by only or both
    record_only_topic_list = []
    current_only_topic_list = []
    both_topic_list = []

    for topic in record_topic_list:
        if topic == "":
            continue
        if topic in current_topic_list:
            both_topic_list.append(topic)
        else:
            record_only_topic_list.append(topic)

    for topic in current_topic_list:
        if topic == "":
            continue
        if topic not in record_topic_list:
            current_only_topic_list.append(topic)

    for topic in record_only_topic_list:
        print(f"{topic} \033[33m(only RECORD LIST)\033[0m")

    for topic in current_only_topic_list:
        print(f"{topic} \033[33m(only CURRENT TOPIC)\033[0m")

    for topic in both_topic_list:
        print(topic)


if __name__ == "__main__":
    main()
