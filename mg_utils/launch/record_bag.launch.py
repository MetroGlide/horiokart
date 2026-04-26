#!/usr/bin/env python3

import os
import datetime

import launch
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.logging import get_logger
from ament_index_python.packages import get_package_share_directory

from mg_utils.launch_argument import LaunchArgumentCreator


def get_record_dir_name():
    return datetime.datetime.now().strftime("%Y%m%d") + "_bag"


def prepare_record_dir(output_dir, record_dir_name):
    record_dir_path = os.path.join(output_dir, record_dir_name)
    get_logger("launch").info("record_dir_path: {}".format(record_dir_path))
    if not os.path.exists(record_dir_path):
        os.makedirs(record_dir_path)
    return record_dir_path


def get_record_bag_path(record_dir_path, record_bag_base_name):
    max_bag_num = 0
    for file_name in os.listdir(record_dir_path):
        if os.path.isdir(os.path.join(record_dir_path, file_name)):
            if not file_name.startswith(record_bag_base_name):
                continue
            try:
                num = int(file_name.split("_")[1])
                if num > max_bag_num:
                    max_bag_num = num
            except Exception:
                pass
    record_bag_num = max_bag_num + 1
    record_bag_name = record_bag_base_name + str(record_bag_num)
    return os.path.join(record_dir_path, record_bag_name)


def get_topic_list(pkg_share):
    topic_list_path = os.path.join(
        pkg_share, "params", "record_topic_list.txt")
    if not os.path.exists(topic_list_path):
        get_logger("launch").info(
            "topic_list_path not found: {}".format(topic_list_path))
        return []
    with open(topic_list_path, "r") as f:
        topic_list = f.read().splitlines()
    return list(filter(lambda x: x != "", topic_list))


def launch_setup(context, *args, **kwargs):
    caller_pkg_name = LaunchConfiguration("caller_pkg_name").perform(context)
    record_bag_base_name = LaunchConfiguration(
        "record_bag_base_name").perform(context)

    pkg_share = get_package_share_directory(caller_pkg_name)
    output_dir = os.environ["ROSBAG_PATH"]

    record_dir_path = prepare_record_dir(output_dir, get_record_dir_name())
    record_bag_path = get_record_bag_path(
        record_dir_path, record_bag_base_name)
    topic_list = get_topic_list(pkg_share)

    simulation_lc = LaunchConfiguration("simulation")

    record_bag_launch = launch.actions.ExecuteProcess(
        cmd=["ros2", "bag", "record", "-s", "mcap",
             "-o", record_bag_path] + topic_list,
        output="screen",
        condition=launch.conditions.UnlessCondition(simulation_lc),
    )
    record_bag_sim_launch = launch.actions.ExecuteProcess(
        cmd=["ros2", "bag", "record", "-s", "mcap",
             "--use-sim-time",
             "-o", record_bag_path] + topic_list,
        output="screen",
        condition=launch.conditions.IfCondition(simulation_lc),
    )

    return [record_bag_launch, record_bag_sim_launch]


def generate_launch_description():
    launch_argument_creator = LaunchArgumentCreator()

    launch_argument_creator.create(
        "caller_pkg_name",
        default="mg_navigation",
        description="Package name to load record_topic_list.txt from",
    )
    launch_argument_creator.create(
        "record_bag_base_name",
        default="navigation_",
        description="Base name for recorded bag directory",
    )
    launch_argument_creator.create(
        "simulation",
        default=EnvironmentVariable("SIMULATION"),
        description="Use simulation time",
    )

    return LaunchDescription([
        *launch_argument_creator.get_created_declare_launch_args(),
        OpaqueFunction(function=launch_setup),
    ])
