#!/usr/bin/env python3

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


class LaunchArgument:
    def __init__(self, name, default=None, description=None):
        self._launch_config = LaunchConfiguration(name, default=default)
        self._declare_launch_arg = DeclareLaunchArgument(
            name, default_value=default, description=description
        )

    @property
    def launch_config(self):
        return self._launch_config

    @property
    def declare_launch_arg(self):
        return self._declare_launch_arg


class LaunchArgumentCreator:
    def __init__(self):
        self._declare_launch_args = []

    def create(self, name, default=None, description=None) -> LaunchArgument:
        launch_argument = LaunchArgument(name, default, description)
        self._declare_launch_args.append(launch_argument.declare_launch_arg)

        return launch_argument

    def get_created_declare_launch_args(self) -> list:
        return self._declare_launch_args
