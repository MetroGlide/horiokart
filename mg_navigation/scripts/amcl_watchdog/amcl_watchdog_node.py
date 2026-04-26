#!/usr/bin/env python3

"""Wrapper script to launch the package-installed AMCL watchdog node.

This thin wrapper imports the implementation from the installed python
package so that `ros2 run` (which installs the script into a different
location) can still find package modules via the Python package install.
"""
from mg_navigation.amcl_watchdog.node import main


if __name__ == '__main__':
    main()
