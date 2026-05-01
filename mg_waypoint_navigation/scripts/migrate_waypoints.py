#!/usr/bin/env python3
"""v1 YAML ウェイポイントファイルを v2.0 フォーマットに変換するスクリプト

使用方法:
  python3 migrate_waypoints.py <input.yaml> <output.yaml>
  python3 migrate_waypoints.py <input.yaml>          # input_v2.yaml として出力
"""
import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser(
        description="Migrate waypoint YAML from v1 to v2.0 format"
    )
    parser.add_argument("input", help="Path to v1 YAML file")
    parser.add_argument("output", nargs="?",
                        help="Path to output v2.0 YAML file")
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"Error: {args.input} does not exist", file=sys.stderr)
        sys.exit(1)

    output_path = args.output
    if output_path is None:
        base, ext = os.path.splitext(args.input)
        output_path = base + "_v2" + ext

    import yaml

    with open(args.input, "r") as f:
        raw = yaml.safe_load(f)

    if isinstance(raw, dict) and raw.get("version") == "2.0":
        print(f"Input file is already v2.0 format. Nothing to do.")
        sys.exit(0)

    if not isinstance(raw, list):
        print(f"Error: expected a YAML list, got {type(raw).__name__}", file=sys.stderr)
        sys.exit(1)

    from mg_waypoint_navigation.waypoint_v1_compat import convert_v1_waypoint
    from mg_waypoint_navigation.waypoint import WaypointList, WaypointsSaver

    waypoints = WaypointList()
    for wp_raw in raw:
        waypoints.add(convert_v1_waypoint(wp_raw))

    saver = WaypointsSaver(output_path)
    saver.save(waypoints)

    print(f"Migrated {waypoints.get_size()} waypoints -> {output_path}")


if __name__ == "__main__":
    main()
