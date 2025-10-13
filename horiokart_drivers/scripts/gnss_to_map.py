
import os
import math
import yaml
import pyproj
import numpy as np
from PIL import Image, ImageDraw


# --- argparseでパラメータ受け取り ---
import argparse


def parse_args():
    parser = argparse.ArgumentParser(
        description="rosbagのNavSatFixをmap座標に変換し画像化")
    parser.add_argument('--transform', type=float, nargs=3,
                        default=[4013878.997909, 104375.799266, 1.70080],
                        metavar=('X', 'Y', 'YAW'), help='UTM→map変換行列 [x y yaw]')
    parser.add_argument('--bag', type=str,
                        default="/root/ros2_data/rosbag/TC2025/20251004/rosbag2_2025_10_04-02_19_02",
                        help='rosbag2ディレクトリ')
    parser.add_argument('--map-img', type=str,
                        default="/root/ros2_data/map_20251004/map1/map.pgm",
                        help='map画像パス')
    parser.add_argument('--map-yaml', type=str,
                        default="/root/ros2_data/map_20251004/map1/map.yaml",
                        help='map.yamlパス')
    parser.add_argument('--output-dir', type=str,
                        default="/root/ros2_data/map_20251004/map_gnss",
                        help='画像保存先ディレクトリ')
    parser.add_argument('--topic', type=str,
                        default="/gps/fix",
                        help='NavSatFixトピック名')
    parser.add_argument('--utm-zone', type=int, default=54,
                        help='UTMゾーン番号')
    return parser.parse_args()

# --- rosbag2_pyによるNavSatFix抽出 ---


def extract_navsatfix_from_bag(bag_path, topic):
    try:
        import rosbag2_py
    except ImportError:
        raise ImportError(
            "rosbag2_pyが必要です。pip install rosbag2_py でインストールしてください。")
    from sensor_msgs.msg import NavSatFix
    import rclpy.serialization
    from rclpy.serialization import deserialize_message

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    msgs = []
    while reader.has_next():
        (topic_name, data, t) = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, NavSatFix)
            msgs.append(msg)
    return msgs

# --- WGS84→UTM変換 ---


def wgs84_to_utm(msg, utm_proj):
    x, y = utm_proj(msg.longitude, msg.latitude)
    return x, y

# --- UTM→map座標変換 ---


def utm_to_map(utm_x, utm_y, transform):
    x, y, yaw = transform
    map_x = x + math.cos(yaw) * utm_x - math.sin(yaw) * utm_y
    map_y = y + math.sin(yaw) * utm_x + math.cos(yaw) * utm_y
    print(f"utm({utm_x:.2f}, {utm_y:.2f}) -> map({map_x:.2f}, {map_y:.2f})")
    return map_x, map_y

# --- map画像・yamlからorigin, resolution取得 ---


def load_map_and_params(map_img_path, map_yaml_path):
    with open(map_yaml_path, 'r') as f:
        yml = yaml.safe_load(f)
    origin = yml['origin']  # [x, y, theta]
    resolution = yml['resolution']
    img = Image.open(map_img_path).convert('RGBA')
    return img, origin, resolution

# --- map座標→画像ピクセル座標 ---


def mapxy_to_pixel(map_x, map_y, origin, resolution, img_size):
    # map原点は画像左下、画像は左上原点
    px = int((map_x - origin[0]) / resolution)
    py = img_size[1] - int((map_y - origin[1]) / resolution)
    print(f"map({map_x:.2f}, {map_y:.2f}) -> pixel({px}, {py})")
    return px, py

# --- 透過画像にプロット ---


def plot_points_on_transparent(map_points, origin, resolution, img_size, radius=3, color=(255, 0, 0, 255)):
    img = Image.new('RGBA', img_size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    for mx, my in map_points:
        px, py = mapxy_to_pixel(mx, my, origin, resolution, img_size)
        draw.ellipse(
            [(px-radius, py-radius), (px+radius, py+radius)], fill=color)
    return img

# --- map画像にプロット ---


def plot_points_on_map(map_img, map_points, origin, resolution, radius=3, color=(255, 0, 0, 255)):
    img = map_img.copy()
    draw = ImageDraw.Draw(img)
    for mx, my in map_points:
        px, py = mapxy_to_pixel(mx, my, origin, resolution, img.size)
        draw.ellipse(
            [(px-radius, py-radius), (px+radius, py+radius)], fill=color)
    return img

# --- 座標リストをyaml保存 ---


def save_points_yaml(map_points, out_path):
    data = {'map_points': [[float(mx), float(my)] for mx, my in map_points]}
    with open(out_path, 'w') as f:
        yaml.safe_dump(data, f)
    print(f"座標をyaml保存: {out_path}")


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    # 1. rosbagからNavSatFix抽出
    navsat_msgs = extract_navsatfix_from_bag(args.bag, args.topic)
    print(f"NavSatFix msgs: {len(navsat_msgs)}件")

    # 2. GNSS→UTM
    utm_proj = pyproj.Proj(proj='utm', zone=args.utm_zone,
                           ellps='WGS84', south=False)
    utm_points = [wgs84_to_utm(msg, utm_proj) for msg in navsat_msgs]

    # 3. UTM→map
    map_points = [utm_to_map(ux, uy, args.transform) for ux, uy in utm_points]

    # 4. map画像・yaml取得
    map_img, origin, resolution = load_map_and_params(
        args.map_img, args.map_yaml)

    # 5. 画像生成
    img_trans = plot_points_on_transparent(
        map_points, origin, resolution, map_img.size)
    img_on_map = plot_points_on_map(map_img, map_points, origin, resolution)

    # 6. 保存
    img_trans.save(os.path.join(args.output_dir,
                   'gnss_points_transparent.png'))
    img_on_map.save(os.path.join(args.output_dir, 'gnss_points_on_map.png'))
    save_points_yaml(map_points, os.path.join(
        args.output_dir, 'gnss_points.yaml'))

    print("Complete!")


if __name__ == "__main__":
    main()
