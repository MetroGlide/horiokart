import os
import argparse
from PIL import Image, ImageDraw
import numpy as np
import pyproj
import yaml
import math

# ROS / rosbag imports
import rosbag2_py
from rclpy.serialization import deserialize_message
from ublox_msgs.msg import NavSTATUS
from sensor_msgs.msg import NavSatFix


def extract_navstatus_from_bag(bag_path, topic, start_time=None, end_time=None):
    # uses module-level imports: rosbag2_py, NavSTATUS, deserialize_message
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    msgs = []
    base_time_ns = None
    while reader.has_next():
        (topic_name, data, t) = reader.read_next()
        if base_time_ns is None:
            base_time_ns = t
        # t is in nanoseconds
        elapsed_sec = (t - base_time_ns) / 1e9
        # skip before start_time
        if start_time is not None and elapsed_sec < start_time:
            continue
        # stop after end_time (sequential reader: safe to break)
        if end_time is not None and elapsed_sec > end_time:
            break
        if topic_name == topic:
            msg = deserialize_message(data, NavSTATUS)
            msgs.append(msg)
    return msgs


# NavStatusのgps_fix値ごとの色定義
GPS_FIX_COLORS = {
    0: (128, 128, 128, 255),  # NO_FIX: gray
    1: (255, 255, 0, 255),   # DEAD_RECKONING_ONLY: yellow
    2: (0, 128, 255, 255),   # 2D_FIX: blue
    3: (0, 255, 0, 255),     # 3D_FIX: green
    4: (128, 0, 255, 255),   # GPS+DR: purple
    5: (255, 0, 0, 255),     # TIME_ONLY_FIX: red
}


def match_navstatus_to_navsatfix(navsat_msgs, navstatus_msgs):
    # NavSatFix.header.stamp.sec/nanosecとNavStatus.i_tow(ミリ秒)で最も近いものを紐付け
    def navsatfix_time(msg):
        return msg.header.stamp.sec * 1e3 + msg.header.stamp.nanosec / 1e6

    def navstatus_time(msg):
        return msg.i_tow
    navstatus_sorted = sorted(navstatus_msgs, key=navstatus_time)
    result = []
    for nav in navsat_msgs:
        t = navsatfix_time(nav)
        # 最も近いNavStatusを探す
        closest = min(navstatus_sorted, key=lambda ns: abs(
            navstatus_time(ns)-t)) if navstatus_sorted else None
        result.append(closest)
    return result


# --- argparseでパラメータ受け取り ---


def parse_args():
    parser = argparse.ArgumentParser(
        description="rosbagのNavSatFixをmap座標に変換し画像化")
    parser.add_argument('--bag', type=str,
                        default="/root/ros2_data/rosbag/20251129/rosbag2_2025_11_29-04_25_25/",
                        help='rosbag2ディレクトリ')
    parser.add_argument('--input-dir', type=str,
                        default="/root/ros2_data/map",
                        help='mapやstatic transformの入力ディレクトリ (default: /root/ros2_data/map)')
    parser.add_argument('--label', type=str,
                        required=True,
                        help='generate_static_transforms.py の output にある transform の label を指定')
    parser.add_argument('--static-transforms', type=str,
                        default="gnss_to_map_static_transforms.yaml",
                        help='generate_static_transforms.py が出力する transforms YAML ファイル名 (input-dirと結合して使用)')
    parser.add_argument('--output-dir', type=str,
                        default="/root/ros2_data/map/map_gnss",
                        help='画像保存先ディレクトリ')
    parser.add_argument('--topic', type=str,
                        default="/gps/fix",
                        help='NavSatFixトピック名')
    parser.add_argument('--utm-zone', type=int, default=54,
                        help='UTMゾーン番号')
    parser.add_argument('--start-time', type=float, default=None,
                        help='rosbag開始からの経過秒で抽出開始 (float seconds, optional)')
    parser.add_argument('--end-time', type=float, default=None,
                        help='rosbag開始からの経過秒で抽出終了 (float seconds, optional)')
    return parser.parse_args()

# --- rosbag2_pyによるNavSatFix抽出 ---


def extract_navsatfix_from_bag(bag_path, topic, start_time=None, end_time=None):
    # uses module-level imports: rosbag2_py, NavSatFix, deserialize_message
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    msgs = []
    base_time_ns = None
    while reader.has_next():
        (topic_name, data, t) = reader.read_next()
        if base_time_ns is None:
            base_time_ns = t
        elapsed_sec = (t - base_time_ns) / 1e9
        if start_time is not None and elapsed_sec < start_time:
            continue
        if end_time is not None and elapsed_sec > end_time:
            break
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
    return map_x, map_y

# --- map画像・yamlからorigin, resolution取得 ---


def load_map_and_params(map_yaml_path):
    with open(map_yaml_path, 'r') as f:
        yml = yaml.safe_load(f)
    origin = yml['origin']  # [x, y, theta]
    resolution = yml['resolution']
    image_entry = yml.get('image')
    if not image_entry:
        raise RuntimeError(f"map yaml {map_yaml_path} に 'image' エントリがありません")
    # image_entry may be relative to the map_yaml location
    if not os.path.isabs(image_entry):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_entry)
    else:
        image_path = image_entry
    img = Image.open(image_path).convert('RGBA')
    return img, origin, resolution, os.path.basename(image_path)


def load_static_transforms(transforms_yaml_path):
    if not os.path.isfile(transforms_yaml_path):
        raise FileNotFoundError(
            f"static transforms yaml not found: {transforms_yaml_path}")
    with open(transforms_yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    transforms = data.get('transforms', []) if data else []
    return transforms

# --- map座標→画像ピクセル座標 ---


def mapxy_to_pixel(map_x, map_y, origin, resolution, img_size):
    # map原点は画像左下、画像は左上原点
    px = int((map_x - origin[0]) / resolution)
    py = img_size[1] - int((map_y - origin[1]) / resolution)
    return px, py

# --- 透過画像にプロット ---


def plot_points_on_transparent(map_points, navsat_msgs, navstatus_msgs, origin, resolution, img_size, radius=3):
    img = Image.new('RGBA', img_size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    for i, (mx, my) in enumerate(map_points):
        px, py = mapxy_to_pixel(mx, my, origin, resolution, img_size)
        msg = navsat_msgs[i]
        navstatus = navstatus_msgs[i] if navstatus_msgs else None
        color = GPS_FIX_COLORS.get(
            getattr(navstatus, 'gps_fix', None), (255, 0, 0, 255))

        # draw point
        draw.ellipse(
            [(px-radius, py-radius), (px+radius, py+radius)], fill=color)

        # draw covariance ellipse
        if hasattr(msg, 'position_covariance') and msg.position_covariance[0] > 0 and msg.position_covariance[4] > 0:
            cov_x = msg.position_covariance[0]
            cov_y = msg.position_covariance[4]
            sigma_x = math.sqrt(cov_x)
            sigma_y = math.sqrt(cov_y)
            ellipse_rx = int(1 * sigma_x / resolution)
            ellipse_ry = int(1 * sigma_y / resolution)
            draw.ellipse([(px-ellipse_rx, py-ellipse_ry), (px+ellipse_rx,
                         py+ellipse_ry)], outline=(0, 0, 255, 128), width=2)
    return img

# --- map画像にプロット ---


def plot_points_on_map(map_img, map_points, navsat_msgs, navstatus_msgs, origin, resolution, radius=3):
    img = map_img.copy()
    draw = ImageDraw.Draw(img)
    for i, (mx, my) in enumerate(map_points):
        px, py = mapxy_to_pixel(mx, my, origin, resolution, img.size)
        msg = navsat_msgs[i]
        navstatus = navstatus_msgs[i] if navstatus_msgs else None
        color = GPS_FIX_COLORS.get(
            getattr(navstatus, 'gps_fix', None), (255, 0, 0, 255))

        # draw point
        draw.ellipse(
            [(px-radius, py-radius), (px+radius, py+radius)], fill=color)

        # draw covariance ellipse
        if hasattr(msg, 'position_covariance') and msg.position_covariance[0] > 0 and msg.position_covariance[4] > 0:
            cov_x = msg.position_covariance[0]
            cov_y = msg.position_covariance[4]
            sigma_x = math.sqrt(cov_x)
            sigma_y = math.sqrt(cov_y)
            ellipse_rx = int(1 * sigma_x / resolution)
            ellipse_ry = int(1 * sigma_y / resolution)
            draw.ellipse([(px-ellipse_rx, py-ellipse_ry), (px+ellipse_rx,
                         py+ellipse_ry)], outline=(0, 0, 255, 128), width=2)

            # draw covariance value text.
            cov_text = f"{cov_x*10000:.2f}cm², {cov_y*10000:.2f}cm²"
            # draw.text((px + ellipse_rx + 2, py - ellipse_ry - 2),
            #           cov_text, fill=(0, 0, 0, 255))
        else:
            if hasattr(msg, 'position_covariance'):
                print(
                    f"Warning: NavSatFix msg {i} has non-positive covariance values. Covariance: {msg.position_covariance}")
            else:
                print(
                    f"Warning: NavSatFix msg {i} has no covariance information")

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
    navsat_msgs = extract_navsatfix_from_bag(
        args.bag, args.topic, start_time=args.start_time, end_time=args.end_time)
    print(f"NavSatFix msgs: {len(navsat_msgs)}件")

    # NavStatusトピック名（仮: /ublox_gps/navstatus）
    navstatus_topic = '/navstatus'
    navstatus_msgs = extract_navstatus_from_bag(
        args.bag, navstatus_topic, start_time=args.start_time, end_time=args.end_time)
    print(f"NavStatus msgs: {len(navstatus_msgs)}件")
    navstatus_for_navsat = match_navstatus_to_navsatfix(
        navsat_msgs, navstatus_msgs)

    # 2. GNSS→UTM
    utm_proj = pyproj.Proj(proj='utm', zone=args.utm_zone,
                           ellps='WGS84', south=False)
    utm_points = [wgs84_to_utm(msg, utm_proj) for msg in navsat_msgs]

    # 3. static transforms YAML から transform を取得 (labelで選択)
    transforms_yaml_path = os.path.join(
        args.input_dir, args.static_transforms)
    transforms = load_static_transforms(transforms_yaml_path)
    # label に一致する transform を探す
    selected_transform = None
    selected_map_name = None
    for t in transforms:
        if t.get('label') == args.label:
            selected_transform = t.get('transform')
            selected_map_name = t.get('map_name')
            break
    if selected_transform is None:
        if len(transforms) == 1:
            selected_transform = transforms[0].get('transform')
            selected_map_name = transforms[0].get('map_name')
            print(
                f"label に一致する transform が見つからなかったため、唯一の transform (label={transforms[0].get('label')}) を使用します")
        else:
            raise RuntimeError(
                f"label={args.label} に一致する transform が {transforms_yaml_path} に見つかりません")

    # 4. UTM→map
    map_points = [utm_to_map(ux, uy, selected_transform)
                  for ux, uy in utm_points]

    # 5. map画像・yaml取得 (map yamlは input-dir と結合して読み込む)
    if not selected_map_name:
        raise RuntimeError("selected_map_name が未設定です")
    map_yaml_path = os.path.join(args.input_dir, selected_map_name)
    map_img, origin, resolution, map_image_name = load_map_and_params(
        map_yaml_path)
    map_w, map_h = map_img.size

    # 5. GNSS点群の描画範囲を計算
    map_xs = [p[0] for p in map_points]
    map_ys = [p[1] for p in map_points]
    min_x = min(min(map_xs), origin[0])
    max_x = max(max(map_xs), origin[0] + map_w * resolution)
    min_y = min(min(map_ys), origin[1])
    max_y = max(max(map_ys), origin[1] + map_h * resolution)
    margin = 2.0  # [m] 余白
    min_x -= margin
    max_x += margin
    min_y -= margin
    max_y += margin

    # 6. 新しい画像サイズ・originを計算
    new_w = int(math.ceil((max_x - min_x) / resolution))
    new_h = int(math.ceil((max_y - min_y) / resolution))
    new_origin = [min_x, min_y, origin[2] if len(origin) > 2 else 0.0]
    print(f"拡張後画像サイズ: {new_w}x{new_h}, origin: {new_origin}")

    # 7. 拡張後ベース画像生成
    new_img = Image.new('RGBA', (new_w, new_h), (255, 255, 255, 0))
    # 既存map画像を新しい画像の正しい位置に貼り付け
    old_offset_x = int((origin[0] - min_x) / resolution)
    old_offset_y = new_h - int((origin[1] - min_y) / resolution) - map_h
    new_img.paste(map_img, (old_offset_x, old_offset_y))

    # 8. GNSS点・誤差円を新しい画像上に描画（NavStatus色分け対応）
    img_trans = plot_points_on_transparent(
        map_points, navsat_msgs, navstatus_for_navsat, new_origin, resolution, (new_w, new_h))
    img_on_map = plot_points_on_map(
        new_img, map_points, navsat_msgs, navstatus_for_navsat, new_origin, resolution)

    # 9. 保存
    img_trans.save(os.path.join(args.output_dir,
                   'gnss_points_transparent.png'))
    img_on_map.save(os.path.join(args.output_dir, 'gnss_points_on_map.png'))
    save_points_yaml(map_points, os.path.join(
        args.output_dir, 'gnss_points.yaml'))

    # 10. 新しいmap.yamlも保存（ベースマップ情報も追記）
    base_map_info = {
        'base_map_image': map_image_name,
        'base_map_yaml': os.path.basename(selected_map_name),
        'base_map_origin': origin,
        'base_map_resolution': resolution,
        'base_map_size': [map_w, map_h],
    }
    new_map_yaml = {
        'image': 'gnss_points_on_map.png',
        'resolution': resolution,
        'origin': new_origin,
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'base_map': base_map_info
    }
    with open(os.path.join(args.output_dir, 'gnss_points_on_map.yaml'), 'w') as f:
        yaml.safe_dump(new_map_yaml, f)
    print("Complete!")


if __name__ == "__main__":
    main()
