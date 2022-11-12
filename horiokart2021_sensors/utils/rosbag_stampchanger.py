import rosbag
import os
import pprint


bag_file_name = r"/root/ros_data/20221106/slam_2022-11-06-04-59-06.bag"
f_name, suf = os.path.splitext(os.path.basename(bag_file_name))
output_file_name = os.path.join(
    os.path.dirname(bag_file_name),
    f_name+"_output"+suf
)

# bagのclockをstampにする
with rosbag.Bag(output_file_name, "w") as outbag:
    for topic, msg, t in rosbag.Bag(bag_file_name).read_messages():
        print("---")
        print(topic)
        pprint.pprint(msg)
        if topic == "/tf" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        else:
            if not msg._has_header:
                print(f"topic[{topic}] is not have header")
                outbag.write(topic, msg, t)
                continue
            
            # frame_id先頭"/"除去
            if hasattr(msg.header, "frame_id") and msg.header.frame_id !="" and msg.header.frame_id[0] == "/":
                msg.header.frame_id = msg.header.frame_id[1:]
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
