import os
import rosbag
from alive_progress import alive_bar
import fire
import sys
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage
import rospy
import geometry_msgs.msg as geometry_msgs

TF_TOPICS = ["/tf", "/tf_static"]  # Default tf topics to check

def check_double_timestamp(bag_file, topic, frame_id=None):
    print(f"Checking for double timestamps in '{bag_file}' on topic '{topic}'")
    if frame_id is  not None:
        print(f"Only checking with same frame_id: {frame_id}")
    times = []

    # Open the input bag file
    with rosbag.Bag(bag_file, 'r') as inbag:
        topics_in_bag = inbag.get_type_and_topic_info().topics.keys()
        if topic not in topics_in_bag:
            print(f"The specified topic '{topic}' does not exist in the bag. Exiting.")
            sys.exit(1)

        n = inbag.get_message_count()
        with alive_bar(n) as bar:
            for topic, msg, t in inbag.read_messages():
                    bar()
                    if topic == topic:
                         if topic in TF_TOPICS:
                             for tf_msg in msg.transforms:
                                 if frame_id is not None and tf_msg.header.frame_id != frame_id:
                                     continue
                                 if tf_msg.header.stamp in times:
                                     print(f"Double timestamp found: {tf_msg.header.stamp}")
                                     print("Frame id: ", tf_msg.header.frame_id)
                                 times.append(tf_msg.header.stamp)
                    elif hasattr(msg, "header"):
                        if msg.header.stamp in times:
                            print(f"Double timestamp found: {msg.header.stamp}")
                        times.append(msg.header.stamp)
                    else:
                        print(f"No timestamp found in message for topic: {topic}")
                        sys.exit(1)

if __name__ == "__main__":
    fire.Fire(check_double_timestamp)
