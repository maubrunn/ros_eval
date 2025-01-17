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

TF_TOPIC = "/tf"  # Default tf topic to add

def add_tf(input_bag_file, output_bag_file=None, odom_topic="/odom", rate=None, frame_id="map", child_frame_id="base_link"):
    print(f"Adding tf from '{odom_topic}' to '{frame_id}' -> '{child_frame_id}' in '{input_bag_file}' and save to '{output_bag_file}'")
    if output_bag_file is None:
        print(f"No output specified. Going to overwrite old bag: {input_bag_file}\n"
              "Do you want to continue? [Y/n]")
        if str(input()).lower() not in ["", "y"]:
            print("Exiting")
            sys.exit(1)
        output_bag_file = input_bag_file

    # Create a temporary file for the output bag
    temp_bag_file = f"{os.path.dirname(output_bag_file)}/tmp.bag"
    added_timestamps = []
    stamp = -1
    init = False or rate is None

    try:
        # Open the input bag file
        with rosbag.Bag(input_bag_file, 'r') as inbag:
            topics_in_bag = inbag.get_type_and_topic_info().topics.keys()
            if odom_topic not in topics_in_bag:
                print(f"The specified odom topic '{odom_topic}' does not exist in the bag. Exiting.")
                sys.exit(1)

            n = inbag.get_message_count()
            with alive_bar(n) as bar:
                with rosbag.Bag(temp_bag_file, 'w') as outbag:
                    for topic, msg, t in inbag.read_messages():
                        bar()
                        outbag.write(topic, msg, t)
                        if init and topic == odom_topic:

                                # print("Added timestamps: ", added_timestamps)
                                # print("Newest stamp: ", newest_stamp.to_sec())
                                # sys.exit(1)
                            if rate is None:
                                stamp = msg.header.stamp
                            tf_msg = geometry_msgs.TransformStamped()
                            tf_msg.header.stamp = stamp
                            tf_msg.header.frame_id = frame_id
                            tf_msg.child_frame_id = child_frame_id
                            
                            # Populate the transform (example assumes identity transform)
                            tf_msg.transform.translation.x = msg.pose.position.x
                            tf_msg.transform.translation.y = msg.pose.position.y
                            tf_msg.transform.translation.z = msg.pose.position.z
                            tf_msg.transform.rotation = msg.pose.orientation
                            if rate is None:
                                stamp += rospy.Duration(1 / rate)
                            # Write the tf message to the /tf topic
                            outbag.write(TF_TOPIC, TFMessage([tf_msg]), t)
                        elif topic == "/clock":
                            stamp = msg.clock
                            init = True
                        # elif topic == TF_TOPIC:
                        #     if msg.transforms[0].child_frame_id == child_frame_id:
                        #         print("What is this?")
                        #     time = msg.transforms[0].header.stamp.to_sec()
                        #     if time > newest_stamp.to_sec():
                        #         newest_stamp = msg.transforms[0].header.stamp
                        # elif hasattr(msg, "header") and msg.header.stamp.to_sec() > newest_stamp.to_sec() and msg.header.stamp.to_sec() - newest_stamp.to_sec() < 10:
                        #         newest_stamp = msg.header.stamp
                        
        
        # If everything is successful, replace the original bag with the new one
        if input_bag_file == output_bag_file:
            print("Deleting file:", input_bag_file)
            os.remove(input_bag_file)
        os.rename(temp_bag_file, output_bag_file)
        print(f"Saved new bag as: {output_bag_file}")

    except Exception as e:
        print(f"An error occurred: {e}")
        if os.path.exists(temp_bag_file):
            os.remove(temp_bag_file)
        sys.exit(1)

if __name__ == "__main__":
    fire.Fire(add_tf)
