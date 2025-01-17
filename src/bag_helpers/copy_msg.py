import rosbag
import os

root = "/home/moe/data/"

source_bag = rosbag.Bag(os.path.join(root,'carla/eval/plotbags/real.bag'), 'r')
target_bag = rosbag.Bag(os.path.join(root,'GokartBags/241204/fast_lap.bag'), 'r')
output_bag = rosbag.Bag(os.path.join(root,'carla/eval/plotbags/real_with_waypoints.bag' ), 'w')
print("Copying messages from {} and {} to {}".format(source_bag.filename, target_bag.filename, output_bag.filename))
try:
    # Copy all messages from the second bag
    some_t = None
    for topic, msg, t in source_bag.read_messages():
        output_bag.write(topic, msg, t)
        if not some_t:
            some_t = t
    
    # Copy only the desired topic from the first bag
    for topic, msg, t in target_bag.read_messages(topics=['/global_waypoints']):
        t = some_t
        output_bag.write(topic, msg, t)
finally:
    source_bag.close()
    target_bag.close()
    output_bag.close()