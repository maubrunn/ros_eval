import rosbag
from alive_progress import alive_bar

def filter_bag(input_bag_file, output_bag_file, topics, start=-1, end=-1):
    print(
        f"Filtering bag:\t{input_bag_file}\
            Output bag:\t{output_bag_file}\
            Topics:\t{topics}\
            Start:\t{start}\
            End:\t{end}")
    # Open the input bag file
    with rosbag.Bag(input_bag_file, 'r') as inbag:

        # Get the list of topics in the input bag
        topics_in_bag = inbag.get_type_and_topic_info().topics.keys()
        n  = inbag.get_message_count()
        # Validate specified topics

        with alive_bar(n) as bar:
            # Open the output bag file
            with rosbag.Bag(output_bag_file, 'w') as outbag:
                # Iterate through messages in the input bag
                i = 0
                for topic, msg, t in inbag.read_messages():
                    i += 1
                    bar()
                    if topic in topics:
                        if (start == -1 or start <= t.to_sec()) and (end == -1  or t.to_sec() <= end):
                            outbag.write(topic, msg, t)

    print("Filtered bag created:", output_bag_file)

if __name__ == "__main__":
    root = "/home/moe/data/"
    input_bags = [
        "GokartBags/241204/fast_lap2.bag",
        "carla/eval/carla/carla_tt08.bag",
        "carla/eval/pbl/pbl_tt08.bag"
    ]
    output_bags = [
        "carla/eval/plotbags/real.bag",
        "carla/eval/plotbags/carla.bag",
        "carla/eval/plotbags/pbl.bag"
    ]
    topics = [
               "/car_state/odom",
               "/global_waypoints",
                ]

    times = [
        [1733320156.068997, 1733320181.5276232],
        [711.180252628, 744.360251909],
        [240.805351712, 264.879797597]
    ]
    for i in range(len(input_bags)):
        filter_bag(root + input_bags[i], root + output_bags[i], topics, times[i][0], times[i][1])