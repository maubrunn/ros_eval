import rosbag
from alive_progress import alive_bar

def filter_bag(input_bag_file, output_bag_file, topics):
    # Open the input bag file
    with rosbag.Bag(input_bag_file, 'r') as inbag:
        # Get the list of topics in the input bag
        topics_in_bag = inbag.get_type_and_topic_info().topics.keys()
        n  = inbag.get_message_count()
        # Validate specified topics
        invalid_topics = set(topics) - set(topics_in_bag)
        print(topics_in_bag)
        if invalid_topics:
            print("Invalid topics specified:", invalid_topics)
            return

        with alive_bar(n) as bar:
            # Open the output bag file
            with rosbag.Bag(output_bag_file, 'w') as outbag:
                # Iterate through messages in the input bag
                i = 0
                for topic, msg, t in inbag.read_messages():
                    i += 1
                    bar()
                    # Check if the topic is one of the specified topics
                    if topic in topics:
                        # Write the message to the output bag
                        outbag.write(topic, msg, t)

    print("Filtered bag created:", output_bag_file)

if __name__ == "__main__":
    input_bag_file = "/home/moe/bagfiles/gokart/herbie/map_winti_5.bag"  # Change this to the path of your input bag file
    output_bag_file = "/home/moe/bagfiles/gokart/herbie/map_5_ackermann.bag"  # Change this to the desired path of your output bag file
    topics = [
               "/vesc/high_level/ackermann_cmd_mux/input/nav_1"
                ]

    filter_bag(input_bag_file, output_bag_file, topics)
