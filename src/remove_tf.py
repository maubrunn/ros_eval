import rosbag
from alive_progress import alive_bar
import fire
import sys

TF_TOPICS = ["/tf", "/tf_static"]

def remove_tf(input_bag_file, output_bag_file=None):
    if output_bag_file == None:        
        print(f"No output specified going to overwrite old bag: {input_bag_file}\n" 
              "Do you want to continue? [Y/n]")
        if str(input()).lower() not in ["", "y"]:
            print("Exiting")
            sys.exit(1)
        output_bag_file = input_bag_file

            
    # Open the input bag file
    with rosbag.Bag(input_bag_file, 'r') as inbag:
        # Get the list of topics in the input bag
        topics_in_bag = inbag.get_type_and_topic_info().topics.keys()
        tf_flag = True
        for tf_topic in TF_TOPICS:
            tf_flag = tf_flag and tf_topic in set(topics_in_bag)
        if not tf_flag:
            print("No tf message in bag, doing nothing")
            sys.exit(1)

        n  = inbag.get_message_count()
        with alive_bar(n) as bar:
            # Open the output bag file
            with rosbag.Bag(output_bag_file, 'w') as outbag:
                # Iterate through messages in the input bag
                for topic, msg, t in inbag.read_messages():
                    bar()
                    # Check if the topic is one of the specified topics
                    if topic in TF_TOPICS:
                        continue
                    # Write the message to the output bag
                    outbag.write(topic, msg, t)

    print("Bag without tf created:", output_bag_file)



if __name__ == "__main__":
    fire.Fire(remove_tf)
