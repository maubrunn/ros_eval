
import os
from os import path
import rosbag
from alive_progress import alive_bar
import fire
import sys
import tempfile

TF_TOPICS = ["/tf", "/tf_static"]

def remove_tf(input_bag_file, output_bag_file=None):
    print("There is a bug where the wrong bag is deleted, first debuig this")
    sys.exit(1)
    if output_bag_file is None:        
        print(f"No output specified. Going to overwrite old bag: {input_bag_file}\n" 
              "Do you want to continue? [Y/n]")
        if str(input()).lower() not in ["", "y"]:
            print("Exiting")
            sys.exit(1)
        output_bag_file = input_bag_file

    # Create a temporary file for the output bag
    temp_bag_file = f"{os.path.dirname(output_bag_file)}/tmp.bag"
    try:
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

            n = inbag.get_message_count()
            with alive_bar(n) as bar:
                # Open the temporary output bag file
                with rosbag.Bag(temp_bag_file, 'w') as outbag:
                    # Iterate through messages in the input bag
                    for topic, msg, t in inbag.read_messages():
                        bar()
                        # Check if the topic is one of the specified topics
                        if topic in TF_TOPICS:
                            continue
                        # Write the message to the output bag
                        outbag.write(topic, msg, t)

        # If everything is successful, replace the original bag with the new one
        print("Bag without tf created:", output_bag_file)
        
        # Remove the original file if it's not the same as the temporary file
        os.remove(output_bag_file)
        
        # Rename the temporary bag file to the original file name
        os.rename(temp_bag_file, output_bag_file)
        print(f"Replaced original bag with: {output_bag_file}")
    
    except Exception as e:
        print(f"An error occurred: {e}")
        # Clean up if anything goes wrong
        if os.path.exists(temp_bag_file):
            os.remove(temp_bag_file)
        sys.exit(1)

if __name__ == "__main__":
    fire.Fire(remove_tf)
