#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import pandas as pd
from tf.transformations import euler_from_quaternion
from alive_progress import alive_bar
import os
import shutil
import time

plt.rcParams.update({'font.size': 26})
TOPICS = ['/vesc/high_level/ackermann_cmd_mux/input/nav_1']
FIELDS =  [['drive.acceleration']]
FIELD_DESCRIPTORS = {
    'drive.acceleration': 'Commanded Acceleration [m/s^2]'
}

PATH_ROOT = '/home/moe/tmp/'

BAGS = [
    'mpc_test2.bag',
    'manual1_converted.bag',
]

RESULTS_PATH = "accel_test"

start_offset = 0


def get_data(bag_path):
    start_time = None
    data = []
    with rosbag.Bag(bag_path) as bag:
        n = 0
        for t in TOPICS:
            n += bag.get_message_count(t)
        if n == 0:
            print("No messages in bag")
            return

        with alive_bar(n) as bar:
            for topic, msg, t in bag.read_messages(topics=TOPICS):
                if start_time is None:
                    start_time = t.to_sec()

                if t.to_sec() - start_time < start_offset:
                    bar()
                    continue


                data_dict = {
                    "time": t.to_sec() - start_time,
                }

                topic_index = TOPICS.index(topic)


                for field in FIELDS[topic_index]:
                    subfields = field.split('.')
                    all_fields = msg.__getattribute__(subfields[0])
                    for subfield in subfields[1:]:
                        all_fields = all_fields.__getattribute__(subfield)
                    data_dict[field] = all_fields

            
                data.append(data_dict)
                bar()
    return  pd.DataFrame(data)


def plot_data(data, path, x_label, y_label, col_bag):
    figure= plt.figure( figsize=[10, 10])
    plt1 = figure.add_subplot(1, 1, 1)
    plt1.margins(x=0, y=0)
    for i in range(0, len(data)):
        plt1.plot(data[i]['time'], data[i][col_bag], label=BAGS[i])
    plt1.set_xlabel(x_label)
    plt1.set_ylabel(y_label)
    plt1.grid()
    plt1.set_xticklabels([])
    plt1.legend()
    plt.savefig(path, bbox_inches='tight', pad_inches=0.1)

def main():
    if len(FIELDS) != len(TOPICS):
        print("FIELDS and TOPICS must have the same length")
        return
    total_fields = sum(len(topic_fields) for topic_fields in FIELDS)
    if total_fields != len(FIELD_DESCRIPTORS):
        print("FIELDS and FIELD_DESCRIPTORS must have the same length")
        return
    dirpath = os.path.join(os.getcwd(),'plots', RESULTS_PATH)
    if os.path.exists(dirpath) and os.path.isdir(dirpath):
        print(dirpath, " exists, do you want to overwrite it? (y/[n])")
        if input() == 'y':
            shutil.rmtree(dirpath)
        else:
            print("Exiting")
            return
    os.mkdir(dirpath)
    data = []
    for bag_name in BAGS:
        print('Evaluating: ', bag_name)
        data.append(get_data(PATH_ROOT + bag_name))
    for field in FIELD_DESCRIPTORS.keys():
        plot_data(data, os.path.join(dirpath, field + '.pdf'), 'Time [s]', FIELD_DESCRIPTORS[field], field)

if __name__ == '__main__':
    main()
