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
TOPICS = ['/car_state/odom']
x_field = 'pose.pose.position.x'
y_field = 'pose.pose.position.y'
vel_x_field = 'twist.twist.linear.x'
vel_y_field = 'twist.twist.linear.y'

PATH_ROOT = '/home/moe/tmp/rosbags/nuc6_carla_comparison/'

BAGS = [
    'tt_80.bag',
    'carla.bag',
]

RESULTS_PATH = "trajectory_eval"

start_offset = 0

def get_data_from_field(msg, field):
    subfields = field.split('.')
    all_fields = msg.__getattribute__(subfields[0])
    for subfield in subfields[1:]:
        all_fields = all_fields.__getattribute__(subfield)
    return all_fields


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

                data_dict['x'] = get_data_from_field(msg, x_field)
                data_dict['y'] = get_data_from_field(msg, y_field)
                data_dict['vel_x'] = get_data_from_field(msg, vel_x_field)
                data_dict['vel_y'] = get_data_from_field(msg, vel_y_field)
            
                data.append(data_dict)
                bar()
    return  pd.DataFrame(data)


def plot_data(data, path, x_label, y_label, x_col, y_col):
    figure= plt.figure( figsize=[10, 10])
    plt1 = figure.add_subplot(1, 1, 1)
    plt1.margins(x=0, y=0)
    for i, d in enumerate(data):
        plt1.plot(d[x_col], d[y_col], label=BAGS[i])
    plt1.set_xlabel(x_label)
    plt1.set_ylabel(y_label)
    plt1.legend()
    plt1.grid()
    plt.savefig(path, bbox_inches='tight', pad_inches=0.1)

def main():
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
    plot_data(data, os.path.join(dirpath, 'trajectory.pdf'), 'Position X [m]', 'Position Y [m]', 'x', 'y')
    plot_data(data, os.path.join(dirpath, 'velocities_x.pdf'), 'Time [s]', 'Velocity [m/s]', 'time', 'vel_x')
    plot_data(data, os.path.join(dirpath, 'velocities_y.pdf'), 'Time [s]', 'Velocity [m/s]', 'time', 'vel_y')

if __name__ == '__main__':
    main()
