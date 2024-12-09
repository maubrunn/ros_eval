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
ODOM_TOPIC = '/car_state/odom'
ACKERMANN_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'
PATH_ROOT = '/home/moe/tmp/rosbags/carla_comparison/'
GOKART_ODOM_TOPIC = '/bumblebee/se2_localization/gokart_state'

BAGS = [
    'carla_tt_70.bag',
    'tt_70.bag'
]
LABELS = ["carla", 'nuc']

RESULTS_PATH = "carla_nuc_eval"

second_bag_offset = 0
max_time = 10

normalize = False

start_offsets = [0, 5]



def get_yaw(msg):
    return euler_from_quaternion([
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ])[2]

def set_init_values(msg, idsc):
    if idsc:
        x_init = msg.pose2d.x
        y_init = msg.pose2d.y
        yaw_init = msg.pose2d.theta
    else:
        x_init = msg.pose.pose.position.x
        y_init = msg.pose.pose.position.y
        yaw_init = get_yaw(msg)
    return x_init, y_init, yaw_init

def get_data(bag_path, i):
    x_init = 0
    y_init = 0
    yaw_init = 0
    start_time = None
    data = []
    once = True
    with rosbag.Bag(bag_path) as bag:
        odom_topic = GOKART_ODOM_TOPIC if 'idsc' in bag_path else ACKERMANN_TOPIC
        n = bag.get_message_count(odom_topic)
        start_offset = start_offsets[i]
        with alive_bar(n) as bar:
            for _, msg, t in bag.read_messages(topics=[odom_topic]):
                if start_time is None:
                    start_time = t.to_sec()
                    if normalize:
                        x_init, y_init, yaw_init = set_init_values(msg, 'idsc' in bag_path)


                if t.to_sec() - start_time < start_offset:
                    bar()
                    if normalize:
                        x_init, y_init, yaw_init = set_init_values(msg, 'idsc' in bag_path)
                    continue

                if t.to_sec() - start_time > max_time + start_offset:
                    break

            # add to dataframe
                data.append({
                    "time" : t.to_sec() - start_time - start_offset,
                    "acc_x": msg.drive.speed
                })
                bar()
    return  pd.DataFrame(data)


def plot_data(data, path, title, x_label, y_label, col_bag):
    figure= plt.figure( figsize=[10, 10])
    plt1 = figure.add_subplot(1, 1, 1)
    plt1.margins(x=0, y=0)
    plt1.plot(data[0]['time'], data[0][col_bag], label=LABELS[0])
    if len(data) > 1:
        plt1.plot(data[1]['time'], data[1][col_bag], label=LABELS[1])
    # plt1.title.set_text(title)
    plt1.set_xlabel(x_label)
    plt1.set_ylabel(y_label)
    plt1.grid()
    plt1.legend()
    # plt1.set_xticklabels([])
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
    for i, bag_name in enumerate(BAGS):
        print('Evaluating: ', bag_name)
        data.append(get_data(PATH_ROOT + bag_name, i))

    plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_speed.pdf"), f"Speed vx [m/s]", 'Time [s]', 'v [m/s]', "acc_x")
    # plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_positionx.pdf"), f"position x [m]", 'Time [s]', 'x [m]', "position_x")
    # plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_positiony.pdf"), f"position y [m]", 'Time [s]', 'y [m]', "position_y")
    # plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_velocityx.pdf"), f"velocity x [m/s]", 'Time [s]', 'vx [m/s]', "velocity_x")
    # plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_velocityy.pdf"), f"velocity y [m/s]", 'Time [s]', 'vy [m/s]', "velocity_y")
    # plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_yaw.pdf"), f"yaw [rad]", 'Time [s]', 'yaw [rad]', "yaw")
    # plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_yawdot.pdf"), f"Angular Velocity [rad]", 'Time [s]', 'Angular velocity z [rad / s]', "vyaw")

if __name__ == '__main__':
    main()
