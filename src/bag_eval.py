#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import pandas as pd
from tf.transformations import euler_from_quaternion
from alive_progress import alive_bar
import os
import shutil
from plot_helpers.plot import plot_data

ODOM_TOPIC = '/car_state/odom'
ACKERMANN_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'
PATH_ROOT = '/home/moe/data/carla/se_eval/gokart'


if PATH_ROOT[-1] != '/':
    PATH_ROOT += '/'

BAGS = [
    'carlamap_bb.bag',
    'wintimap_bb.bag'
]
LABELS = ["sim", 'real']

RESULTS_PATH = "carla_se_gokart"

second_bag_offset = 0
max_time = 120000

normalize = False

start_offsets = [10, 10]
# start_offsets = [0, 0]



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
        odom_topic = GOKART_ODOM_TOPIC if 'idsc' in bag_path else ODOM_TOPIC
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
                    "position_x": msg.pose.pose.position.x - x_init,
                    "position_y": msg.pose.pose.position.y - y_init,
                    "velocity_x": msg.twist.twist.linear.x,
                    "velocity_y": msg.twist.twist.linear.y,
                    "yaw": get_yaw(msg) - yaw_init,
                    "vyaw": msg.twist.twist.angular.z,
                })
                bar()
    return  pd.DataFrame(data)

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

    # plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_speed.pdf"), f"Speed vx [m/s]", 'time [s]', 'v [m/s]', "acc_x")
    plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_positionx.pdf"), f"Position", 'time [s]', 'x [m]', "position_x", LABELS)
    plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_positiony.pdf"), f"Position", 'time [s]', 'y [m]', "position_y", LABELS)
    plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_velocityx.pdf"), f"Velocity", 'time [s]', 'vx [m/s]', "velocity_x", LABELS)
    plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_velocityy.pdf"), f"Velocity", 'time [s]', 'vy [m/s]', "velocity_y", LABELS)
    plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_yaw.pdf"), f"Orientation", 'time [s]', 'yaw [rad]', "yaw", LABELS)
    plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_yawdot.pdf"), f"Angular Velocity", 'time [s]', 'angular velocity z [rad / s]', "vyaw", LABELS)

if __name__ == '__main__':
    main()
