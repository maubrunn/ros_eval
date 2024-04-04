#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import pandas as pd
from tf.transformations import euler_from_quaternion
from alive_progress import alive_bar
import os
import shutil

plt.rcParams.update({'font.size': 26})
ODOM_TOPIC = '/car_state/odom'
PATH_ROOT = '/home/moe/bagfiles/gokart/bb/se-test/'
GOKART_ODOM_TOPIC = '/bumblebee/se2_localization/gokart_state'

BAGS = [
    'idscloc_fast.bag',
    'loc_fast.bag'
]
LABELS = ["pbl", 'idsc']

RESULTS_PATH = "idsc_eval_fast"

start_offset = 5
max_time = 0
x_init = 0
y_init = 0

def get_data(bag_path):
    df = pd.DataFrame(columns=["time, position_x, position_y, velocity_x, velocity_y"])
    start_time = None
    with rosbag.Bag(bag_path) as bag:
        odom_topic = GOKART_ODOM_TOPIC if 'idsc' in bag_path else ODOM_TOPIC
        n = bag.get_message_count(odom_topic)
        with alive_bar(n) as bar:
            for _, msg, t in bag.read_messages(topics=[odom_topic]):

                    if start_time is None:
                        start_time = t.to_sec()
                        if 'idsc' in bag_path:
                            x_init = msg.pose2d.x
                            y_init = msg.pose2d.y
                        else:
                            x_init = msg.pose.pose.position.x
                            y_init = msg.pose.pose.position.y
                    if t.to_sec() - start_time < start_offset or (max_time > 0 and t.to_sec() - start_time > max_time):
                        bar()
                        if 'idsc' in bag_path:
                            x_init = msg.pose2d.x
                            y_init = msg.pose2d.y
                        else:
                            x_init = msg.pose.pose.position.x
                            y_init = msg.pose.pose.position.y
                        continue

                # add to dataframe
                    if 'idsc' in bag_path:
                        df = pd.concat([df, pd.DataFrame({
                            "time": t.to_sec() - start_time,
                            "position_x": msg.pose2d.x - x_init,
                            "position_y": msg.pose2d.y - y_init,
                            "velocity_x": msg.pose2d_dot.x,
                            "velocity_y": msg.pose2d_dot.y,
                            "yaw": msg.pose2d.theta,
                            "vyaw": msg.pose2d_dot.theta,
                        }, index=[0])], axis=0, ignore_index=True)
                    else:
                        yaw = euler_from_quaternion([
                                msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w])[2]
                        df = pd.concat([df, pd.DataFrame({
                            "time": t.to_sec() - start_time,
                            "position_x": msg.pose.pose.position.x - x_init,
                            "position_y": msg.pose.pose.position.y - y_init,
                            "velocity_x": msg.twist.twist.linear.x,
                            "velocity_y": msg.twist.twist.linear.y,
                            "yaw": yaw,
                            "vyaw": msg.twist.twist.angular.z,
                        }, index=[0])], axis=0, ignore_index=True)
                    bar()
    return df
    

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
    plt1.set_xticklabels([])
    plt.savefig(path, bbox_inches='tight', pad_inches=0.1)

def main():
    j =0

    dirpath = os.path.join(os.getcwd(),'plots', RESULTS_PATH)
    if os.path.exists(dirpath) and os.path.isdir(dirpath):
        print(dirpath, " exists, do you want to delete it? (y/[n])")
        if input() == 'y':
            shutil.rmtree(dirpath)
            os.mkdir(dirpath)
    else:
        os.mkdir(dirpath)
    data = []
    for bag_name in BAGS:
        print('Evaluating: ', bag_name)
        data.append(get_data(PATH_ROOT + bag_name))
    plot_data(data, os.path.join(dirpath, "positionx.pdf"), f"position x [m]", 'Time [s]', 'x [m]', "position_x")
    plot_data(data, os.path.join(dirpath, "positiony.pdf"), f"position y [m]", 'Time [s]', 'y [m]', "position_y")
    plot_data(data, os.path.join(dirpath, "velocityx.pdf"), f"velocity x [m/s]", 'Time [s]', 'vx [m/s]', "velocity_x")
    plot_data(data, os.path.join(dirpath, "velocityy.pdf"), f"velocity y [m/s]", 'Time [s]', 'vy [m/s]', "velocity_y")
    plot_data(data, os.path.join(dirpath, "yaw.pdf"), f"yaw [rad]", 'Time [s]', 'yaw [rad]', "yaw")
    plot_data(data, os.path.join(dirpath, "yawdot.pdf"), f"Angular Velocity [rad]", 'Time [s]', 'Angular velocity z [rad / s]', "vyaw")
    
if __name__ == '__main__':
    main()