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
PATH_ROOT = '/home/moe/bagfiles/gokart/bb/se-test/'
GOKART_ODOM_TOPIC = '/bumblebee/se2_localization/gokart_state'

BAGS = [
    'loc_mapping.bag',
]
LABELS = ["pbl"]

RESULTS_PATH = "pbl_eval_mean"

start_offset = 180
max_time = 0


def calculate_mean(df, col, dt=5):
    mean = []
    print('calculating mean...')
    with alive_bar(len(df['time'])) as bar:
        for i in range(0, len(df['time'])):
            # get all  rows of col where col time is between df[time][i] and df[time][i] + dt
            mean.append(df[(df['time'] >= df['time'][i]) & (df['time'] < df['time'][i] + dt)][col].mean())
            bar()
    return mean

def calculate_variance(df, col, mean_col):
    variance = []
    print('calculating variance...')
    variance = abs(df[col] - df[mean_col])
    return variance.mean()

def get_data(bag_path):
    start_time = None
    data = []
    with rosbag.Bag(bag_path) as bag:
        odom_topic = GOKART_ODOM_TOPIC if 'idsc' in bag_path else ODOM_TOPIC
        n = bag.get_message_count(odom_topic)
        with alive_bar(n) as bar:
            for _, msg, t in bag.read_messages(topics=[odom_topic]):
                if start_time is None:
                    start_time = t.to_sec()
                if t.to_sec() - start_time < start_offset:
                    bar()
                    continue
                # add to dataframe
                if 'idsc' in bag_path:
                    data.append({
                        "time": t.to_sec() - start_time,
                        "velocity_x": msg.pose2d_dot.x,
                    })
                else:
                    data.append({
                        "time": t.to_sec() - start_time,
                        "velocity_x": msg.twist.twist.linear.x,
                    })
                bar()
    data = pd.DataFrame(data)
    data['mean'] = calculate_mean(data, 'velocity_x')
    var = calculate_variance(data, 'velocity_x', 'mean')
    return  data, var
    

def plot_data(data, path, title, x_label, y_label, col_bag, mean=False):
    figure= plt.figure( figsize=[10, 10])
    plt1 = figure.add_subplot(1, 1, 1)
    plt1.margins(x=0, y=0)
    plt1.plot(data[0]['time'], data[0][col_bag], label=LABELS[0])
    if len(data) > 1:
        plt1.plot(data[1]['time'], data[1][col_bag], label=LABELS[1])
        if mean:
            plt1.plot(data[1]['time'], data[1]['mean'], label='mean', linestyle='--')

    if mean:
        plt1.plot(data[0]['time'], data[0]['mean'], label='mean', linestyle='--')
    # plt1.title.set_text(title)
    plt1.set_xlabel(x_label)
    plt1.set_ylabel(y_label)
    plt1.grid()
    plt1.legend()
    plt1.set_xticklabels([])
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
        df, var = get_data(PATH_ROOT + bag_name)
        with open(os.path.join(dirpath, "var.txt"), 'w') as f:
            f.write('Var: ' + str(var) + '\n')
            f.close()
        data.append(df)
    plot_data(data, os.path.join(dirpath, f"{RESULTS_PATH}_velocityx.pdf"), f"velocity x [m/s]", 'Time [s]', 'vx [m/s]', "velocity_x", True)
if __name__ == '__main__':
    main()