#!/usr/bin/env python3
from sklearn.neighbors import KDTree
import rosbag
import pandas as pd
from tf.transformations import euler_from_quaternion
from alive_progress import alive_bar
import os
import fire
import numpy as np
import json
import sys

from utils.files import get_text_of_last_dir
from utils.metrics import pearson_correlation, cross_correlation, kld
from plot_helpers.plot import plot_data


def compare(df1, df2):
    tree = KDTree(df2[['position_x', 'position_y']].to_numpy())
    _, indices = tree.query(df1[["position_x", 'position_y']].to_numpy(), k=1)
    start_idx = indices[0][0]
    new_indices = list(range(start_idx, len(df2)))
    new_indices.extend(list(range(start_idx)))
    old_df = df2.copy()
    df2 = df2.iloc[new_indices]
    df2.reset_index(inplace=True)
    
    error = 0
    for i in range(len(df1)):
        error += (df1.iloc[i]["position_x"] - df2.iloc[i]["position_x"])**2 + (df1.iloc[i]["position_y"] - df2.iloc[i]["position_y"])**2
    print(f"Mean Squared Error: {error/len(df1)}")
    corrs = {}
    for col in df1.columns:
        if col == "time":
            continue
        corrs[col] = {}
        # corr = pearson_correlation(df1[col], df2[col])
        # if corr is not None:
        #     corrs[col]['pearson'] = corr
        
        cross = cross_correlation(df1[col], df2[col])
        if cross is not None:
            corrs[col]['cross'] = cross

        kldiv = kld(df1[col], old_df[col])
        if kldiv is not None:
            corrs[col]['kld'] = kldiv
        
    return df2,  corrs


def calculate_metrics(data, idx):
    gt_df = data[idx]

    total_corrs = []
    
    for i, df in enumerate(data):
        if i == idx:
            total_corrs.append({})
            continue
        print(f"Comparing with {i}")
        updated_df, corrs = compare(gt_df, df)
        data[i] = updated_df
        total_corrs.append(corrs)
    return data, total_corrs



def get_yaw(msg):
    return euler_from_quaternion([
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ])[2]


def get_data(bag_path):
    start_time = None
    data = []
    with rosbag.Bag(bag_path) as bag:
        odom_topic = "/car_state/odom"
        n = bag.get_message_count(odom_topic)
        with alive_bar(n) as bar:
            for _, msg, t in bag.read_messages(topics=[odom_topic]):
                if start_time is None:
                    start_time = t.to_sec()

                data.append({
                    "time" : t.to_sec() - start_time,
                    "position_x": msg.pose.pose.position.x,
                    "position_y": msg.pose.pose.position.y,
                    "velocity_x": msg.twist.twist.linear.x,
                    "velocity_y": msg.twist.twist.linear.y,
                    "yaw": get_yaw(msg),
                    "vyaw": msg.twist.twist.angular.z,
                })
                bar()
    return  pd.DataFrame(data)

def main(dir):
    last_dir = get_text_of_last_dir(dir)
    dirpath = os.path.join(os.getcwd(),'plots/se_comparison/', last_dir)
    if not os.path.exists(dirpath):
        print("making directory: ", dirpath)
        os.mkdir(dirpath)
    data = []
    files = []
    for file in os.listdir(dir):
        if file.endswith(".bag"):
            files.append(file)
            print('Evaluating: ', file)
            data.append(get_data(os.path.join(dir, file)))

    print("Will compute the correlation now. Which bag is the real bag?")
    for i, f in enumerate(files):
        print(f"{i}:\t{f}")
    try:
        idx = int(input())
    except:
        print("Exiting")
        sys.exit(0)
    data, all_corrs = calculate_metrics(data, idx)

    out_json = {}
    for i, f in enumerate(files):
        if i == idx:
            continue
        out_json[f] = all_corrs[i]

    with open(os.path.join(dirpath, "correlations.json"), "w") as f:
        f.write(json.dumps(out_json, indent=4))

        

    plot_data(
        data,
        os.path.join(dirpath, "se_comp_x.pdf"),
        "SE Comparison",
        "idx",
        "pos x",
        "position_x",
        files,
        x_col="pd_indices"
    )

    plot_data(
        data,
        os.path.join(dirpath, "se_comp_y.pdf"),
        "SE Comparison",
        "idx",
        "pos y",
        "position_y",
        files,
        x_col="pd_indices"
    )

    plot_data(
        data,
        os.path.join(dirpath, "se_comp_vel.pdf"),
        "SE Comparison",
        "idx",
        "vel",
        "velocity_x",
        files,
        x_col="pd_indices"
    )


if __name__ == '__main__':
    fire.Fire(main)
