#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import pandas as pd
from tf.transformations import euler_from_quaternion
from alive_progress import alive_bar
import os
import shutil
import time
import numpy as np
from plot_helpers.colors import get_colors
from plot_helpers.plot import plot_data

WAYPOINTS = True
WAYPOINTS_TOPIC = '/global_waypoints'


COLORS = get_colors()

TOPICS = ['/car_state/odom']
x_field = 'pose.pose.position.x'
y_field = 'pose.pose.position.y'

VELOCITIES = True
vel_x_field = 'twist.twist.linear.x'
vel_y_field = 'twist.twist.linear.y'

PATH_ROOT = '/home/moe/data/carla/eval/plotbags'

if PATH_ROOT[-1] != '/':
    PATH_ROOT += '/'

BAGS = [
    'real_with_waypoints.bag',
    'carla.bag',
    'pbl.bag'
]

LABELS = ['real', 'pbl', 'carla']

TRANSFORMS = [[4.44582387,  0.83516225, -0.01552528],
     None, None]

# OFFSETS = [
#     [-3, -2],
#     [-4.5, -1],
# ]
OFFSETS = [
    [0, 0],
    [0, 0],
]

RESULTS_PATH = "trajectory_eval"

start_offset = 0

def get_data_from_field(msg, field):
    subfields = field.split('.')
    all_fields = msg.__getattribute__(subfields[0])
    for subfield in subfields[1:]:
        all_fields = all_fields.__getattribute__(subfield)
    return all_fields


def get_waypoints(wpnts_msg):
    data = []
    for wpnt in wpnts_msg.wpnts:
        data.append({
            "wpnt_x": wpnt.x_m,
            "wpnt_y": wpnt.y_m,
            "wpnt_vx": wpnt.vx_mps
        })
    return data


def transform_coords(coords, transform):
    if transform is None:
        return coords
    dx, dy, theta = transform
    rotation_matrix = [
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ]
    return coords @ rotation_matrix + np.array([dx, dy])

def get_data(bag_path, offset=[0, 0], transform=None):
    start_time = None
    data = []
    waypoints = False if WAYPOINTS else True
    if not waypoints:
        TOPICS.append(WAYPOINTS_TOPIC)
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
                if topic == WAYPOINTS_TOPIC:
                    if not waypoints:
                        waypoints = get_waypoints(msg)
                        data.extend(waypoints)
                        

                    bar()
                elif topic == TOPICS[0]:
                    data_dict = {
                        "time": t.to_sec() - start_time,
                    }

                    data_dict['x'] = get_data_from_field(msg, x_field) + offset[0]
                    data_dict['y'] = get_data_from_field(msg, y_field) + offset[1]
                    if VELOCITIES:
                        data_dict['vel_x'] = get_data_from_field(msg, vel_x_field)
                        data_dict['vel_y'] = get_data_from_field(msg, vel_y_field)
                    data.append(data_dict)
                    bar()
    

    
    data_df = pd.DataFrame(data)
    transformed_coords = transform_coords(data_df[['x', 'y']].to_numpy(), transform)
    if transform is not None:
        print(data_df[['x', 'y']].to_numpy())
        print(transformed_coords)
    data_df['x_orig'] = data_df['x']
    data_df['y_orig'] = data_df['y']
    data_df['x'] = transformed_coords[:, 0]
    data_df['y'] = transformed_coords[:, 1]

    return data_df

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
        offset = [0, 0]
        if i != 0:
            offset = OFFSETS[i-1]
        data.append(get_data(PATH_ROOT + bag_name, offset, TRANSFORMS[i]))
    if WAYPOINTS:
        for i in range(len(data)):
                waypoint_pd = pd.DataFrame(data[i])
                waypoint_pd["x_orig"] = waypoint_pd["wpnt_x"]
                waypoint_pd["y_orig"] = waypoint_pd["wpnt_y"]
                plot_data(
                    [data[i], waypoint_pd], 
                    os.path.join(dirpath, f'trajectory_{BAGS[i].strip(".bag")}.pdf'),
                    'NPC Trajectory',
                    'position x [m]', 
                    'position y [m]',
                    'y_orig', 
                    ["npc trajectory", "waypoints"], 
                    x_col='x_orig',
                    second_linestyle='--',
                    pad=True
                )

    plot_data(
        data, 
        os.path.join(dirpath, f'trajectory.pdf'), 
        'Trajectory',
        'Position X [m]', 'Position Y [m]',
        'y', 
        LABELS,
        x_col='x',
        pad=True)
        
    if WAYPOINTS:    
        plot_data(
            data, 
            os.path.join(dirpath, 'trajectory_waypoints.pdf'), 
            'NPC Trajectory',
            'Position X [m]', 
            'Position Y [m]', 
            'wpnt_y',
            LABELS,
            x_col='wpnt_x',
            pad=True)
                
        plot_data(
                data,
                os.path.join(dirpath, 'velocities_waypoints.pdf'),
                "title",
                "Time [s]",
                'Velocity [m/s]',
                'wpnt_vx',
                LABELS,
                x_col='wpnt_x',
                pad=True
            )
    
    if VELOCITIES:

        plot_data(
            data, 
            os.path.join(dirpath, 'velocities_x.pdf'), 
            "Velocity [m/s]",
            'Time [s]', 
            'Velocity [m/s]', 
            'vel_x',
            LABELS
            )

if __name__ == '__main__':
    main()
