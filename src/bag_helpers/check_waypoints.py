#!/usr/bin/env python3
import os
import sys

import json
import pandas as pd
from scipy.optimize import minimize
from sklearn.metrics import mean_squared_error
from scipy.spatial import KDTree
import numpy as np

from utils.rosbag import get_waypoints_from_bag as get_data
from utils.files import check_and_create_dir, read_global_waypoints
from plot_helpers.plot import plot_data

WAYPOINTS_TOPIC = '/global_waypoints'
PATH_ROOT = ''


if PATH_ROOT[-1] != '/':
    PATH_ROOT += '/'


LABELS= ["carla", "real"]

BAGS = [
    'herbie_carla.bag',
    'fast_lap.bag',
    # 'f110_raw_winti.bag'
]

FROM_MAP = [None, None]
MAP_PATH_ROOT = ''
RESULTS_PATH = "waypoint_comp"


def align_and_calculate_error(df1, df2):
    """
    Align two trajectories and calculate the alignment error using RANSAC.

    Parameters:
        df1 (pd.DataFrame): First trajectory with columns ['x', 'y'].
        df2 (pd.DataFrame): Second trajectory with columns ['x', 'y'].

    Returns:
        float: The mean squared error after alignment.
        np.ndarray: Transformation matrix [rotation, translation].
    """
   # Extract coordinates
    coords1 = df1[['x', 'y']].to_numpy()
    coords2 = df2[['x', 'y']].to_numpy()
    # Define the transformation function
    def transform(coords, dx, dy, theta):
        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        return coords @ rotation_matrix.T + np.array([dx, dy])

    def alignment_error(params):
        dx, dy, theta = params
        transformed_coords2 = transform(coords2, dx, dy, theta)
        tree = KDTree(transformed_coords2)
        distances, _ = tree.query(coords1, k=1)
        return np.mean(distances**2)

    initial_guess = [0, 0, 0]
    result = minimize(alignment_error, initial_guess, method='BFGS')
    dx, dy, theta = result.x
    aligned_coords2 = transform(coords2, dx, dy, theta)
    tree = KDTree(aligned_coords2)
    _, indices = tree.query(coords1, k=1)
    matched_coords2 = aligned_coords2[indices]
    mse = mean_squared_error(coords1, matched_coords2)
    return mse, np.array([dx, dy, theta]), matched_coords2, indices

def rewrite_waypoints(new_wnpts, map_name, transformation_matrix):
    json_path = os.path.join('/home/moe/data/maps', map_name, 'global_waypoints.json')
    file_content = json.loads(open(json_path).read())
    arr = []
    for i in range(len(new_wnpts)):
        wpnt = {}
        transformed_yaw = new_wnpts.iloc[i]['psi_rad'] + transformation_matrix[2]
        wpnt['x_m'] = new_wnpts.iloc[i]['x']
        wpnt['y_m'] = new_wnpts.iloc[i]['y']
        wpnt['psi_rad'] = transformed_yaw
        wpnt['vx_mps'] = new_wnpts.iloc[i]['vx_mps']
        wpnt['ax_mps2'] = new_wnpts.iloc[i]['ax_mps2']
        wpnt['s_m'] = new_wnpts.iloc[i]['s_m']
        arr.append(wpnt)
    file_content['global_traj_wpnts_iqp']['wpnts'] = arr

    with open(json_path, 'w') as f:
        f.write(json.dumps(file_content, indent=4))
    return pd.DataFrame(file_content['global_traj_wpnts_iqp']['wpnts'])

        
def main():
    dirpath = os.path.join(os.getcwd(),'plots', RESULTS_PATH)
    check_and_create_dir(dirpath)
    data = []
    wpnts = []
    for i, bag_name in enumerate(BAGS):
        print('Evaluating: ', bag_name)
        d, w = get_data(PATH_ROOT + bag_name)
        data.append(d)
        wpnts.append(w)

    for i, map in enumerate(FROM_MAP):
        d = []
        if map is not None:
            map_path = os.path.join(MAP_PATH_ROOT, map)
            wpnts = read_global_waypoints(map_path)
            for w in wpnts:
                d.append({"x": w["x_m"], "y": w["y_m"]})
            data[i] = pd.DataFrame(d)   
    mse, transformation_matrix, coords2, indices = align_and_calculate_error(data[0], data[1])

    print("Mean squared error after alignment:", mse)
    print("Transformation matrix:", transformation_matrix)
    df2 = pd.DataFrame(coords2, columns=['x', 'y'])
    df2['s_m'] = data[1]['s'].iloc[indices].to_numpy()
    df2['psi_rad'] = data[1]['yaw'].iloc[indices].to_numpy()
    df2['vx_mps'] = data[1]['vx'].iloc[indices].to_numpy()
    df2['ax_mps2'] = data[1]['ax'].iloc[indices].to_numpy()

    df2.drop_duplicates(subset=['x', 'y'], keep='first', inplace=True)
    s_max = df2['s_m'].max()
    df2['s_m'] = np.linspace(0, s_max, len(df2))
    
    plot_data(
        [data[0], df2],
        os.path.join(dirpath, 'waypoints_aligned.pdf'), 
        'waypoints',
        'Position X [m]', 
        'Position Y [m]', 
        'y',
        LABELS,
        x_col='x',
        pad=True)

    yes = input("Do you want to write the new waypoints to the map? (y/[n])")
    if yes != 'y':
        sys.exit(0)

    wpnt_df = rewrite_waypoints(df2, 'carla_new', transformation_matrix)
    wpnt_df['x'] = wpnt_df['x_m']
    wpnt_df['y'] = wpnt_df['y_m']
    plot_data(
        [wpnt_df, df2],
        os.path.join(dirpath, 'waypoints_aligned_new.pdf'), 
        'waypoints',
        'Position X [m]', 
        'Position Y [m]', 
        'y',
        ["pbl", "gokart"],
        x_col='x',
        pad=True)
    plot_data(
        [wpnt_df, df2],
        os.path.join(dirpath, 'yaw.pdf'), 
        'waypoints',
        's [m]', 
        'yaw [m]', 
        'psi_rad',
        ["pbl", "gokart"],
        x_col='s_m')

    plot_data(
        [wpnt_df, df2],
        os.path.join(dirpath, 'ax.pdf'),
        'ax',
        'Position S [m]',
        'ax [mps2]',
        'ax_mps2',
        ["pbl", "gokart"],
        x_col='s_m'
    )
    



if __name__ == '__main__':
    main()
