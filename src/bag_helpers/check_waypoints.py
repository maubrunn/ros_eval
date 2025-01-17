#!/usr/bin/env python3
import rosbag
import pandas as pd
import os
import shutil
from scipy.optimize import minimize
from sklearn.metrics import mean_squared_error
from scipy.spatial import KDTree
from readwrite_global_waypoints.rw_global_waypoints_src import GlobalWaypoints, write_global_waypoints, read_global_waypoints
import numpy as np
from f110_msgs.msg import WpntArray, Wpnt
import json
#add plor_helpers as module to oath
import sys
sys.path.append(os.path.join(os.getcwd(), 'src'))


from plot_helpers.plot import plot_data
WAYPOINTS_TOPIC = '/global_waypoints'

PATH_ROOT = '/home/moe/data/'

BAGS = [
    'carla/eval/pbl_new/pbl.bag',
    'GokartBags/241204/fast_lap.bag'
]

FROM_MAP = ["carla", None]

RESULTS_PATH = "gokart_waypoint"

def get_waypoints(wpnts_msg):
    data = []
    for wpnt in wpnts_msg.wpnts:
        data.append({
            "x": wpnt.x_m,
            "y": wpnt.y_m,
            "yaw": wpnt.psi_rad,
            "vx": wpnt.vx_mps,
            "ax": wpnt.ax_mps2,
            "s": wpnt.s_m,
        })
    return data


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

    # Define the error function to minimize
    def alignment_error(params):
        dx, dy, theta = params
        transformed_coords2 = transform(coords2, dx, dy, theta)

        # Use KDTree to find nearest neighbors
        tree = KDTree(transformed_coords2)
        distances, _ = tree.query(coords1, k=1)

        # Calculate the mean squared error
        return np.mean(distances**2)

    # Initial guess for [dx, dy, theta]
    initial_guess = [0, 0, 0]

    # Minimize the alignment error
    result = minimize(alignment_error, initial_guess, method='BFGS')

    # Extract the optimal parameters
    dx, dy, theta = result.x

    # Calculate the final aligned coordinates and error
    aligned_coords2 = transform(coords2, dx, dy, theta)
    tree = KDTree(aligned_coords2)
    _, indices = tree.query(coords1, k=1)
    matched_coords2 = aligned_coords2[indices]
    mse = mean_squared_error(coords1, matched_coords2)
    

    return mse, np.array([dx, dy, theta]), matched_coords2, indices


def get_data(bag_path):
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=WAYPOINTS_TOPIC):
            return pd.DataFrame(get_waypoints(msg)), msg

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
    if os.path.exists(dirpath) and os.path.isdir(dirpath):
        print(dirpath, " exists, do you want to overwrite it? (y/[n])")
        if True or input() == 'y':
            shutil.rmtree(dirpath)
        else:
            print("Exiting")
            return
    os.mkdir(dirpath)
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
            wpnts = read_global_waypoints(map)
            for w in wpnts.global_traj_wpnts_iqp.wpnts:
                d.append({"x": w.x_m, "y": w.y_m})
            data[i] = pd.DataFrame(d)   
    mse, transformation_matrix, coords2, indices = align_and_calculate_error(data[0], data[1])

    print("Mean squared error after alignment:", mse)
    print("Transformation matrix:", transformation_matrix)
    df2 = pd.DataFrame(coords2, columns=['x', 'y'])
    df2['s_m'] = data[1]['s'].iloc[indices].to_numpy()
    df2['psi_rad'] = data[1]['yaw'].iloc[indices].to_numpy()
    df2['vx_mps'] = data[1]['vx'].iloc[indices].to_numpy()
    df2['ax_mps2'] = data[1]['ax'].iloc[indices].to_numpy()

    #sort by s
    # df2 = df2.sort_values(by=['x'], ignore_index=True)
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
        ["pbl", "gokart"],
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
