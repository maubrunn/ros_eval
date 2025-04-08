import pandas as pd
import alive_progress
from utils.math import get_yaw
import rosbag
import numpy as np
from typing import List
from sensor_msgs.msg import Image


def get_markers_before(bagpath, topic, time):
    last_valid_markers = None

    with rosbag.Bag(bagpath) as bag:
        messages = bag.read_messages(topics=[topic])
        for _, msg, _ in messages:
            t = msg.markers[0].header.stamp.to_sec()
            if t > time:
                break
            last_valid_markers = msg.markers

    if not last_valid_markers:
        return pd.DataFrame(columns=["x", "y"])

    return pd.DataFrame.from_records(
        ((m.pose.position.x, m.pose.position.y) for m in last_valid_markers), 
        columns=["x", "y"]
    )

def get_markers(bagpath, topic):
    last_valid_markers = None

    with rosbag.Bag(bagpath) as bag:
        messages = bag.read_messages(topics=[topic])
        data = []
        for _, msg, _ in messages:
            for m in msg.markers:
                data.append({
                    "x": m.pose.position.x,
                    "y": m.pose.position.y,
                })
        return pd.DataFrame(data)




def get_opponents_trajectories(bagpath, topic="/perception/opp_trajectories", last_only=True):
    with rosbag.Bag(bagpath) as bag:
        last_msg = None
        msgs = []
        for _, msg, _ in bag.read_messages(topics=[topic]):
            msgs.append(msg)
        if len(msgs) == 0:
            raise ValueError("No messages found in bag for topic: ", topic)

    if last_only:
        msgs = [msgs[-1]]
    data = []
    for m in msgs:
        traj_data = []
        for i, opp in enumerate(m.trajectories):
            opp_data = []
            print(i)
            for wpnt in opp.oppwpnts:
                new_wpnt = {
                    "x": wpnt.x_m,
                    "y": wpnt.y_m,
                    "var": wpnt.d_var,
                    "vs": wpnt.proj_vs_mps,
                    "v_var": wpnt.vs_var,
                    "d": wpnt.d_m,
                    "s": wpnt.s_m,
                }
                opp_data.append(new_wpnt)
            traj_data.append(pd.DataFrame(opp_data))
        data.append(traj_data)
    return data


def get_topics_in_bag(bagpath: str) -> List[str]:
    '''
    Get a list of topics in a rosbag.
    
    Parameters:
    bagpath (str): The path to the rosbag file.
        
    Returns:
        List[str]: The list of topics
    '''
    
    with rosbag.Bag(bagpath) as bag:
        return bag.get_type_and_topic_info().topics.keys()

def create_frenet_converter(bag, raceline_topic = "/global_waypoints"):
    from frenet_conversion import FrenetConverter
    raceline, _ = get_waypoints_from_bag(bag, waypoints_topic=raceline_topic)
    raceline_x = np.array(raceline["x"])
    raceline_y = np.array(raceline["y"])
    return FrenetConverter(raceline_x, raceline_y)

def get_images_as_list(
    bagpath: str, 
    image_topic: str) -> List[Image]:
    '''
    Get a list of images from a rosbag.
    
    Parameters:
    bagpath (str): The path to the rosbag file.
    image_topic (str): The topic to extract images from.
        
    Returns:
        List[Image]: The list of images
    '''
    
    with rosbag.Bag(bagpath) as bag:
        return [msg for _, msg, _ in bag.read_messages(topics=[image_topic])]
    

def get_detections_from_bag(bagpath, fc, detection_topic="/perception/obstacles"):
    with rosbag.Bag(bagpath) as bag:
        n = bag.get_message_count(detection_topic)
        with alive_progress.alive_bar(n, title="Detections") as bar:
            data = {}
            for _, msg, _ in bag.read_messages(topics=detection_topic):
                bar()
                for i, o in enumerate(msg.obstacles):
                    opp_s = (o.s_start + o.s_end) / 2
                    opp_d = (o.d_right + o.d_left) / 2
                    x, y = fc.get_cartesian(opp_s, opp_d)
                    id = o.id
                    key = "opp" + str(id)
                    if key in data:
                        data[key]["x"].append(x)
                        data[key]["y"].append(y)
                        data[key]["time"].append(msg.header.stamp.to_sec())
                        data[key]["s"].append(opp_s)
                        data[key]["d"].append(opp_d)
                        data[key]["vs"].append(o.vs)
                    else:
                        data[key] = {
                            "x": [x],
                            "y": [y],
                            "time": [msg.header.stamp.to_sec()],
                            "s": [opp_s],
                            "d": [opp_d],
                            "vs": [o.vs],
                        }
            return [pd.DataFrame(data[key]) for key in data.keys()]
        print("No detections found in bag: ", bagpath)
        raise ValueError("No detections found in bag")

def get_trackbounds_from_bag(bagpath, trackbounds_topic="/trackbounds/markers"):
    exterior = []
    interior = []
    switched = False
    old_pos = None
    with rosbag.Bag(bagpath) as bag:
        for _, msg, _ in bag.read_messages(topics=trackbounds_topic):
            n = len(msg.markers)
            with alive_progress.alive_bar(n, title="Trackbounds") as bar:
                for bound in msg.markers:
                    bar()
                    if switched:
                        interior.append({
                            "x": bound.pose.position.x,
                            "y": bound.pose.position.y,
                        })
                    else:
                        x = bound.pose.position.x
                        y = bound.pose.position.y
                        if old_pos is not None:
                            if (old_pos[0] - x) ** 2 + (old_pos[1] - y) ** 2 > 2.0:
                                switched = True
                                interior.append({
                                    "x": x,
                                    "y": y,
                                })
                                continue
                        old_pos = (x, y)
                        exterior.append({
                            "x": x,
                            "y": y,
                        })
            
                return pd.DataFrame(exterior), pd.DataFrame(interior)
            
        print("No trackbounds found in bag: ", bagpath)
        raise ValueError("No trackbounds found in bag")

def get_waypoints_from_bag(bagpath, waypoints_topic="/global_waypoints"):
    with rosbag.Bag(bagpath) as bag:
        for _, msg, _ in bag.read_messages(topics=waypoints_topic):
            data = []
            for wpnt in msg.wpnts:
                data.append({
                    "x": wpnt.x_m,
                    "y": wpnt.y_m,
                    "yaw": wpnt.psi_rad,
                    "vx": wpnt.vx_mps,
                    "ax": wpnt.ax_mps2,
                    "s": wpnt.s_m,
                })
            return pd.DataFrame(data), msg
        print("No waypoints found in bag: ", bagpath)
        raise ValueError("No waypoints found in bag")

def get_states_from_bag(bagpath, odom_topic="/car_state/odom", frenet=False, normalize_time=False):
    """
    Get a pandas DataFrame from a rosbag.

    Parameters:
        bag (rosbag.Bag): The rosbag to extract data from.

    Returns:
        pd.DataFrame: The extracted data.
    """
    if frenet:
        data = {
            'time': [],
            's': [],
            'd': [],
            'vs': []
        }
    else:
        data = {
            'time': [],
            'position_x': [],
            'position_y': [],
            'velocity_x': [],
            'velocity_y': [],
            'yaw': [],
            'yaw_rate': []
        }
    column_names = list(data.keys())
    start_time = 0
    with rosbag.Bag(bagpath) as bag:
        n = bag.get_message_count(odom_topic)
        with alive_progress.alive_bar(n, title="Getting states") as bar:
            for _, msg, _ in bag.read_messages(topics=[odom_topic]):
                if normalize_time and start_time == 0:
                    start_time = msg.header.stamp.to_sec()
                data[column_names[0]].append(msg.header.stamp.to_sec() - start_time)
                data[column_names[1]].append(msg.pose.pose.position.x)
                data[column_names[2]].append(msg.pose.pose.position.y)
                data[column_names[3]].append(msg.twist.twist.linear.x)
                if not frenet:
                    data[column_names[4]].append(msg.twist.twist.linear.y)
                    data[column_names[5]].append(get_yaw(msg))
                    data[column_names[6]].append(msg.twist.twist.angular.z)
                bar()
    if len(data[column_names[0]]) == 0:
        raise ValueError("No data found in bag for topic: ", odom_topic)
    return pd.DataFrame(data)
