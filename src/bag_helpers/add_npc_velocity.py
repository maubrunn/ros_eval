    import os
    import rosbag
    from alive_progress import alive_bar
    import fire
    import math
    import sys
    import rospy
    from nav_msgs.msg import Odometry
    from f110_msgs.msg import WpntArray
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
    from readwrite_global_waypoints import read_global_waypoints


    def calculate_scalar_error(waypoints, poses, initial_guess):
        """
        Calculate the scalar error between the velocity data in waypoints and poses.

        Args:
            waypoints (list): List of waypoints with ground truth velocity and position.
            poses (list): List of poses extracted from the bagfile.
            initial_guess (float): Initial guess for the velocity scalar.

        Returns:
            float: The calculated error for the velocity scalar.
        """
        error = 0.0
        total_points = min(len(waypoints), len(poses)) - 1  # Avoid index out of range
        
        for i in range(total_points):
            # Waypoint and pose at current index
            waypoint = waypoints[i]
            current_pose = poses[i]
            next_pose = poses[i + 1]
            
            # Time difference
            dt = (next_pose.header.stamp - current_pose.header.stamp).to_sec()

            # Compute the expected distance traveled based on the scalar
            direction_x = waypoint.x_m - current_pose.pose.position.x
            direction_y = waypoint.y_m - current_pose.pose.position.y
            direction_norm = math.sqrt(direction_x ** 2 + direction_y ** 2)
            
            if direction_norm > 0:
                direction_x /= direction_norm
                direction_y /= direction_norm

                # Calculate expected pose
                expected_x = current_pose.pose.position.x + direction_x * waypoint.vx_mps * initial_guess * dt
                expected_y = current_pose.pose.position.y + direction_y * waypoint.vx_mps * initial_guess * dt

                # Compute the error
                error += (expected_x - next_pose.pose.position.x) ** 2 + (expected_y - next_pose.pose.position.y) ** 2

        return error / total_points

    def guess_velocity_scalar(recorded_poses, waypoints, initial_guess=0.4):
        """
        Guess the velocity scalar by minimizing the positional error between recorded poses and waypoints.
        
        Args:
            recorded_poses (list): List of recorded pose messages.
            waypoints (list): List of waypoint messages.
            initial_guess (float): Initial guess for the velocity scalar.

        Returns:
            float: The best guess for the velocity scalar.
        """

        best_scalar = initial_guess
        min_error = calculate_scalar_error(waypoints, recorded_poses, initial_guess)
        for scalar in [initial_guess * (1.0 + i * 0.01) for i in range(-100, 101)]:
            error = calculate_scalar_error(waypoints, recorded_poses, initial_guess)
            if error < min_error:
                min_error = error
                best_scalar = scalar
                print(f"New best scalar: {best_scalar}, error: {min_error}")

        return best_scalar

    def calculate_odom_with_waypoints(input_bag_file, map_name, output_bag_file=None, pose_topic="/carla_interface/traffic_manager/npc_1/pose", odom_topic="/carla_interface/traffic_manager/npc_1/odom"):
        """
        Calculate odometry messages using global waypoints and guess velocity scalar.

        Args:
            input_bag_file (str): Path to the input ROS bag file.
            map_name (str): Name of the map to load global waypoints.
            output_bag_file (str, optional): Path to save the output ROS bag file. Defaults to overwriting the input file.
            pose_topic (str): Topic containing the pose data.
            odom_topic (str): Topic to write the odometry messages.
        """
        print(f"Calculating odometry from '{input_bag_file}' with waypoints from map '{map_name}'.")

        if output_bag_file is None:
            print(f"No output specified. Going to overwrite old bag: {input_bag_file}\n"
                "Do you want to continue? [Y/n]")
            if str(input()).lower() not in ["", "y"]:
                print("Exiting")
                sys.exit(1)
            output_bag_file = input_bag_file

        temp_bag_file = f"{os.path.dirname(output_bag_file)}/tmp.bag"

        try:
            # Load waypoints
            _, _, _, _, _, glb_wpnts, _, _, _, _, _ = read_global_waypoints(map_name)
            waypoints = glb_wpnts.wpnts

            with rosbag.Bag(input_bag_file, 'r') as inbag:
                topics_in_bag = inbag.get_type_and_topic_info().topics.keys()
                if pose_topic not in topics_in_bag:
                    print(f"The specified pose topic '{pose_topic}' does not exist in the bag. Exiting.")
                    sys.exit(1)

                n = inbag.get_message_count()
                recorded_poses = []

                with alive_bar(n) as bar:
                    with rosbag.Bag(temp_bag_file, 'w') as outbag:
                        for topic, msg, t in inbag.read_messages():
                            bar()
                            outbag.write(topic, msg, t)

                            if topic == pose_topic:
                                recorded_poses.append(msg)

                        # Guess velocity scalar
                        print("Guessing velocity scalar...")
                        scalar = guess_velocity_scalar(recorded_poses, waypoints)
                        print(f"Guessed scalar: {scalar}")

                        # Re-process the bag to calculate odometry
                        for pose, waypoint in zip(recorded_poses, waypoints):
                            vx_body = waypoint.vx_mps * scalar
                            angular_velocity = waypoint.kappa_radpm * waypoint.vx_mps * scalar

                            odom_msg = Odometry()
                            odom_msg.header = pose.header
                            odom_msg.header.frame_id = "map"
                            odom_msg.child_frame_id = "base_link"
                            odom_msg.pose.pose = pose.pose
                            odom_msg.twist.twist.linear.x = vx_body
                            odom_msg.twist.twist.linear.y = 0.0
                            odom_msg.twist.twist.angular.z = angular_velocity
                            outbag.write(odom_topic, odom_msg, pose.header.stamp)


            if input_bag_file == output_bag_file:
                print("Deleting file:", input_bag_file)
                os.remove(input_bag_file)
            os.rename(temp_bag_file, output_bag_file)
            print(f"Saved new bag as: {output_bag_file}")

        except Exception as e:
            print(f"An error occurred: {e}")
            if os.path.exists(temp_bag_file):
                os.remove(temp_bag_file)
            sys.exit(1)

    if __name__ == "__main__":
        fire.Fire(calculate_odom_with_waypoints)
