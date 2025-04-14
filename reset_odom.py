def reset_odometry(self):
    """Reset odometry to the origin."""
    try:
        reset_odom_service = roslibpy.Service(self.ros, f'/{self.robot_name}/reset_pose', 'irobot_create_msgs/ResetPose')
        result = reset_odom_service.call(roslibpy.ServiceRequest())
        print("ODOM Reset:", result)

        self.initial_yaw = None
        self.yaw = 0

        print(f"Inital Yaw angle set to 0: Yaw {self.yaw}")
    except Exception as e:
        print(f"Failed to reset odometry: {e}") 