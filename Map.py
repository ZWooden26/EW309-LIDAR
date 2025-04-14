import roslibpy
import time
import math

class OccupancyGridMapper:
    def __init__(self, ip, port, robot_name, topic_name):
        self.ros = roslibpy.Ros(host=ip, port=port)
        self.robot_name = robot_name
        self.topic_name = topic_name
        self.map_topic = roslibpy.Topic(self.ros, f'/{robot_name}/{topic_name}', 'nav_msgs/OccupancyGrid')
        self.odom = roslibpy.Topic(self.ros, f'/{robot_name}/odom', 'nav_msgs/Odometry')
        self.lidar = roslibpy.Topic(self.ros, f'/{robot_name}/scan', 'sensor_msgs/LaserScan')

        self.ranges = []
        self.angs = []
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.initial_x = None
        self.initial_y = None

    def callback_odom(self, msg):
        try:
            self.x = msg.get('pose').get('pose').get('position').get('x', 0)
            self.y = msg.get('pose').get('pose').get('position').get('y', 0)
            if self.initial_x is None:
                self.initial_x = self.x
            if self.initial_y is None:
                self.initial_y = self.y
            o = msg.get('pose').get('pose').get('orientation')
            x_o = o.get('x', 0)
            y_o = o.get('y', 0)
            z_o = o.get('z', 0)
            w_o = o.get('w', 1)
            self.yaw = math.atan2(2 * (w_o * z_o + x_o * y_o), 1 - 2 * (y_o**2 + z_o**2))
        except Exception as e:
            print(f"Error in odometry callback: {e}")

    def callback_lidar(self, msg):
        try:
            angle_min = msg.get('angle_min', 0)
            angle_max = msg.get('angle_max', 0)
            angle_increment = msg.get('angle_increment', 0)
            self.ranges = msg.get('ranges', [])
            if self.ranges:
                self.angs = [angle_min + i * angle_increment for i in range(len(self.ranges))]
            else:
                self.angs = []
        except Exception as e:
            print(f"Error in LiDAR callback: {e}")

    def make_grid(self):
        res = 0.05
        Wid = 50
        hgt = 100

        if not hasattr(self, 'data'):
            self.data = [-1] * (Wid * hgt)  # Initialize with -1 only once

        if not self.ranges or not self.angs:
            print("Warning: LiDAR data is empty. Skipping grid generation.")
            return None

        for r, a in zip(self.ranges, self.angs):
            if r > 0:
                x_cell = int(((self.x - self.initial_x) + r * math.cos(a + self.yaw)) / res)
                y_cell = int(((self.y - self.initial_y) + r * math.sin(a + self.yaw)) / res)
                if 0 <= x_cell < Wid and 0 <= y_cell < hgt:
                    self.data[y_cell * Wid + x_cell] = 100

        grid_mesg = {
            'header': {
                'frame_id': f'{self.robot_name}/odom',
                'stamp': {
                    'secs': int(time.time()),
                    'nsecs': 0
                }
            },
            'info': {
                'map_load_time': {
                    'secs': int(time.time()),
                    'nsecs': 0
                },
                'resolution': res,
                'width': Wid,
                'height': hgt,
                'origin': {
                    'position': {
                        'x': self.initial_x if self.initial_x is not None else 0.0,
                        'y': self.initial_y if self.initial_y is not None else 0.0,
                        'z': 0.0
                    },
                    'orientation': {
                        'x': 0.0,
                        'y': 0.0,
                        'z': 0.0,
                        'w': -1.0
                    }
                }
            },
            'data': self.data
        }
        return grid_mesg

    def run(self):
        self.ros.run()
        self.reset_odometry()
        self.odom.subscribe(self.callback_odom)
        self.lidar.subscribe(self.callback_lidar)

        while self.ros.is_connected:
            grid_message = self.make_grid()
            if grid_message:
                self.map_topic.publish(roslibpy.Message(grid_message))
                print('Publishing map with data:', grid_message['data'][:10])
            else:
                print("Skipping map publication due to missing data.")
            time.sleep(3)

        self.ros.terminate()

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

if __name__ == "__main__":
    IP = '192.168.8.104'
    portnumb = 9012
    robot_name = 'juliet'
    topic_name = 'maplol'

    mapper = OccupancyGridMapper(IP, portnumb, robot_name, topic_name)
    mapper.run()