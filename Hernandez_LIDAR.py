import roslibpy
import time
import math
import numpy as np
from collections import deque

# Connect to Ros
ip = '192.168.8.104'
port = 9012

robot_name = 'foxtrot'

ros = roslibpy.Ros(host=ip, port=port)
ros.run()

"""Reset odometry to the origin."""
try:
    reset_odom_service = roslibpy.Service(ros, f'/{robot_name}/reset_pose', 'irobot_create_msgs/ResetPose')
    result = reset_odom_service.call(roslibpy.ServiceRequest())
    print("ODOM Reset:", result)

    initial_yaw = None
    yaw = 0

    print(f"Inital Yaw angle set to 0: Yaw {yaw}")
except Exception as e:
    print(f"Failed to reset odometry: {e}")

class Lidar:
    def __init__(self, ros, robot_name='foxtrot'):
        self.robot_name = robot_name
        self.ros = ros
        # create topics
        self.odom_topic = roslibpy.Topic(ros, f'/{robot_name}/odom', 'nav_msgs/Odometry')
        self.lidar_topic = roslibpy.Topic(ros, f'/{robot_name}/scan', 'sensor_msgs/LaserScan')
        self.map_topic = roslibpy.Topic(ros, f'/{robot_name}/hernandezmap', 'nav_msgs/OccupancyGrid')
        # odom parameters
        self.ranges = []
        self.angles = []
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # mapping parameters
        self.resolution = 0.1  # meter/cell
        self.width = 100
        self.height = 200
        self.data = [-1] * (self.width * self.height)
        # deque for storing only latest scans/odom
        self.buffer = 50
        self.scan_data = deque(maxlen=self.buffer)
        self.odom_data = deque(maxlen=self.buffer)
        self.history = deque(maxlen=10)

    # callback functions
    def callback_odom(self, message):
        stamp = message['header']['stamp']  # extract time stamp
        timestamp = stamp['sec'] + stamp['nanosec'] * 1e-9
        self.x = message.get('pose').get('pose').get('position').get('x')
        self.y = message.get('pose').get('pose').get('position').get('y')
        o = message.get('pose').get('pose').get('orientation')
        qx = o.get('x', 0)
        qy = o.get('y', 0)
        qz = o.get('z', 0)
        qw = o.get('w', 1)
        self.yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))
        self.odom_data.append({
            'timestamp': timestamp,
            'position': {
                'x': self.x,
                'y': self.y},
            'yaw': self.yaw
        })

    def callback_scan(self, message):
        stamp = message['header']['stamp']
        timestamp = stamp['sec'] + stamp['nanosec'] * 1e-9
        angle_min = message.get('angle_min', 0)
        angle_max = message.get('angle_max', 0)
        angle_increment = message.get('angle_increment', 0)
        ranges = message.get('ranges', [])
        angles = [angle_min + i * angle_increment for i in range(len(ranges))]
        mask = np.array(ranges) < 0.1
        self.ranges = np.array(ranges)[~mask]
        self.angles = np.array(angles)[~mask]
        self.scan_data.append({
            'timestamp': timestamp,
            'ranges': self.ranges,
            'angles': self.angles
        })

    # functions
    def subscribe(self):
        self.odom_topic.subscribe(self.callback_odom)
        self.lidar_topic.subscribe(self.callback_scan)

    def unsubscribe(self):
        self.lidar_topic.unsubscribe()
        self.odom_topic.unsubscribe()

    def get_latest_odom(self):
        return list(self.odom_data)

    def get_latest_scan(self):
        return list(self.scan_data)

    def lidar_to_map(self):
        for odom in self.odom_data:
            for scan in self.scan_data:
                if abs(odom['timestamp'] - scan['timestamp']) <= 0.05:
                    x = odom['position']['x']
                    y = odom['position']['y']
                    yaw = odom['yaw']
                    ranges = scan['ranges']
                    angles = scan['angles']
                    break
                else:
                    continue

        xs = [x + r * math.cos(yaw + a) for r, a in zip(ranges, angles)]
        ys = [y + r * math.sin(yaw + a) for r, a in zip(ranges, angles)]
        self.middle = [self.width / 2, self.height / 2]
        x_idx = [int(x / self.resolution + self.middle[0]) for x in xs]
        y_idx = [int(y / self.resolution + self.middle[1]) for y in ys]

        points = set(zip(x_idx, y_idx))
        self.history.append(points)
        for y in range(self.height):
            for x in range(self.width):
                index = y * self.width + x
                if (x, y) in points:
                    self.data[index] = 100
                elif (x, y) not in points and (x, y) not in self.history:
                    self.data[index] = 0
                else:
                    continue

    def make_grid(self):
        for odom in reversed(self.odom_data):
            for scan in reversed(self.scan_data):
                if abs(odom['timestamp'] - scan['timestamp']) <= 0.05:
                    self.x = odom['position']['x']
                    self.y = odom['position']['y']
                    self.yaw = odom['yaw']
                    self.ranges = scan['ranges']
                    self.angs = scan['angles']
                    break
            else:
                continue
            break

        res = 0.025  # meters per cell
        Wid = 500
        hgt = 500

        self.data = [-1] * (Wid * hgt)

        origin_x = Wid // 2
        origin_y = hgt // 2

        for r, a in zip(self.ranges, self.angs):
            if r > 0:
                beam_end_x = self.x + r * math.cos(a + self.yaw)
                beam_end_y = self.y + r * math.sin(a + self.yaw)

                steps = int(r / res)
                for i in range(steps):
                    intermediate_x = self.x + i * res * math.cos(a + self.yaw)
                    intermediate_y = self.y + i * res * math.sin(a + self.yaw)
                    x_cell = int(origin_x + intermediate_x / res)
                    y_cell = int(origin_y - intermediate_y / res)

                    if 0 <= x_cell < Wid and 0 <= y_cell < hgt:
                        index = y_cell * Wid + x_cell
                        if self.data[index] == -1:
                            self.data[index] = 0  # Free space

                x_cell = int(origin_x + beam_end_x / res)
                y_cell = int(origin_y - beam_end_y / res)
                if 0 <= x_cell < Wid and 0 <= y_cell < hgt:
                    index = y_cell * Wid + x_cell
                    self.data[index] = 100  # Obstacle

                red_r = r + 0.1
                red_x = self.x + red_r * math.cos(a + self.yaw)
                red_y = self.y + red_r * math.sin(a + self.yaw)
                red_cell_x = int(origin_x + red_x / res)
                red_cell_y = int(origin_y - red_y / res)
                if 0 <= red_cell_x < Wid and 0 <= red_cell_y < hgt:
                    red_index = red_cell_y * Wid + red_cell_x
                    if self.data[red_index] == -1:
                        self.data[red_index] = 90  # Red zone

        robot_x_cell = int(origin_x + self.x / res)
        robot_y_cell = int(origin_y - self.y / res)
        box_radius = 3

        for dx in range(-box_radius, box_radius + 1):
            for dy in range(-box_radius, box_radius + 1):
                cx = robot_x_cell + dx
                cy = robot_y_cell + dy
                if 0 <= cx < Wid and 0 <= cy < hgt:
                    index = cy * Wid + cx
                    self.data[index] = 50  # Robot

        grid_msg = {
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
                        'x': -Wid * res / 2,
                        'y': -hgt * res / 2,
                        'z': 0.0
                    },
                    'orientation': {
                        'x': 0.0,
                        'y': 0.0,
                        'z': 0.0,
                        'w': 1.0
                    }
                }
            },
            'data': self.data
        }
        return grid_msg

    def update_map(self):
        self.map_topic.publish(roslibpy.Message(self.make_grid()))

if __name__ == '__main__':
    lidar = Lidar(ros, robot_name)
    lidar.subscribe()
    print("Connected to LIDAR, mapping data...")
    while ros.is_connected:
        time.sleep(2)
        lidar.update_map()

    # Cleanup
    lidar.unsubscribe()
    ros.terminate(); 
