import roslibpy
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import random

# Connect to Ros
ip = '192.168.8.104'
port = 9012

robot_name = 'juliet'

ros = roslibpy.Ros(host=ip, port=port)
ros.run()

class Lidar:
    def __init__ (self):
        self.robot_name = robot_name
        #create topics
        self.odom_topic = roslibpy.Topic(ros, f'/{robot_name}/odom', 'nav_msgs/Odometry')
        self.lidar_topic = roslibpy.Topic(ros, f'/{robot_name}/scan', 'sensor_msgs/LaserScan')
        self.map_topic = roslibpy.Topic(ros, f'/{robot_name}/map_wooden', 'nav_msgs/OccupancyGrid')
        #odom parameters
        self.ranges = []
        self.angles = []
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        #mapping parameters
        self.resolution = 0.1 #meter/cell
        self.width = 100
        self.height = 50
        self.data = []

    # callback functions
    def callback_odom(self, message):
        self.x = message.get('pose').get('pose').get('position').get('x')
        self.y = message.get('pose').get('pose').get('position').get('y')
        o = message.get('pose').get('pose').get('orientation')
        qx = o.get('x')
        qy = o.get('y')
        qz = o.get('z')
        qw = o.get('w')
        self.yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
        #print(f"x: {x}, y: {y}, yaw: {yaw}")

    def callback_scan(self, message):
        angle_min = message.get('angle_min')
        angle_max = message.get('angle_max')
        angle_increment = message.get('angle_increment')
        ranges = message.get('ranges')
        angles = [angle_min + i*angle_increment for i in range(len(ranges))]
        mask = np.array(ranges) < 0.1
        self.ranges = np.array(ranges)[~mask]
        self.angles = np.array(angles)[~mask]
    
    #functions
    def subscribe(self):
        self.odom_topic.subscribe(self.callback_odom)
        self.lidar_topic.subscribe(self.callback_scan)
    
    def unsubscribe(self):
        self.lidar_topic.unsubscribe()
        self.odom_topic.unsubscribe()

    def lidar_to_map(self):
        xs = [self.x + r*math.cos(self.yaw + a) for r,a in zip(self.ranges, self.angles)]
        ys = [self.y + r*math.sin(self.yaw + a) for r,a in zip(self.ranges, self.angles)]
        middle = [self.width/2, self.height/2]
        x_idx = [int(xs[i]/self.resolution + middle[0]) for i in range(len(xs))]
        y_idx = [int(ys[i]/self.resolution + middle[1]) for i in range(len(ys))]

        for y in range(self.height):
            for x in range(self.width):
                index = int(y*(self.width+1) + x)
                print(index)
                if x in x_idx and y in y_idx:
                    self.data[index] = 100
                else:
                    self.data[index] = -1

    def make_grid(self):
        self.lidar_to_map()
        grid_msg = {'header': {'frame_id': f'{self.robot_name}/Odometry'},
                    'info': {'map_load_time': {'secs': int(time.time()), 'nsecs': 0},
                            'resolution': self.resolution,
                            'width': self.width,
                            'height': self.height,
                            'origin': {'position': {'x': 0.0, 'y': 0.0, 'z':0.0}, 'orientation': {'x':0.0, 'y':0.0, 'z': 0.0 , 'w':1.0}}
                            },
                    'data': self.data}
        return grid_msg
    
    def publish_map(self):
        self.map_topic.publish(roslibpy.Message(self.make_grid()))
        print("Publishing Map")

if __name__ == '__main__':
    Lidar = Lidar()
    Lidar.subscribe()
    while ros.is_connected:
        Lidar.publish_map()
        time.sleep(1)

    # cleanup
    Lidar.unsubscribe()
    ros.terminate()