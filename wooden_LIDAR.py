import roslibpy
import time
import math
import numpy as np

# Connect to Ros
ip = '192.168.8.104'
port = 9012

robot_name = 'india'

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
    def __init__ (self, ros, robot_name='juliet'):
        self.robot_name = robot_name
        self.ros = ros
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
        self.height = 200
        self.data = [-1] * (self.width*self.height)

    # callback functions
    def callback_odom(self, message):
        self.x = message.get('pose').get('pose').get('position').get('x')
        self.y = message.get('pose').get('pose').get('position').get('y')
        o = message.get('pose').get('pose').get('orientation')
        qx = o.get('x', 0)
        qy = o.get('y', 0)
        qz = o.get('z', 0)
        qw = o.get('w', 1)
        self.yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

    def callback_scan(self, message):
        angle_min = message.get('angle_min', 0)
        angle_max = message.get('angle_max', 0)
        angle_increment = message.get('angle_increment', 0)
        ranges = message.get('ranges', [])
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

#    def lidar_to_map(self):
#        xs = [self.x + r*math.cos(self.yaw + a) for r,a in zip(self.ranges, self.angles)]
#        ys = [self.y + r*math.sin(self.yaw + a) for r,a in zip(self.ranges, self.angles)]
#        print(xs)
#        middle = [self.width/2, self.height/2]
#        x_idx = [int(xs[i]/self.resolution + middle[0]) for i in range(len(xs))]
#        y_idx = [int(ys[i]/self.resolution + middle[1]) for i in range(len(ys))]

#        for y in range(self.height):
#            for x in range(self.width):
#                index = int(y*(self.width+1) + x)
#                print(index)
#                if x in x_idx and y in y_idx:
#                    self.data[index] = 100
#                else:
#                    self.data[index] = -1

    def lidar_to_map(self):
        xs = [self.x + r*math.cos(self.yaw + a) for r,a in zip(self.ranges, self.angles)]
        ys = [self.y + r*math.sin(self.yaw + a) for r,a in zip(self.ranges, self.angles)]
        ts = [math.atan2(y, x) for x, y in zip(xs, ys)]
        self.middle = [self.width/2, self.height/2]
        x_idx = [int(x / self.resolution + self.middle[0]) for x in xs]
        y_idx = [int(y / self.resolution + self.middle[1]) for y in ys]

        points = set(zip(x_idx, y_idx))
        for y in range(self.height):
            for x in range(self.width):
                index = y * self.width + x
                if x == self.middle[0] and y == self.middle[1]:
                    self.data[index] = 50
                elif (x, y) in points:
                    self.data[index] = 100
                else:
                    self.data[index] = -1

    def make_grid(self):
        self.lidar_to_map()
        grid_msg = {'header': {'frame_id': f'{self.robot_name}/odom'},
                    'info': {'map_load_time': {'secs': int(time.time()), 'nsecs': 0},
                            'resolution': self.resolution,
                            'width': self.width,
                            'height': self.height,
                            'origin': {'position': {'x': 0.0, 'y': 0.0, 'z':0.0}, 'orientation': {'x':0.0, 'y':0.0, 'z': 0.0 , 'w':-1.0}}
                            },
                    'data': self.data}
        return grid_msg
    
    def publish_map(self):
        self.map_topic.publish(roslibpy.Message(self.make_grid()))       

if __name__ == '__main__':
    lidar = Lidar(ros, robot_name)
    lidar.subscribe()
    print("Connected to LIDAR, mapping data...")
    while ros.is_connected:
        time.sleep(1)
        lidar.publish_map()

    # cleanup
    lidar.unsubscribe()
    ros.terminate()