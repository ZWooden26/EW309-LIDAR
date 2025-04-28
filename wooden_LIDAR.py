import roslibpy
import time
import math
import numpy as np
from collections import deque
import threading

# Connect to Ros
ip = '192.168.8.104'
port = 9012

robot_name = 'juliet'

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
        self.lock = threading.Lock()
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
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        #mapping parameters
        self.resolution = 0.1 #meter/cell
        self.width = 600
        self.height = 600
        self.data = [-1] * (self.width*self.height)
        #deque for storing only latest scans/odom
        self.buffer = 20
        self.scan_data = deque(maxlen=self.buffer)
        self.odom_data = deque(maxlen=self.buffer)
        self.history = deque(maxlen=10)

    # callback functions
    def callback_odom(self, message):
        stamp = message['header']['stamp'] # extract time stamp
        timestamp = stamp['sec'] + stamp['nanosec'] * 1e-9
        self.x = message.get('pose').get('pose').get('position').get('x')
        self.y = message.get('pose').get('pose').get('position').get('y')
        o = message.get('pose').get('pose').get('orientation')
        qx = o.get('x', 0)
        qy = o.get('y', 0)
        qz = o.get('z', 0)
        qw = o.get('w', 1)
        self.yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
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
        angles = [angle_min + i*angle_increment for i in range(len(ranges))]
        mask = np.array(ranges) < 0.1
        self.ranges = np.array(ranges)[~mask]
        self.angles = np.array(angles)[~mask]
        self.scan_data.append({
            'timestamp': timestamp,
            'ranges': self.ranges,
            'angles': self.angles
        })
        
    
    #functions
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
    
    def get_synch_data(self):
        synchronized = []   
        for odom in self.odom_data: 
            for scan in self.scan_data:
                if abs(odom['timestamp'] - scan['timestamp']) <= 0.05: 
                    synchronized.append((odom, scan))
                    
                else:
                    continue
        return synchronized

    
    def lidar_to_map(self):
         # Wait for synchronized data
        while len(self.get_synch_data()) == 0:
            time.sleep(0.05)  # Small sleep to prevent busy-waiting
        with self.lock:
            synchronized = self.get_synch_data()
            idx = len(synchronized) - 1
            odom = synchronized[idx][0]  # Get the latest odom data
            scan = synchronized[idx][1]  # Get the latest scan data
            
            if self.initial_x is None:
                self.initial_x = odom['position']['x']
                self.initial_y = odom['position']['y']
                self.initial_yaw = odom['yaw']
            
            x = odom['position']['x'] - self.initial_x
            y = odom['position']['y'] - self.initial_y
            yaw = odom['yaw'] - self.initial_yaw
            ranges = scan['ranges']
            angles = scan['angles']

            # Map center
            self.middle = [self.width // 2, self.height // 2]
            #robot position
            robot_x = int((x / self.resolution) + self.middle[0])
            robot_y = int((y / self.resolution) + self.middle[1])

            #for r, a in zip(ranges, angles):
            #    if r > 0:
            #        end_x = x + r * math.cos(yaw + a)
            #        end_y = y + r * math.sin(yaw + a)
            #        steps = int(r/self.resolution)
            #        for i in range(steps):
            #            intermediate_x = x + (i * self.resolution * math.cos(a + yaw))
            #            intermediate_y = y + (i * self.resolution * math.sin(a + yaw))
            #            x_cell = int(robot_x + intermediate_x / self.resolution)
            #            y_cell = int(robot_y + intermediate_y / self.resolution)                        

            #            if 0 <= x_cell < self.width and 0 <= y_cell < self.height:
            #                index = y_cell * self.width + x_cell
            #                self.data[index] = 0  # Free space

            #        x_hit = int(robot_x + (end_x / self.resolution))
            #        y_hit = int(robot_y + (end_y / self.resolution))
            #        if 0 <= x_hit < self.width and 0 <= y_hit < self.height:
            #            index = y_hit * self.width + x_hit
            #            self.data[index] = 100  # Obstacle
            
            ###
            xs = [x + r*math.cos(yaw + a) for r,a in zip(ranges, angles)]
            ys = [y + r*math.sin(yaw + a) for r,a in zip(ranges, angles)]

            steps = [int(r/self.resolution) for r in ranges]
            steps_idx = [(i) for i in range(steps)]
            path_x = [x + (i * self.resolution * math.cos(a + yaw)) for i,a in zip(steps_idx, angles)] #free space vectors
            path_y = [y + (i * self.resolution * math.sin(a + yaw)) for i,a in zip(steps_idx, angles)]


            x_idx = [int(x / self.resolution + self.middle[0]) for x in xs] #Obstacle hitboxes
            y_idx = [int(y / self.resolution + self.middle[1]) for y in ys]
            points = set(zip(x_idx, y_idx))
            
            for y in range(self.height):
                for x in range(self.width):
                    index = y * self.width + x
                    if (x, y) in points:
                        self.data[index] = 100 #Obstacle
                    elif x in path_x and y in path_y:
                        self.data[index] = 0 #Free Space
                    else:
                        continue

            box_radius = 1
            for dx in range(-box_radius, box_radius + 1):
                for dy in range(-box_radius, box_radius + 1):
                    cx = robot_x + dx
                    cy = robot_y + dy
                    if 0 <= cx < self.width and 0 <= cy < self.height:
                        index = cy * self.width + cx
                        self.data[index] = 50  # Robot position

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
    
    def update_map(self):
        self.map_topic.publish(roslibpy.Message(self.make_grid()))

if __name__ == '__main__':
    lidar = Lidar(ros, robot_name)
    lidar.subscribe()
    print("Connected to LIDAR, mapping data...")
    while ros.is_connected:
        time.sleep(5)
        lidar.update_map()
        print('Sent map data')

    # cleanup
    lidar.unsubscribe()
    ros.terminate()