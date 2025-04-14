import roslibpy
import time
import math
import matplotlib.pyplot as plt
from random import randint
print("libraries imported")

class RosMapPublisher:
    def __init__(self, ip, port, robot_name, topic_name, width=100, height=50):
        self.ip = ip
        self.port = port
        self.robot_name = robot_name
        self.topic_name = topic_name
        self.width = width
        self.height = height

        self.latest_ranges = []
        self.latest_angs = []
        self.latest_yaw = 0.0
        self.latest_pos = (0.0, 0.0)

        self.ros = roslibpy.Ros(host=self.ip, port=self.port)
        self.map_topic = None
        self.odom_topic = None
        self.scan_topic = None

    def connect(self):
        self.ros.run()
        print(f"Connected to ROS: {self.ros.is_connected}")
        self.map_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/{self.topic_name}', 'nav_msgs/OccupancyGrid')
        self.odom_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/odom', 'nav_msgs/Odometry')
        self.scan_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/scan', 'sensor_msgs/LaserScan')

    def subscribe_topics(self):
        self.odom_topic.subscribe(self.callback_odom)
        print("Subscribed to odom topic")
        self.scan_topic.subscribe(self.callback_scan)
        print("Subscribed to scan topic")

    def callback_odom(self, message):
        pos = message['pose']['pose']['position']
        orientation = message['pose']['pose']['orientation']

        x = pos['x']
        y = pos['y']

        qx = orientation['x']
        qy = orientation['y']
        qz = orientation['z']
        qw = orientation['w']
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        print(f"Position: {x}, {y}, Yaw: {yaw}")
        self.latest_yaw = yaw
        self.latest_pos = (x, y)

    def callback_scan(self, message):
        angle_increment = message['angle_increment']
        angle_min = message['angle_min']
        self.latest_ranges = message['ranges']
        self.latest_angs = [angle_min + i * angle_increment for i in range(len(self.latest_ranges))]

    def generate_map_data(self):
        return [randint(-1, 100) for _ in range(self.width * self.height)]

    def make_grid_msg(self):
        grid_data = self.generate_map_data()
        print(f"Map data generated: {grid_data[:10]}...")

        msg = {
            "header": {
                "frame_id": f"{self.robot_name}/{self.topic_name}",
                "stamp": {
                    "secs": int(time.time()),
                    "nsecs": 0
                }
            },
            "info": {
                "map_load_time": {
                    "secs": int(time.time()),
                    "nsecs": 0
                },
                "width": self.width,
                "height": self.height,
                "resolution": 0.1,
                "origin": {
                    "position": {"x": 0.0, "y": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            },
            "data": grid_data
        }
        self.map_topic.publish(msg)
        print("Publishing map data")

    def plot_scan(self):
        x, y = self.latest_pos
        #print(f"Latest position: {x}, {y}")
        yaw = self.latest_yaw
        #print(f"Latest yaw: {self.latest_yaw}")
        ranges = self.latest_ranges
        angs = self.latest_angs
        print(f"Latest ranges: {ranges[:10]}...")
        print(f"Latest angles: {angs[:10]}...")
        xs = [x + r * math.cos(yaw + a) for r, a in zip(ranges, angs) if r > 0.1]
        ys = [y + r * math.sin(yaw + a) for r, a in zip(ranges, angs) if r > 0.1]

        print(f"X coordinates: {xs}")
        print(f"Y coordinates: {ys}")

        #plt.figure()
        #plt.plot(xs, ys, 'r.')
        #plt.title('Laser Scan Data')
        #plt.xlabel('X (m)')
        #plt.ylabel('Y (m)')
        #plt.axis('equal')
        #plt.grid(True)
        #plt.show()

    def publish_loop(self):
        t = 0
        while self.ros.is_connected and t < 50:
            try:
                print("Waiting for data...")
                time.sleep(3)

                self.plot_scan()
                print("Plotting complete")
                self.make_grid_msg()
                print("Grid message created")

            except KeyboardInterrupt:
                print("Interrupted. Closing connection.")
                self.cleanup()
            t += 1
            print(f"Loop iteration: {t}")
    def cleanup(self):
        self.map_topic.unadvertise()
        #self.odom_topic.unsubscribe(self.callback_odom(message=None))
        #self.scan_topic.unsubscribe(self.callback_scan(message=None))
        print("Unsubscribed from topics")
        self.ros.terminate()

if __name__ == "__main__":
    node = RosMapPublisher(
        ip='192.168.8.104',
        port=9012,
        robot_name='juliet',
        topic_name='Hernandez'
    )
    node.connect()

    node.subscribe_topics()

    node.publish_loop()
    node.cleanup()