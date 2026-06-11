#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

class SpooferNode(Node):
    def __init__(self):
        super().__init__('pc_spoofer')
        # Listen to the Unity bag
        self.sub = self.create_subscription(PointCloud2, '/lidar/points', self.callback, 10)
        # Publish the fixed data to a new topic for Super-LIO
        self.pub = self.create_publisher(PointCloud2, '/lidar/points_fixed', 10)
        self.get_logger().info('Spoofer active: Translating Unity points with 0.1s sweeping time...')

    def callback(self, msg):
        # Extract the basic X, Y, Z from Unity
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points: return

        new_points = []
        num_points = len(points)
        
        # Put the 0.1 second sweep math back in!
        # This perfectly mimics the physical rotation of a LiDAR scanner
        time_step = 0.1 / num_points 
        
        for i, p in enumerate(points):
            # Inject X, Y, Z, Fake Intensity (50.0), and the Sweeping Time offset
            new_points.append([p[0], p[1], p[2], 50.0, float(i * time_step)])

        # Define the exact hardware memory layout Super-LIO expects
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
        ]

        # Repackage and send!
        new_msg = pc2.create_cloud(msg.header, fields, new_points)
        self.pub.publish(new_msg)

def main():
    rclpy.init()
    node = SpooferNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
