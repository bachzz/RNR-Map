import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import cv2

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


class ExtractData(Node):
    def __init__(self):
        super().__init__('extract_data')
        
        self.bridge = CvBridge()
        
        self.odom_sub = self.create_subscription(Odometry, "/demo/odom", self.odom_sub_callback, 10)
        self.im_rgb_sub = self.create_subscription(Image, "/demo/camera_depth/image_raw", self.im_rgb_sub_callback, 10)
        self.im_depth_sub = self.create_subscription(Image, "/demo/camera_depth/depth/image_raw", self.im_depth_sub_callback, 10)

        self.data_out = {'position':[], 'rotation': [], 'rgb':[], 'depth': []}

    def odom_sub_callback(self, msg):
        print(f"[odom] ") #{msg}")
        # breakpoint()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        self.data_out['position'].append([x, y, z])
        self.data_out['rotation'].append([qw, qx, qy, qz])
    
    def im_rgb_sub_callback(self, msg):
        print(f"[im_rgb] ")
        im_rgb = self.bridge.imgmsg_to_cv2(msg)
        # im_rgb = cv2.resize(im_rgb, (128, 128))
        self.data_out['rgb'].append(im_rgb)
 
    
    def im_depth_sub_callback(self, msg):
        print(f"[im_depth] ") #{msg}")
        # breakpoint()
        im_depth = self.bridge.imgmsg_to_cv2(msg)
        im_depth[np.isinf(im_depth)] = 0
        # im_depth = cv2.resize(im_depth, (128, 128))
        im_depth = im_depth / im_depth.max()
        im_depth = im_depth[:, :, np.newaxis]
        
        self.data_out['depth'].append(im_depth)
 
    
    def shutdown_callback(self):
        print('[*] shutting down ...')
        self.data_out['position'] = np.array(self.data_out['position'])
        self.data_out['rotation'] = np.array(self.data_out['rotation'])
        self.data_out['rgb'] = np.array(self.data_out['rgb'])
        self.data_out['depth'] = np.array(self.data_out['depth'])
        np.save('data_out.npy', self.data_out)
        

def main(args=None):
    rclpy.init(args=args)
    node = ExtractData()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_callback()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
