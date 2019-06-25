#!/usr/bin/python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from time import time
from cv_bridge import CvBridge
import argparse
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
from ncappzoo.apps.live_object_detector import live_object_detector


class Radar:
    def __init__(self, scan_topic='/mmWaveDataHdl/RScan', depth_topic='/camera/depth/image'):
        rospy.init_node('listener')
        self.scan_topic = scan_topic
        self.depth_topic = depth_topic
        rospy.Subscriber(self.scan_topic, PointCloud2, self.scan)
        rospy.Subscriber(self.depth_topic, Image, self.imaging)
        self.x = []
        self.y = []
        self.z = []
        self.intensity = []
        self.image = []
        self.bridge = CvBridge()

    def imaging(self, depth):
        image = self.bridge.imgmsg_to_cv2(depth)
        cv2.imshow("Imshow", image)
        cv2.waitKey(3)
        self.image = np.nan_to_num(image)

    def scan(self, cloud):
        xyz_generator = pc2.read_points_list(cloud, skip_nans=True, field_names=("x", "y", "z", "intensity"))
        x = []
        y = []
        z = []
        intensity = []
        for xyz in xyz_generator:
            x.append(xyz.x)
            y.append(xyz.y)
            z.append(xyz.z)
            intensity.append(xyz.intensity)
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity

    def run(self, hold, hold_time, sampling):
        fig = plt.figure(1, figsize=(10, 10))
        ax = plt.axes(projection='3d')
        r = rospy.Rate(100)
        time_base = time()
        ax.view_init(-90, 0)
        data = [[], [], [], []]
        live_object_detector.dec()
        while not rospy.is_shutdown():
            if (not hold) or (time()-time_base > hold_time):
                ax.clear()
                time_base = time()
                if sampling:
                    np.save('radar_xyzi', np.nan_to_num(np.array(data)))
                    np.save('depth', np.array(self.image))
                    cv2.destroyAllWindows()
                    print('Finish Sampling')
                    break
            ax.set_xlim3d(0, 2.5)
            ax.set_xlabel('x')
            ax.set_ylim3d(-5, 5)
            ax.set_ylabel('y')
            ax.set_zlim3d(-5, 5)
            ax.scatter(self.x, self.y, self.z, s=3, marker="o")
            data[0] += self.x
            data[1] += self.y
            data[2] += self.z
            data[3] += self.intensity
            fig.canvas.draw()
            plt.pause(0.01)



if __name__ == "__main__":
    parse = argparse.ArgumentParser(description="mmWave Point Cloud Visualizer",
                                    usage='use "%(prog)s --help" for more information',
                                    formatter_class=argparse.RawTextHelpFormatter)
    parse.add_argument('--refresh', type=float, default=2.0, help="max running time")
    parse.add_argument('--hold', type=bool, default=False, help='Hold plot or not')
    parse.add_argument('--sampling', type=bool, default=False, help='Sampling')
    args = parse.parse_args()
    mmwave = Radar()
    mmwave.run(hold=args.hold, hold_time=args.refresh, sampling=args.sampling)
    print("ROS Stopped")
    # live_object_detector.
    rospy.spin()