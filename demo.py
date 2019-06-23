import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import argparse


class Radar:
    def __init__(self, scan_topic='/mmWaveDataHdl/RScan'):
        rospy.init_node('listener')
        self.scan_topic = scan_topic
        rospy.Subscriber(self.scan_topic, PointCloud2, self.scan)

        self.x = []
        self.y = []
        self.z = []
        self.intensity = []

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
        self.x, self.y, self.z, self.intensity = x, y, z, intensity

    def run(self, hold=True):
        fig = plt.figure(figsize=(20,20))
        ax = plt.axes(projection='3d')
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            if not hold:
                ax.clear()
            ax.set_xlim3d(0, 2.5)
            ax.set_xlabel('x')
            ax.set_ylim3d(-5, 5)
            ax.set_ylabel('y')
            ax.set_zlim3d(-5, 5)
            # ax.set_zlabel('z')
            ax.scatter(self.x, self.y, self.z)
            fig.canvas.draw()
            plt.pause(0.1)
            # r.sleep()


if __name__ == "__main__":
    parse = argparse.ArgumentParser(description="mmWave Point Cloud Visualizer",
                                    usage='use "%(prog)s --help" for more information',
                                    formatter_class=argparse.RawTextHelpFormatter)
    # parse.add_argument('File', help="max running time")
    parse.add_argument('--hold', type=str, default='False', help='Hold plot or not')
    args = parse.parse_args()
    mmwave = Radar()
    mmwave.run(hold=False)