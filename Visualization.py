import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import argparse
import struct
import os.path


class PointCloud:
    def __init__(self, data_file_name):
        self.data_file_name = data_file_name
        self.x = []
        self.y = []
        self.z = []
        self.intensity = []
        self.data = pd.read_csv(data_file_name, header=None, sep=' ', keep_default_na=True)[11:-1].values

        for line in self.data:
            self.x.append(eval(line[0]))
            self.y.append(eval(line[1]))
            self.z.append(eval(line[2]))
            self.intensity.append(eval(line[3]))

    def plot(self):
        fig = plt.figure()
        if np.sum(self.z) == 0:
            plt.plot(self.x, self.y)
        else:
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(self.x, self.y, self.z)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
        ax.set_title(self.data_file_name)


class Depth:
    def __init__(self, data_file_name):
        self.raw_data = pd.read_csv(data_file_name)
        self.data = self.raw_data.transpose()[9:].values
        self.img = []
        self.height = self.raw_data['field.height'][0]
        self.width = self.raw_data['field.width'][0]

    def plot(self):
        for idx in range(0, len(self.data), 2):
            float32_str = '0b' + np.binary_repr(self.data[idx][0], width=8) + np.binary_repr(self.data[idx + 1][0], width=8)
            self.img.append(struct.unpack('I', struct.pack('I', int(float32_str, 2)))[0])
        self.img = np.array(self.img)
        self.img = self.img.reshape(self.height, self.width)
        plt.imshow(self.img, cmap='gray')


if __name__ == "__main__":
    parse = argparse.ArgumentParser(description="mmWave Point Cloud Visualizer",
                                    usage='use "%(prog)s --help" for more information',
                                    formatter_class=argparse.RawTextHelpFormatter)
    parse.add_argument('File', help="max running time")
    parse.add_argument('--option', type=str, default='radar', help='the type of data is visualized(depth/radar)')
    args = args = parse.parse_args()

    # my_path = os.path.abspath(os.path.dirname(__file__))
    # data = PointCloud(my_path + '/' + args.File)

    if args.option == 'radar':
        data = PointCloud(args.File)
    elif args.option == 'depth':
        data = Depth(args.File)
    data.plot()
    plt.show(block=True)
