#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray

def plot_dist(msg):

    dist = msg.data 

    # X values
    x = list(range(len(dist)))

    # plotting
    plt.bar(x, dist, color='skyblue', edgecolor='black')
    plt.xlabel('x')
    plt.ylabel('Probability p(x)')
    plt.title('Histogram of p(x)')
    plt.xticks(x)  # show all x labels
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    
    plt.show()

    return 0


def main():

    rospy.init_node("vis")

    rospy.Subscriber("/float_array", Float32MultiArray, plot_dist)

    rospy.spin()

if __name__ == '__main__':
    main()