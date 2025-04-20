#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String, Float32MultiArray

arr_msg = Float32MultiArray()

def callback(data):
    global prob_dist
    msg = data.data
    ch = msg[0].upper()
    steps = int(msg[1:])
    
    # get updated prob dist
    prob_dist = getProbDist(prob_dist, ch, steps)

    # define array msg
    arr_msg.data = prob_dist

    # publish prob dist
    pub.publish(arr_msg)
    print(arr_msg)
    return 0

def calMotionModel(lst,steps):
    for step in range(steps):
        newProbList = np.zeros(20)
        newProbList[0] = lst[0]
        newProbList[1] = 0.2*lst[1] + 0.8*lst[0]
        for i in range(2,20,1):
            newProbList[i] = 0.2*lst[i] + 0.8*lst[i-1] + 0.2*lst[i-2]
            
        lst = newProbList
        step += 1

    return newProbList*(1/np.sum(newProbList))

def calLeftMotionModel(lst,steps):
    for step in range(steps):
        newProbList = np.zeros(20)
        for i in range(0,18,1):
            newProbList[i] = 0.2*lst[i] + 0.8*lst[i+1] + 0.2*lst[i+2]
        newProbList[18] = 0.2*lst[18] + 0.8*lst[19]
        newProbList[19] = lst[19]
            
        lst = newProbList
        step += 1

    return newProbList*(1/np.sum(newProbList))

def calSensorModel(lst,stepNo):
    
    # geting probability list for given observation
    probList = np.zeros(20)
    
    if stepNo == 0:
        probList[0] = 0.4
        probList[1] = 0.2
        probList[2] = 0.1
    
    elif stepNo == 1:
        probList[0] = 0.2
        probList[1] = 0.4
        probList[2] = 0.2
        probList[3] = 0.1
    
    elif stepNo in range(2,18):
        probList[stepNo] = 0.4
        probList[stepNo-1],probList[stepNo+1] = 0.2,0.2
        probList[stepNo-2],probList[stepNo+2] = 0.1,0.1
    
    elif stepNo == 18:
        probList[16] = 0.1
        probList[17] = 0.2
        probList[18] = 0.4
        probList[19] = 0.2
    
    elif stepNo == 19:
        probList[17] = 0.1
        probList[18] = 0.2
        probList[19] = 0.4
        
    # getting prob dist list
    probDistList = np.zeros(20)
    
    for i in range(20):
        probDistList[i] = (probList[i]*lst[i])/np.dot(probList,lst)
        
    return probDistList
    

def getProbDist(lst,chr,num):

    if chr == "R":
        prob_dist = calMotionModel(lst,num)
       
    if chr == "L":
        prob_dist = calLeftMotionModel(lst,num)

    if chr == "X":
        prob_dist = calSensorModel(lst,num)       

    return prob_dist

def main():
    global pub, prob_dist
    rospy.init_node("pub_dist")
    pub = rospy.Publisher("/float_array", Float32MultiArray, queue_size=10)

   # initial prob dist
    prob_dist = np.full(20, 1/361)
    prob_dist[10] = 342/361

    rospy.Subscriber("/chatter", String, callback)
    rospy.sleep(1)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass