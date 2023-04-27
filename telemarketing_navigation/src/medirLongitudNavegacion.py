#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
import time
import rospy
import numpy as np

LongitudTotal=0
longitud_anterior=0
cont=0

def callback(path_msg):
    global longitud_anterior
    global LongitudTotal
    global cont
    path_length=0
    for i in range(len(path_msg.poses) - 1):
        position_a_x = path_msg.poses[i].pose.position.x
        position_b_x = path_msg.poses[i+1].pose.position.x
        position_a_y = path_msg.poses[i].pose.position.y
        position_b_y = path_msg.poses[i+1].pose.position.y

        path_length += np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y- position_a_y), 2))
    if(longitud_anterior==0):
        longitud_anterior=path_length
    LongitudTotal+=-path_length+longitud_anterior
    longitud_anterior=path_length
    if cont==0:
        print("Longitud total del camino global: ",path_length)
    if path_length<=0.2:
        print("Longitud total del camino local: ",LongitudTotal)
    cont+=1

rospy.init_node('suscriptor')
rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, callback)
rospy.spin()

  