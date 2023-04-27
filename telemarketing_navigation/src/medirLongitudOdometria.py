#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult

import time
import rospy
import numpy as np

LongitudTotal=0
position_x_anterior=0
position_y_anterior=0

def callback(odometry_msg):
    global LongitudTotal
    global position_x_anterior
    global position_y_anterior
    
    position_x = odometry_msg.pose.pose.position.x
    position_y = odometry_msg.pose.pose.position.y

    if(position_x_anterior==0 and position_y_anterior==0):
        position_x_anterior=position_x
        position_y_anterior=position_y
    LongitudTotal += np.sqrt(np.power((position_x - position_x_anterior), 2) + np.power((position_y- position_y_anterior), 2))

    position_x_anterior = position_x
    position_y_anterior = position_y

def callback2(mensaje_llegada):
    print(LongitudTotal)

rospy.init_node('suscriptor')
rospy.Subscriber("/odom", Odometry, callback)
rospy.Subscriber("move_base/result", MoveBaseActionResult, callback2)
rospy.spin()



