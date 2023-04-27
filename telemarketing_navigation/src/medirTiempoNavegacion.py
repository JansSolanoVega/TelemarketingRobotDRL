#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseActionResult
import time
import rospy

tiempo_inicio_navegacion=0

def callback1(mensaje_goal):
    global tiempo_inicio_navegacion
    tiempo_inicio_navegacion=time.time()
    #print(mensaje_goal)
    print(tiempo_inicio_navegacion)

def callback2(mensaje_llegada):
    tiempo_navegacion=time.time()-tiempo_inicio_navegacion
    print(tiempo_navegacion)
    #print(mensaje_llegada)

rospy.init_node('suscriptor')
rospy.Subscriber("move_base/goal", MoveBaseActionGoal, callback1)
rospy.Subscriber("move_base/result", MoveBaseActionResult, callback2)
rospy.spin()

  