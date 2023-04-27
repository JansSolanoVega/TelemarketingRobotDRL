#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import message_filters
import math 
distancia=list()
entro_zona_colision=list()
for i in range(1000):
    distancia.append(0)
    entro_zona_colision.append(0)
cantidad_colisiones=0
def callback(odometria, model_states):
    global entro_zona_colision
    global cantidad_colisiones
    threshold_value=0.623
    pos_robot_x=odometria.pose.pose.position.x
    pos_robot_y=odometria.pose.pose.position.y
    for i in range(len(model_states.name)):
        sentence="Modelo: "
        nombre=model_states.name[i]
        sentence+=nombre
        pos_modelo_x=round(model_states.pose[i].position.x,2)
        pos_modelo_y=round(model_states.pose[i].position.y,2)
        pos_modelo_z=round(model_states.pose[i].position.z,2)
        #sentence+=", Posicion: ("+str(pos_modelo_x)+","+str(pos_modelo_y)+","+str(pos_modelo_z)+")"

        distancia=math.sqrt((pos_robot_x-pos_modelo_x)**2+(pos_robot_y-pos_modelo_y)**2)
        sentence+=", Distancia: "+str(round(distancia,2))
        #print(sentence)
        if nombre!="telemarketing" and nombre!="ground_plane":
            #if nombre=="actor7":
                #print(entro_zona_colision)
            if distancia<threshold_value and entro_zona_colision[i]==0:
                entro_zona_colision[i]=1
                cantidad_colisiones+=1
                print("El actor acaba de colisionar con el objeto: "+nombre)
                print(pos_modelo_x,pos_modelo_y)
            if distancia>threshold_value:
                entro_zona_colision[i]=0
    
#Para hacer el message_filters ambos topicos deben tener un mensaje para ambos al mismo tiempo

odom_sub = message_filters.Subscriber('odom', Odometry)
model_sub = message_filters.Subscriber('gazebo/model_states', ModelStates,)

rospy.init_node('topic_subscriber')
ts = message_filters.ApproximateTimeSynchronizer([odom_sub, model_sub], 10,0.1,allow_headerless=True)
ts.registerCallback(callback)
rospy.spin()