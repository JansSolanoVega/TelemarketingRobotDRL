#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseActionResult
from move_base_msgs.msg import MoveBaseActionGoal
import time
import numpy as np
from gazebo_msgs.msg import ModelStates
import message_filters
import math 

distancia=list()
entro_zona_colision=list()
entro_zona_ego_score=list()
tiempos_entrada_ego_score=list()

tiempo_inicio_navegacion=0
for i in range(1000):
    distancia.append(0)
    entro_zona_colision.append(0)
    entro_zona_ego_score.append(0)
    tiempos_entrada_ego_score.append(0)
cantidad_colisiones=0
tiempo_ego_score=0

##COLISIONES Y EGO SCORE
def callback(odometria, model_states):
    global entro_zona_colision
    global entro_zona_ego_score
    global tiempos_entrada_ego_score
    global cantidad_colisiones
    global tiempo_ego_score
    threshold_value_colision=0.623
    r_robot=0.670/2
    threshold_value_ego_score=threshold_value_colision+r_robot
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
            if distancia<threshold_value_colision and entro_zona_colision[i]==0:
                entro_zona_colision[i]=1
                cantidad_colisiones+=1
                print("El actor acaba de colisionar con el objeto: "+nombre)
            if distancia>threshold_value_colision:
                entro_zona_colision[i]=0

            if distancia<threshold_value_ego_score and entro_zona_ego_score[i]==0:
                entro_zona_ego_score[i]=1
                tiempos_entrada_ego_score[i]=time.time()
                print("El robot acaba de entrar al rango peligroso del objeto: "+nombre)
            if distancia>threshold_value_ego_score and entro_zona_ego_score[i]==1:
                entro_zona_ego_score[i]=0
                tiempo_dentro_ego_score=time.time()-tiempos_entrada_ego_score[i]
                print("El robot acaba de salir del rango peligroso del objeto: "+nombre)
                print("EStuvo dentro: ",tiempo_dentro_ego_score)
                tiempo_ego_score+=tiempo_dentro_ego_score

##INICIO                
def callback1(mensaje_goal):
    global tiempo_inicio_navegacion,cantidad_colisiones
    tiempo_inicio_navegacion=time.time()
    cantidad_colisiones=0

##LLEGADA 
def callback2(mensaje_llegada):
    tiempo_navegacion=time.time()-tiempo_inicio_navegacion
    print("Tiempo total de navegacion: "+str(tiempo_navegacion)+" segundos")
    print("Cantidad de colisiones: "+str(cantidad_colisiones))
    ego_score=tiempo_ego_score*100/tiempo_navegacion
    print("EgoScore: ",ego_score,"%")
    
rospy.init_node('suscriptor')

odom_sub = message_filters.Subscriber('odom', Odometry)
model_sub = message_filters.Subscriber('gazebo/model_states', ModelStates,)
ts = message_filters.ApproximateTimeSynchronizer([odom_sub, model_sub], 10,0.1,allow_headerless=True)
ts.registerCallback(callback)
rospy.Subscriber("move_base/goal", MoveBaseActionGoal, callback1)
rospy.Subscriber("move_base/result", MoveBaseActionResult, callback2)

rospy.spin()