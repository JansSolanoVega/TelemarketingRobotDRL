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
import pandas as pd 

filename="mpc_0.2_4"
navegador="mpc"

##INICIALIZACION
distancia=list()
entro_zona_colision=list()
entro_zona_ego_score=list()
tiempos_entrada_ego_score=list()
tiempo_inicio_navegacion=0

##CREACION DE DATAFRAME PARA EL CSV
lista=[]
cantidad_navegaciones_exportar_csv=11
current_navegaciones=0

diccionario={"teb":"/move_base/TebLocalPlannerROS/global_plan","dwa":"/move_base/DWAPlannerROS/global_plan","mpc":"/move_base/MpcLocalPlannerROS/global_plan"}
for i in range(1000):
    distancia.append(0)
    entro_zona_colision.append(0)
    entro_zona_ego_score.append(0)
    tiempos_entrada_ego_score.append(0)
cantidad_colisiones=0
tiempo_ego_score=0

LongitudTotal=0
position_x_anterior=0
position_y_anterior=0
tiempo_inicio_navegacion=0
LongitudTotalGlobalPlanner=0
cont=0

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
    global LongitudTotal,cont,tiempo_ego_score
    tiempo_inicio_navegacion=time.time()
    cantidad_colisiones=0
    LongitudTotal=0
    cont=0
    tiempo_ego_score=0
##LLEGADA 
def callback2(mensaje_llegada):
    global lista,current_navegaciones
    tiempo_navegacion=time.time()-tiempo_inicio_navegacion
    print("Tiempo total de navegacion: "+str(tiempo_navegacion)+" segundos")
    print("Cantidad de colisiones: "+str(cantidad_colisiones))
    ego_score=tiempo_ego_score*100/tiempo_navegacion
    print("EgoScore: ",ego_score,"%")
    print("Recorrido total de navegacion: "+str(LongitudTotal)+" metros")
    print("Recorrido total de global planner: "+str(LongitudTotalGlobalPlanner)+" metros")
    current_navegaciones+=1
    if(current_navegaciones>1):
        lista.append({
            'Tiempo_total': tiempo_navegacion,
            'Cantidad_Colisiones': cantidad_colisiones,
            'EGO_SCORE': ego_score,
            'Recorrido_total': LongitudTotal,
            'Recorrido global path': LongitudTotalGlobalPlanner
        })
    print(lista)
    if(current_navegaciones==(cantidad_navegaciones_exportar_csv)):
        df = pd.DataFrame(lista)
        print(df)
        df.to_csv('/home/zetans/ros_ws/src/Telemarketing_Robot/telemarketing_navigation/src/resultados/'+filename+'.csv')
##RECORRIDO TOTAL
def callback3(odometry_msg):
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

##RECORRIDO GLOBAL PLANNER
def callback4(path_msg):
    global cont
    global LongitudTotalGlobalPlanner
    path_length=0
    for i in range(len(path_msg.poses) - 1):
        position_a_x = path_msg.poses[i].pose.position.x
        position_b_x = path_msg.poses[i+1].pose.position.x
        position_a_y = path_msg.poses[i].pose.position.y
        position_b_y = path_msg.poses[i+1].pose.position.y

        path_length += np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y- position_a_y), 2))
    if cont==0:
        LongitudTotalGlobalPlanner=path_length
    cont+=1

rospy.init_node('suscriptor')

odom_sub = message_filters.Subscriber('odom', Odometry)
model_sub = message_filters.Subscriber('gazebo/model_states', ModelStates,)
ts = message_filters.ApproximateTimeSynchronizer([odom_sub, model_sub], 10,0.1,allow_headerless=True)
ts.registerCallback(callback)

rospy.Subscriber("move_base/goal", MoveBaseActionGoal, callback1)
rospy.Subscriber("move_base/result", MoveBaseActionResult, callback2)
rospy.Subscriber("/odom", Odometry, callback3)
rospy.Subscriber(diccionario[navegador], Path, callback4)

rospy.spin()