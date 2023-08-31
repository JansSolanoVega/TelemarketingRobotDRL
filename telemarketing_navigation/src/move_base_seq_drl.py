#!/usr/bin/env python
# license removed for brevity
__author__ = 'fiorellasibona'
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionResult

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
import numpy as np
import time

class MoveBaseSeq():

    def __init__(self):
        rospy.init_node('move_base_sequence')
        self.llego_a_la_meta=0


        numero_intentos=int(rospy.get_param('move_base_seq/numero_intentos'))
        point_inicial=rospy.get_param('move_base_seq/p_inicial')
        point_final=rospy.get_param('move_base_seq/p_final')
        points_seq=list(point_inicial)
        

        angulos=rospy.get_param('move_base_seq/angle_sequence')
        yaweulerangles_seq=[angulos[0]]

        for i in range(numero_intentos):
            if i%2==0:
                points_seq.extend(point_final)
                yaweulerangles_seq.extend(angulos)
            else:
                points_seq.extend(point_inicial)

        self.tiempo_transcurrido=0
        self.tiempo_anterior=time.time()
        quat_seq = list()
        self.pose_seq = list()
        self.goal_cnt = 0
        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()
        
    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
        #print("Posicion x"+str(self.pose_seq[self.goal_cnt].position.x))
        a=1
    def quaternion_to_euler(self,x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)

        return X, Y, Z
    def nueva_meta_llegada(self,odometry_msg):
        position_x = odometry_msg.pose.pose.position.x
        position_y = odometry_msg.pose.pose.position.y

        cuat_x=odometry_msg.pose.pose.orientation.x
        cuat_y=odometry_msg.pose.pose.orientation.y
        cuat_z=odometry_msg.pose.pose.orientation.z
        cuat_w=odometry_msg.pose.pose.orientation.w
        roll,pitch,yaw=self.quaternion_to_euler(cuat_x,cuat_y,cuat_z,cuat_w)

        cuat_x_deseado=self.pose_seq[self.goal_cnt].orientation.x
        cuat_y_deseado=self.pose_seq[self.goal_cnt].orientation.y
        cuat_z_deseado=self.pose_seq[self.goal_cnt].orientation.z
        cuat_w_deseado=self.pose_seq[self.goal_cnt].orientation.w
        roll,pitch,angulo_deseado=self.quaternion_to_euler(cuat_x_deseado,cuat_y_deseado,cuat_z_deseado,cuat_w_deseado)

        distancia_referencia=0.3
        distancia=np.sqrt(np.power((position_x - self.pose_seq[self.goal_cnt].position.x), 2) + np.power((position_y- self.pose_seq[self.goal_cnt].position.y), 2))
        
        error_angulo_deseado=0.3
        error_angulo=abs(yaw-angulo_deseado)
        if(distancia<distancia_referencia):
            self.goal_cnt += 1
            print("llegueeee")
            #Accion de llegada
            llegada=MoveBaseActionResult()
            llegada.status.text="ksjakdjas"
            self.pub.publish(llegada)
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
        #print(self.goal_cnt)
    def done_cb(self, status, result):
        #print(status)
        #print(result)
        #print("raa")
        self.tiempo_transcurrido=time.time()-self.tiempo_anterior  
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")
        if(self.tiempo_transcurrido>10):
            self.goal_cnt += 1
        if status == 3:
            self.tiempo_anterior=time.time()
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.Subscriber("/odom", Odometry, self.nueva_meta_llegada)
        self.pub = rospy.Publisher("move_base/result", MoveBaseActionResult, queue_size=10)
        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")