#!/usr/bin/env python
# license removed for brevity
__author__ = 'fiorellasibona'
import rospy
import math
import numpy as np

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import time
from gazebo_msgs.msg import ModelStates


class MoveBaseSeq():

    def __init__(self):
        rospy.init_node('move_base_sequence')

        #self.pos_start_x=0
        #self.pos_start_y=-6

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
    '''
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
    '''
    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
        #print(str(self.pose_seq[self.goal_cnt]))
        a=1
    def done_cb(self, status, result):
        print(status)
        print(result)
        print("raa")
        self.tiempo_transcurrido=time.time()-self.tiempo_anterior  
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")
        if(self.tiempo_transcurrido>10):
            self.goal_cnt += 1
        if status == 3:
            self.tiempo_anterior=time.time()
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            
            '''
            while 1:
                distancia=np.sqrt((self.pos_modelo_x-self.pos_start_x)**2+(self.pos_modelo_y-self.pos_start_y)**2)
                roll,pitch,yaw=self.quaternion_to_euler(self.cuat_x,self.cuat_y,self.cuat_z,self.cuat_w)

                #print("Distancia: ",distancia," Angulo: ",yaw)

                if(distancia<0.1 and (yaw<0)):
                    break
            '''
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
    '''
    def callback(self,model_states):
        for i in range(len(model_states.name)):
            nombre=model_states.name[i]
            if nombre=="actor1":
                self.pos_modelo_x=round(model_states.pose[i].position.x,2)
                self.pos_modelo_y=round(model_states.pose[i].position.y,2)
                self.cuat_x=model_states.pose[i].orientation.x
                self.cuat_y=model_states.pose[i].orientation.y
                self.cuat_z=model_states.pose[i].orientation.z
                self.cuat_w=model_states.pose[i].orientation.w
                #print(self.pos_modelo_x,", ",self.pos_modelo_y)
    '''
    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo("Posicion: "+str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        #rospy.Subscriber("gazebo/model_states", ModelStates, self.callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")