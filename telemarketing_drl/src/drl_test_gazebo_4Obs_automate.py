#!/usr/bin/env python3
import rospy
import numpy as np
import math 
import time
import actionlib

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import euler_from_quaternion
from stable_baselines import DQN, A2C, PPO2, SAC
import os
from rospkg import RosPack

TIME_FREQ = 0.1
MIN_DISTANCE_TO_GOAL = 0.3
class DRL_test:
    def __init__(self, path_temp_model, algorithm, continuous_actions, numObsNoLaser):
        self.continuous_space=continuous_actions
        self.algorithm = algorithm
        self.numObsNoLaser = numObsNoLaser
        self.ranges = None
        self.success=0

        rospy.init_node('drl_gazebo', anonymous=True)

        rospy.Subscriber('/scan', LaserScan, self.read_laser_data_gazebo)
        rospy.Subscriber('/odom', Odometry, self.read_odometry)
        #rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.goal_x=0; self.goal_y=0
        if algorithm=="DQN":
            self.model = DQN.load(path_temp_model)
        elif algorithm=="A2C":
            self.model = A2C.load(path_temp_model)
        elif algorithm=="PPO2":
            self.model = PPO2.load(path_temp_model)
        elif algorithm=="SAC":
            self.model = SAC.load(path_temp_model)        
        
        self.server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.execute, False)
        self.server.start()
        

    def goal_callback(self, data):
        data = data.goal.target_pose
        goal_pose = data.pose.position
        self.goal_x = goal_pose.x;self.goal_y = goal_pose.y
        print("Nueva meta asignada")
        
        while self.distance_to_goal<0.3 or self.ranges is None:
            pass
        self.eval_model()

    def read_laser_data_gazebo(self, data):
        self.ranges = np.array(data.ranges)
        self.ranges = list(np.where(np.isinf(self.ranges), 10.0, self.ranges))

        self.num_lidar_samples = len(self.ranges)
        #print(num_lidar_samples)
        valor_minimo = min(self.ranges)
        indice_valor_minimo = self.ranges.index(valor_minimo)
        self.distance_to_nearest_obstacle = valor_minimo
        self.angular_error_nearest_obstacle = (indice_valor_minimo*240.0/self.num_lidar_samples)-120.0

    def read_odometry(self, data):
        #a=time.time()
        self.positionCurrent = data.pose
        self.x = self.positionCurrent.pose.position.x
        self.y = self.positionCurrent.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.angle = np.degrees(yaw)
        self.angle_to_goal = np.degrees(math.atan2(self.goal_y - self.y, self.goal_x - self.x))

        self.distance_to_goal = np.sqrt((self.x-self.goal_x)**2+(self.y-self.goal_y)**2)
        self.angular_error = self.angle_to_goal-self.angle
        if self.angular_error>180:
            self.angular_error=-(360-self.angular_error)
        #print(self.x, self.y)
        #print("Distancia: ", self.distance_to_goal)
        #print("Error posicion angular: ", self.angular_error)
        #print("time executing odometry: ",time.time()-a)
        
    def eval_model(self):
        twist = Twist()
        t_anterior=0
        self.success=0

        while self.distance_to_goal>MIN_DISTANCE_TO_GOAL:
            #a=time.time()
            if (time.time()-t_anterior)>TIME_FREQ:
                obs = self.ranges.copy()
                obs.append(self.distance_to_goal);obs.append(self.angular_error)
                if self.numObsNoLaser>2:
                    obs.append(self.distance_to_nearest_obstacle);obs.append(self.angular_error_nearest_obstacle)
                obs = np.reshape(obs, (1, 1, self.num_lidar_samples+self.numObsNoLaser))
                action, _ = list(self.model.predict(obs))
                
                if self.continuous_space:
                    twist.linear.x = action[0][0]; twist.linear.y = 0.0; twist.linear.z = 0.0
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = action[0][1]
                else:
                    action_discrete_map = {
                        "forward": [0.4, 0],
                        "left": [0.3, 0.3],
                        "right": [0.3, -0.3],
                        "strong_left": [0, 0.6],
                        "strong_right": [0, -0.6],
                        "backward": [-0.4, 0],
                        "stop": [0, 0]
                    }
                    list_actions = list(action_discrete_map.items())
                    twist.linear.x = list_actions[action[0]][1][0]; twist.linear.y = 0.0; twist.linear.z = 0.0
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = list_actions[action[0]][1][1]

                #print("Time executing model: ", time.time()-a)
                self.pub.publish(twist) 
                t_anterior=time.time()     

        twist.linear.x = 0; twist.angular.z = 0
        self.pub.publish(twist)
        print("Meta Alcanzada por el planner")
    def execute(self, goal):
        
        rospy.loginfo('Received goal: {}'.format(goal.target_pose.pose))
        
        data = goal.target_pose
        goal_pose = data.pose.position
        self.goal_x = goal_pose.x;self.goal_y = goal_pose.y
        print("Nueva meta asignada")
        
        while self.distance_to_goal<0.3 or self.ranges is None:
            pass
        self.eval_model()
        
        # result_msg = MoveBaseActionResult()
        # result_msg.status.status = actionlib.GoalStatus.SUCCEEDED
        # self.server.set_succeeded(result_msg) 

        # result_msg.status.status = actionlib.GoalStatus.PREEMPTED
        # self.server.set_preempted(result_msg) 
        
    
    

if __name__ == '__main__':
    try:
        packageName = "telemarketing_drl"
        rospack = RosPack(); package_path = rospack.get_path(packageName)
        fileName = "best_model_SAC_24_Dynamic.Human.Random_3e-1.10.10_Continuous_25e-5_ConstantActiveReward.10_MaxAvoidanceReward.10_RateDecay.5_WithoutHitReward_re.8e-1_240Lidar_WithoutTF_DobleStep_0.2"
        algo = DRL_test(path_temp_model=os.path.join(package_path, "src/models", fileName), algorithm="SAC",#best_model_a2c_laser_corrected
                    continuous_actions=1, numObsNoLaser=4)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    
