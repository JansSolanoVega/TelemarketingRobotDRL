import rospy
import numpy as np
import math 
import time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from tf.transformations import euler_from_quaternion
from stable_baselines import DQN, A2C, PPO2
class DRL_test:
    def __init__(self, path_temp_model, algorithm, continuous_actions, goal, numObsNoLaser):
        self.continuous_space=continuous_actions
        self.algorithm = algorithm
        self.goal_x = goal[0];self.goal_y = goal[1]
        self.numObsNoLaser = numObsNoLaser

        rospy.init_node('drl_gazebo', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.read_laser_data_gazebo)
        rospy.Subscriber('/odom', Odometry, self.read_odometry)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        if algorithm=="DQN":
            self.model = DQN.load(path_temp_model)
        elif algorithm=="A2C":
            self.model = A2C.load(path_temp_model)
        elif algorithm=="PPO2":
            self.model = PPO2.load(path_temp_model)
        
        self.eval_model()
        rospy.spin()
        
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
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.angle = np.degrees(yaw)
        self.angle_to_goal = np.degrees(math.atan2(self.goal_y - self.y, self.goal_x - self.x))

        self.distance_to_goal = np.sqrt((self.x-self.goal_x)**2+(self.y-self.goal_y)**2)
        self.angular_error = self.angle_to_goal-self.angle
        if self.angular_error>180:
            self.angular_error=-(360-self.angular_error)
        print(self.x, self.y)
        #print("Distancia: ", self.distance_to_goal)
        #print("Error posicion angular: ", self.angular_error)
        #print("time executing odometry: ",time.time()-a)
        
    def eval_model(self):
        twist = Twist()
        t_anterior=0
        while self.distance_to_goal>0.3:
            #a=time.time()
            if (time.time()-t_anterior)>0.1:
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
        

    

if __name__ == '__main__':
    try:
        algo = DRL_test(path_temp_model="best_model_DQN_StaticEnvironmentDiscrete_RestartHit_5Obstacles_RewardReverseNone", algorithm="DQN",#best_model_a2c_laser_corrected
                    continuous_actions=0, goal=(6,6), numObsNoLaser=4)
        
    except rospy.ROSInterruptException:
        pass
    
