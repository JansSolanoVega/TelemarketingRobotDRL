import rospy
import numpy as np
import math
from tf.transformations import euler_from_quaternion
from gym import spaces

from gym.envs.registration import register
from geometry_msgs.msg import Point
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
from openai_ros.robot_envs import telemarketing_env


timestep_limit_per_episode = 10000 # Can be any Value

register(
        id='TelemarketingEmpty-v0',
        entry_point='telemarketing_empty:TelemarketingEmptyEnv',
        max_episode_steps=timestep_limit_per_episode,
    )

class TelemarketingEmptyEnv(telemarketing_env.TelemarketingEnv):
    def __init__(self):
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/telemarketing/config",
                               yaml_file_name="telemarketing_empty.yaml")
        # ROSLauncher(rospackage_name="telemarketing_description",
        #             launch_file_name="gazebo.launch",
        #             ros_ws_abspath = rospy.get_param("/telemarketing/ros_ws_abspath", None))
         # Here we will add any init functions prior to starting the MyRobotEnv

        ros_ws_abspath = rospy.get_param("/telemarketing/ros_ws_abspath", None)
        super(TelemarketingEmptyEnv, self).__init__(ros_ws_abspath)

        # Only variable needed to be set here
        number_actions = rospy.get_param('/telemarketing/n_actions')
        self.action_space = spaces.Discrete(number_actions)
        
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-np.inf, np.inf)
        
        # Actions and Observations
        self.max_linear_speed = rospy.get_param('/telemarketing/max_linear_speed')
        self.linear_speed_turning = rospy.get_param('/telemarketing/linear_speed_turning')
        self.max_angular_speed = rospy.get_param('/telemarketing/max_angular_speed')
        self.angular_speed_turning = rospy.get_param('/telemarketing/angular_speed_turning')
        self.init_linear_forward_speed = rospy.get_param('/telemarketing/init_linear_forward_speed')
        self.init_linear_turn_speed = rospy.get_param('/telemarketing/init_linear_turn_speed')

        self.max_laser_distance = rospy.get_param('/telemarketing/max_laser_distance')
        self.number_observations = rospy.get_param('/telemarketing/n_observations')
        
        # Get Desired Point to Get
        self.desired_point = Point()
        self.desired_point.x = rospy.get_param("/telemarketing/desired_pose/x")
        self.desired_point.y = rospy.get_param("/telemarketing/desired_pose/y")
        self.desired_point.z = rospy.get_param("/telemarketing/desired_pose/z")
        
        # We create two arrays based on the binary values that will be assigned
        # In the discretization method.
        laser_scan = self.get_laser_scan()
        rospy.logdebug("laser_scan len===>" + str(len(laser_scan.ranges)))

        # We only use two integers
        self.observation_space = spaces.Box(low=0, high=self.max_laser_distance, shape=(1, self.number_observations), dtype=np.float)

        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))

        # Rewards
        self.reward_goal = rospy.get_param("/telemarketing/reward_goal")
        self.reward_towards_goal = rospy.get_param("/telemarketing/reward_towards_goal")
        self.reward_away_from_goal = rospy.get_param("/telemarketing/reward_away_from_goal")
        self.reward_hit = rospy.get_param("/telemarketing/reward_hit")
        self.reward_time_out = rospy.get_param("/telemarketing/reward_time_out")
        self.reverse_reward = rospy.get_param("/telemarketing/reverse_reward")
        self.min_laser_distance_for_crashing = rospy.get_param("/telemarketing/min_laser_distance_for_crashing")

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_base( self.init_linear_forward_speed,
                        self.init_linear_turn_speed,
                        epsilon=0.05,
                        update_rate=10)

        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        self.cumulated_reward = 0.0
        # Set to false Done, because its calculated asyncronously
        self._episode_done = False
        
        odometry = self.get_odom()
        self.previous_distance_from_des_point = self.get_distance_from_desired_point(odometry.pose.pose.position)

        self.cumulated_steps = 0.0

    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the Telemarketing
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """
        discrete_action_list = ["forward", "left", "right", "strong_left", "strong_right", "backward", "stop"]
        # action_discrete_map = {
        #                 "forward": [0.4, 0],
        #                 "left": [0.3, 0.3],
        #                 "right": [0.3, -0.3],
        #                 "strong_left": [0, 0.6],
        #                 "strong_right": [0, -0.6],
        #                 "backward": [-0.4, 0],
        #                 "stop": [0, 0]
        #             }
        rospy.logdebug("Start Set Action ==>"+str(action))
        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
        if action == 0: #forward
            linear_speed = self.max_linear_speed
            angular_speed = 0.0
        elif action == 1: #left
            linear_speed = self.linear_speed_turning
            angular_speed = self.angular_speed_turning
        elif action == 2: #right
            linear_speed = self.linear_speed_turning
            angular_speed = -1*self.angular_speed_turning
        elif action == 3: #strong_left
            linear_speed = 0.0
            angular_speed = self.max_angular_speed
        elif action == 4: #strong_right
            linear_speed = 0.0
            angular_speed = -1*self.max_angular_speed
        elif action == 5: #backward
            linear_speed = -1*self.max_linear_speed
            angular_speed = 0.0
        elif action == 6: #stop
            linear_speed = 0.0
            angular_speed = 0.0
        self.last_action=discrete_action_list[action]
        
        self.move_base(linear_speed, angular_speed, epsilon=0.05, update_rate=10)
        
        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        TurtleBot2Env API DOCS
        :return:
        """
        rospy.logdebug("Start Get Observation ==>")
        # Odometry data
        odometry = self.get_odom()
        x_position = odometry.pose.pose.position.x
        y_position = odometry.pose.pose.position.y
        orientation_q = odometry.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        angle = np.degrees(yaw)

        #OBSERVATIONS
        # We get the laser scan data
        laser_scan = np.array(self.get_laser_scan().ranges)
        laser_scan = list(np.where(np.isinf(laser_scan), 10.0, laser_scan))
        
        #Distance to goal
        self.current_position = Point()
        self.current_position.x = x_position
        self.current_position.y = y_position
        self.current_position.z = 0.0
        distance_from_des_point = self.get_distance_from_desired_point(self.current_position)
                                                                        
        #Angle error                                                 
        angle_to_goal = self.angle_to_goal(self.current_position)
        angular_error = angle_to_goal-angle
        if angular_error>180:
            angular_error=-(360-angular_error)

        observations = laser_scan.copy()
        #print(observations)
        observations.append(distance_from_des_point);observations.append(angular_error)

        #rospy.logdebug("Observations==>"+str(observations))
        rospy.logdebug("END Get Observation ==>")
        return observations
        

    def _is_done(self, observations):
        '''
        current_position = Point()
        current_position.x = observations[-2]
        current_position.y = observations[-1]
        current_position.z = 0.0'''
        #print(self.current_position.x, "--", self.current_position.y)
        if self.is_in_desired_position(self.current_position):
            self._episode_done = True
            rospy.logwarn("GOAL REACHED==>")
            #self.desired_point.x = np.round(np.random.uniform(-8, 8), 1)
            #self.desired_point.y = np.round(np.random.uniform(-8, 8), 1)
            #rospy.logwarn("NEW GOAL==>", "x: ",self.desired_point.x, "y: ", self.desired_point.y)
            #print("ga")
        else:
            self._episode_done = self.has_crashed(self.min_laser_distance_for_crashing)
            if self._episode_done:
                rospy.logwarn("Telemarketing has crashed==>")
            else:
                rospy.logwarn("Telemarketing didnt crash at least ==>")  
        return self._episode_done

    def _compute_reward(self, observations, done):
        reward = 0
        '''current_position = Point()
        current_position.x = observations[-2]
        current_position.y = observations[-1]
        current_position.z = 0.0'''

        distance_from_des_point = self.get_distance_from_desired_point(self.current_position)
        distance_difference =  distance_from_des_point - self.previous_distance_from_des_point


        if not done:
                
            # If there has been a decrease in the distance to the desired point, we reward it
            if distance_difference < 0.0:
                rospy.logwarn("Towards goal")
                reward += self.reward_towards_goal
            else:
                rospy.logerr("Away goal")
                reward -= self.reward_away_from_goal
                
        else:
            
            if self.is_in_desired_position(self.current_position):
                reward = self.reward_goal
            else:
                reward = -1*self.reward_hit


        self.previous_distance_from_des_point = distance_from_des_point


        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))
        
        return reward


    # Internal TaskEnv Methods    
        
    def is_in_desired_position(self,current_position, epsilon=0.15):
        """
        It return True if the current position is similar to the desired poistion
        """
        
        is_in_desired_pos = False
        
        
        x_pos_plus = self.desired_point.x + epsilon
        x_pos_minus = self.desired_point.x - epsilon
        y_pos_plus = self.desired_point.y + epsilon
        y_pos_minus = self.desired_point.y - epsilon
        
        x_current = current_position.x
        y_current = current_position.y
        
        #print(x_current,"-",)
        x_pos_are_close = (x_current <= x_pos_plus) and (x_current > x_pos_minus)
        y_pos_are_close = (y_current <= y_pos_plus) and (y_current > y_pos_minus)
        
        is_in_desired_pos = x_pos_are_close and y_pos_are_close
        
        return is_in_desired_pos
        
        
    def get_distance_from_desired_point(self, current_position):
        """
        Calculates the distance from the current position to the desired point
        :param start_point:
        :return:
        """
        distance = self.get_distance_from_point(current_position,
                                                self.desired_point)
    
        return distance
    
    def get_distance_from_point(self, pstart, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = np.array((pstart.x, pstart.y, pstart.z))
        b = np.array((p_end.x, p_end.y, p_end.z))
    
        distance = np.linalg.norm(a - b)
    
        return distance
    
    def angle_to_goal(self, current_position):
        return np.degrees(math.atan2(self.desired_point.y - current_position.y, self.desired_point.x - current_position.x))


