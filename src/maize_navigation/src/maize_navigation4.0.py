#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField,Joy
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Twist, Point32
from std_msgs.msg import Bool


class FieldRobotNavigator:
    def __init__(self):
        rospy.init_node('field_robot_navigator')

        # Set up subscribers and publishers
        rospy.Subscriber('/merged_point_cloud', PointCloud2, self.point_cloud_callback)
        rospy.Subscriber('/teleop/automatic_mode', Bool, self.automatic_mode_callback)
        rospy.Subscriber('/obstacle/obstacle_automatic_mode', Bool, self.obstacle_automatic_mode_callback)
        rospy.Subscriber('/teleop/cmd_vel', Twist, self.teleop_cmd_vel_callback)
        rospy.Subscriber('/teleop/movement_sequence', Joy, self.pattern_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.points_pub = rospy.Publisher('/field_points', PointCloud2, queue_size=1)

        # Initialize member variables
        self.robot_pose = None
        self.points = None

        self.box = rospy.set_param('box', 'drive')
        self.both_sides = rospy.set_param('both_sides', 'both')

        # Initialize parameters
        self.x_min = rospy.get_param('x_min_drive_in_row')
        self.x_max = rospy.get_param('x_max_drive_in_row')
        self.y_min = rospy.get_param('y_min_drive_in_row')
        self.y_max = rospy.get_param('y_max_drive_in_row')

        self.row_width = rospy.get_param('row_width')
        self.drive_out_dist =rospy.get_param('drive_out_dist')
        self.max_dist_in_row = rospy.get_param('max_dist_in_row')
        self.critic_row = rospy.get_param('critic_row')

        self.vel_linear_drive =rospy.get_param('vel_linear_drive')
        self.vel_linear_count= rospy.get_param('vel_linear_count')
        self.vel_linear_turn =rospy.get_param('vel_linear_turn')

        self.last_state = 'drive_in_row'
        
        # Initialize state variables
        self.current_state = 'manual_mode'
        self.pattern = rospy.get_param('pattern')
        self.automatic_mode=False
        self.obstacle_automatic_mode= True
        self.teleop_cmd_vel=Twist()
        self.teleop_cmd_vel.linear.x = 0
        self.teleop_cmd_vel.angular.z = 0

        self.driven_row = 0
        


    def point_cloud_callback(self, msg):
        points = []
        both_sides = rospy.get_param('both_sides')
        min_distance = float('inf')  # initialize minimum distance with infinity

        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            point = Point32(*p)
            distance = np.sqrt(point.x**2 + point.y**2)  # calculate Euclidean distance
            if distance < min_distance:
                min_distance = distance
            if both_sides == 'both':
                if self.y_min < np.abs(point.y) < self.y_max and self.x_min < point.x < self.x_max:
                    points.append(point)
            elif both_sides == 'L':
                if -self.y_max < point.y < -self.y_min and self.x_min < point.x < self.x_max:
                    points.append(point)
            elif both_sides == 'R':
                if self.y_min < point.y < self.y_max and self.x_min < point.x < self.x_max:
                    points.append(point)
        self.min_dist = min_distance
        self.points = points

        # Publish self.points
        header = msg.header
        header.frame_id = "laserFront"
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]
        points = [(p.x, p.y, p.z) for p in self.points]
        cloud = point_cloud2.create_cloud(header, fields, points)
        self.points_pub.publish(cloud)


    def navigate(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.points is None:
                rate.sleep()
                continue
            else:
                if self.current_state == 'drive_in_row':
                    self.drive_in_row()
                elif self.current_state == 'turn_and_exit':
                    self.turn_and_exit()
                elif self.current_state == 'counting_rows':
                    self.counting_rows()
                elif self.current_state == 'turn_to_row':
                    self.turn_to_row()
                elif self.current_state == 'manual_mode':
                    self.manual_mode()
                    

                rate.sleep()

    def pattern_callback(self,msg):
    # Callback function to process the received message
        buttons = msg.buttons
        pattern = []

        # Iterate over the buttons list with a step size of 2
        for i in range(0, len(buttons), 2):
            button_count = buttons[i]
            button_value = buttons[i+1]
            if button_value==1:
                button_value='L'
            else:
                button_value='R'    
            pattern.append([button_count, button_value])
            self.pattern = pattern

    def automatic_mode_callback(self,msg):
    # Callback function to process the received message
        self.automatic_mode = msg.data
    
    def obstacle_automatic_mode_callback(self,msg):
        self.obstacle_automatic_mode = msg.data

    def teleop_cmd_vel_callback(self,msg):
    # Callback function to process the received message
        self.teleop_cmd_vel.linear.x = msg.linear.x
        self.teleop_cmd_vel.angular.z = msg.angular.z

    def manual_mode(self):
        rospy.loginfo("Manual mode...")
        cmd_vel = Twist()
        cmd_vel = self.teleop_cmd_vel
        self.cmd_vel_pub.publish(cmd_vel)
        if self.automatic_mode==True and self.obstacle_automatic_mode==True:
            self.current_state='drive_in_row'#self.last_state

    def drive_in_row(self):
        rospy.loginfo("Driving in row...")
        # Calculate the average distance to the robot on both sides within x and y limits
        rospy.set_param('box', 'drive')
        rospy.set_param('both_sides', 'both')
        left_y = [p.y for p in self.points if p.y < 0]
        right_y = [p.y for p in self.points if p.y >= 0]
        left_dist = np.mean(np.abs(left_y)) if len(left_y) > 1 else np.inf #left is negative usually
        right_dist = np.mean(np.abs(right_y)) if len(right_y) > 1 else np.inf
        if np.isinf(left_dist) and np.isinf(right_dist):
            # Not enough data to calculate center
            rospy.loginfo("At least one side has no maize")
            rospy.loginfo("Reached the end of a row.")
            cmd_vel = Twist()
            cmd_vel.linear.x = self.vel_linear_drive
            time = self.drive_out_dist / self.vel_linear_drive
            start_time = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - start_time) < time:
                self.cmd_vel_pub.publish(cmd_vel)
                rospy.loginfo("Leaving the row...")
                rospy.sleep(0.1)

            self.x_min = rospy.get_param('x_min_turn_and_exit')
            self.x_max = rospy.get_param('x_max_turn_and_exit')
            self.y_min = rospy.get_param('y_min_turn_and_exit')
            self.y_max = rospy.get_param('y_max_turn_and_exit')
            rospy.set_param('box', 'exit')
            rospy.set_param('both_sides', self.pattern[self.driven_row][1])
            self.driven_row += 1 
            self.current_state = 'turn_and_exit'
        else:
            if np.isinf(left_dist):
                left_dist=self.row_width-right_dist
            if np.isinf(right_dist):
                right_dist=self.row_width-left_dist
                
            # Calculate the actual distance to the center of both sides
            center_dist = (right_dist - left_dist) / 2.0
            rospy.loginfo("Distance to center: %f", center_dist)
         # Adjust the angular velocity to center the robot between the rows
            cmd_vel = Twist()
            cmd_vel.angular.z = -center_dist*3*self.vel_linear_drive
            if np.abs(center_dist)>0.15:
                cmd_vel.linear.x = 0.1
                if    np.abs(center_dist) > 0.20:
                    cmd_vel.linear.x = 0
                    rospy.logwarn('Too close to row!!!')
            else:
                cmd_vel.linear.x = self.vel_linear_drive*(self.max_dist_in_row-np.abs(center_dist))/self.max_dist_in_row
            rospy.loginfo("Publishing to cmd_vel: %s", cmd_vel)
        self.cmd_vel_pub.publish(cmd_vel)
        if self.automatic_mode==False or self.obstacle_automatic_mode==False:
            self.last_state=self.current_state
            self.current_state='manual_mode'
        

    def turn_and_exit(self):
        rospy.loginfo("Turn and exit...")
        # Calculate the average distance to the robot on both sides within x and y limits
        points_x = [p.x for p in self.points]
        x_mean = np.mean(points_x) if len(points_x) > 0 else np.inf 
        rospy.loginfo("xmean: %f", x_mean)
        if -0.1 < x_mean < 0.1:
            cmd_vel = Twist()
            rospy.loginfo("Aligned to the rows...")
            self.x_min = rospy.get_param('x_min_counting_rows')
            self.x_max = rospy.get_param('x_max_counting_rows')
            self.y_min = rospy.get_param('y_min_counting_rows')
            self.y_max = rospy.get_param('y_max_counting_rows')
            if self.pattern[self.driven_row-1][0]==1:
                if self.driven_row in self.critic_row:
                    self.x_min = rospy.get_param('x_min_turn_to_row_critic')
                    self.x_max = rospy.get_param('x_max_turn_to_row_critic')
                    self.y_min = rospy.get_param('y_min_turn_to_row_critic')
                    self.y_max = rospy.get_param('y_max_turn_to_row_critic')
                    rospy.set_param('box', 'turn_crit')
                else:
                    self.x_min = rospy.get_param('x_min_turn_to_row')
                    self.x_max = rospy.get_param('x_max_turn_to_row')
                    self.y_min = rospy.get_param('y_min_turn_to_row')
                    self.y_max = rospy.get_param('y_max_turn_to_row')
                    rospy.set_param('box', 'turn')

                rospy.set_param('both_sides', 'both')
                self.current_state = 'turn_to_row'
            else:    
                rospy.set_param('box', 'count')
                self.row_counter = 1
                self.previous_row = 1
                self.actual_row = 1
                self.actual_dist = self.min_dist
                self.current_state = 'counting_rows'

        else:
            if 0 <= self.driven_row < len(self.pattern):
                if self.pattern[self.driven_row-1][1] == 'L':
                    # Calculate the actual distance to the center of both sides
                    rospy.loginfo("Turning left until parallel...")
                    # Adjust the angular velocity to center the robot between the rows
                    cmd_vel = Twist()
                    cmd_vel.linear.x = self.vel_linear_turn
                    radius = self.row_width/2
                    cmd_vel.angular.z = self.vel_linear_turn/radius
                elif self.pattern[self.driven_row-1][1] == 'R':
                    rospy.loginfo("Turning right until parallel...")
                    cmd_vel = Twist()
                    cmd_vel.linear.x = self.vel_linear_turn
                    radius = -self.row_width/2
                    cmd_vel.angular.z = self.vel_linear_turn/radius
                else:
                    rospy.logwarn("Invalid direction at driven_row index")
                    cmd_vel = Twist()
            else:
                rospy.loginfo("Pattern is now finished")
                cmd_vel = Twist()
                

            rospy.loginfo("Publishing to cmd_vel: %s", cmd_vel)

        self.cmd_vel_pub.publish(cmd_vel)
        if self.automatic_mode==False or self.obstacle_automatic_mode==False:
            self.last_state=self.current_state
            self.current_state='manual_mode'

    def counting_rows(self):
        rospy.loginfo("counting rows...")
        # Calculate the average distance to the robot on both sides within x and y limits
        if self.pattern[self.driven_row-1][0] == self.row_counter:
            rospy.loginfo("start turning to row...")
            if self.driven_row in self.critic_row:
                self.x_min = rospy.get_param('x_min_turn_to_row_critic')
                self.x_max = rospy.get_param('x_max_turn_to_row_critic')
                self.y_min = rospy.get_param('y_min_turn_to_row_critic')
                self.y_max = rospy.get_param('y_max_turn_to_row_critic')
                rospy.set_param('box', 'turn_crit')
            else:
                self.x_min = rospy.get_param('x_min_turn_to_row')
                self.x_max = rospy.get_param('x_max_turn_to_row')
                self.y_min = rospy.get_param('y_min_turn_to_row')
                self.y_max = rospy.get_param('y_max_turn_to_row')
                rospy.set_param('box', 'turn')
            rospy.set_param('both_sides', 'both')
            
            self.current_state = 'turn_to_row'
        else:
            if self.pattern[self.driven_row-1][1]=='L':
                gain=2.5
            elif self.pattern[self.driven_row-1][1]=='R':
                gain=-2.5

            # Calculate the actual distance to the center of both sides
            rospy.loginfo("Holding Distance, driving parallel")
            # Adjust the angular velocity to center the robot between the rows
            cmd_vel = Twist()
            cmd_vel.linear.x = self.vel_linear_count
            rospy.loginfo("No. of points in Box %i", len(self.points))
            if  len(self.points)>1:
                 # Compute the shortest y distance
                cmd_vel.angular.z = gain*(self.min_dist - self.actual_dist)
                rospy.loginfo("Gap to desired distance:%f",self.min_dist - self.actual_dist)
                self.actual_row = 1
            else:
                    cmd_vel.angular.z = 0
                    self.actual_row = 0
            if self.actual_row > self.previous_row:
               self.row_counter+=1
               rospy.loginfo("Increment row_counter to %i", self.row_counter)

            rospy.loginfo("Passing row %i of %i", self.row_counter, self.pattern[self.driven_row-1][0])
            self.previous_row=self.actual_row     
            rospy.loginfo("Publishing to cmd_vel: %s", cmd_vel)
            self.cmd_vel_pub.publish(cmd_vel)
            if self.automatic_mode==False or self.obstacle_automatic_mode==False:
                self.last_state=self.current_state
                self.current_state='manual_mode'
                


    def turn_to_row(self):
        rospy.loginfo("Turn to row...")
        # Calculate the average distance to the robot on both sides within x and y limits
        points_y = [p.y for p in self.points]
        y_mean = np.mean((points_y)) if len(points_y) > 0 else np.inf #left is negative usually
        rospy.loginfo("ymean: %f", y_mean)
        if -0.1 < y_mean < 0.1:
            cmd_vel = Twist()
            rospy.loginfo("Start driving in row...")
            self.x_min = rospy.get_param('x_min_drive_in_row')
            self.x_max = rospy.get_param('x_max_drive_in_row')
            self.y_min = rospy.get_param('y_min_drive_in_row')
            self.y_max = rospy.get_param('y_max_drive_in_row')
            self.current_state = 'drive_in_row'
        else:
            if 0 <= self.driven_row < len(self.pattern):
                if self.pattern[self.driven_row-1][1] == 'L':
                    gain = 1
                elif self.pattern[self.driven_row-1][1] == 'R':
                    gain=-1
                else:
                    rospy.logwarn("Invalid direction at driven_row index")
                    cmd_vel = Twist()

                rospy.loginfo("Turning right until parallel...")
                cmd_vel = Twist()
                cmd_vel.linear.x = self.vel_linear_turn
                radius = gain*self.row_width/2
                cmd_vel.angular.z = self.vel_linear_turn/radius
            else:
                rospy.logwarn("Driven_row index out of range")
                cmd_vel = Twist()

            rospy.loginfo("Publishing to cmd_vel: %s", cmd_vel)

        self.cmd_vel_pub.publish(cmd_vel)
        if self.automatic_mode==False or self.obstacle_automatic_mode==False:
            self.last_state=self.current_state
            self.current_state='manual_mode'


if __name__ == '__main__':
    navigator = FieldRobotNavigator()
    navigator.navigate()
