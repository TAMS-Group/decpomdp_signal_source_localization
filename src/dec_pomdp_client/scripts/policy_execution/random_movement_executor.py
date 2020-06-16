#!/usr/bin/env python
import rospy
import random
import numpy as np
from executor import Executor
from std_msgs.msg import Int64
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class RandomMovement(Executor): 
    """
    This class will make the 
    """
    def execute_random_movement(self, number_of_steps):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        starting_pose = self.get_current_loc()
        current_location = min(self.locations, key=lambda loc: np.sqrt(
                                                        (self.locations[loc].x - starting_pose.pose.position.x)**2 +
                                                        (self.locations[loc].y - starting_pose.pose.position.y)**2
                                                    )
        )
        for step in range(number_of_steps.data):
            rospy.loginfo("Next location is %d with coordinates x = %f and y = %f ", current_location, self.locations[current_location].x, self.locations[current_location].y)
            goal.target_pose.pose.position.x = self.locations[current_location].x
            goal.target_pose.pose.position.y = self.locations[current_location].y
            goal.target_pose.pose.orientation.w = 1
            goal.target_pose.header.stamp = rospy.get_rostime()
            self.move_to_goal(goal)
            measurement_value = self.take_measurement()
            possible_moves = self.allowed_moves[current_location]
            current_location = random.choice(possible_moves)





    def __init__(self):
        super(RandomMovement, self).__init__()

        movement_graph_file_path = rospy.get_param('movement_graph_file_path')
        #Read nodes
        location_file = open(movement_graph_file_path + "locations.txt", "r")
        self.locations = dict()
        for line in location_file.readlines():
            line_content = [float(x) for x in line.split(" ")]
            self.locations[int(line_content[0])] = Point(line_content[1],line_content[2], 0.0)
        location_file.close()

        #Read Allowed moves
        allowed_moves_file = open(movement_graph_file_path + "allowed_moves.txt", "r")
        self.allowed_moves = dict()
        for line in allowed_moves_file.readlines():
            line_content = [int(x) for x in line.split(" ")]
            self.allowed_moves[line_content.pop(0)] = line_content
        allowed_moves_file.close()
        robot_name = rospy.get_param("robot_name")
        self.random_movement_subscriber =  rospy.Subscriber(robot_name + "/randomMovement", Int64, self.execute_random_movement)

if __name__ == "__main__":
    rospy.init_node("random_movement")
    random_movement_executor = RandomMovement()
    rospy.spin()