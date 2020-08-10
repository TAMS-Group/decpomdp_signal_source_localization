#!/usr/bin/env python
import rospy
import random
import numpy as np
import actionlib
from executor import Executor
from std_msgs.msg import Int64
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from dec_pomdp_msgs.msg import Measurements, ExecuteRandomMovementAction, ExecuteRandomMovementFeedback, ExecuteRandomMovementResult, ExecuteRandomMovementGoal, ExecutionState
from dec_pomdp_msgs.srv import GetExecutionStatus

class RandomMovement(Executor):
    """
    This class will make the
    """
    _feedback = ExecuteRandomMovementFeedback()
    _result = ExecuteRandomMovementResult()

    def execute_random_movement(self, random_movement_goal):
        number_of_steps = random_movement_goal.number_of_steps
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        starting_pose = self.get_current_loc()
        rospy.loginfo("starting pose is x = %f and y = %f", starting_pose.pose.position.x, starting_pose.pose.position.y)
        current_location = min(self.locations, key=lambda loc: np.sqrt(
                                                        (self.locations[loc].x - starting_pose.pose.position.x)**2 +
                                                        (self.locations[loc].y - starting_pose.pose.position.y)**2
                                                    )
        )
        #start at Random location instead of nearest as above
        current_location = random.choice(range(len(self.locations)))
        for step in range(number_of_steps):
            rospy.loginfo("Next location is %d with coordinates x = %f and y = %f ", current_location, self.locations[current_location].x, self.locations[current_location].y)
            goal.target_pose.pose.position.x = self.locations[current_location].x
            goal.target_pose.pose.position.y = self.locations[current_location].y
            goal.target_pose.pose.orientation.w = 1
            goal.target_pose.header.stamp = rospy.get_rostime()
            self.move_to_goal(goal)
            measurement_value = self.take_measurement()
            possible_moves = self.allowed_moves[current_location]
            current_location = random.choice(possible_moves)
            self._feedback.step = step
            self._feedback.current_measurements = self.measurements
            self.random_movement_action_server.publish_feedback(self._feedback)
        self._result.measurements = self.measurements
        self.measurements = Measurements()
        self._current_status = ExecutionState.IDLE
        self.random_movement_action_server.set_succeeded(self._result)





    def __init__(self):
        super(RandomMovement, self).__init__()
        mgfp_param_name = rospy.search_param('movement_graph_file_path')
        movement_graph_file_path = rospy.get_param(mgfp_param_name)
        #initialize Execution status feedback service
        self.execution_status_service = rospy.Service('get_random_move_execution_status', GetExecutionStatus, self.get_execution_status)
        #Read nodes
        location_file = open(movement_graph_file_path + "/locations.txt", "r")
        self.locations = dict()
        for line in location_file.readlines():
            line_content = [float(x) for x in line.split(" ")]
            self.locations[int(line_content[0])] = Point(line_content[1],line_content[2], 0.0)
        location_file.close()

        #Read Allowed moves
        allowed_moves_file = open(movement_graph_file_path + "/allowed_moves.txt", "r")
        self.allowed_moves = dict()
        for line in allowed_moves_file.readlines():
            line_content = [int(x) for x in line.split(" ")]
            self.allowed_moves[line_content.pop(0)] = line_content
        allowed_moves_file.close()
        robot_name = rospy.get_param("robot_name")
        self.random_movement_action_server = actionlib.SimpleActionServer('execute_random_movement', ExecuteRandomMovementAction, execute_cb=self.execute_random_movement, auto_start=False)
        self.random_movement_action_server.start()

if __name__ == "__main__":
    rospy.init_node("random_movement")
    random_movement_executor = RandomMovement()
    rospy.spin()
