#!/usr/bin/env python
import rospy
from dec_pomdp_msgs.srv import StartExperiment

class ExperimentRecorder:
    def record_experiments(self, number, random_movement=False):
        for experiment in range(experiments_to_record):
            rospy.loginfo("Experiment number %d", experiment)
            try:
                self.start_experiment(random_movement)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s"%e)

    def __init__(self, experiments_to_record):
        rospy.wait_for_service('start_experiment')
        self.start_experiment = rospy.ServiceProxy('start_experiment', StartExperiment)
