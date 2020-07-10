#!/usr/bin/env python
import rospy
from dec_pomdp_msgs.srv import StartExperiment, RecordExperiments
from std_srvs.srv import Empty

class ExperimentRecorder:
    def record_experiments(self, request):
        for experiment in range(request.number_of_experiments):
            rospy.loginfo("Experiment number %d", experiment)
            try:
                self.start_experiment(request.random_movement, request.simulate_measurements)
                self.reset_measurement_evaluation()
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s"%e)

    def __init__(self):
        rospy.wait_for_service('start_experiment')
        self.start_experiment = rospy.ServiceProxy('start_experiment', StartExperiment)
        rospy.wait_for_service('reset_measurement_evaluation')
        self.reset_measurement_evaluation = rospy.ServiceProxy('reset_measurement_evaluation', Empty)
        self.record_service = rospy.Service('record_experiments', RecordExperiments, self.record_experiments)

if __name__ == "__main__":
    rospy.init_node('experiment_recorder')
    policy_executioner = ExperimentRecorder()
    rospy.spin()
