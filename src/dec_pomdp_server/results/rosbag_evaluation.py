#!/usr/bin/env python
import rospy
import rosbag
import matplotlib.pyplot as plt

def evaluateRosbag(name):
    bag = rosbag.Bag(name)
    errors = []
    for topic, msg, t in bag.read_messages(topics=['/measurements_evaluation/result']):
        if (msg.result.error > 0.0):
            errors.append(msg.result.error)
    bag.close()
    print('Rosbag %s has %d experiments' % (name, len(errors)))
    return errors

if __name__ == '__main__':
    rospy.init_node('rosbag_evaluation')
    rand_errors_exp1 = evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/100RandomMovementExperiments.bag')
    rand_errors_exp2 = evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/100RandomMovementExperimentsV2.bag')
    rand_errors_exp3 = evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/100RandomMovementExperimentsV3.bag')
    joined_error_list = rand_errors_exp1 + rand_errors_exp2 + rand_errors_exp3 
    joined_dec_pomdp_error_list = evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperiments.bag')
    joined_dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV2.bag')
    joined_dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV3.bag')
    joined_dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV4.bag')
    joined_dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV5.bag')
    joined_dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV6.bag')
    joined_dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV7.bag')
    joined_dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV8.bag')
    joined_dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV9.bag')
    joined_dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV10.bag')
    joined_dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV11.bag')
    data = [joined_error_list, joined_dec_pomdp_error_list]
    fig, ax = plt.subplots()
    ax.set_title('%d Random Movement and %d dec POMDP experiments' % (len(joined_error_list), len(joined_dec_pomdp_error_list)))
    ax.boxplot(data, labels=["Random Movement", "dec-POMDP"])
    plt.show()
    rospy.spin()