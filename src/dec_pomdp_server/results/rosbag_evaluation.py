#!/usr/bin/env python
import rospy
import rosbag
import matplotlib.pyplot as plt
import numpy as np

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
    # Experiment 1
    # random_error_list = evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/100RandomMovementExperiments.bag')
    # random_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/100RandomMovementExperimentsV2.bag')
    # random_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/100RandomMovementExperimentsV3.bag')
    # dec_pomdp_error_list = evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperiments.bag')
    # dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV2.bag')
    # dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV4.bag')
    # dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV3.bag')
    # dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV5.bag')
    # dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV6.bag')
    # additional info
    # dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV7.bag')
    # dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV8.bag')
    # dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV9.bag')
    # dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV10.bag')
    # dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/50decPOMDPExperimentsV11.bag')
    
    #Experiment 2
    dec_pomdp_error_list = evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/DecPOMDPNo1.bag')
    dec_pomdp_error_list += evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/DecPOMDPNo2.bag')
    random_error_list = evaluateRosbag('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/RandomMovementAttempt2-V1.bag')
    data = [random_error_list, dec_pomdp_error_list]
    fig, ax = plt.subplots()
    plt.rcParams.update({'font.size': 22})
    ax.set_title('%d Random Movement and %d dec POMDP experiments' % (len(random_error_list), len(dec_pomdp_error_list)))
    ax.set_ylabel('Weighted Root Mean Squared Error in m')
    boxplot = ax.boxplot(data, labels=["Random Movement", "dec-POMDP"], widths=(0.4, 0.4))
    print('Mean of random error %f and mean of dec POMDP error %f' % (np.mean(random_error_list), np.mean(dec_pomdp_error_list)))
    print('Boxplot random error median %f and Boxplot dec-POMDP error median %f' % (boxplot['medians'][0].get_ydata()[0], boxplot['medians'][1].get_ydata()[0]))
    plt.show()
    rospy.spin()