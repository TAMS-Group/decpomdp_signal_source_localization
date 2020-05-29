#!/usr/bin/env python
import subprocess
import numpy as np
import re
import rospy
from scipy.stats import rice

class SignalStrengthMeasurer:
    MAX_ATTEMPTS = 10

    def __init__(self):
        self.last_link_quality_min = 0
        self.last_link_quality_max = 0
        self.last_signal_level = 0
        self.rate = rospy.Rate(0.5) #TODO CHANGE BACK FOR NON SIMULATION 0.5

    def takeMeasurement(self, ESSID):
        tries = 0
        while not rospy.is_shutdown():
            measurements = self.getAllSignalStrengths()
            essid_string = 'ESSID:"' + ESSID + '"'
            measurements_for_essid = filter(lambda x: essid_string in x, measurements)
            dBms = []
            for measurement in measurements_for_essid:
                dBm = int(measurement[0].split('=')[1])
                dBms.append(dBm)
            sorted_dBms = np.sort(np.array(dBms))
            if len(sorted_dBms) > 0:
                return sorted_dBms[0]
            elif(tries > self.MAX_ATTEMPTS):
                rospy.logwarn('Attempt unsuccessful. Maximum number of attempts reached')
                return 0
            else:
                tries += 1
                rospy.logwarn('Attempt to get measurement failed. retrying...')
                self.rate.sleep()

    def takeSimulatedMeasurement(self, ESSID, distance, v=2.4e9, b=0.009, loc=-7.001, scale=12.551):
        rss_base = 147.55 - 20.0 * np.log10(v) - 20.0 * np.log10(distance)
        rss = rss_base - rice.rvs(b, loc=loc, scale=scale)[0]
        self.rate.sleep()
        return rss

    def getAllSignalStrengths(self):
        cmd = ["iwlist", "scanning"]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        points = proc.stdout.read()
        CellArray = points.split("Cell")
        Output = []
        for cell in CellArray:
            cellinfo = cell.split()
            cellinfo = list(filter(
                lambda x: re.match(r"ESSID*", x) or re.match(r"level*", x),
                cellinfo
            ))
            Output.append(cellinfo)
        return Output


if __name__ == '__main__':
    measurer = SignalStrengthMeasurer()
    ESSID = input("ESSID = ?")
    print(measurer.takeMeasurement(ESSID))
    result = input("Continue ? ")
    print(result)
