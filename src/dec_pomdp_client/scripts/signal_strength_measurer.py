#!/usr/bin/env python
import subprocess
import numpy as np
import re

class SignalStrengthMeasurer:
    def __init__(self):
        self.last_link_quality_min = 0
        self.last_link_quality_max = 0
        self.last_signal_level = 0

    def takeMeasurement(self, ESSID):
        measurements = self.getAllSignalStrengths()
        essid_string = 'ESSID:"' + ESSID + '"'
        measurements_for_essid = filter(lambda x: essid_string in x, measurements)
        dBms = []
        for measurement in measurements_for_essid:
            dBm = int(measurement[0].split('=')[1])
            dBms.append(dBm)
        #TODO continue here
        sorted_dBms = np.sort(np.array(dBms))
        if len(sorted_dBms) > 0:
            return sorted_dBms[0]
        else:
            return False

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
