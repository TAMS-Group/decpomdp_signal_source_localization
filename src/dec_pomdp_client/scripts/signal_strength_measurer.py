#!/usr/bin/env python
import subprocess

class SignalStrengthMeasurer:
    def __init__(self):
        self.last_link_quality_min = 0
        self.last_link_quality_max = 0
        self.last_signal_level = 0

    def takeMeasurement(self):
        cmd = subprocess.Popen(["iwconfig"], stdout=subprocess.PIPE)
        outs, errs = cmd.communicate()
        output = outs.split()
        link_quality = output[output.index(b"Link") + 1].decode("utf-8")
        signal_level = output[output.index(b"Signal") + 1].decode("utf-8")
        #cmd.terminate()
        link_quality = link_quality.split('=')[1]
        link_quality = link_quality.split('/')
        self.last_link_quality_min = link_quality[0]
        self.last_link_quality_max = link_quality[1]
        self.last_signal_level = int(signal_level.split('=')[1])
        return link_quality[0], link_quality[1], signal_level.split('=')[1]
