#!/usr/bin/env python

#    signal_strength_visualization.py
#
#	 Copyright 2020 Tobias Krueger
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import matplotlib.pyplot as plt
import json
import numpy as np

if __name__ == '__main__':
    with open('/home/tobias/Documents/BachelorThesis Visualization/measurements2.json', 'r') as json_file:
        data = json.load(json_file)
        x_orientations = []
        x_time = []
        y_signal_strengths = []
        measurements = sorted(data["measurements"], key=lambda x: x["time_secs"])
        measurements = filter(lambda x: x["time_secs"]> 0, measurements)
        starting_time = measurements[0]["time_secs"]
        for measurement in measurements:
            if measurement["time_secs"] == 0:
                continue
            x_time.append(measurement["time_secs"] - starting_time)
            x_orientations.append(measurement["orientation"])
            y_signal_strengths.append(measurement["signal_strength"])
        plt.ylabel("signal strengths in dBm")
        plt.xlabel("elapsed time")
        plt.plot(x_time, y_signal_strengths)
        plt.show()
        last_time = measurements[0]["time_secs"] + measurements[0]["time_nsecs"] * 10**(-9)
        times_between_measurements = []
        for measurement in measurements:
            time = measurement["time_secs"] + measurement["time_nsecs"] * 10**(-9) - last_time
            last_time = measurement["time_secs"] + measurement["time_nsecs"] * 10**(-9)
            if time <= 0:
                continue
            times_between_measurements.append(time)
        plt.ylabel("average time between measurements in secs")
        plt.boxplot(times_between_measurements)
        print(times_between_measurements)
        plt.show()
        plt.xlabel("orientation (1.0 is pointing at the source)")
        plt.ylabel("signal strength in dbm")
        plt.scatter(x_orientations, y_signal_strengths)
        plt.show()
            
            
