#!/usr/bin/env python
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
            
            
