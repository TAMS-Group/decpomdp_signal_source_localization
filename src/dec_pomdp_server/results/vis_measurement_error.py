import numpy as np 
import json
import matplotlib.pyplot as plt

if __name__ == "__main__":
    data={}
    with open('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/measurement_results.json') as json_file:
        data = json.load(json_file)
    for measurements in data:
        num_of_measurements = []
        error = []
        for item in data[measurements]:
            num_of_measurements.append(item["num_measurements"])
            error.append(item["error"])
        plt.plot(num_of_measurements, error, label=measurements)
    plt.legend()
    plt.show()