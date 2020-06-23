import numpy as np 
import json
import matplotlib.pyplot as plt

if __name__ == "__main__":
    data={}
    colors=['b', 'g', 'r', 'y']
    rand_colors = ['m', 'y', 'k', 'c']
    with open('/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_server/results/measurement_results.json') as json_file:
        data = json.load(json_file)
    for measurements in data:
        num_of_measurements = []
        error = []
        for item in data[measurements]:
            num_of_measurements.append(item["num_measurements"])
            error.append(item["error"])
        line_type = '-'
        if "Random" in measurements:
            line_type = rand_colors.pop() + '--'
        else:
            line_type = colors.pop() + '-'

        plt.plot(num_of_measurements, error, line_type, label=measurements)
    plt.xlabel('number of measurements evaluated')
    plt.xticks(np.arange(0, 401, 40))
    plt.ylabel('Root mean squared error in m')
    plt.legend()
    plt.show()