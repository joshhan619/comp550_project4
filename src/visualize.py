import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def read_geometric_path(filename):
    all_data = []
    path = os.getcwd().split('/')[:-1]
    path.append('data')
    path = '/'.join(path)
    with open(os.path.join(path, filename), "r") as f:
        for line in f:
            data = line.strip().split(' ')
            data[0] = float(data[0])
            data[1] = float(data[1])
            all_data.append(data)

    return pd.DataFrame(all_data, columns=["theta", "omega"])

def visualize_pendulum(data1, data2, data3, planner):
    fig, ax= plt.subplots(1,3, sharey=True)
    ax[0].plot(data1["theta"], data1["omega"], 'b-')
    ax[0].title.set_text(f"Torque range [-3, 3]")

    ax[1].plot(data2["theta"], data2["omega"], 'b-')
    ax[1].title.set_text(f"Torque range [-5, 5]")

    ax[2].plot(data3["theta"], data3["omega"], 'b-')
    ax[2].title.set_text(f"Torque range [-10, 10]")

    start = data1.iloc[0].to_numpy()
    goal = data1.iloc[data1.shape[0]-1].to_numpy()
    for i in range(3):
        ax[i].plot(start[0], start[1], 'bo')
        ax[i].plot(goal[0], goal[1], 'bo')
    fig.text(0.5, 0.04, "theta", ha="center")
    fig.text(0.04, 0.5, "omega", va="center", rotation="vertical")
    plt.suptitle(f"{planner} pendulum solution trajectory")
    plt.show()
        

if __name__ == '__main__':
    planner = 'RRT'
    data1 = read_geometric_path(f"{planner}_pendulum_t3.txt")
    data2 = read_geometric_path(f"{planner}_pendulum_t5.txt")
    data3 = read_geometric_path(f"{planner}_pendulum_t10.txt")
    visualize_pendulum(data1, data2, data3, planner)

    planner = 'KPIECE'
    data4 = read_geometric_path(f"{planner}_pendulum_t3.txt")
    data5 = read_geometric_path(f"{planner}_pendulum_t5.txt")
    data6 = read_geometric_path(f"{planner}_pendulum_t10.txt")
    visualize_pendulum(data4, data5, data6, planner)