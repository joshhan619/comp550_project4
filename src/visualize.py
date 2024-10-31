import os
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt


def read_geometric_path(filename):
    all_data = []
    path = os.getcwd().split('/')[:-1]
    # Read from data folder
    path.append('data')
    path = '/'.join(path)
    with open(os.path.join(path, filename), "r") as f:
        for line in f:
            data = line.strip().split(' ')
            for i in range(len(data)):
                data[i] = float(data[i])

            all_data.append(data)

    return pd.DataFrame(all_data)

def visualize_pendulum(data1, data2, data3, planner):
    # 3 axes, one for each torque
    fig, ax= plt.subplots(1,3, sharey=True)
    ax[0].plot(data1[0], data1[1], 'b-')
    ax[0].title.set_text(f"Torque range [-3, 3]")

    ax[1].plot(data2[0], data2[1], 'b-')
    ax[1].title.set_text(f"Torque range [-5, 5]")

    ax[2].plot(data3[0], data3[1], 'b-')
    ax[2].title.set_text(f"Torque range [-10, 10]")

    # Visualize the same start and goal states for all graphs
    start = np.array([[-math.pi/2], [0]])
    goal = np.array([[math.pi/2], [0]])
    for i in range(3):
        ax[i].plot(start[0], start[1], 'bo')
        ax[i].plot(goal[0], goal[1], 'bo')
    fig.text(0.5, 0.04, "theta", ha="center")
    fig.text(0.04, 0.5, "omega", va="center", rotation="vertical")
    plt.suptitle(f"{planner} pendulum solution trajectory")
    plt.show()

def visualize_car(data1, data2, data3):
    fig, ax = plt.subplots(1,3, sharey=True)
    # Rectangular obstacles are tuples (x, y, w, h) where (x,y) is coordinate of lower left corner
    obstacles = [(-6, -10, 10, 4), (-6, 6, 10, 2), (-6, -4, 5, 8), (2, -4, 4, 8)]

    # Visualize the same start and goal states for all graphs
    start = np.array([[-5], [-5]])
    goal = np.array([[4], [5]])
    for i in range(3):
        ax[i].set_xbound(-6, 6)
        ax[i].set_ybound(-10, 10)
        for obs in obstacles:
            # Plot obstacles
            x,y,w,h = obs
            ax[i].plot([x, x], [y, y+h], 'r-')
            ax[i].plot([x, x+w], [y, y], 'r-')
            ax[i].plot([x+w, x+w], [y, y+h], 'r-')
            ax[i].plot([x, x+w], [y+h, y+h], 'r-')
        
        # Plot start and goal states
        ax[i].plot(start[0], start[1], 'bo')
        ax[i].plot(goal[0], goal[1], 'bo')

    # Plot RRT trajectory
    ax[0].plot(data1[0], data1[1], 'b-')
    ax[0].title.set_text(f"RRT car solution trajectory")
    for i in range(data1.shape[0]):
        theta = data1.iloc[i, 2]
        curr_pos = data1.iloc[i, :2].to_numpy().reshape((2, 1))
        # Draw an arrow in the direction of the robot's orientation
        arrow = np.array([[math.cos(theta)], [math.sin(theta)]])
        ax[0].arrow(curr_pos[0, 0], curr_pos[1, 0], arrow[0, 0], arrow[1, 0], head_width=0.1)

    # Plot KPIECE trajectory
    ax[1].plot(data2[0], data2[1], 'b-')
    ax[1].title.set_text(f"KPIECE car solution trajectory")
    for i in range(data2.shape[0]):
        theta = data2.iloc[i, 2]
        curr_pos = data2.iloc[i, :2].to_numpy().reshape((2, 1))
        # Draw an arrow in the direction of the robot's orientation
        arrow = np.array([[math.cos(theta)], [math.sin(theta)]])
        ax[1].arrow(curr_pos[0, 0], curr_pos[1, 0], arrow[0, 0], arrow[1, 0], head_width=0.1)

    # Plot RGRRT trajectory
    ax[2].plot(data3[0], data3[1], 'b-')
    ax[2].title.set_text(f"RGRRT car solution trajectory")
    for i in range(data3.shape[0]):
        theta = data3.iloc[i, 2]
        curr_pos = data3.iloc[i, :2].to_numpy().reshape((2, 1))
        # Draw an arrow in the direction of the robot's orientation
        arrow = np.array([[math.cos(theta)], [math.sin(theta)]])
        ax[2].arrow(curr_pos[0, 0], curr_pos[1, 0], arrow[0, 0], arrow[1, 0], head_width=0.1)

    
    fig.text(0.5, 0.04, "x", ha="center")
    fig.text(0.04, 0.5, "y", va="center", rotation="vertical")
    plt.show()

if __name__ == '__main__':
    # Visualize RRT planner solutions for each torque in the pendulum system
    planner = 'RRT'
    data1 = read_geometric_path(f"{planner}_pendulum_t3.txt")
    data2 = read_geometric_path(f"{planner}_pendulum_t5.txt")
    data3 = read_geometric_path(f"{planner}_pendulum_t10.txt")
    visualize_pendulum(data1, data2, data3, planner)

    # Visualize KPIECE planner solutions for each torque in the pendulum system
    planner = 'KPIECE'
    data4 = read_geometric_path(f"{planner}_pendulum_t3.txt")
    data5 = read_geometric_path(f"{planner}_pendulum_t5.txt")
    data6 = read_geometric_path(f"{planner}_pendulum_t10.txt")
    visualize_pendulum(data4, data5, data6, planner)

    # Visualize KPIECE planner solutions for each torque in the pendulum system
    planner = 'RGRRT'
    data7 = read_geometric_path(f"{planner}_pendulum_t3.txt")
    data8 = read_geometric_path(f"{planner}_pendulum_t5.txt")
    data9 = read_geometric_path(f"{planner}_pendulum_t10.txt")
    visualize_pendulum(data7, data8, data9, planner)

    # Visualize RRT and KPIECE solutions in the car system
    data10 = read_geometric_path("RRT_car.txt")
    data11 = read_geometric_path("KPIECE_car.txt")
    data12 = read_geometric_path("RGRRT_car.txt")
    visualize_car(data10, data11, data12)
