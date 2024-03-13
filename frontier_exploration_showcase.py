import matplotlib.pyplot as plt
import hexapod_robot.HexapodRobot as hexapod
import hexapod_explorer.HexapodExplorer as explorer
import hexapod_robot.HexapodController as controller
from messages import *
import time

from Explorer import *

SIMULATION_DURAION = 20


def show_goals(gridmap: OccupancyGrid, ax0: plt.Axes, ax1:plt.Axes, ax2: plt.Axes) -> None:
    goals_kmeans = expl.find_free_edge_frontiers(gridmap, True)
    goals_normal = expl.find_free_edge_frontiers(gridmap, False)
    goals_mutal = expl.find_inf_frontiers(gridmap, 1.0)

    ax1.cla()
    gridmap.plot(ax1)
    ax1.scatter([goal.position.x for goal in goals_kmeans], [goal.position.y for goal in goals_kmeans], c='m', s=40)

    ax0.cla()
    gridmap.plot(ax0)
    ax0.scatter([goal.position.x for goal in goals_normal], [goal.position.y for goal in goals_normal], c='y', s=40)

    ax2.cla()
    gridmap.plot(ax2)
    mutal_info = [goal.orientation.z for goal in goals_mutal]
    px = [goal.position.x for goal in goals_mutal]
    py = [goal.position.y for goal in goals_mutal]
    ax2.scatter(px, py, c=mutal_info, cmap='viridis', s=40)
    for (x, y, mutal_info) in zip(px, py, mutal_info):
        ax2.annotate(str(round(mutal_info, 2)), (x, y), color='magenta')


if __name__ == "__main__":
    rand_goal = Pose(Vector3(0, -3.5, 0), Quaternion(1, 0, 0, 0))
    expl = explorer.HexapodExplorer()

    ex0 = Explorer(0, reactive=True)
    ex0.start()
    gridmap = None
    start_time = time.time()

    plt.ion()
    fig, ax = plt.subplots(1, 4, figsize=(12, 4))

    print("Starting the exploration.")
    ex0.set_and_follow_goal(rand_goal, initial_goal=True)
    while (time.time() - start_time) < SIMULATION_DURAION:
        if ex0.odometry_ready():
            gridmap = expl.fuse_laser_scan_resizing(gridmap, ex0.get_laser_scan(), ex0.get_odometry())

        ax[0].cla()
        ex0.visualize_robot(ax[0], gridmap)
        show_goals(gridmap, ax[1], ax[2], ax[3])

        ax[0].set_title("Robot position and goal")
        ax[1].set_title("Normal frontiers")
        ax[2].set_title("K-Means frontiers")
        ax[3].set_title("Mutal information frontiers")
        fig.suptitle("Time: {:.{}f} s".format(time.time() - start_time, 2))
        plt.show()
        plt.pause(0.5)

    ex0.stop()
    print("Explorers stopped.")