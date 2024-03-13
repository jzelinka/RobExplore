import matplotlib.pyplot as plt
import hexapod_robot.HexapodRobot as hexapod
import hexapod_explorer.HexapodExplorer as explorer
import hexapod_robot.HexapodController as controller
from messages import *
import time

from Explorer import *

USE_MUTAL_INFO = True

if __name__ == "__main__":
    rand_goal = Pose(Vector3(0, -3.5, 0), Quaternion(1, 0, 0, 0))
    expl = explorer.HexapodExplorer()

    fig, ax = plt.subplots(1, 3, figsize=(10, 6))
    print("Starting explorers.")
    ex0 = Explorer(0, reactive=False, use_mutal_info=USE_MUTAL_INFO, ax = ax[1])

    print("Explorers turning on.")
    ex0.start()

    while not ex0.odometry_ready():
        pass

    gridmap = None
    start_time = time.time()

    plt.ion()

    # allow the robots to collect some initial data into the gridmap
    print("Getting initial laser scans.")
    rotation_done = False
    while (time.time() - start_time) < 3.0:
        ex0.initial_rotation()
        gridmap = ex0.update_gridmap(gridmap, expl)
        ax[0].cla()
        ex0.visualize_robot(ax[0], gridmap)
        fig.suptitle("Time: {:.{}f} s".format(time.time() - start_time, 2))
        plt.show()
        plt.pause(0.5)

    print("Starting the exploration.")
    while not ex0.all_goals_explored:
        # pay attention to put everything inside odometry ready()
        if ex0.odometry_ready():
            gridmap = expl.fuse_laser_scan_resizing(gridmap, ex0.get_laser_scan(), ex0.get_odometry())
            ex0.update(gridmap, expl)

        # start plotting the gridmap
        ax[0].cla()
        ex0.visualize_robot(ax[0], gridmap)
        ex0.vis_replanning(ax[2], gridmap, expl)
        fig.suptitle("Time: {:.{}f} s".format(time.time() - start_time, 2))
        plt.show()
        plt.pause(0.5)

    print("Exploration finished.")
    time.sleep(5)
    ex0.stop()
    print("Explorers stopped.")