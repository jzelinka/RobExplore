import matplotlib.pyplot as plt
import hexapod_robot.HexapodRobot as hexapod
import hexapod_explorer.HexapodExplorer as explorer
import hexapod_robot.HexapodController as controller
import argparse
import pathlib
from messages import *
import time

import copy

SIM_IDXS = [0, 1]
MIN_POS = True
USE_MUTAL_INFO = True

SIMULATION_STEP = 0.5
# crutial for the simulation
ROBOT_SIZE = 0.5
SAVE_FIGS = False

def poses_near(pose1: Pose, pose2: Pose) -> bool:
    x1 = pose1.position.x
    y1 = pose1.position.y
    x2 = pose2.position.x
    y2 = pose2.position.y
    return (x1 - x2)**2 + (y1 - y2)**2 < ROBOT_SIZE ** 2

class Explorer:
    def __init__(self,
                 id: int,
                 reactive: bool = True,
                 kmeans: bool = True,
                 use_mutal_info: bool = False,
                 ax: plt.Axes = None,
                 min_pos = False) -> None:
        self.robot = hexapod.HexapodRobot(id)
        self.goal = None
        self.initial_goal = None
        self.path = None
        self.name = "Explorer " + str(id)
        self.reactive = reactive
        self.kmeans = kmeans
        self.other_robots = []
        self.all_goals_explored = False
        self.use_mutal_info = use_mutal_info
        self.ax = ax
        self.min_pos = min_pos
    
    def set_other_robots(self, other_robots):
        self.other_robots = other_robots
    
    def initial_rotation(self) -> None:
        cmd = Twist()
        cmd.angular.z = 5
        self.robot.move(cmd)


    def start(self):
        self.robot.turn_on()
        self.robot.start_navigation()

        while not self.odometry_ready():
            pass

    def stop(self):
        self.robot.stop_navigation()
        self.robot.turn_off()

    def set_and_follow_goal(self, new_goal: Pose, initial_goal: bool = False):
        print(self.name, "Setting goal.")
        print(new_goal.position.x, new_goal.position.y)
        if initial_goal:
            self.initial_goal = new_goal

        self.goal = new_goal
        if self.reactive:
            self.robot.goto_reactive(self.goal)
        else:
            self.robot.goto(self.goal)

    def get_laser_scan(self) -> LaserScan:
        return self.robot.laser_scan_

    def get_odometry(self) -> Odometry:
        return self.robot.odometry_

    def odometry_ready(self) -> bool:
        ret = self.get_odometry() is not None
        ret = ret and self.get_laser_scan() is not None
        return ret
    
    def update_gridmap(self, gridmap: OccupancyGrid, expl: explorer.HexapodExplorer) -> OccupancyGrid:
        if self.odometry_ready():
            return expl.fuse_laser_scan_resizing(gridmap, self.get_laser_scan(), self.get_odometry())
        return gridmap
    
    def update_path(self, gridmap: OccupancyGrid, expl: explorer.HexapodExplorer) -> bool:
        start = self.prepare_start(gridmap)
        inflated_map = expl.grow_obstacles(gridmap, ROBOT_SIZE, robot = start)

        final_goal = copy.deepcopy(self.goal)
        if self.path is not None and len(self.path.poses) > 0:
            final_goal = copy.deepcopy(self.path.poses[-1])

        path = expl.plan_path(inflated_map, start, final_goal)

        if path is None:
            print("Path doesn't exist any more, have to replan.")
            self.path = None
            self.goal = None
            return False

        simple_path = expl.simplify_path(inflated_map, path)

        if simple_path is not None:
            for pose in simple_path.poses:
                pose.position.x += gridmap.origin.position.x
                pose.position.y += gridmap.origin.position.y
            
            potential_goal = simple_path.poses[1]

            if poses_near(potential_goal, self.goal):
                return True

            simple_path.poses = simple_path.poses[1:]
            self.goal = simple_path.poses.pop(0)
            self.path = simple_path
            return True
        else:
            self.path = None
            self.goal = None
            return False

    def update(self, gridmap: OccupancyGrid, expl: explorer.HexapodExplorer) -> None:
        if self.all_goals_explored:
            print(self.name, "All goals explored.")
            return

        # robot reached the goal
        if self.robot.navigation_goal is None:
            print("Robot reached the goal. Have to find a new one.")
            self.goal = None

        if self.goal is not None and self.goal != self.initial_goal:
            if self.reactive:
                # don't update path if using reactive
                return

            old_goal = self.goal
            if self.update_path(gridmap, expl):
                if old_goal != self.goal:
                    self.set_and_follow_goal(self.goal)
                return
            # else continue to find new path

        # called every time step to check if i need to set a new goal
        # check if goal is none or the initial
        if self.goal is None and self.path is not None and len(self.path.poses) > 0:
            print("Getting a goal from path.")
            new_goal = self.path.poses.pop(0)
            self.set_and_follow_goal(new_goal)
            return
        
        # select a new goal from a path if the search was successful
        if self.find_path(gridmap, expl):
            print("Found a path.")
            new_goal = self.path.poses.pop(0)
            self.set_and_follow_goal(new_goal)
        else:
            print("didnt find a path, probably explored all goals.")
            if not self.all_goals_explored:
                print("Exploring all goals.")
                self.all_goals_explored = True
                # just stay in place as it would other wise exhaust the planning
                new_goal = copy.deepcopy(self.get_odometry().pose)
                self.set_and_follow_goal(new_goal)
                return

    def goal_already_in_other_path(self, goal: Pose)->bool:
        if len(self.other_robots) == 0:
            return False
        
        for robot in self.other_robots:
            if robot.path is not None and len(robot.path.poses) > 0:
                for pose in robot.path.poses:
                    if poses_near(pose, goal):
                        return True

            if robot.goal is not None and poses_near(robot.goal, goal):
                return True

        return False
    
    def plot_restart(self):
        self.ax.cla()
        self.ax.set_title(self.name + " path selection")
    
    def plot_inflated_map(self, inflated_map: OccupancyGrid):
        if self.ax is None:
            return
        inflated_map.plot(self.ax)
    
    def plot_goals(self, possible_goals):
        if self.ax is None:
            return
        gx = [goal.position.x for goal in possible_goals]
        gy = [goal.position.y for goal in possible_goals]
        mutal_info = [goal.orientation.z for goal in possible_goals]
        if self.use_mutal_info:
            self.ax.scatter(gx, gy, c=mutal_info, s=20)
            if not self.min_pos:
                for (x, y, mi) in zip(gx, gy, mutal_info):
                    self.ax.annotate(str(round(mi, 2)), (x, y), color='magenta')
        else:
            self.ax.scatter(gx, gy, c="g", s=20)
    
    def plot_goal_rank(self, possible_goal, rank):
        if self.ax is None:
            return
        px = possible_goal.position.x
        py = possible_goal.position.y
        self.ax.annotate(str(rank), (px, py), color='magenta')
    
    def plot_path(self, path: Path, c = 'r'):
        if self.ax is None:
            return
        x = [pose.position.x for pose in path.poses]
        y = [pose.position.y for pose in path.poses]
        self.ax.plot(x, y, c)
    
    def get_simple_path(self, infl_map: OccupancyGrid, expl: explorer.HexapodExplorer, start: Pose, goal: Pose) -> Path:
        # just to make sure
        goal = copy.deepcopy(goal)
        path = expl.plan_path(infl_map, start, goal)
        if path is None:
            return None

        simple_path = expl.simplify_path(infl_map, path)

        if simple_path is None:
            return None

        for pose in simple_path.poses:
            pose.position.x += infl_map.origin.position.x
            pose.position.y += infl_map.origin.position.y

        return simple_path


    def find_path(self, gridmap: OccupancyGrid, expl: explorer.HexapodExplorer) -> bool:
        # returns true if the search was successful
        inflated_map = expl.grow_obstacles(gridmap, ROBOT_SIZE, robot=self.prepare_start(gridmap))
        possible_goals = expl.find_inf_frontiers(gridmap, self.kmeans)
        possible_goals = [goal for goal in possible_goals if not poses_near(goal, self.get_odometry().pose)]

        if len(self.other_robots) == 0:
            self.min_pos = False

        self.plot_restart()
        self.plot_inflated_map(inflated_map)
        self.plot_goals(possible_goals)

        if possible_goals is None:
            return False

        shortest_path = None
        shortest_path_length = None
        best_mutal_info = None
        best_min_pos = None

        for goal in possible_goals:
            simple_path = self.get_simple_path(inflated_map, expl, self.prepare_start(gridmap), goal)
            if simple_path is None:
                continue
            cur_len = explorer.path_length(simple_path)

            count_robots_closer = 0
            if self.min_pos:
                robot: Explorer
                for robot in self.other_robots:
                    inflated_map = expl.grow_obstacles(gridmap, ROBOT_SIZE, robot.prepare_start(gridmap))
                    simple_path2 = robot.get_simple_path(inflated_map, expl, robot.prepare_start(gridmap), goal)
                    assert inflated_map.origin.position.x == gridmap.origin.position.x, "Origins are not the same."
                    if simple_path2 is not None and explorer.path_length(simple_path2) < cur_len:
                        count_robots_closer += 1
                self.plot_goal_rank(goal, count_robots_closer)

            self.plot_path(simple_path)

            if self.goal_already_in_other_path(goal):
                print("Goal already in other path. Skipping.")
                continue

            # init
            if shortest_path is None:
                shortest_path = simple_path
                shortest_path_length = cur_len
                best_mutal_info = goal.orientation.z
                best_min_pos = count_robots_closer
                continue
            
            # select based on min pos
            if self.min_pos: 
                # best rank
                if count_robots_closer < best_min_pos:
                    shortest_path = simple_path
                    shortest_path_length = cur_len
                    best_mutal_info = goal.orientation.z
                    best_min_pos = count_robots_closer
                    continue
                # worst rank => skip
                elif count_robots_closer > best_min_pos:
                    continue
            
            # select based on mutal info
            if self.use_mutal_info:
                if goal.orientation.z > best_mutal_info:
                    shortest_path = simple_path
                    shortest_path_length = cur_len
                    best_mutal_info = goal.orientation.z
                    continue
            # select based on length
            elif cur_len < shortest_path_length:
                shortest_path = simple_path
                shortest_path_length = cur_len

        if shortest_path is None:
            return False
        
        self.plot_path(shortest_path, 'g')

        shortest_path.poses = shortest_path.poses[1:]
        self.path = shortest_path
        return True

    def prepare_start(self, gridmap: OccupancyGrid) -> Pose:
        origin_x = gridmap.origin.position.x
        origin_y = gridmap.origin.position.y
        pose = copy.deepcopy(self.get_odometry().pose)
        pose.position.x -= origin_x
        pose.position.y -= origin_y
        return pose
    
    def visualize_robot(self, ax: plt.Axes, gridmap: OccupancyGrid) -> None:
        # pose = copy.deepcopy(self.get_odometry().pose)
        pose = self.get_odometry().pose
        goal = self.goal

        ax.scatter(pose.position.x, pose.position.y, c='b', s=20)
        ax.annotate(self.name, (pose.position.x, pose.position.y), textcoords="offset points", xytext=(0, 5))
        if self.goal is not None:
            ax.scatter(goal.position.x, goal.position.y, c='r', s=20)
            ax.annotate(self.name + " goal", (goal.position.x, goal.position.y), textcoords="offset points", xytext=(0, 5))
            if self.path is not None:
                px = [pose.position.x for pose in self.path.poses]
                py = [pose.position.y for pose in self.path.poses]
                ax.plot(px, py, 'g')
                if len(px) > 0:
                    ax.plot([goal.position.x, px[0]], [goal.position.y, py[0]], 'g')
            ax.plot([goal.position.x, pose.position.x], [goal.position.y, pose.position.y], 'g')
            circle = plt.Circle((goal.position.x, goal.position.y), controller.DELTA_DISTANCE, color='r', fill=False)
            ax.add_artist(circle)
        gridmap.plot(ax)
    
    def vis_replanning(self, ax:plt.Axes, gridmap: OccupancyGrid, expl: explorer.HexapodExplorer) -> None:
        ax.cla()
        start = self.prepare_start(gridmap)
        inflated_map = expl.grow_obstacles(gridmap, ROBOT_SIZE, robot=start)

        final_goal = copy.deepcopy(self.goal)
        if self.path is not None and len(self.path.poses) > 0:
            final_goal = copy.deepcopy(self.path.poses[-1])

        inflated_map.plot(ax)
        ax.scatter(final_goal.position.x, final_goal.position.y, c='r', s=20)
        start = self.prepare_start(gridmap)

        path = expl.plan_path(inflated_map, start, final_goal)

        if path is not None:
            simple_path = expl.simplify_path(inflated_map, path)

            if simple_path is None:
                print("No simple path.")
                return

            start = self.get_odometry().pose
            ax.scatter(start.position.x, start.position.y, c='b', s=20)

            if simple_path is not None:
                map_orig_x = gridmap.origin.position.x
                map_orig_y = gridmap.origin.position.y
                px = [pose.position.x + map_orig_x for pose in simple_path.poses]
                py = [pose.position.y + map_orig_y for pose in simple_path.poses]
                ax.plot(px, py, 'g')
        else:
            print("No path found.===========")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Explorer")
    parser.add_argument("-s", "--save", action="store_true", help="Save the figures to the figs directory.", default=False)
    parser.add_argument("-m", "--mutal_info", action="store_true", help="Use mutal info for goal selection.", default=False)
    parser.add_argument("-p", "--min-pos", action="store_true", help="Use min pos navigation.", default=False)

    args = parser.parse_args()
    SAVE_FIGS = args.save
    USE_MUTAL_INFO = args.mutal_info
    MIN_POS = args.min_pos

    if SAVE_FIGS:
        pathlib.Path("figs").mkdir(exist_ok=True)

    ax_size = (5, 6)

    expl = explorer.HexapodExplorer()
    count_axes = 1 + len(SIM_IDXS)
    fig, ax = plt.subplots(1, count_axes, figsize=(ax_size[0] * count_axes, ax_size[1]))

    explorers = []
    print("Starting explorers.")
    for i, sim_idx in enumerate(SIM_IDXS):
        explorers.append(Explorer(sim_idx,
                                  reactive=False,
                                  min_pos=MIN_POS,
                                  use_mutal_info = USE_MUTAL_INFO,
                                  ax = ax[i + 1]))

    # set the other robots for each explorer for minpos and greedy
    for i, ex in enumerate(explorers):
        ex.set_other_robots(explorers[:i] + explorers[i+1:])

    ex : Explorer
    for ex in explorers:
        ex.start()

    print("Robots started.")
    while not all([ex.odometry_ready() for ex in explorers]):
        pass
    gridmap = None

    image_idx = 0
    start_time = time.time()
    plt.ion()
    print("Getting initial laser scans.")
    while (time.time() - start_time) < 1.0:
        ax[0].cla()
        for ex in explorers:
            ex.initial_rotation()
            if ex.odometry_ready():
                gridmap = ex.update_gridmap(gridmap, expl)
                ex.visualize_robot(ax[0], gridmap)
        
        if SAVE_FIGS:
            fname = "figs/vis_{:04d}.png".format(image_idx)
            plt.savefig(fname)
            image_idx += 1

        fig.suptitle("Time: {:.{}f} s".format(time.time() - start_time, 2))
        plt.show()
        plt.pause(SIMULATION_STEP)

    print("Starting the exploration.")
    while not all([ex.all_goals_explored for ex in explorers]):
        for ex in explorers:
            if ex.odometry_ready():
                gridmap = expl.fuse_laser_scan_resizing(gridmap, ex.get_laser_scan(), ex.get_odometry())
                ex.update(gridmap, expl)

        ax[0].cla()
        ax[0].set_title("Real time vis")
        for ex in explorers:
            ex.visualize_robot(ax[0], gridmap)

        if SAVE_FIGS:
            fname = "figs/vis_{:04d}.png".format(image_idx)
            plt.savefig(fname)
            image_idx += 1
        fig.suptitle("Time: {:.{}f} s".format(time.time() - start_time, 2))
        plt.tight_layout()
        plt.show()
        plt.pause(SIMULATION_STEP)


    print("Exploration finished.")
    time.sleep(1)
    for ex in explorers:
        ex.stop()

    print("Explorers stopped.")