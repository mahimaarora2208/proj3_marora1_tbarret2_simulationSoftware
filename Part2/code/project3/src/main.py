#!/usr/bin/env python
"""main.py - The top-level code for ENPM661 Project 3"""


from math import atan2, pi, sqrt
from geometry_msgs.msg import Twist
import time
import a_star
import obstacle_map
import visualization
import rospy
import bot_controller 

# TODO - find this constant somewhere in the documentation
TURTLEBOT_ROBOT_RADIUS = 0.033
STEP_SIZE = 10 
print("Please enter the start and goal poses as comma-separated integers, such as: 50, 220, 30")
start_point_str = input("Please input the starting point and orientation: ")
start_coord = start_point_str.split(",")
start_x = int(start_coord[0].strip())
start_y = int(start_coord[1].strip())
start_x += 5
start_y += 5
start_theta = int(start_coord[2].strip())
start_point = (start_x, start_y), start_theta

current_x_pos = start_x
current_y_pos = start_y
# start_point = (150, 100), 0

goal_point_str = input("Please input the goal point: ")
goal_coord = goal_point_str.split(",")
goal_x = int(goal_coord[0].strip())
goal_y = int(goal_coord[1].strip())
goal_x += 5
goal_y += 5
goal_point = (goal_x, goal_y)

# goal_point = (250, 100)

rpm_str = input("Please enter the desired rpms: ")
rpms = rpm_str.split(",")
rpm1 = int(rpms[0])
rpm2 = int(rpms[1])

clearance_str = input("Finally, please enter the desired clearance: ")
clearance = float(clearance_str.strip())

print("Start point: " + str(start_point))
print("Goal point: " + str(goal_point))


o_map = obstacle_map.generate_obstacle_map(clearance + TURTLEBOT_ROBOT_RADIUS)
# o_map = obstacle_map.generate_obstacle_map(0) # TODO - what should we put here?

visualizer = visualization.PathPlanningVisualizer()
visualizer.draw_obstacle_map(o_map)

a = a_star.AStar(o_map, goal_point, rpm1, rpm2)
t0 = time.time()
path, cost, motor_inputs = a.generate_path(start_point, goal_point, lambda x: visualizer.draw_visited_node(x))
if path:
    print("Found a path from start to goal point!")

print("Time required: " + str(time.time() - t0))

visualizer.draw_path(path)
visualizer.write_video_file()

# Allow enough time to finish flushing the buffer to the AVI output file.
time.sleep(2)

for motor_input in motor_inputs:
    print(motor_input)

# Controller for bot
controller = bot_controller.BotController()
new_list = []
for val in path:
    new_list.append((val.coordinates[0] - 5, val.coordinates[1] - 5))

x = range(0, len(new_list), STEP_SIZE)
for n in x:
    controller.go_to_goal(new_list[n][0], new_list[n][1])
    print('POINTS TAKEN FOR REACHING GOAL: ', (new_list[n][0], new_list[n][1]))
    time.sleep(5)


