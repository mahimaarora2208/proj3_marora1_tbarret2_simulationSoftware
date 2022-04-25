"""Map_Traverser.py - Contains methods for traversing the map in 8 directions."""


import math
import node_description


RELATIVE_THETAS = [60, 30, 0, -30, -60]


class MapTraverser(object):
    def __init__(self, obstacle_map, step_size, goal_coordinate):
        self.obstacle_map = obstacle_map
        self.step_size = step_size
        self.goal_coord = goal_coordinate


    def get_valid_neighbors(self, node):
        x, y = node.coordinates
        current_theta = node.theta

        valid_neighbors = []
        for dtheta in RELATIVE_THETAS:
            new_theta = current_theta + dtheta
            new_theta = new_theta % 360
            dx = self.step_size * math.cos(new_theta * math.pi / 180)
            dy = self.step_size * math.sin(new_theta * math.pi / 180)
            newx = x + dx
            newy = y + dy

            if not self.obstacle_map.is_coordinate_occupied((newx, newy)):
                new_cost_to_come = node.cost_to_come + self.step_size

                x_to_goal = self.goal_coord[0] - newx
                y_to_goal = self.goal_coord[1] - newy
                cost_to_go = math.sqrt(x_to_goal**2 + y_to_goal**2)

                new_node_description = node_description.NodeDescription((newx, newy),
                                                                        theta=new_theta,
                                                                        cost_to_come=new_cost_to_come,
                                                                        cost_to_go=cost_to_go,
                                                                        parent_node=node)
                valid_neighbors.append(new_node_description)

        return valid_neighbors
