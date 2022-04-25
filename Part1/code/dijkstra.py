"""Dijkstra.py - Implementation of Dijkstra's algorithm."""


import time
import heapq

import map_traverser
import node_description


class Dijkstra(object):
    def __init__(self, obstacle_map):
        self.obstacle_map = obstacle_map
        self.open = []
        # heapq.heapify(self.open)
        self.closed = set()

        self.traverser = map_traverser.MapTraverser(obstacle_map)

        self.coord_to_lowest_cost = {}


    def generate_path(self, start_coordinate, goal_coordinate, update_callback):

        if self.obstacle_map.is_coordinate_occupied(start_coordinate):
            print("The starting coordinate " + str(start_coordinate) + " is inside an obstacle!")
            return [], -1

        if self.obstacle_map.is_coordinate_occupied(goal_coordinate):
            print("The goal coordinate " + str(goal_coordinate) + " is inside an obstacle!")
            return [], -1

        start_node = node_description.NodeDescription(start_coordinate, cost_to_come=0, parent_node=None)
        heapq.heappush(self.open, (start_node.get_cost(), start_node))

        # Get neighbor nodes
        while True: # TODO - stop when the heap is empty

            _, this_node = heapq.heappop(self.open)
            if this_node in self.closed:
                continue

            self.closed.add(this_node)

            if this_node.coordinates not in self.coord_to_lowest_cost:
                self.coord_to_lowest_cost[this_node.coordinates] = this_node
            elif self.coord_to_lowest_cost[this_node.coordinates].get_cost() > this_node.get_cost():
                self.coord_to_lowest_cost[this_node.coordinates] = this_node

            update_callback(this_node)

            if this_node.coordinates == goal_coordinate:
                return self.backtrack(this_node)

            free_neighbors = self.traverser.get_valid_neighbors(this_node)
            new_free_neighbors = [x for x in free_neighbors if x not in self.closed]

            for neighbor in new_free_neighbors:
                heapq.heappush(self.open, (neighbor.get_cost(), neighbor))


    def backtrack(self, goal_node):

        backtrack_nodes = []
        this_node = goal_node
        while this_node:
            backtrack_nodes.insert(0, this_node)
            this_node = self.coord_to_lowest_cost[this_node.coordinates].parent_node

        return backtrack_nodes, goal_node.get_cost()


if __name__ == '__main__':
    import obstacle_map
    o_map = obstacle_map.generate_obstacle_map()

    d = Dijkstra(o_map)
    path, cost = d.generate_path((1, 1), (20, 20), lambda x: None)

    print("Cost to get from start point to goal point: " + str(cost))
    print("The path:")
    for node in path:
        print(node)
