#!/usr/bin/python
import rospy
from my_planner.srv import MakePlan, MakePlanResponse
import numpy as np
import heapq as hq
import math
from itertools import count

diagonal_distance = 2 ** .5
class Dijkstra(object): 
    
    def __init__(self):
        self.make_plan_service = rospy.Service("/move_base/GlobalPlannerPython/make_plan", MakePlan, self.make_plan)

    @staticmethod
    def idx_to_coord(idx, map_width):
        return idx % map_width, idx // map_width

    @staticmethod
    def coord_to_idx(x, y, map_width):
        return y * map_width + x

    @staticmethod
    def retrieve_neighbours(current_index, map_width, map_height):
        x, y = Dijkstra.idx_to_coord(current_index, map_width)

        orthogonal_neighbours = []
        diagonal_neighbours = []
        for d_x in [-1, 0, 1]:
            for d_y in [-1, 0, 1]:
                if 0 <= (x + d_x) < map_width and 0 <= (y + d_y) < map_height and not ((x + d_x) == x and (y + d_y) == y):
                    neighbour_idx = Dijkstra.coord_to_idx(x + d_x, y + d_y, map_width)
                    if 0 in [d_x, d_y]:
                        orthogonal_neighbours.append(neighbour_idx)
                    else:
                        diagonal_neighbours.append(neighbour_idx)

        return orthogonal_neighbours, diagonal_neighbours

    def calculate_path_with_dijkstra(self, start_index, goal_index, map_width, map_height, costmap):
        """
        Implementation of Dijkstra algorithm inspired by
        https://networkx.github.io/documentation/latest/_modules/networkx/algorithms/shortest_paths/astar.html#astar_path

        """
        c = count()
        heap = [(0, next(c), start_index, 0, None)]

        # Maps enqueued nodes to distance of discovered paths.
        # We avoid inserting the node into the queue too many times.
        enqueued = {}

        # Maps explored nodes to parent closest to the source.
        explored = {}
        while heap:
            # Pop the smallest item from queue.
            _, _, current_index, distance, parent_index = hq.heappop(heap)

            if current_index == goal_index:
                path = [current_index]
                node = parent_index
                while node is not None:
                    path.append(node)
                    node = explored[node]
                path.reverse()
                return path

            if current_index in explored:
                # Do not override the parent of starting node
                if explored[current_index] is None:
                    continue

                # Skip bad paths that were enqueued before finding a better one
                q_cost = enqueued[current_index]
                if q_cost < distance:
                    continue

            explored[current_index] = parent_index

            orth_neighbours, diag_neighbours = self.retrieve_neighbours(current_index, map_width, map_height)
            # print("For %d there are %d neigbours" % (current_index, len(neighbours)))
            neighbours = zip(orth_neighbours, [False] * len(orth_neighbours)) + \
                         zip(diag_neighbours, [True] * len(diag_neighbours))

            for neighbour_idx, diagonal in neighbours:
                n_cost = costmap[neighbour_idx] + distance + (diagonal_distance if diagonal else 1)
                if neighbour_idx in enqueued:
                    q_cost = enqueued[neighbour_idx]
                    if q_cost <= n_cost:
                        continue
                enqueued[neighbour_idx] = n_cost
                hq.heappush(heap, (n_cost, next(c), neighbour_idx, n_cost, current_index))
        return "went wrong"

    def make_plan(self, req):
        ## This is the data you get from the request
        costmap = req.costmap_ros   ## The costmap, a single array version of an image
        width = req.width
        height = req.height
        start_index = req.start
        goal_index = req.goal

        # Calculate the path
        path = self.calculate_path_with_dijkstra(start_index, goal_index, width, height, costmap)

        # Make a response object
        resp = MakePlanResponse()
        resp.plan = path
        
        print 'done'
        return resp


if __name__ == "__main__":
    rospy.init_node("dijkstra_planner")
    
    dijkstra = Dijkstra()
    
    rospy.spin()
