#!/usr/bin/python
import rospy
from my_planner.srv import MakePlan, MakePlanResponse
import numpy as np
import heapq as hq
import math
from itertools import count


class Dijkstra(object): 
    
    def __init__(self):
        self.make_plan_service = rospy.Service("/move_base/GlobalPlannerPython/make_plan", MakePlan, self.make_plan)

    @staticmethod
    def retrieve_neighbours(current_index, map_width, map_height):
        current_x, current_y = (current_index % map_width, current_index // map_width)

        check_coords = lambda x, y: 0 <= x < map_width and 0 <= y < map_height and not (x == current_x and y == current_y)
        coord_to_idx = lambda x, y: y * map_width + x

        return [coord_to_idx(current_x + x_diff, current_y + y_diff) for x_diff in [-1, 0, 1] for y_diff in [-1, 0, 1]
                if check_coords(current_x + x_diff, current_y + y_diff)]

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

            neighbours = self.retrieve_neighbours(current_index, map_width, map_height)
            # print("For %d there are %d neigbours" % (current_index, len(neighbours)))
            for neighbour_idx in neighbours:
                n_cost = 1 + costmap[neighbour_idx] + distance
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
