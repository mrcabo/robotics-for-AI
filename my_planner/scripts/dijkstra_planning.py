#!/usr/bin/python
import rospy
from my_planner.srv import MakePlan, MakePlanResponse
import numpy as np
import heapq
import math


class Dijkstra(object): 
    
    def __init__(self):
        self.make_plan_service = rospy.Service("/move_base/GlobalPlannerPython/make_plan", MakePlan, self.make_plan)

    def make_plan(self, req):
        ## This is the data you get from the request
        costmap = req.costmap_ros   ## The costmap, a single array version of an image
        width = req.width
        height = req.height
        map_size = height * width
        start_index = req.start
        goal_index = req.goal

        print(costmap)
        print(start_index)
        print(goal_index)

                
        #make a response object
        resp = MakePlanResponse()
        resp.plan = [] #your plan, which should contain the index values representing the path
        
        print 'done'
        return resp


if __name__ == "__main__":
    rospy.init_node("dijkstra_planner")
    
    dijkstra = Dijkstra()
    
    rospy.spin()
