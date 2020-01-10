#!/usr/bin/env python
import rospy
import math
import numpy as np
from uav_search.srv import *
from uav_search.msg import *

class WaypointGenerator:

    def __init__(self):    
        self.spacing = 10 #rospy.get_param("~spacing")
        self.height = 3 #rospy.get_param("~height")

    def generatePoints(self):
        if self.search_pattern == "spiral":
            return self.generatePointsSpiral()
        elif self.search_pattern == "levy":
            return self.generatePointsLevy()
        elif self.search_pattern == "lawn":
            return self.generatePointsLawn()

    def generatePointsSpiral(self, length_x, length_y):
        if length_y >= length_x:
            directions=[[1,0],[0,-float(length_x)/length_y],[-1,0],[0,float(length_x)/length_y]]
        else:
            directions=[[float(length_y)/length_x,0],[0,-1],[-float(length_y)/length_x,0],[0,1]]
        num_points=max(length_x/self.spacing, length_y/self.spacing)
        start = np.array([[self.spacing/2, 0, self.height, 0]])
        points = np.array(start)
        for i in (range(1,int(num_points))):
            a=points[i-1][0]
            b=points[i-1][1]
            y=points[i-1][0]+np.array(directions[(i-1)%4][0])*((num_points-i)*self.spacing)
            x=points[i-1][1]+np.array(directions[(i-1)%4][1])*((num_points-i)*self.spacing)
            xnew=a+b-x
            ynew=y+b-a
            orientation = np.arctan2(ynew-b, xnew-a)
            # set orientation for the previous point
            points[i-1][3] = orientation
            # comment above line and add orientation instead of zero to put orientation in current point
            points=np.vstack((points,np.array([[xnew, ynew, self.height, 0]])))
        return points

    def generatePointsLawn(self, length_x, length_y):
        directions=[[1,1],[1,-1]]
        num_points=length_x/self.spacing+1
        start = np.array([[0, 0, self.height, 0]])
        points = np.array(start)
        for i in (range(1,int(num_points))):
            a=points[i-1][0]
            b=points[i-1][1]
            x=points[i-1][0]+np.array(directions[(i-1)%2][0])*self.spacing
            y=points[i-1][1]+np.array(directions[(i-1)%2][1])*length_y
            orientation = np.arctan2(y-b, x-a)
            # set orientation for the previous point
            points[i-1][3] = orientation
            # comment above line and add orientation instead of zero to put orientation in current point
            points=np.vstack((points,np.array([[x, y, self.height, 0]])))
        return points

    def generatePointsLevy(self, length_x, length_y, sigma):
        start = np.array([[0, 0, self.height, 0]])
        # change this to get more points
        num_points = 50
        points = np.array(start)
        for i in range(1,num_points):
            a=points[i-1][0]
            b=points[i-1][1]
            x=a
            y=b
            count = 0
            while True:
                x = a
                y = b
                scaling = np.random.uniform(0,1)**(-1/sigma)
                spacing = self.spacing * scaling
                if i == 1:
                    angle = np.random.uniform(0, 0.25) * (2 * math.pi)
                else:
                    angle = np.random.uniform(0, 1) * (2 * math.pi)                
                direction_x = math.cos(angle)
                direction_y = math.sin(angle)
                x = x + direction_x * spacing
                y = y + direction_y * spacing
                if x >= 0 and x <= length_x and y >= 0 and y <= length_y:
                    break
                count += 1
                if count > 100:
                    break
            orientation = np.arctan2(y-b, x-a)
            # set orientation for the previous point
            points[i-1][3] = orientation
            # comment above line and add orientation instead of zero to put orientation in current point
            points=np.vstack((points,np.array([[x,y, self.height, 0]])))
        return points

    def handle_get_points(self, req):
        self.spacing = req.request.spacing
        self.height = req.request.height
        if req.request.pattern == "spiral":
            points = self.generatePointsSpiral(req.request.size_x, req.request.size_y)
        elif req.request.pattern == "levy":
            points = self.generatePointsLevy(req.request.size_x, req.request.size_y, 1)
        elif req.request.pattern == "lawn":
            points = self.generatePointsLawn(req.request.size_x, req.request.size_y)
        else:
            return
        response = PatternPoints()
        response.size_x = points.shape[0]
        response.size_y = points.shape[1]
        response.data = np.ndarray.flatten(points)
        return GetPointsResponse(response)

    def service_server(self):
        service = rospy.Service('get_points', GetPoints, self.handle_get_points)
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('uav_search_pattern_generator')
    gen = WaypointGenerator()
    gen.service_server()