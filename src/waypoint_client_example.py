#!/usr/bin/env python
from uav_search.srv import *
from uav_search.msg import *
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Rectangle

def draw_orientation(points, ax):
    for point in points:
        ax.arrow(point[0], point[1], 3*np.cos(point[3]), 3*np.sin(point[3]),lw=2, head_width=1, head_length=1.5, fc='k', ec='k')

def get_points_client(request):
    rospy.wait_for_service('get_points')
    try:
        pointse_service = rospy.ServiceProxy('get_points', GetPoints)
        response = pointse_service(request)
        return response.response
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return None


if __name__ == "__main__":
    print("Will do the test now")
    try:
        while True:
            pattern_type = raw_input("Pattern type: ")
            size_x = raw_input("Size_x: ")
            size_y = raw_input("Size_y: ")
            spacing = raw_input("Spacing: ")
            height = raw_input("Height:")
            # pattern_type = "spiral"
            # size_x = 100.0
            # size_y = 100.0
            # spacing = 5.0
            # height = 13.1
            request = PointsRequest()
            request.pattern = pattern_type
            request.size_x = float(size_x)
            request.size_y = float(size_y)
            request.spacing = float(spacing)
            request.height = float(height)
            points_response = get_points_client(request)
            dim_x = points_response.size_x
            dim_y = points_response.size_y
            data = np.asarray(points_response.data)
            points = np.reshape(data, (dim_x, dim_y))
            fig = plt.figure()
            ax = fig.add_subplot(111)
            axis_color = 'lightgoldenrodyellow'
            [line] = ax.plot(points[:,0], points[:,1], marker="o", markerfacecolor="r", linestyle='-')
            line.set_xdata(points[:, 0])
            line.set_ydata(points[:, 1])
            [p.remove() for p in reversed(ax.patches)]
            ax.add_patch(Rectangle((0, 0), request.size_x, request.size_y, alpha=0.2))
            draw_orientation(points, ax)
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw_idle()
            plt.show()
    except KeyboardInterrupt:
        print("Print exiting")