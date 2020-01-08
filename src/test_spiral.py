import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.widgets import Slider, Button, RadioButtons, Rectangle

xs=[]
ys=[]

fig = plt.figure()
ax = fig.add_subplot(111)
axis_color = 'lightgoldenrodyellow'
#ax.set_xlim([-100, 100])
#ax.set_ylim([-100,100])

length=10
spacing=1
#directions=[[1,0],[0,-1],[-1,0],[0,1]]
directions=[[1,1],[1,-1]]
startinglabel="corner"
txt=plt.gcf().text(0.1, 0.9, 'Total distance='+"{0:.2f}".format(0)+"m", fontsize=14)
def updatePlot():
    points=generate()
    line.set_xdata(points[:, 0])
    line.set_ydata(points[:, 1])
    [p.remove() for p in reversed(ax.patches)]
    global startinglabel
    if startinglabel == "center":
        ax.add_patch(Rectangle((0-int(amp_slider.val)/2, 0-int(amp_slider.val)/2), int(amp_slider.val), int(amp_slider.val), alpha=0.2))
    else:
        ax.add_patch(Rectangle((0, 0), int(amp_slider.val), int(amp_slider2.val), alpha=0.2))
    ax.relim()
    # update ax.viewLim using the new dataLim
    ax.autoscale_view()
    fig.canvas.draw_idle()
def generate():
    global startinglabel
    if startinglabel == "center":
        points = generatepoints(int(amp_slider.val), amp_slider1.val)
    else:
        points = generatepointsreverse(int(amp_slider.val), int(amp_slider2.val), amp_slider1.val)
    return points
def generatepoints(length,spacing,height=3):
    if (spacing < 10e-7):
        spacing = 10e-3
    length = length / spacing
    start = np.array([[0, 0, 1, 0]])
    points = np.array(start)
    distance_travelled = 0
    for i in range(1,int(length)):
        a=points[i-1][0]
        b=points[i-1][1]
        x=points[i-1][0]+np.array(directions[(i-1)%2][0])*(i*length)
        y=points[i-1][1]+np.array(directions[(i-1)%2][1])*(i*length)
        points=np.vstack((points,np.array([[a+b-y,x+b-a, height, 0]])))
        distance_travelled += np.linalg.norm(np.array([a, b]) - np.array([x, y]))
        xs.append(a+b-y)
        ys.append(x+b-a)
    message = 'Total distance=' + "{0:.2f}".format(distance_travelled) + "m"
    txt.set_text(message)
    return points

def generatepointsreverse(length_x,length_y,spacing,height=3):

    if(spacing<10e-7):
        spacing=10e-3
    num_points=length_x/spacing+1
    start = np.array([[0, 0, height, 0]])
    points = np.array(start)
    distance_travelled = 0
    for i in (range(1,int(num_points))):
        a=points[i-1][0]
        b=points[i-1][1]
        x=points[i-1][0]+np.array(directions[(i-1)%2][0])*spacing
        y=points[i-1][1]+np.array(directions[(i-1)%2][1])*length_y
        xnew=x#a+b-y
        ynew=y#x+b-a
        distance_travelled += np.linalg.norm(np.array([a, b]) - np.array([x, y]))
        xs.append(xnew)
        ys.append(ynew)
        points=np.vstack((points,np.array([[xnew, ynew, height, 0]])))
    message = 'Total distance=' + "{0:.2f}".format(distance_travelled) + "m"
    txt.set_text(message)
    return points
# Plot non-ordered points
points=generatepointsreverse(length, length, spacing)
[line] = ax.plot(points[:,0], points[:,1], marker="o", markerfacecolor="r", linestyle='-')
#rospy.init_node('insert_object',log_level=rospy.INFO)


# Define an action for modifying the line when any slider's value changes
def sliders_on_changed(val):
    updatePlot()
amp_0 = length
amp_slider_ax  = fig.add_axes([0.25, 0.15, 0.65, 0.03], facecolor=axis_color)
amp_slider = Slider(amp_slider_ax, 'Search space dimension x', 1, 800, valinit=amp_0,valfmt= "%d")
amp_slider.on_changed(sliders_on_changed)

amp_slider_ax2  = fig.add_axes([0.25, 0.2, 0.65, 0.03], facecolor=axis_color)
amp_slider2 = Slider(amp_slider_ax2, 'Search space dimension y', 1, 800, valinit=amp_0,valfmt= "%d")
amp_slider2.on_changed(sliders_on_changed)

amp_1 = 1
amp_slider_ax1  = fig.add_axes([0.25, 0.1, 0.65, 0.03], facecolor=axis_color)
amp_slider1 = Slider(amp_slider_ax1, 'Spacing', 0, 10, valinit=amp_1)
amp_slider1.on_changed(sliders_on_changed)
# Add a button for resetting the parameters
reset_button_ax = fig.add_axes([0.8, 0.05, 0.1, 0.04])
reset_button = Button(reset_button_ax, 'Reset', color=axis_color, hovercolor='0.975')
def reset_button_on_clicked(mouse_event):
    amp_slider1.reset()
    amp_slider.reset()
    amp_slider2.reset()
reset_button.on_clicked(reset_button_on_clicked)


publish_trajectory_button_ax = fig.add_axes([0.8, 0.0, 0.1, 0.04])
publish_trajectory_button = Button(publish_trajectory_button_ax, 'SendTrajectory', color=axis_color, hovercolor='0.975')
def publish_trajectory_button_on_clicked(mouse_event):
    points=generate()
    rospy.init_node("topp_trajectory_call_example")
    RequestTrajectory(points)
publish_trajectory_button.on_clicked(publish_trajectory_button_on_clicked)

return_home_button_ax = fig.add_axes([0.02, 0.0, 0.1, 0.04])
return_home_button = Button(return_home_button_ax, 'ReturnHome', color=axis_color, hovercolor='0.975')
def return_home_button_on_clicked(mouse_event):
    points=np.array([0,0,1,0])
    rospy.init_node("topp_trajectory_call_example")
    RequestTrajectory(points,home=True)
return_home_button.on_clicked(return_home_button_on_clicked)

spawn_boxes_button_ax = fig.add_axes([0.02, 0.05, 0.1, 0.04])
spawn_boxes_button = Button(spawn_boxes_button_ax, 'SpawnBoxes', color=axis_color, hovercolor='0.975')
def spawn_boxes_button_on_clicked(mouse_event):
    spawn_boxes()
spawn_boxes_button.on_clicked(spawn_boxes_button_on_clicked)


color_radios_ax = fig.add_axes([0.025, 0.5, 0.15, 0.15], facecolor=axis_color)
color_radios = RadioButtons(color_radios_ax, ('center', 'corner'), active=1)
def color_radios_on_clicked(label):
    global startinglabel
    startinglabel=label
    updatePlot()
color_radios.on_clicked(color_radios_on_clicked)
ax.add_patch(Rectangle((0, 0), int(amp_slider.val), int(amp_slider.val),alpha=0.2))
plt.show()
