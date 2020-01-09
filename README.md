# Search-strategies-for-UAV
Installation Instructions
-------------------------
Tested on ROS Melodic, should work on other ROS platforms too. 

Client example uses Matplotlib for visualization of received points.

Launching the waypoint generator
-----------

Launch the waypoint generator

```
$ roslaunch uav_search waypoint_generator.launch
```
When waypoint generator is launched, the *get_points* service should be available. Then, you can call that service to get waypoints.

Calling the waypoint generator service
-----------
To call the *get_points* service, you need to create a message of type *PointsRequest*, which has the following fields:

1. *pattern* - type string, can attain one of the following values:

  * *spiral* - generates waypoints in shape of a spiral

  * *lawn* - generates waypoints in shape of lawn mower pattern

  * *levy* - generates a randomized pattern of 50 points

2. *size_x* - type float64, denotes the horizontal size of the area
3. *size_y* - type float64, denotes the vertical size of the area
4. *spacing* - parameter of pattern generator (for spiral and lawn it denotes how much apart are the segments of the trajectory, for levy it denotes how big are the jumps between random points)
5. *height* - desired height for the points

An example is given in the *wapoint_client_example.py*, and boils down to the following code:

```python
	request = PointsRequest()
	request.pattern = string(pattern_type)
	request.size_x = float(size_x)
	request.size_y = float(size_y)
	request.spacing = float(spacing)
	request.height = float(height)
	points_response = get_points_client(request)
```
You can also run the example:
```
$ rosrun uav_search waypoint_client_example.py
```

Unpacking the point data
-----------
The response of the *get_points* service is a message of type *PatternPoints* which has the following fields:
1. *size_x* - number of points, type int64
2. *size_y* - dimension of each point, type int64
3. *data* - type float64[], vector of data

To unpack the points, use *numpy.reshape* function:
```python
	points_response = get_points_client(request)
	dim_x = points_response.size_x
	dim_y = points_response.size_y
	data = np.asarray(points_response.data)
	points = np.reshape(data, (dim_x, dim_y))
```