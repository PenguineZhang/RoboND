## Project: Search and Sample Return

[//]: # (Image References)

[image1]: misc/color_thresh.png


### Project Objective
This project is to navigate the Rover autonomously to explore a simulated environment and locate samples.

The simulation and the autonomy development are inspired by the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html)

### Notebook Analysis
## 1. Image processing
`color_thresh` function uses hard-coded values to define RGB boundaries in order to identify target pixels for navigatable terrain, obstacles, and rock:
```
ground_thresh = (img[:,:,0] >= rgb_thresh[0]) & (img[:,:,1] >= rgb_thresh[1]) & (img[:,:,2] >= rgb_thresh[2])
rock_thresh = (100 <= img[:,:,0]) & (img[:,:,0] <= 245) & \
    			  (90 <= img[:,:,1]) & (img[:,:,1] <= 245) & \
    			  (0 <= img[:,:,2]) & (img[:,:,2] <= 60)
bstacle_thresh = (img[:,:,0] < 140) & (img[:,:,1] < 140) & (img[:,:,2] < 140)
```
By using this RGB thresholding values, it's able to produce the following thresholded image:

![color_thresh][image1]

Obstacles include the true obstacles appeared in the image and black pixels that do not appear in the image (two black triangles on the side of the warped image).

## 2. Mapping
`process_image(img)` consists of a series of actions to acquire warped image and update the worldmap. It first defines the boundary for warping:
```
dst_size = 5 
bottom_offset = 6
source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
```
The source and destination are later fed into `perspect_transform` to obtain the warped image.

```
source, destination = define_box(img)
warped = perspect_transform(img, source, destination)
```
The warped image is passed to `color_thresh` for target identification, which outputs three variables corresponding three targe regions on the image as mentioned in "Image processing" section. 

Once the regions are identified, coordinate transforms are performed. The tranforms includes rover-centric and world frame coordinate transforms:
```
# convert image pixels to rover-centric coordinates
xpix_ground, ypix_ground = rover_coords(ground_select)
xpix_obstacle, ypix_obstacle = rover_coords(obstacle_select)
xpix_rock, ypix_rock = rover_coords(rock_select)

# convert rover-centric pixels to world coordinates, given rover's pose information
x_pix_ground_world, y_pix_ground_world = pix_to_world(xpix_ground, ypix_ground, xpos, ypos, yaw, 200, 10)
x_pix_obstacle_world, y_pix_obstacle_world = pix_to_world(xpix_obstacle, ypix_obstacle, xpos, ypos, yaw, 200, 10)
x_pix_rock_world, y_pix_rock_world = pix_to_world(xpix_rock, ypix_rock, xpos, ypos, yaw, 200, 10)    
```
Once the coordinate transformation is complete, one is able to update the worldmap:
```
data.worldmap[y_pix_ground_world, x_pix_ground_world, 0] += 1
data.worldmap[y_pix_obstacle_world, x_pix_obstacle_world, 1] += 1
data.worldmap[y_pix_rock_world, x_pix_rock_world, 2] += 1
```

### Video Demo
[Click Here](https://www.youtube.com/watch?v=ZW1d9I3rd2Y) for video demo. The simulation is run under 1920 x 1080 resolution and "Fantastic" graphics quality, giving 27 FPS. 

### Autonomous Navigation and Mapping
## 1. Perception step
`perception_step()` is called in the `drive_rover.py` script, in which the Rover receives new images from the simulator and updates the navigable ground and world map. 

`perception_step()` resembles the `process_image()` in Notebook Analysis section, except that Rover's vision map needed to be updated to the latest thresholded image.
```
Rover.vision_image[:,:,0] = obstacle_thresholded
Rover.vision_image[:,:,1] = rock_thresholded
Rover.vision_image[:,:,2] = ground_thresholded
```
Also the ground pixels are converted into polar coordinates:
```
Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix_ground, ypix_ground) 
```
where `xpix_ground` and `ypix_ground` are the ground pixels in rover-centric coordinate and used to determine Rover's steering angles.

## 2. Decision step
`decision_step()` is also called in the `driver_rover.py` script after `perception_step` is completed. There are three modes in the decision-making process:

**In "forward" mode**
1. If the Rover has enough space to move (navigatable ground pixels are greater than pre-defined threshold), it moves forward with maximum throttle until it has reached maximum speed. When the Rover stucks in dead-end or does not have space to move, brake is applied and it goes to "stop" mode.

2. To stablize the steering, outlier rejection is utlized based on the percentile of `nav_angles`. 
```
def remove_outlier(angles):
	lower_bound = np.percentile(angles, 25)
	upper_bound = np.percentile(angles, 75)
	
	angles = [i for i in angles if (i >= lower_bound and i <= upper_bound)]
	return np.mean(angles)
```
In this case, angles between 25% and 75% in the percentile scale are chosen to calculate the steering angle, which is the average valid angles. 

3. Rover can sometimes be stuck for several reasons. If the Rover's absolute velocity is less than 0.05 but the throttle is non-zeros, the stuck timer starts ticking. When the Rover couldn't get out of the state, it will transition to "unstuck" mode.

**In "stop" mode**
1. Rover applies hard brake for immediate stop if it still has momentum.
2. When it come to a dead-end, Rover performs 4-wheel-turning until it has space to move. To make sure the space is efficient, Rover backs up straight a bit before it transitions to "forward" mode.


**In "unstuck" mode**
```
Rover.mode == 'unstuck':
	Rover.throttle = -Rover.throttle_set
	Rover.brake = 0
	Rover.steer = 0
	if (time.time() - Rover.unstuck_timer) > 3:
		Rover.mode = 'forward'
```
1. Rover reaches complete stop and attempts to move backward straight.
2. When Rover backs up for 3 seconds, it should be able to pass obstacles.