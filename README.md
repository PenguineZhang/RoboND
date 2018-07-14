## Project: Search and Sample Return

[//]: # (Image References)

[image1]: misc/color_thresh.png


### Project Objective
This project is to navigate the Rover autonomously to explore a simulated environment and locate samples.

The simulation and the autonomy development are inspired by the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html)

### Notebook Analysis
## 1. Image processing
`color_thresh` function defined as follows:
```
# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    ground_select = np.zeros_like(img[:,:,0])
    obstacle_select = np.zeros_like(img[:,:,0])
    rock_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    ground_thresh = (img[:,:,0] >= rgb_thresh[0]) \
                & (img[:,:,1] >= rgb_thresh[1]) \
                & (img[:,:,2] >= rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    ground_select[ground_thresh] = 1
    
    # rock (yellow objects) threshold
    rock_thresh = (100 <= img[:,:,0]) & (img[:,:,0] <= 245) & \
    			  (90 <= img[:,:,1]) & (img[:,:,1] <= 245) & \
    			  (0 <= img[:,:,2]) & (img[:,:,2] <= 60)
    rock_select[rock_thresh] = 1
    
    # obstacle (wall/mountain) threshold
    obstacle_thresh = (img[:,:,0] < 140) & \
                  (img[:,:,1] < 140) & \
                  (img[:,:,2] < 140)
    obstacle_select[obstacle_thresh] = 1
    
    # Return the binary image
    return ground_select, obstacle_select, rock_select
```
In this function, ground has already been identified as when the RGB pixel values are greater than (160, 160, 160), respectively. For the case of rock and obstacle, the RGB values for rock range from([100, 245], [90, 245], [0, 60]), respectively, while the RGB values for obstacle are ([0, 140], [0, 140], [0, 140]), respectively. The result are shown after homography transform below:

![color_thresh][image1]

Obstacles include the true obstacles appeared in the image and black pixels that do not appear in the image (two black triangles on the side of the warped image).

## 2. Mapping
```
# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img

	img = Rover.img
	xpos = Rover.pos[0]
	ypos = Rover.pos[1]
	yaw = Rover.yaw

    # 1) Define source and destination points for perspective transform
	source, destination = define_box(img)

    # 2) Apply perspective transform
	warped = perspect_transform(img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
	ground_select, obstacle_select, rock_select = color_thresh(warped)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
	Rover.vision_image[:,:,0] = obstacle_select
	Rover.vision_image[:,:,1] = rock_select
	Rover.vision_image[:,:,2] = ground_select

    # 5) Convert map image pixel values to rover-centric coords
	xpix_obstacle, ypix_obstacle = rover_coords(obstacle_select)
	xpix_rock, ypix_rock = rover_coords(rock_select)
	xpix_ground, ypix_ground = rover_coords(ground_select)

    # 6) Convert rover-centric pixel values to world coordinates
	x_pix_obstacle_world, y_pix_obstacle_world = pix_to_world(xpix_obstacle, ypix_obstacle, xpos, ypos, yaw, 200, 10)
	x_pix_rock_world, y_pix_rock_world = pix_to_world(xpix_rock, ypix_rock, xpos, ypos, yaw, 200, 10)
	x_pix_ground_world, y_pix_ground_world = pix_to_world(xpix_ground, ypix_ground, xpos, ypos, yaw, 200, 10)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
	Rover.worldmap[y_pix_obstacle_world, x_pix_obstacle_world, 0] += 1
	Rover.worldmap[y_pix_rock_world, x_pix_rock_world, 1] += 1
	Rover.worldmap[y_pix_ground_world, x_pix_ground_world, 2] += 1
    
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
	Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix_ground, ypix_ground) 

	return Rover
```

For each image generated from the simulator, the function first loads the image and Rover's pose information, then transforms and translate the image to Rover-centric and world coordinate frame. The code is broken down as follows:

```
source, destination = define_box(img)
warped = perspect_transform(img, source, destination)
```
When Rover's camera generates a new image, it must be transformed into top-down view using perspective transformation, as shown in Figure 1, which helps coordinate transform later in the code.

```
ground_select, obstacle_select, rock_select = color_thresh(warped)
Rover.vision_image[:,:,0] = obstacle_select
Rover.vision_image[:,:,1] = rock_select
Rover.vision_image[:,:,2] = ground_select
``` 
The warped image is thresholded into three regions: ground, obstacle, and rock using three thresholding RGB values. These regions are stored in Rover's vision_image attribute.

```
xpix_obstacle, ypix_obstacle = rover_coords(obstacle_select)
xpix_rock, ypix_rock = rover_coords(rock_select)
xpix_ground, ypix_ground = rover_coords(ground_select)

x_pix_obstacle_world, y_pix_obstacle_world = pix_to_world(xpix_obstacle, ypix_obstacle, xpos, ypos, yaw, 200, 10)
x_pix_rock_world, y_pix_rock_world = pix_to_world(xpix_rock, ypix_rock, xpos, ypos, yaw, 200, 10)
x_pix_ground_world, y_pix_ground_world = pix_to_world(xpix_ground, ypix_ground, xpos, ypos, yaw, 200, 10)
```
Once the three regions are identified, the pixels are translated into Rover-centric and world frame coordinates system given Rover's current pose information (x, y, yaw).

```
Rover.worldmap[y_pix_obstacle_world, x_pix_obstacle_world, 0] += 1
Rover.worldmap[y_pix_rock_world, x_pix_rock_world, 1] += 1
Rover.worldmap[y_pix_ground_world, x_pix_ground_world, 2] += 1
```
Rover's worldmap can be updated by using the calculated world frame coordinates of ground, obstacles, and rocks.

### Video Demo
[Click Here](https://www.youtube.com/watch?v=ZW1d9I3rd2Y) for video demo. The simulation is run under 1920 x 1080 resolution and "Fantastic" graphics quality, giving 27 FPS. 

### Autonomous Navigation and Mapping
## 1. Perception step
As mentioned in Noteboo