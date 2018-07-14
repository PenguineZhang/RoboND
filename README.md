## Project: Search and Sample Return

[//]: # (Image References)

[image1]: misc/color_thresh.png


### Project Objective
This project is to navigate the Rover autonomously to explore a simulated environment and locate samples.

The simulation and the autonomy development are inspired by the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html)

### Video Demo
[Click Here](https://www.youtube.com/watch?v=ZW1d9I3rd2Y)

### Notebook Analysis
In the function 'color_thresh', ground has already been identified as when the RGB pixel values are greater than (160, 160, 160), respectively. For the case of rock and obstacle, the RGB values for rock range from([100, 245], [90, 245], [0, 60]), respectively, while the RGB values for obstacle are ([0, 140], [0, 140], [0, 140]), respectively. The result are shown after homography transform below:

![color_thresh][image1]