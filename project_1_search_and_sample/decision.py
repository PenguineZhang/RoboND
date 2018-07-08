import numpy as np
from perception import rover_coords, pix_to_world
import cv2


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

	# Implement conditionals to decide what to do given perception data
	# Here you're all set up with some basic functionality but you'll need to
	# improve on this decision tree to do a good job of navigating autonomously!
	print 'current Rover mode: ' + Rover.mode
	print 'current Rover pos: ' + str(Rover.pos[0]) + ' ' +str(Rover.pos[1])
	# Example:
	# Check if we have vision data to make decisions with
	if Rover.nav_angles is not None:
		# Check for Rover.mode status
		if Rover.mode == 'forward': 
			# Check the extent of navigable terrain
			if len(Rover.nav_angles) >= Rover.stop_forward:  
				# If mode is forward, navigable terrain looks good 
				# and velocity is below max, then throttle 
				if len(Rover.vision_image[:,:,1].nonzero()[0]) > 10:
					Rover.mode = 'pickup'

				else:
					if Rover.vel < Rover.max_vel:
						# Set throttle value to throttle setting
						Rover.throttle = Rover.throttle_set
					else: # Else coast
					    Rover.throttle = 0

					Rover.brake = 0
					# # Set steering to average angle clipped to the range +/- 15
					Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
			# If there's a lack of navigable terrain pixels then go to 'stop' mode
			elif len(Rover.nav_angles) < Rover.stop_forward:
				# Set mode to "stop" and hit the brakes!
				Rover.throttle = 0
				# Set brake to stored brake value
				Rover.brake = Rover.brake_set
				Rover.steer = 0
				Rover.mode = 'stop'
			
			elif abs(Rover.vel) < 0.05 and Rover.throttle != 0:
				Rover.mode = 'stop'

		# If we're already in "stop" mode then make different decisions
		elif Rover.mode == 'stop':
			# If we're in stop mode but still moving keep braking
			if Rover.vel > 0.2:
				Rover.throttle = 0
				Rover.brake = Rover.brake_set
				Rover.steer = 0
			# If we're not moving (vel < 0.2) then do something else
			elif Rover.vel <= 0.2:
				# Now we're stopped and we have vision data to see if there's a path forward
				if len(Rover.nav_angles) < Rover.go_forward:
					Rover.throttle = 0
					# Release the brake to allow turning
					Rover.brake = 0
					# Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
					Rover.steer = -15 # Could be more clever here about which way to turn

				# If we're stopped but see sufficient navigable terrain in front then go!
				if len(Rover.nav_angles) >= Rover.go_forward:
					# Set throttle back to stored value
					Rover.throttle = Rover.throttle_set
					# Release the brake
					Rover.brake = 0
					# Set steer to mean angle
					Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
					Rover.mode = 'forward'
		
		elif Rover.mode == 'pickup':
			print 'in pickup mode'
			
			rock_vision_x, rock_vision_y = Rover.vision_image[:,:,1].nonzero()

			angle_x = Rover.vision_image[:,:,1].shape[0] - np.mean(rock_vision_x)
			angle_y = Rover.vision_image[:,:,1].shape[1]/2 - np.mean(rock_vision_y)

			angle = np.arctan2(angle_y, angle_x) * 180 / np.pi

			if np.isnan(angle):
				print 'angle is nan, return to forward'

			elif angle > 5:
				print 'turning left'
				Rover.throttle = 0
				Rover.brake = 0
				Rover.steer = 15
			elif angle < -5:
				print 'turn right'
				Rover.throttle = 0
				Rover.brake = 0
				Rover.steer = -15
			else:
				print 'angle aligned'
				Rover.steer = 0
				Rover.throttle = 0.1

				if Rover.near_sample == 1:
					Rover.throttle = 0
					Rover.brake = Rover.brake_set
					Rover.mode = 'stop'

	# Just to make the rover do something 
	# even if no modifications have been made to the code
	else:
		Rover.throttle = Rover.throttle_set
		Rover.steer = 0
		Rover.brake = 0
        
	# If in a state where want to pickup a rock send pickup command
	if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
		Rover.send_pickup = True
		Rover.mode = 'forward'

	return Rover

