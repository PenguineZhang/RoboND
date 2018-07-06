import numpy as np


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
				rock_x, rock_y = Rover.worldmap[:,:,1].nonzero()
				dist = np.sqrt(rock_x**2 + rock_y**2)
				mean_dist = np.mean(dist)
				print "mean_dist = " + str(mean_dist)

				if len(rock_x) != 0:
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

        # If we are already in 'pickup' mode then try to acquire the rock
		elif Rover.mode == 'pickup':
			print 'in pickup mode'
			rock_y, rock_x = Rover.worldmap[:,:,1].nonzero()
			rock_rover_x = Rover.pos[0]-rock_x
			rock_rover_y = Rover.pos[1]-rock_y
			angles = np.arctan2(rock_rover_y, rock_rover_x)
			dist = np.sqrt(rock_rover_x**2 + rock_rover_y**2)
			mean_dist = np.mean(dist)
			mean_angle = np.mean(angles * 180/np.pi)
			# print "current rock pos: " + str(rock_x) + ' ' + str(rock_y)
			print "current distance to rock: " + str(mean_dist)
			print "current angle to rock: " + str(mean_angle)
			if mean_dist > 3:
				Rover.steer = np.clip(mean_angle, -15, 15)
				Rover.throttle = Rover.throttle_set
				Rover.brake = 0
			else:
				Rover.steer = 0
				Rover.send_pickup = True

			if len(rock_x) != 0:
				Rover.mode = 'forward'

	# Just to make the rover do something 
	# even if no modifications have been made to the code
	else:
		Rover.throttle = Rover.throttle_set
		Rover.steer = 0
		Rover.brake = 0
        
	# If in a state where want to pickup a rock send pickup command
	if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
		Rover.send_pickup = True

	return Rover

