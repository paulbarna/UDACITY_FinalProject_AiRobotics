# ----------
# Part Three
#
# Now you'll actually track down and recover the runaway Traxbot. 
# In this step, your speed will be about twice as fast the runaway bot,
# which means that your bot's distance parameter will be about twice that
# of the runaway. You can move less than this parameter if you'd 
# like to slow down your bot near the end of the chase. 
#
# ----------
# YOUR JOB
#
# Complete the next_move function. This function will give you access to 
# the position and heading of your bot (the hunter); the most recent 
# measurement received from the runaway bot (the target), the max distance
# your bot can move in a given timestep, and another variable, called 
# OTHER, which you can use to keep track of information.
# 
# Your function will return the amount you want your bot to turn, the 
# distance you want your bot to move, and the OTHER variable, with any
# information you want to keep track of.
# 
# ----------
# GRADING
# 
# We will make repeated calls to your next_move function. After
# each call, we will move the hunter bot according to your instructions
# and compare its position to the target bot's true position
# As soon as the hunter is within 0.01 stepsizes of the target,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot. 
#
# As an added challenge, try to get to the target bot as quickly as 
# possible. 

from robot import *
from math import *
from matrix import *
import random

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 
# next state function
	F = matrix([[1., 1., 0.],
                [0., 1., 0.],
                [0., 0., 1.]])  
    # measurement function
	H = matrix([[1., 0., 0.],
				[0., 0., 1.]])
    # measurement uncertainty matrix
	R = matrix([[measurement_noise, 0.],
				[0., measurement_noise]])
    # identity matrix
	I = matrix([[1., 0., 0.],
				[0., 1., 0.],
				[0., 0., 1.]]) 
    
	if not OTHER:
		OTHER={}
		OTHER["initialState"] = matrix([[0.],
						                [0.],
					                    [0.]])  
		OTHER["initialUncertainty"] = matrix([[1000., 0., 0.],
					                          [0., 1000., 0.],
					                          [0., 0., 1000.]])
		OTHER["Measuremnt"]=[]
		

	OTHER["Measuremnt"].append(target_measurement)
	
	heading = []
	distance =[]
	turningPredict=0
	headingPredict=0
	
	#print len(OTHER["Measuremnt"])
	
    #if the length of the measurement list is 1 pass the current values
	if len(OTHER["Measuremnt"]) == 1:
		x0 = OTHER["Measuremnt"][0][0]
		y0 = OTHER["Measuremnt"][0][1]
		xy_estimate = (x0, y0)
    #if the length of the measurement list is 2, distance and heading are returned with 0 
	elif len(OTHER["Measuremnt"]) == 2:
		distance= [0]
		heading= [0]
	else:
        # otherwise calculate the distance and the heading offsets with respect to the previous measurement values
		for i in range(len(OTHER["Measuremnt"])-1):
			Initial = (OTHER["Measuremnt"][i][0], OTHER["Measuremnt"][i][1])
			Next = (OTHER["Measuremnt"][i+1][0], OTHER["Measuremnt"][i+1][1])
			distance.append(distance_between(Next, Initial))
			heading.append((atan2(OTHER["Measuremnt"][i+1][1]-OTHER["Measuremnt"][i][1], OTHER["Measuremnt"][i+1][0]-OTHER["Measuremnt"][i][0]))%(2*pi))

	
	if len(OTHER["Measuremnt"]) != 1:
		
		x=OTHER["initialState"]
		P=OTHER["initialUncertainty"]
		PreviousHeading=x.value[0][0]
		PreviousDistance=x.value[2][0]

		#estimate the current heading by applying the turning correction with respet to the previous value
		turningPredict = heading[len(heading)-1] - PreviousHeading
		
		if (turningPredict + (PreviousHeading // (2 * pi))*(2 * pi) + pi) < 0:
			turningPredict =(PreviousHeading // (2 * pi))*(2 * pi) + (2 * pi)
		elif (turningPredict + (PreviousHeading // (2 * pi))*(2 * pi) - pi) < 0:
			turningPredict =(PreviousHeading // (2 * pi))*(2 * pi)
			
		headingPredict=heading[len(heading)-1] + turningPredict

		#estimate distance based on previous measuremnts
		distancePredict = sum(distance) / len(distance)
		
		
		# I'm using the estimated heading and distance within the Kalman initial state vector to predicte the next position
		# measure
		z = matrix([[headingPredict],
                    [distancePredict]])
		y = z - H * x
		S = H * P * H.transpose() + R
		K = P * H.transpose() * S.inverse()
		x = x + (K * y)
		P = (I - K * H) * P
		# predict
		x = F * x
		P = F * P * F.transpose()
		
		OTHER["initialState"]=x
		OTHER["initialUncertainty"]=P
	
		
		#compute the estimated x and y 

		xPredict = target_measurement[0] + x.value[2][0] * cos(x.value[0][0])
		yPredict = target_measurement[1] + x.value[2][0] * sin(x.value[0][0])
		xy_estimate = (xPredict, yPredict)

	
	distance = distance_between(hunter_position, xy_estimate)
	
	# Not move faster than allowed!
	if distance > max_distance:
		distance = max_distance
	
	#compute the turning angle by subtracting the hunter heading from the heading between the target and the hunter position
	turning = angle_trunc(get_heading(hunter_position, xy_estimate) - hunter_heading)
 
    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
	return turning, distance, OTHER

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 1.94 * target_bot.distance # 1.94 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)
        
        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1            
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught

def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all 
    the target measurements, hunter positions, and hunter headings over time, but it doesn't 
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables
    
    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

print demo_grading(hunter, target, next_move)





