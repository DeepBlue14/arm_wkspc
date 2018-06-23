#!/usr/bin/python

from random import shuffle
from pprint import pprint
# Number of trials
# 10 objects per trial
# Pick 6 up, leave 4 behind
# 30 objects
# Each object must receive the same number of pick-up attempts

#Objects, and whether they can be grasped horizontally
#Things that cannot be grasped horizontally can't be on the middle shelf
objects = [("apple juice", True), ("ball", True), ("battery", True), ("blue squirtbottle", False),
			("computer mouse", False), ("dog toy", False), ("drill", False), ("flashlight", False), 
			("fruits bottle", True), ("hammer", False), ("hex key set", False), 
		   ("long vacuum", False), ("magazine", False), ("medicine box", True), ("monkey", True), 
		   ("newspaper", False), ("nutrition shake", True), ("robot", True), ("rocket", True), 
		   ("salt", True), ("screw", True), ("screwdriver", False), ("short vacuum", False), 
		   ("sunscreen bottle", True), ("tissues", False), ("tomato paste", True), ("toothpaste", False), 
		   ("vitamin C", True), ("white pepper", True), ("white squirtbottle", False)]


objCount = len(objects)
shuffle(objects)

#Step needs to be an even divisor of perTrial, or not every object will get the same number of attempts
perTrial = 10
step = 2

# Each trial is 10 objects:
# 3 on small table
# 3 on big table
# 2 on top shelf
# 2 on middle shelf

trials = []
for x in range(0, objCount, step):
	trial = []
	for y in range(0, perTrial):
		trial.append(objects[(x+y) % len(objects)])
	print trial
	trials.append(trial)

#Check that for all trials, the objects on the middle shelf are able to be grasped from the side
for trial in trials:
	#index_list = range(len(trial))
	middle_shelf = trial[0:2]
	#print index_list
	#import pdb; pdb.set_trace()
	print trial
	for thingIndex, thing in enumerate(middle_shelf):
		if not thing[1]:
			#Find something else in this trial that CAN be stood upright
			for goodThingIndex, goodThing in enumerate(trial[2:]):
				if goodThing[1]:
					print "swapping {0}, {1}".format(thing[0], goodThing[0])
					#Swap and get out of this loop
					temp = trial[thingIndex]
					trial[thingIndex] = trial[goodThingIndex + 2]
					trial[goodThingIndex + 2] = temp
					break
	#print index_list
	print trial
	print "----"
	
#Test that everything gets used equally
testCounts = {}
for trial in trials:
	for item in trial:
		if item in testCounts.keys():
			testCounts[item] += 1
		else:
			testCounts[item] = 1
pprint(testCounts)

for trial in trials:
	#Print out what goes where
	middle_shelf = trial[0:2]
	print "Middle shelf:",middle_shelf

	top_shelf = trial[2:4]
	print "Top shelf: ",top_shelf

	big_table = trial[4:7]
	print "Big table: ",big_table

	little_table = trial[7:]
	print "Little table: ",little_table

	print "-----"
