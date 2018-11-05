#!/usr/bin/env python

import random
import roslib; roslib.load_manifest('jenga_robot_gazebo')
import rospy
from std_msgs.msg import Empty, Int32, String
from jenga_robot_gazebo.srv import Move, MoveResponse

#Leng's Edits
outFile = open("optimalResult.txt", "w")

jenga_move = None
reset = None

waitingForServer = True
state = None

iteration = 0
t = 0
move1 = 0
move2 = 1

def reset_callback(empty):
	global waitingForServer
	global state
	global t
	global iteration

	waitingForServer = True
	state = None

	print("FALLEN!")

	global move1
	global move2
	
	move1 = 0
	move2 = 1
	iteration += 1
	outFile.write(str(iteration) + "," + str(t) +"\n") 
	
	if iteration == 100: rospy.signal_shutdown("end of iteration")
	t = 0


def state_pub_callback(res):
	global waitingForServer
	global state

	state = res.data

	for a in state:
		if (a == "0"):
			return
		
	waitingForServer = False


if __name__=='__main__':

	rospy.init_node('random_jenga_player')

	rospy.sleep(2.0)

	rospy.wait_for_service("/game_msgs/move")

	jenga_move = rospy.ServiceProxy("/game_msgs/move", Move)
	reset = rospy.Publisher("/game_msgs/rebuild_stack", Empty)

	rospy.Subscriber("/game_msgs/reset", Empty, reset_callback)
	rospy.Subscriber("/game_msgs/state", String, state_pub_callback)


	waitingForServer = True

	desired_frequency = rospy.get_param('~frequency', default = 1.5)
	sleep_time = 1.0 / desired_frequency


	block_count = 100

	while not rospy.is_shutdown():
		rospy.sleep(sleep_time)

		if not waitingForServer:
			
			# count the number of blocks we have

			
			print("Moving block " + str(move1))

			try:
				res = jenga_move(move1)
				state = res.data

				t += 1

				if (move1 == move2):
					move2 = move1 + 1
				else:
					move1 = move2

			except rospy.service.ServiceException:
				pass

			block_count = 0

			for entry in state[0:len(state)-3]:
				if entry == '1':
					block_count += 1

			if (move1 >= block_count): 
				waitingForServer = True
				reset.publish()
				print("Optimal Reached - Resetting Tower")

	outFile.close()

	print("Done")
