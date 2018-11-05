#!/usr/bin/env python


import roslib; roslib.load_manifest('jenga_robot_gazebo')
import rospy
from std_msgs.msg import Empty, Int32, String
from jenga_robot_gazebo.srv import Move, MoveResponse

outFile = open("randomResult.txt", "w")

jenga_move = None

waitingForServer = True
state = None
t = 0
iteration = 0

def reset_callback(empty):
	global waitingForServer
	global state
	global t
	global iteration

	waitingForServer = True
	state = None
	print(str(iteration) + "    " + str(t-2))
	outFile.write(str(iteration) + "," + str(t-2) + "\n")
	iteration += 1

	if (iteration == 101): rospy.signal_shutdown("end of iteration")
	t = 0

	print("FALLEN!")

def state_pub_callback(res):
	global waitingForServer
	global state

	waitingForServer = False
	state = res.data


if __name__=='__main__':

	rospy.init_node('random_jenga_player')

	rospy.sleep(2.0)

	rospy.wait_for_service("/game_msgs/move")

	jenga_move = rospy.ServiceProxy("/game_msgs/move", Move)

	rospy.Subscriber("/game_msgs/reset", Empty, reset_callback)
	rospy.Subscriber("/game_msgs/state", String, state_pub_callback)


	waitingForServer = True

	desired_frequency = rospy.get_param('~frequency', default = 1.5)
	sleep_time = 1.0 / desired_frequency

	while not rospy.is_shutdown():
		rospy.sleep(sleep_time)

		if not waitingForServer:


			# count the number of blocks we have

			block_count = 0
			print(state)
			for entry in state[0:len(state)-3]:
				if entry == '1':
					block_count += 1

			# pick a random block and move it


			import random

			msg = Move()
			msg.data = random.randint(0, block_count-1)
		
			print("Moving block " + str(msg.data))

			try:
				res = jenga_move(msg.data)
			except rospy.service.ServiceException:
				pass
			state = res.data
			t += 1
outFile.close()

