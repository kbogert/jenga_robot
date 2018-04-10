#!/usr/bin/env python


import roslib; roslib.load_manifest('jenga_robot_gazebo')
import rospy
from std_msgs.msg import Empty, Int32, String
from jenga_robot_gazebo.srv import Move, MoveResponse

jenga_move = None

waitingForServer = True
state = None

move1 = 0
move2 = 1

def reset_callback(empty):
	global waitingForServer
	global state

	waitingForServer = True
	state = None

	print("FALLEN!")

	global move1
	global move2

	move1 = 0
	move2 = 1

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

	desired_frequency = rospy.get_param('~frequency', default = 5.0)
	sleep_time = 1.0 / desired_frequency


	block_count = 100

	while not rospy.is_shutdown() and move1 < block_count:
		rospy.sleep(sleep_time)

		if not waitingForServer:

			# count the number of blocks we have

			block_count = 0

			for entry in state[0:len(state)]:
				if entry == '1':
					block_count += 1
			
			print("Moving block " + str(move1))

			try:
				res = jenga_move(move1)
			except rospy.service.ServiceException:
				pass
			state = res.data


			if (move1 == move2):
				move2 = move1 + 1
			else:
				move1 = move2

	print("Done")
