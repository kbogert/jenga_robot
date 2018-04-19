#!/usr/bin/env python


import roslib; roslib.load_manifest('jenga_robot_gazebo')
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Empty, Int32
from geometry_msgs.msg import *
from jenga_robot_gazebo.srv import Move, MoveResponse

block_urdf = None
block_pose = None
spawn_srv = None
delete_srv = None
model_states = None
set_model_state = None

blocks_list = []

# need the store the state of all blocks, for building the state message
# need to provide an actionlib interface for removing a block and placing it on top

stack_state = []
block_pos_to_id = {}


def spawn_new_block(new_id, pose):

	spawn_srv(new_id, block_urdf, "", pose, "world")

def spawn_all_blocks(num_levels):

	# if blocks have already been spawned once, don't spawn again, instead move them to their correct positions

	if len(blocks_list) > 0:
		rebuild_tower(num_levels)
		return

	rotated = False
	count = 0

        next_pose = Pose()
    
        next_pose.position.x = block_pose.position.x
        next_pose.position.y = block_pose.position.y
        next_pose.position.z = block_pose.position.z
    
        next_pose.orientation.x = block_pose.orientation.x
        next_pose.orientation.y = block_pose.orientation.y
        next_pose.orientation.z = block_pose.orientation.z
        next_pose.orientation.w = block_pose.orientation.w

	rotation_swap = Pose()

        rotation_swap.orientation.x = 0.499
        rotation_swap.orientation.y = -0.499
        rotation_swap.orientation.z = 0.501
        rotation_swap.orientation.w = 0.501

	
	for i in range(num_levels):

		next_pose.position.y = block_pose.position.y
		next_pose.position.x = block_pose.position.x

		stack_state.append([])

		if rotated:

			next_pose.position.y = block_pose.position.y + 0.025
			next_pose.position.x = block_pose.position.x - 0.025

		for j in range(3):
			newid = "block" + str(count)
			spawn_new_block(newid, next_pose)
			if rotated:
				next_pose.position.x += 0.0255 # BLOCK WIDTH is .025
			else:
				next_pose.position.y += 0.0255 # BLOCK WIDTH is .025
			blocks_list.append(newid)
			stack_state[i].append(1)
			block_pos_to_id[count] = newid

			count += 1

		next_pose.position.z += 0.015
		tmp_rotation = next_pose.orientation
		next_pose.orientation = rotation_swap.orientation
		rotation_swap.orientation = tmp_rotation

		rotated = not rotated

	print(block_pos_to_id)


def rebuild_tower(num_levels):
	rotated = False
	count = 0

        next_pose = Pose()
    
        next_pose.position.x = block_pose.position.x
        next_pose.position.y = block_pose.position.y
        next_pose.position.z = block_pose.position.z
    
        next_pose.orientation.x = 0
        next_pose.orientation.y = -0.7071
        next_pose.orientation.z = 0
        next_pose.orientation.w = 0.7071

	rotation_swap = Pose()

        rotation_swap.orientation.x = 0.499
        rotation_swap.orientation.y = -0.499
        rotation_swap.orientation.z = 0.501
        rotation_swap.orientation.w = 0.501

	
	for i in range(num_levels):

		next_pose.position.y = block_pose.position.y
		next_pose.position.x = block_pose.position.x

		stack_state.append([])

		if rotated:

			next_pose.position.y = block_pose.position.y + 0.025
			next_pose.position.x = block_pose.position.x - 0.025

		for j in range(3):
			newid = "block" + str(count)
			#spawn_new_block(newid, next_pose)

			new_block_state = ModelState()
			new_block_state.model_name = newid
			new_block_state.pose = next_pose
			new_block_state.twist = Twist()
			new_block_state.reference_frame = "world"

			set_model_state(new_block_state)


			if rotated:
				next_pose.position.x += 0.0255 # BLOCK WIDTH is .025
			else:
				next_pose.position.y += 0.0255 # BLOCK WIDTH is .025

			stack_state[i].append(1)
			block_pos_to_id[count] = newid

			count += 1

		next_pose.position.z += 0.015
		tmp_rotation = next_pose.orientation
		next_pose.orientation = rotation_swap.orientation
		rotation_swap.orientation = tmp_rotation

		rotated = not rotated



def delete_all_blocks():

#	for block in blocks_list:
#		delete_srv(block)

#	blocks_list[:] = []

	next_pose = Pose()
	next_pose.position.x = block_pose.position.x
	next_pose.position.y = block_pose.position.y + 0.5
	next_pose.position.z = block_pose.position.z

	next_pose.orientation.x = block_pose.orientation.x
	next_pose.orientation.y = block_pose.orientation.y
	next_pose.orientation.z = block_pose.orientation.z
	next_pose.orientation.w = block_pose.orientation.w


	for blockid in blocks_list:
		new_block_state = ModelState()
		new_block_state.model_name = blockid
		new_block_state.pose = next_pose
		new_block_state.twist = Twist()
		new_block_state.reference_frame = "world"

		set_model_state(new_block_state)

	block_pos_to_id = {}

def hasFallen():


	if len(blocks_list) < 3:
		return False

	# check if there are any blocks outside of the stack's dimensions

	for block in blocks_list:
		pos =  model_states(block, "world").pose.position

		if (pos.x < block_pose.position.x - (0.025 * 2.5)) or ( pos.x > block_pose.position.x + (0.025 * 4.5)) or ( pos.y < block_pose.position.y - (0.025 * 1)) or (pos.y > block_pose.position.y + (0.025 * 4.5) ):

			print("Fall detected")
			print(block)
			print(pos)
			return True

	# check if any of the levels have all zeros

	for level in stack_state:
		total = 0
		for entry in level:
			total += entry

		if total == 0:
			return True
	return False

	# average the z-height of each block's position, if below half of the ideal block height, we've fallen

#	block_height = 0.015
#	total_height = 0

#	for block in blocks_list:
#		total_height += model_states(block, "world").pose.position.z - block_pose.position.z
	
#	total_height /= 3

#	avg_height = total_height / (len(blocks_list) / 3)
#
#	print(str(avg_height) + " " + str((len(blocks_list) / 3) /2 * block_height)) 
#
#	if avg_height < (len(blocks_list) / 3) /2 * block_height:
#		# we've fallen
#		return True
#
#	return False

def getBlockPose(model_states, id):

	return model_states(id, "world").pose

def generateStateString():
	s = ""


	for level in stack_state:
		for entry in level:
			s += str(entry)

	return s

def publishState(state_pub):

	toPub = std_msgs.msg.String()

	toPub.data = generateStateString()

	state_pub.publish(toPub)


def moveBlock(req):

	blocknum = req.data

	print("Moving block " + str(req.data))

	blockCount = 0
	blockpos = 0

	for level in stack_state:

		if blockCount > blocknum:
			break

		for entry in level:
			if entry == 1:
				blockCount += 1

			if blockCount > blocknum:
				break

			blockpos += 1	
			
	blockid = str(block_pos_to_id[blockpos])

#	delete_srv(blockid)
	
#	print("Start from")
#	print(model_states(blockid, "world"))

	stack_state[int(blockpos / 3)][blockpos % 3] = 0

#	rospy.sleep(0.5)

	# now the block has been removed, figure out where to place it	


	allFull = True

	for entry in stack_state[len(stack_state) - 1]:
		if entry == 0:
			allFull = False
			break

	if allFull:
		
		# the upper level is full, add on to our stack state in prep for 
		stack_state.append([0,0,0])
		

	whichRotation = len(stack_state) % 2 == 0
	z_height = block_pose.position.z + (len(stack_state) - 1) * 0.015

	pos = 0

	for entry in stack_state[len(stack_state) - 1]:
		if (entry == 1):
			pos += 1
		else:
			break


        next_pose = Pose()
    
        next_pose.position.x = block_pose.position.x
        next_pose.position.y = block_pose.position.y
        next_pose.position.z = z_height

	if not whichRotation:
	        next_pose.orientation.x = 0
	        next_pose.orientation.y = -0.7071
	        next_pose.orientation.z = 0
	        next_pose.orientation.w = 0.7071

		next_pose.position.y += 0.0255 * pos
	else:
        	next_pose.orientation.x = 0.499
        	next_pose.orientation.y = -0.499
        	next_pose.orientation.z = 0.501
        	next_pose.orientation.w = 0.501

		next_pose.position.y = block_pose.position.y + 0.025
		next_pose.position.x = block_pose.position.x - 0.025
	
		next_pose.position.x += 0.0255 * pos
		
#	spawn_new_block(blockid, next_pose)

	new_block_state = ModelState()
	new_block_state.model_name = blockid
	new_block_state.pose = next_pose
	new_block_state.twist = Twist()
	new_block_state.reference_frame = "world"

	set_model_state(new_block_state)

	del(block_pos_to_id[blockpos])

	block_pos_to_id[pos + (len(stack_state)-1) * 3] = blockid
	stack_state[len(stack_state) - 1][pos] = 1

	toPub = MoveResponse()

	toPub.data = generateStateString()

	return toPub

def resetStack(req):
	delete_all_blocks()

if __name__=='__main__':

	rospy.init_node('jenga_block_manager')

	rospy.sleep(2.0)

	rospy.wait_for_service("gazebo/spawn_urdf_model")
	rospy.wait_for_service("gazebo/delete_model")
	rospy.wait_for_service("gazebo/get_model_state")

	gazebo_spawn_model_srv = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
	gazebo_delete_model_srv = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
	set_model_state = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)

	spawn_srv = gazebo_spawn_model_srv
	delete_srv = gazebo_delete_model_srv

	gazebo_model_states_srv = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
	model_states = gazebo_model_states_srv

	state_pub = rospy.Publisher('/game_msgs/state', std_msgs.msg.String, queue_size=10)
	system_reset_pub = rospy.Publisher('/game_msgs/reset', Empty, queue_size=10)
	rospy.Subscriber("/game_msgs/rebuild_stack", Empty, resetStack)

	s = rospy.Service('/game_msgs/move', Move, moveBlock)

	rospy.sleep(2.0)

	desired_frequency = rospy.get_param('~frequency', default = 1.0)
	stack_height = rospy.get_param('~stack_height', default = 3.0)
#	err = rospy.get_param('~block_height_err', default = 0.005)

	block_pose = Pose()
    
	block_pose.position.x = rospy.get_param('~position_x')
	block_pose.position.y = rospy.get_param('~position_y')
	block_pose.position.z = rospy.get_param('~position_z')
    
	block_pose.orientation.x = rospy.get_param('~orientation_x')
	block_pose.orientation.y = rospy.get_param('~orientation_y')
	block_pose.orientation.z = rospy.get_param('~orientation_z')
	block_pose.orientation.w = rospy.get_param('~orientation_w')


	block_urdf = rospy.get_param('~block_description')

	stack_constructed = False

	sleep_time = 1.0 / desired_frequency
	while not rospy.is_shutdown():
		rospy.sleep(sleep_time)

		# Do we need to destroy all blocks?

		if stack_constructed and hasFallen():
			delete_all_blocks()

			system_reset_pub.publish(Empty())
			rospy.sleep(sleep_time)
			rospy.sleep(sleep_time)
			stack_constructed = False
			stack_state = []

		# Do we need to add another block?
		elif not stack_constructed:
			
			spawn_all_blocks( stack_height )
			stack_constructed = True
			publishState(state_pub)

		else:
			publishState(state_pub)

#		elif stack_constructed:
#
#			import random
#
#			msg = Int32()
#			msg.data = random.randint(0, len(blocks_list) - 1)
#		
#			print("Moving block " + str(msg.data))
#
#			moveBlock(msg)

