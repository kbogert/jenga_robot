#!/usr/bin/env python


import roslib; roslib.load_manifest('widowx_arm_controller')
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from std_msgs.msg import Empty
from geometry_msgs.msg import *

block_urdf = None
base_pose = None

def spawn_new_block(spawn_srv, new_id, block_pose):

	spawn_srv(new_id, block_urdf, "", block_pose, "world")

def spawn_all_blocks(spawn_srv, blocks_list, num_levels):

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

		if rotated:

			next_pose.position.y = block_pose.position.y + 0.025
			next_pose.position.x = block_pose.position.x - 0.025

		for j in range(3):
			newid = "block" + str(count)
			count += 1
			spawn_new_block(spawn_srv, newid, next_pose)
			if rotated:
				next_pose.position.x += 0.0255 # BLOCK WIDTH is .025
			else:
				next_pose.position.y += 0.0255 # BLOCK WIDTH is .025
			blocks_list.append(newid)

		next_pose.position.z += 0.015
		tmp_rotation = next_pose.orientation
		next_pose.orientation = rotation_swap.orientation
		rotation_swap.orientation = tmp_rotation

		rotated = not rotated

def delete_all_blocks(delete_srv, blocks_list):

	for block in blocks_list:
		delete_srv(block)


def hasFallen(model_states, blocks_list):


	if len(blocks_list) < 3:
		return False

	# average the z-height of each block's position, if below half of the ideal block height, we've fallen

	block_height = 0.015
	total_height = 0

	for block in blocks_list:
		total_height += model_states(block, "world").pose.position.z - block_pose.position.z
	
#	total_height /= 3

	avg_height = total_height / (len(blocks_list) / 3)

	if avg_height < (len(blocks_list) / 3) /2 * block_height:
		# we've fallen
		return True

	return False

def getBlockPose(model_states, id):

	return model_states(id, "world").pose

if __name__=='__main__':

	rospy.init_node('jenga_block_manager')

	rospy.sleep(2.0)

	rospy.wait_for_service("gazebo/spawn_urdf_model")
	rospy.wait_for_service("gazebo/delete_model")
	rospy.wait_for_service("gazebo/get_model_state")

	gazebo_spawn_model_srv = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
	gazebo_delete_model_srv = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

	gazebo_model_states_srv = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
	
	block_detection_pub = rospy.Publisher('/game_msgs/new_blocks', PoseStamped, queue_size=10)
	system_reset_pub = rospy.Publisher('/game_msgs/reset', Empty, queue_size=10)

	rospy.sleep(2.0)

	desired_frequency = rospy.get_param('~frequency', default = 1.0)
	add_block_delay = rospy.get_param('~add_block_delay_secs', default = 6.0)
	err = rospy.get_param('~block_height_err', default = 0.005)

	block_pose = Pose()
    
	block_pose.position.x = rospy.get_param('~position_x')
	block_pose.position.y = rospy.get_param('~position_y')
	block_pose.position.z = rospy.get_param('~position_z')
    
	block_pose.orientation.x = rospy.get_param('~orientation_x')
	block_pose.orientation.y = rospy.get_param('~orientation_y')
	block_pose.orientation.z = rospy.get_param('~orientation_z')
	block_pose.orientation.w = rospy.get_param('~orientation_w')


	block_urdf = rospy.get_param('~block_description')

	blocks_list = []
	stack_constructed = False

	sleep_time = 1.0 / desired_frequency
	while not rospy.is_shutdown():
		# Do we need to destroy all blocks?

		if hasFallen(gazebo_model_states_srv, blocks_list):
			delete_all_blocks(gazebo_delete_model_srv, blocks_list)
			blocks_list[:] = []

			system_reset_pub.publish(Empty())
			rospy.sleep(sleep_time)
			rospy.sleep(sleep_time)
			stack_constructed = False

		# Do we need to add another block?
		if not stack_constructed:
			
			spawn_all_blocks(gazebo_spawn_model_srv, blocks_list, 3)
			stack_constructed = True


		rospy.sleep(sleep_time)
