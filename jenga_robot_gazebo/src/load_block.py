#!/usr/bin/env python


import roslib; roslib.load_manifest('jenga_robot_gazebo')
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from std_msgs.msg import Empty
from geometry_msgs.msg import *

block_urdf = None
block_pose = None

def spawn_new_block(spawn_srv, new_id):

	spawn_srv(new_id, block_urdf, "", block_pose, "world")

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
		
	avg_height = total_height / len(blocks_list)

	if avg_height < len(blocks_list) /2 * block_height:
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
	block_num = 0
	last_block_add_time = 0
	block_waiting = False


	sleep_time = 1.0 / desired_frequency
	while not rospy.is_shutdown():
		# Do we need to destroy all blocks?

		if hasFallen(gazebo_model_states_srv, blocks_list):
			delete_all_blocks(gazebo_delete_model_srv, blocks_list)
			last_block_add_time = 0
			blocks_list[:] = []
			block_num = 0
			block_waiting = False

			system_reset_pub.publish(Empty())
			rospy.sleep(sleep_time)
			rospy.sleep(sleep_time)

		# Do we need to add another block?
		if not block_waiting and rospy.get_time() - last_block_add_time > add_block_delay:
			
			block_num += 1
			newid = "block" + str(block_num)
			spawn_new_block(gazebo_spawn_model_srv, newid)
			block_waiting = True
			blocks_list.append(newid)

		# Is a block waiting to be picked up? send new_blocks message


		if block_waiting:

			cur_block_pose = getBlockPose(gazebo_model_states_srv, "block" + str(block_num))
			z_value = cur_block_pose.position.z

			if z_value > block_pose.position.z + err:
				block_waiting = False
				last_block_add_time = rospy.get_time()
			else:

				msg = PoseStamped()
				
#				msg.header.seq = block_num
				msg.header.stamp = rospy.Time.now()
				msg.header.frame_id = "world"
				msg.pose = cur_block_pose

				block_detection_pub.publish(msg)


		rospy.sleep(sleep_time)
