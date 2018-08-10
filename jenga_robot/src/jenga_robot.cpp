
#include <ros/ros.h>
#include <tf/tf.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <widowx_block_manipulation/PickAndPlaceAction.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/String.h>

#include <boost/scoped_ptr.hpp>

#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"




geometry_msgs::PoseStamped block; 
geometry_msgs::PoseStamped adjusted_block;
bool have_block;



namespace widowx_block_manipulation
{

class PickAndPlaceServer
{
private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<widowx_block_manipulation::PickAndPlaceAction> as_;
  std::string action_name_;

  widowx_block_manipulation::PickAndPlaceFeedback     feedback_;
  widowx_block_manipulation::PickAndPlaceResult       result_;
  widowx_block_manipulation::PickAndPlaceGoalConstPtr goal_;

  ros::Publisher target_pose_pub_;
  ros::Subscriber pick_and_place_sub_;
  
  
  

  // Move groups to control arm and gripper with MoveIt!
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::MoveGroupInterface gripper_;
  

  // Parameters from goal
  std::string arm_link;
  double gripper_open;
  double gripper_closed;
  double z_up;

public:


  PickAndPlaceServer(const std::string name) :
    nh_("~"), as_(name, false), action_name_(name), arm_("jenga_robot"), gripper_("jenga_gripper")
  {
    // Register the goal and feedback callbacks
    


    as_.start();

    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_pose", 1, true);
    
  }
  

  static void callback(const geometry_msgs::PoseStamped& msg)
     { 
	block=msg;
	adjusted_block=msg;
	have_block = true;
	//adjusted for gripper positioning
        adjusted_block.pose.orientation.w=0; //0
	adjusted_block.pose.orientation.x=0; //0
	adjusted_block.pose.orientation.y=-1; // -1 
	adjusted_block.pose.orientation.z=0;  //0
     }
  void pickBlock(const std::string &object)
     {
        arm_.planGraspsAndPick(object);
     }
  void placeBlock(const std::string &object)
     {
        arm_.place(object);
     }

  
  /**
   * Move arm to a target pose. Only position coordinates are taken into account; the
   * orientation is calculated according to the direction and distance to the target.
   * @param target Pose target to achieve
   * @return True of success, false otherwise
   */
  bool moveArmTo(const geometry_msgs::Pose& target, double z_addition)
  {


    // Allow some leeway in position (meters) and orientation (radians)
    
    arm_.setGoalPositionTolerance(0.01);
    arm_.setGoalOrientationTolerance(0.01);
    

    // Allow replanning to increase the odds of a solution
    arm_.allowReplanning(true); 
    arm_.clearPoseTargets();

    int attempts = 0;
    ROS_INFO("[pick and place] Move arm to [%.2fx, %.2fy, %.2fz, %.2fyaw]",
             target.position.x, target.position.y, target.position.z, tf::getYaw(target.orientation));

    while (attempts < 20)
    {

      geometry_msgs::PoseStamped modiff_target;
      modiff_target.header.frame_id = "/world"; 
      modiff_target.pose = target;

      modiff_target.pose.position.z += z_addition;


	/*modiff_target.pose.orientation.w=0; //0
	modiff_target.pose.orientation.x=0; //0
	modiff_target.pose.orientation.y=-1; // -1 
	modiff_target.pose.orientation.z=0;  //0
	//modiff_target.pose.position.y=-0.1516; //mirror across x axis from block
	*/

      double x = modiff_target.pose.position.x;
      double y = modiff_target.pose.position.y;
      double z = modiff_target.pose.position.z;
      double d = sqrt(x*x + y*y);
      if (d > 0.3) 
      {
        // Maximum reachable distance by the arm is 30 cm
        ROS_ERROR("Target pose out of reach [%f > %f]", d, 0.3);
        as_.setAborted(result_);
        return false;
      }
      
      

      // Pitch is 90 (vertical) at 10 cm from the arm base; the farther the target is, the closer to horizontalquaternion
      // we point the gripper. Yaw is the direction to the target. We also try some random variations of both to
      // increase the chances of successful planning.
      double rp = M_PI_2 - std::asin((d - 0.1)/0.29); // 0.29 = arm's max reach - vertical pitch distance + ε
      double ry = std::atan2(y, x);



      /*tf::Quaternion q = tf::createQuaternionFromRPY(3.14,
                                                     attempts*fRand(-0.05, +0.05),
                                                     3.14+attempts*fRand(-0.05, +0.05) + ry);

      tf::quaternionTFToMsg(q, modiff_target.pose.orientation);*/

      // Slightly increase z proportionally to pitch to avoid hitting the table with the lower gripper corner
//      ROS_INFO("z increase:  %f  +  %f", modiff_target.pose.position.z, std::abs(std::cos(rp))/50.0);
//      modiff_target.pose.position.z += std::abs(std::cos(rp))/50.0;

      ROS_INFO("Set pose target [%.2f, %.2f, %.2f] [d: %.2f, p: %.2f, y: %.2f]", x, y, modiff_target.pose.position.z, d, rp, ry);

     
     bool check = arm_.setPoseTarget(modiff_target, "wrist_2_link");
      
//      bool check = arm_.setPositionTarget(modiff_target.pose.position.x, modiff_target.pose.position.y, modiff_target.pose.position.z, "wrist_2_link");
//      check = check & arm_.setRPYTarget(0, 0, 3.14 /2, "wrist_2_link");

      if (check == false)
      {
        ROS_ERROR("Set pose target [%.2f, %.2f, %.2f, %.2f] failed",
                  modiff_target.pose.position.x, modiff_target.pose.position.y, modiff_target.pose.position.z,
                  tf::getYaw(modiff_target.pose.orientation));
        as_.setAborted(result_);
        return false;
      }
      
      //moveit::planning_interface::MoveGroupInterface::Plan plan;
      //arm_.plan(plan);
      //ros::Duration(7.0).sleep();//Added to give plan() enough time to complete
     // moveit::planning_interface::MoveItErrorCode result = arm_.execute(plan);
      moveit::planning_interface::MoveItErrorCode result = arm_.move();
      ros::Duration(2.0).sleep();

      if (bool(result) == true)
      {
        return true;
      }
      else
      {
        ROS_ERROR("[pick and place] Move to target failed (error %d) at attempt %d",
                  result.val, attempts + 1);
      }
      attempts++;
    }
   

    ROS_ERROR("[pick and place] Move to target failed after %d attempts", attempts);
    as_.setAborted(result_);
    return false;
  }

  /**
   * Set gripper opening.
   * @param opening Physical opening of the gripper, in meters
   * @return True of success, false otherwise
   */
  bool setGripper(float opening)
  {
    ROS_DEBUG("[pick and place] Set gripper opening to %f", opening);
    if (gripper_.setJointValueTarget("gripper_joint", opening / 2) == false)
    {
      ROS_ERROR("[pick and place] Set gripper opening to %f failed", opening);
      return false;
    }

    moveit::planning_interface::MoveItErrorCode result = gripper_.move();
    if (bool(result) == true)
    {
      return true;
    }
    else
    {
      ROS_ERROR("[pick and place] Set gripper opening failed (error %d)", result.val);
      as_.setAborted(result_);
      return false;
    }
  }

  float fRand(float min, float max)
  {
    return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
  }
};

};





bool add_collision_block(const geometry_msgs::Pose& position, const std::string block_number)
{
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
while(! have_block )
	ros::Duration(0.5).sleep();

  /* Put an object into the scene*/

  /* Define the object message */
moveit_msgs::CollisionObject collision_object;
  /* The TF frame */
collision_object.header.frame_id = "/world";
  /* The id of the object */
collision_object.id = block_number;

  /* Make box at current pose of block */
geometry_msgs::Pose currentBlockPose;
currentBlockPose=position; //block.pose
currentBlockPose.position.z+=.0005; //Added so block is not in collision with base

  /* Block definition */
shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);  
primitive.dimensions[0] = 0.015;
primitive.dimensions[1] = 0.025;
primitive.dimensions[2] = 0.075;

collision_object.primitives.push_back(primitive);
collision_object.primitive_poses.push_back(currentBlockPose);
collision_object.operation=collision_object.ADD;

std::vector<moveit_msgs::CollisionObject> collision_objects;  
collision_objects.push_back(collision_object); 
 
   /*Add object into world*/

planning_scene_interface.addCollisionObjects(collision_objects);
sleep(2.0);

}




int main(int argc, char** argv)
{
ros::init(argc, argv, "jenga_robot");
ros::NodeHandle nh;
ros::Subscriber block_position;



//Attach client
ros::ServiceClient attach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
gazebo_ros_link_attacher::Attach req;

//Detach client
ros::ServiceClient detach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
gazebo_ros_link_attacher::Attach req1;

//define stack location
geometry_msgs::PoseStamped stack;
stack.header.frame_id = "/world"; 
stack.pose.position.x=0;
stack.pose.position.y=0;
stack.pose.position.z=0.3; //initial z position
stack.pose.orientation.w=0;
stack.pose.orientation.x=0;
stack.pose.orientation.y=-1;
stack.pose.orientation.z=0;

//define stack_turned location
geometry_msgs::PoseStamped stack_turned;
stack_turned.header.frame_id = "/world"; 
stack_turned.pose.position.x=0;
stack_turned.pose.position.y=0;
stack_turned.pose.position.z=0.3; //initial z position
stack_turned.pose.orientation.w=0;
stack_turned.pose.orientation.x=0.707;
stack_turned.pose.orientation.y=-0.707;
stack_turned.pose.orientation.z=0;



have_block = false;

widowx_block_manipulation::PickAndPlaceServer server("pick_and_place");
block_position = nh.subscribe("/game_msgs/new_blocks", 1000, server.callback); 

ros::AsyncSpinner spinner(1);
spinner.start();






/************************************************************************************/

add_collision_block(block.pose, "permanent_block");

/*********************************************************************************/


server.setGripper(0.031);

while(! have_block )
	ros::Duration(0.5).sleep();



//PICK UP FIRST BLOCK
// move arm above block
server.moveArmTo(adjusted_block.pose, 0.15); 
// move arm down to block
server.moveArmTo(adjusted_block.pose, 0.1);

//server.setGripper(0.02);

// attach link  
req.request.model_name_1 = "robot";
req.request.link_name_1 = "gripper_2_link";
req.request.model_name_2 = "block1";
req.request.link_name_2 = "jenga_block_link";
if (attach_client.call(req))
  {
    ROS_INFO("Successfully made attach request.");
  }
  else
  {
    ROS_ERROR("Failed to make attach request.");
  }

//MOVE BLOCK TO BOTTOM OF STACK
server.moveArmTo(stack.pose, 0.15);
server.moveArmTo(stack.pose, 0.1);

//detach link 
req1.request.model_name_1 = "robot";
req1.request.link_name_1 = "gripper_2_link";
req1.request.model_name_2 = "block1";
req1.request.link_name_2 = "jenga_block_link";
if (detach_client.call(req1))
  {
    ROS_INFO("Successfully made detach request.");
  }
  else
  {
    ROS_ERROR("Failed to make detach request.");
  }

//PICK UP SECOND BLOCK
// move arm above block
server.moveArmTo(adjusted_block.pose, 0.15); 
// move arm down to block
server.moveArmTo(adjusted_block.pose, 0.1);

//server.setGripper(0.02);

// attach link  
req.request.model_name_1 = "robot";
req.request.link_name_1 = "gripper_2_link";
req.request.model_name_2 = "block2";
req.request.link_name_2 = "jenga_block_link";
if (attach_client.call(req))
  {
    ROS_INFO("Successfully made attach request.");
  }
  else
  {
    ROS_ERROR("Failed to make attach request.");
  }

//MOVE BLOCK TO SECOND POSITION IN STACK AT 90 DEGREES
server.moveArmTo(stack_turned.pose, 0.2);
server.moveArmTo(stack_turned.pose, 0.15);

//detach link 
req1.request.model_name_1 = "robot";
req1.request.link_name_1 = "gripper_2_link";
req1.request.model_name_2 = "block2";
req1.request.link_name_2 = "jenga_block_link";
if (detach_client.call(req1))
  {
    ROS_INFO("Successfully made detach request.");
  }
  else
  {
    ROS_ERROR("Failed to make detach request.");
  }


ros::Duration(0.8).sleep();

ros::waitForShutdown();

spinner.stop();





  
  ROS_INFO("Done");
 

  return 0;
}







