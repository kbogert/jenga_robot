
#include <ros/ros.h>
#include <tf/tf.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <widowx_block_manipulation/PickAndPlaceAction.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/String.h>

#include <boost/scoped_ptr.hpp>

geometry_msgs::PoseStamped block; //New

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
	   
     }

  
  /**
   * Move arm to a target pose. Only position coordinates are taken into account; the
   * orientation is calculated according to the direction and distance to the target.
   * @param target Pose target to achieve
   * @return True of success, false otherwise
   */
  bool moveArmTo(const geometry_msgs::Pose& target)
  {

    arm_.setPoseReferenceFrame("wrist_2_link");

    // Allow some leeway in position (meters) and orientation (radians)
    arm_.setGoalPositionTolerance(0.001);
    arm_.setGoalOrientationTolerance(0.1);

    // Allow replanning to increase the odds of a solution
    arm_.allowReplanning(true);



    int attempts = 0;
    ROS_INFO("[pick and place] Move arm to [%.2fx, %.2fy, %.2fz, %.2fyaw]",
             target.position.x, target.position.y, target.position.z, tf::getYaw(target.orientation));
    while (attempts < 5)
    {
      geometry_msgs::PoseStamped modiff_target;
      modiff_target.header.frame_id = "wrist_2_link";
      modiff_target.pose = target;

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
      // Pitch is 90 (vertical) at 10 cm from the arm base; the farther the target is, the closer to horizontal
      // we point the gripper. Yaw is the direction to the target. We also try some random variations of both to
      // increase the chances of successful planning.
      double rp = M_PI_2 - std::asin((d - 0.1)/0.29); // 0.29 = arm's max reach - vertical pitch distance + ε
      double ry = std::atan2(y, x);


      //x0, y1, z0, w-0.2
      //tf::Quaternion q = tf::createQuaternionFromRPY(0.0,
      //                                               attempts*fRand(-0.05, +0.05) + rp,
      //                                               attempts*fRand(-0.05, +0.05) + ry);
      tf::Quaternion q = tf::createQuaternionFromRPY(3.14,
                                                     attempts*fRand(-0.05, +0.05) /* + rp*/,
                                                     -3.14+attempts*fRand(-0.05, +0.05) + ry);
      tf::quaternionTFToMsg(q, modiff_target.pose.orientation);

      // Slightly increase z proportionally to pitch to avoid hitting the table with the lower gripper corner
      ROS_INFO("z increase:  %f  +  %f", modiff_target.pose.position.z, std::abs(std::cos(rp))/50.0);
      modiff_target.pose.position.z += std::abs(std::cos(rp))/50.0;

      ROS_INFO("Set pose target [%.2f, %.2f, %.2f] [d: %.2f, p: %.2f, y: %.2f]", x, y, z, d, rp, ry);
     bool check = arm_.setPoseTarget(modiff_target, arm_.getEndEffectorLink());

      if (check == false)
      {
        ROS_ERROR("Set pose target [%.2f, %.2f, %.2f, %.2f] failed",
                  modiff_target.pose.position.x, modiff_target.pose.position.y, modiff_target.pose.position.z,
                  tf::getYaw(modiff_target.pose.orientation));
        as_.setAborted(result_);
        return false;
      }

      moveit::planning_interface::MoveItErrorCode result = arm_.move();
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







int main(int argc, char** argv)
{
ros::init(argc, argv, "jenga_robot");
ros::NodeHandle nh;
ros::Subscriber block_position;
ros::AsyncSpinner spinner(1);
spinner.start();
widowx_block_manipulation::PickAndPlaceServer server("pick_and_place");
block_position = nh.subscribe("/game_msgs/new_blocks", 1000, server.callback); //New

server.setGripper(0.03);
ros::Duration(0.8).sleep();
server.moveArmTo(block.pose);
server.setGripper(0.0);
ros::Duration(0.8).sleep();

spinner.stop();





  
  ROS_INFO("Done");
 

  return 0;
}







