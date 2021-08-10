#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from pkg_vb_sim.srv import vacuumGripper
'''imported all the essential packages'''

class Ur5Moveit:

  # Constructor
  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)

    self._planning_group = "ur5_1_planning_group"
    moveit_commander.roscpp_initialize(sys.argv)
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self._commander = moveit_commander.roscpp_initialize(sys.argv)
    self._robot = moveit_commander.RobotCommander()
    self._scene = moveit_commander.PlanningSceneInterface()
    self._group = moveit_commander.MoveGroupCommander(self._planning_group)
    self._display_trajectory_publisher = rospy.Publisher(
          '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

    self._exectute_trajectory_client = actionlib.SimpleActionClient(
          'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    self._exectute_trajectory_client.wait_for_server()

    self._planning_frame = self._group.get_planning_frame()
    self._eef_link = self._group.get_end_effector_link()
    self._group_names = self._robot.get_group_names()


    rospy.loginfo(
          '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
    rospy.loginfo(
          '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
    rospy.loginfo(
          '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

    rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
    

  def add_box(self, timeout=4):
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "Gripper_Base"
    box_pose.pose.orientation.x = 0#wont work
    box_pose.pose.orientation.y = 0
    box_pose.pose.orientation.z = 0
    box_pose.pose.position.z = 0.27#0.24 # translation for y
    box_pose.pose.position.x = 0.75#0.01 # translation for x
    box_pose.pose.position.y = 1#-0.16 # above the frame
    box_name = "box"
    rospy.loginfo('Getting Something')
    scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

  def attach_box(self, timeout = 4):
  	box_name = self.box_name
  	robot = self.robot
  	scene = self.scene
  	_eef_link = self._eef_link
  	_group_names = self._group_names

  	grasping_group = 'vacuumGripper'
  	touch_links = robot.get_link_names(group = grasping_group)
  	scene.attach_box(_eef_link, box_name, touch_links = touch_links)
  	return self.wait_for_state_update(box_is_attached=True, box_is_known = False, timeout = timeout)

  def dettach_box(self, timeout = 4):
  	box_name = self.box_name
  	scene = self.scene
  	_eef_link = self._eef_link
  	scene.remove_attached_object(_eef_link, name = box_name)

  def remove_box(self, timeout = 4):
  	box_name = self.box_name
  	scene = self.scene
  	scene.remove_world_object = (box_name)

  def set_joint_angles(self, arg_list_joint_angles):

      list_joint_values = self._group.get_current_joint_values()
      rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
      rospy.loginfo(list_joint_values)

      self._group.set_joint_value_target(arg_list_joint_angles)
      self._group.plan()
      flag_plan = self._group.go(wait=True)

      list_joint_values = self._group.get_current_joint_values()
      rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
      rospy.loginfo(list_joint_values)

      pose_values = self._group.get_current_pose().pose
      rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
      rospy.loginfo(pose_values)

      if (flag_plan == True):
          rospy.loginfo(
              '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
      else:
          rospy.logerr(
              '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

      return flag_plan

    
  # Destructor
  def __del__(self):
      moveit_commander.roscpp_shutdown()
      rospy.loginfo(
          '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
  # Copy class variables to local variables to make the web tutorials more clear.
  # In practice, you should use the class variables directly unless you have a good
  # reason not to.
  box_name = "trial"
  scene = moveit_commander.PlanningSceneInterface()

  start = rospy.get_time()
  seconds = rospy.get_time()
  while (seconds - start < timeout) and not rospy.is_shutdown():
    # Test if the box is in attached objects
    attached_objects = scene.get_attached_objects([box_name])
    is_attached = len(attached_objects.keys()) > 0

    # Test if the box is in the scene.
    # Note that attaching the box will remove it from known_objects
    is_known = box_name in scene.get_known_object_names()

    # Test if we are in the expected state
    if (box_is_attached == is_attached) and (box_is_known == is_known):
      return True

    # Sleep so that we give other threads time on the processor
    rospy.sleep(0.1)
    seconds = rospy.get_time()

  # If we exited the while loop without returning then we timed out
  return False

def main():

  ur5 = Ur5Moveit()

  lst_joint_angles_1 = [math.radians(0),
                        math.radians(0),
                        math.radians(0),
                        math.radians(0),
                        math.radians(0),
                        math.radians(0)]

  lst_joint_angles_3 = [math.radians(66),
                        math.radians(-81),
                        math.radians(18),
                        math.radians(-120),
                        math.radians(-64),
                        math.radians(0)]

  lst_joint_angles_4 = [math.radians(73),
                        math.radians(-72),
                        math.radians(-55),
                        math.radians(-87),
                        math.radians(-76),
                        math.radians(0)]

  lst_joint_angles_5 = [math.radians(180),
                        math.radians(0),
                        math.radians(0),
                        math.radians(0),
                        math.radians(0),
                        math.radians(0)]

  while not rospy.is_shutdown():

    ur5.add_box()
    rospy.sleep(2)

    ur5.set_joint_angles(lst_joint_angles_3)
    rospy.sleep(2)

    ur5.add_box()
    rospy.sleep(1)

    attach_box = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
    attach_box(True)

    ur5.set_joint_angles(lst_joint_angles_4)
    rospy.sleep(2)

    ur5.set_joint_angles(lst_joint_angles_5)
    rospy.sleep(1)

    attach_box = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
    attach_box(False)

    ur5.dettach_box()
    rospy.sleep(1)

    ur5.set_joint_angles(lst_joint_angles_1)
    rospy.sleep(10)

  del ur5


if __name__ == '__main__':
  main()
