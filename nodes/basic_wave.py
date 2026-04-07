#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from dapple import yumi


print("============ Starting tutorial setup")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()


print("============ Setting move groups ============")
groups = {
    name: moveit_commander.MoveGroupCommander(name) for name in robot.get_group_names()
}

yummels = yumi.Yumi(groups)

#group_left = moveit_commander.MoveGroupCommander("left_arm")
# group_left.set_planner_id("ESTkConfigDefault")

#group_right = moveit_commander.MoveGroupCommander("right_arm")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory, queue_size=10)


print("============ Left arm reference frame: %s ============" % groups['left_arm'].get_planning_frame())
print("============ Left arm end effector link: %s ============" % groups['left_arm'].get_end_effector_link())

print("============ Robot Groups: ============")
print(robot.get_group_names())

# Groups: both_arms, left_arm, left_gripper, right_arm, right_gripper

print("============ Printing robot state ============")
print(robot.get_current_state())
print("============")


print("============ Generating plan_left ============")
group_variable_values = groups['left_arm'].get_current_joint_values()
group_variable_values[0] = 1.0

left_move = yumi.MoveToJointState('left_arm', group_variable_values)
planning_result = yummels.plan(left_move)
# groups['left_arm'].set_joint_value_target(group_variable_values)
# plan_left = groups['left_arm'].plan()

print("============ Waiting while RVIZ displays plan_left... ============")
rospy.sleep(10)

print("============ Visualizing plan_left ============")
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(planning_result.plan)
display_trajectory_publisher.publish(display_trajectory)

print("============ Waiting while plan_left is visualized (again)... ============")
rospy.sleep(5)

yummels.execute(planning_result)


print("============ Generating plan_left ============")
group_variable_values = groups['right_arm'].get_current_joint_values()

group_variable_values[0] = 1.0
groups['right_arm'].set_joint_value_target(group_variable_values)

plan_right = groups['right_arm'].plan()

print("============ Waiting while RVIZ displays plan_left... ============")
rospy.sleep(10)


print("============ Visualizing plan_left ============")
display_trajectory = moveit_msgs.msg.DisplayTrajectory()

display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan_right)
display_trajectory_publisher.publish(display_trajectory)

print("============ Waiting while plan_left is visualized (again)... ============")
rospy.sleep(5)

groups['right_arm'].go(wait=True)


rospy.sleep(5)

yumi.go_neutral(yummels)

# Stop Node
moveit_commander.roscpp_shutdown()