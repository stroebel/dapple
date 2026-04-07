#!/usr/bin/env python

import sys
import signal
from collections import namedtuple

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from dapple import yumi

from collections import namedtuple

PrintQuery = namedtuple('PrintQuery', ['arm', 'kind'])

def signal_handler(sig, frame):
    print('\nInterrupt received! Shutting down...')
    rospy.signal_shutdown("Shutting down")
    sys.exit(0)

# Register the handler for SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)

class SafeExit(Exception):
    pass

ARMS = {'left_arm','right_arm','both_arms'}

# example: 'left_arm joint 2 3.2' -> Set target joint to target value
# example: 'both_arms joints 3 23 22 ...' -> Set all joint values
# example: 'right_arm pose 32 42 ...' -> Set pose value
# example: 'print left_arm joints' -> Print current joint values
def parse_command(cmd):
    # Not validating commands. Running them is validation enough for this
    line = cmd.split()

    if line[0].lower() == 'print':
        arm = line[1].lower()
        if arm not in ARMS:
            raise ValueError("Unknown arm: %s" % arm)
        kind = line[2].lower()
        if kind not in ('joints', 'pose'):
            raise ValueError("Unknown print type: %s (use 'joints' or 'pose')" % kind)
        return PrintQuery(arm=arm, kind=kind)

    group = line[0].lower()

    if group not in ARMS:
        raise ValueError("Unknown arm: %s" % group)

    # handle pose
    if line[1] == 'pose':
        raise ValueError("Pose is broken for now. ")
        # return yumi.MoveToPoseState(arm=group, pose=[float(v) for v in line[2:]])
    # handle simple joint command, i.e. full state
    elif line[1] == 'joints':
        return yumi.MoveToJointState(arm=group, joints=[float(v) for v in line[2:]])

    elif line[1] == 'joint':
        # Not doing any type verification here. Just don't pass in the wrong thing :)
        joint_index = int(line[2])
        # Slightly annoying that I don't have actual joint values here
        joint_val = float(line[3])
        # Also ignoring the rest of the line

        # both_arms need special handling
        if group == 'both_arms':
            # Because Yumi has 7 joints
            joint_index = (joint_index, joint_index + 7) # Handle the joint concat of the arms
            joint_val = (joint_val, joint_val)

        return yumi.MoveToSingleJointState(arm=group, joint=joint_index, state=joint_val)
    else:
        raise ValueError("Unknown command")

def start_control():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('manual_elbows',
                anonymous=True)

    try:
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
    except Exception as e:
        rospy.logerr("Failed to set up robot and scene. Is yumi_launch running?")
        rospy.logerr(e)
        raise SafeExit("Robot is not available")

    groups = {
        name: moveit_commander.MoveGroupCommander(name) for name in robot.get_group_names()
    }

    yummels = yumi.Yumi(groups)

    print("You can issue commands.\n")
    running = True
    while running:
        try:
            cmd = raw_input('> ')
        except Exception as e:
            rospy.logerr(e)
            continue

        if not cmd.strip():
            continue

        if cmd.lower() == "exit":
            raise SafeExit("Shutting down")

        try:
            cmd = parse_command(cmd)
        except (ValueError, IndexError) as e:
            rospy.logwarn("Invalid command: %s" % e)
            continue

        if isinstance(cmd, PrintQuery):
            if cmd.kind == 'joints':
                print(yummels.get_joint_values(cmd.arm))
            else:
                print(yummels.get_current_pose(cmd.arm))
            continue

        print("Moving arm: %s" % cmd.arm)
        try:
            yummels.plan_and_execute(cmd)
        # To capture oob errors
        except Exception as e:
            rospy.logerr(e)
            continue

if __name__ == "__main__":
    try:
        start_control()
    except (KeyboardInterrupt, rospy.ROSInterruptException, SafeExit) as e:
        rospy.loginfo("Shutting down")
    except Exception as e:
        rospy.logerr(e)
    finally:
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("Shutting down")
