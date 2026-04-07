#!/usr/bin/env python

import sys
import os
import signal
import readline
import atexit
from collections import namedtuple

import rospy
import moveit_commander

from dapple import yumi

PrintQuery = namedtuple('PrintQuery', ['arm', 'kind'])

def signal_handler(sig, frame):
    print('\nInterrupt received! Shutting down...')
    rospy.signal_shutdown("Shutting down")
    sys.exit(0)

# Register the handler for SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)

class SafeExit(Exception):
    pass

ARMS = {'left_arm','right_arm','both_arms', 'left_gripper','right_gripper'}
JOINTS_PER_ARM = 7 # This will be sad when using grippers. Might need to do this dynamically later

HELP_TEXT = """Commands:
  <arm> joint <index> <value>       Set a single joint (e.g. left_arm joint 2 3.2)
  <arm> nudge joint <index> <delta> Nudge a joint by delta (e.g. left_arm nudge joint 2 0.1)
  <arm> joints <v1> <v2> ...       Set all joint values
  print <arm> joints               Print current joint values
  print <arm> pose                 Print current pose
  print groups                     Print available move groups
  help                             Show this help
  exit                             Quit

Arms: left_arm, right_arm, both_arms, left_gripper, right_gripper
"""
def parse_command(cmd):
    # Not validating commands. Running them is validation enough for this
    line = cmd.split()

    if line[0].lower() == 'print':
        sub_cmd = line[1].lower()
        if sub_cmd == 'groups':
            return PrintQuery(arm=None, kind='groups')
        
        # Since subcommand is handled, reparse as arm
        # Ugly but fine
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

    # Checking for joint because we might need to handle pose later
    elif line[1] == 'nudge' and line[2] == 'joint':
        joint_index = int(line[3])
        delta = float(line[4])

        if group == 'both_arms':
            joint_index = (joint_index, joint_index + JOINTS_PER_ARM)
            delta = (delta, delta)

        return yumi.NudgeJointState(arm=group, joint=joint_index, delta=delta)

    elif line[1] == 'joint':
        joint_index = int(line[2])
        joint_val = float(line[3])

        if group == 'both_arms':
            joint_index = (joint_index, joint_index + JOINTS_PER_ARM)
            joint_val = (joint_val, joint_val)

        return yumi.MoveToSingleJointState(arm=group, joint=joint_index, state=joint_val)
    else:
        raise ValueError("Unknown command")

def start_control():

    histfile = os.path.expanduser("~/.dapple_history")
    try:
        readline.read_history_file(histfile)
    except IOError:
        pass
    atexit.register(readline.write_history_file, histfile)

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('manual_elbows',
                anonymous=True)

    try:
        robot = moveit_commander.RobotCommander()
    except Exception as e:
        rospy.logerr("Failed to set up robot and scene. Is yumi_launch running?")
        rospy.logerr(e)
        raise SafeExit("Robot is not available")

    groups = {
        name: moveit_commander.MoveGroupCommander(name) for name in robot.get_group_names()
    }

    yummels = yumi.Yumi(groups)

    print("You can issue commands. Type 'help' for usage.\n")
    last_cmd = None
    running = True
    while running:
        try:
            cmd = raw_input('> ')
        except Exception as e:
            rospy.logerr(e)
            continue

        if not cmd.strip():
            if last_cmd is None:
                continue
            cmd = last_cmd
        else:
            last_cmd = cmd

        if cmd.lower() == "exit":
            raise SafeExit("Shutting down")

        if cmd.lower() == "help":
            print(HELP_TEXT)
            continue

        try:
            cmd = parse_command(cmd)
        except (ValueError, IndexError) as e:
            rospy.logwarn("Invalid command: %s" % e)
            continue

        if isinstance(cmd, PrintQuery):
            if cmd.kind == 'joints':
                print(yummels.get_joint_values(cmd.arm))
            elif cmd.kind == 'groups':
                print(yummels.get_groups())
            else:
                print(yummels.get_current_pose(cmd.arm))
            continue

        print("Moving arm: %s" % cmd.arm)
        try:
            yummels.plan_and_execute(cmd)
            print("Result: %s" % yummels.get_joint_values(cmd.arm))
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
