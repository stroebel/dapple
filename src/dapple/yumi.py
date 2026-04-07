from collections import namedtuple

MoveToJointState = namedtuple('MoveToJointState', ['arm','joints'])
MoveToSingleJointState = namedtuple('MoveToSingleJointState', ['arm', 'joint', 'state'])
MoveToPoseState = namedtuple('MoveToPoseState', ['arm','pose'])


MoveResult = namedtuple('MoveResult', ['success','plan','error', 'arm'])

def ok_result(plan, arm):
    return MoveResult(success=True, plan=plan, error=None, arm=arm)

def err_result(error):
    return MoveResult(success=False, plan=None, error=error, arm=None)



class Yumi(object):
    def __init__(self, move_groups):
        self._groups = move_groups

    def execute(self, result):
        if not result.success or result.plan is None:
            return err_result("No valid plan")
        
        group = self._groups[result.arm]
        success = group.execute(result.plan, wait=True)
        return ok_result(result.plan, result.arm) if success else err_result("Execution failed")

    def get_joint_values(self, arm):
        return self._groups[arm].get_current_joint_values()

    def get_current_pose(self, arm):
        return self._groups[arm].get_current_pose()
    
    def get_groups(self):
        return self._groups.keys()

    def plan(self, command):
        if isinstance(command, MoveToSingleJointState):
            return self._plan_move_joint(command.arm, command.joint, command.state)
        if isinstance(command, MoveToJointState):
            return self._plan_move_joints(command.arm, command.joints)
        elif isinstance(command, MoveToPoseState):
            return self._plan_move_cartesian(command.arm, command.pose)
        else:
            raise ValueError("Unknown command")

    def plan_and_execute(self, command):
        return self.execute(self.plan(command))

    def _plan_move_joint(self, arm, joint, state):
        group = self._groups[arm]
        joints = group.get_current_joint_values()

        # both_arms need special handling
        if arm == 'both_arms':
            # joint _should_ be an iterable
            # state _should_ be an iterable
            joints[joint[0]] = state[0]
            joints[joint[1]] = state[1]
        else:
            joints[joint] = state

        group.set_joint_value_target(joints)
        plan = group.plan()

        return ok_result(plan, arm) if plan else err_result("Plan failed")

    def _plan_move_joints(self, arm, joints):
        group = self._groups[arm]
        group.set_joint_value_target(joints)
        # Having the right documentation makes writing the code much easier
        # https://docs.ros.org/en/kinetic/api/moveit_commander/html/move__group_8py_source.html#l00505
        plan =  group.plan()
        # Noetic docs indicate that plans can have success flags. 
        # I am going to keep this in despite the fact that a plan will always return
        return ok_result(plan, arm) if plan else err_result("Plan failed")
    
    def _plan_move_cartesian(self, arm, pose):
        group = self._groups[arm]
        group.set_pose_target(pose)
        plan =  group.plan()
        # See comment above about Noetic success flags for plans
        return ok_result(plan, arm) if plan else err_result("Plan failed")
