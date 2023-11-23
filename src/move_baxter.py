#!/usr/bin/env python

# ROS Python API
import rospy
import sys
import copy
import rospkg

# Baxter SDK that provides high-level functions to control Baxter
import baxter_interface

# Pose is the defacto message to store a 3D point and a quaternion,
# head over ROS API to find out the structure!
# PoseStamped consists of a Header and a Pose messages.
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
    PoseArray
)

from std_msgs.msg import (
    Empty,
)

import moveit_commander


class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")

        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        # We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")
        # self._group.set_num_planning_attempts = 20

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))

        self.gripper_open()
        self._group.set_pose_target(start_angles)
        
        succeded = False
        while succeded == False:
            plan = self._group.plan()
            succeded = self._group.execute(plan)

        print(succeded)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance

        self._group.set_pose_target(approach)
        succeded = False
        while succeded == False:
            plan = self._group.plan()
            succeded = self._group.execute(plan)
    
    def _get_pose(self):
        current_pose = self._limb.endpoint_pose()
        return current_pose['position']

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        # servo up from current pose
        self._group.set_pose_target(ik_pose)
        succeded = False
        while succeded == False:
            plan = self._group.plan()
            succeded = self._group.execute(plan)

    def _servo_to_pose(self, pose):
        # servo down to release
        self._group.set_pose_target(pose)
        succeded = False
        while succeded == False:
            plan = self._group.plan()
            succeded = self._group.execute(plan)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

    def pose_callback(self, pose_msg):
        # An orientation for gripper fingers to be overhead and parallel to the obj
        overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)

        starting_pose = Pose(position=Point(x=0.7, y=0.135, z=0.35),
            orientation=overhead_orientation)

        # Move Baxter to a pre-picking position
        self.move_to_start(starting_pose)

        # Define the block pose. You have to move 2cm down in order to pick it as
        # the centroid is on the surface of the block
        block_pose_pick = Pose(
            position=Point(x=pose_msg.poses[0].position.x, y=pose_msg.poses[0].position.y, z=pose_msg.poses[0].position.z),
            orientation=overhead_orientation)

        # Define the placing pose, 5 cm to the right or left (self.alternate sets the direction)
        block_pose_place = Pose(
            position=Point(x=pose_msg.poses[1].position.x, y=pose_msg.poses[1].position.y, z=pose_msg.poses[1].position.z),
            orientation=overhead_orientation)


        # Execute pick and place
        print("\nPicking...")
        self.pick(block_pose_pick)
        print("\nPlacing...")
        self.place(block_pose_place)
        # Move the arm away from the camera field of view

        #this one gives an error at times
        # picking_pose = Pose(position=Point(x=0.7, y=0.135, z=0.35),
        #     orientation=overhead_orientation)
        # self.move_to_start(picking_pose)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_baxter")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15  # meters
    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    # Move to the desired starting pose
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)

    starting_pose = Pose(
        position=Point(x=0.7, y=0.135, z=0.35),
        orientation=overhead_orientation)
    
    # while pnp._get_pose() != starting_pose.position:
    pnp.move_to_start(starting_pose)
    
    rospy.Subscriber('/chess_baxter/pose', PoseArray, pnp.pose_callback, queue_size=20)



    rospy.loginfo("Node ready!")
    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())
