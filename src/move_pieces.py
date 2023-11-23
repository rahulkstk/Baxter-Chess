#!/usr/bin/env python

import rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
import sys
import copy
from chess_baxter.srv import SpawnPiece

board_setup = None
piece_names = None
piece_positionmap = None
start_coordinates = None

def setup_chessboard(listener, pub_pose):

    spawn_piece = rospy.ServiceProxy("SpawnPiece", SpawnPiece)
    for x in piece_names:
        spawned = spawn_piece(x)
        place_coordinate = start_coordinates[x]
        now = rospy.Time()
        listener.waitForTransform('/base', x, now, rospy.Duration(4.0))
        (trans,rot) = listener.lookupTransform('/base', x, now)
        pointToPick = Point(x=trans[0], y = trans[1], z = trans[2])
        pose_array = PoseArray()
        pose_array.poses.append(Pose(position = pointToPick))
        pose_array.poses.append(Pose(position=(Point(x=place_coordinate[0], y=place_coordinate[1], z=place_coordinate[2]))))
        # print(pose_array.poses[0])
        pub_pose.publish(pose_array)
        rospy.sleep(15)
        # print(pointToPick)

def hardCodedMoves(listener, pub_pose):
    
    spawn_piece = rospy.ServiceProxy("SpawnPiece", SpawnPiece)
    now = rospy.Time()
    listener.waitForTransform('/base', 'R0', now, rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform('/base', 'R0', now)
    pointToPick = Point(x=trans[0], y = trans[1], z = trans[2])
    pose_array = PoseArray()
    pose_array.poses.append(Pose(position = pointToPick))
    pose_array.poses.append(Pose(position=(Point(x=piece_positionmap['20'][0], y=piece_positionmap['20'][1], z=trans[2]))))
    pub_pose.publish(pose_array)

    listener.waitForTransform('/base', 'k2', now, rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform('/base', 'k2', now)
    pointToPick = Point(x=trans[0], y = trans[1], z = trans[2])
    pose_array = PoseArray()
    pose_array.poses.append(Pose(position = pointToPick))
    pose_array.poses.append(Pose(position=(Point(x=piece_positionmap['33'][0], y=piece_positionmap['33'][1], z=trans[2]))))
    pub_pose.publish(pose_array)

    listener.waitForTransform('/base', 'P6', now, rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform('/base', 'P6', now)
    pointToPick = Point(x=trans[0], y = trans[1], z = trans[2])
    pose_array = PoseArray()
    pose_array.poses.append(Pose(position = pointToPick))
    pose_array.poses.append(Pose(position=(Point(x=piece_positionmap['36'][0], y=piece_positionmap['36'][1], z=trans[2]))))
    pub_pose.publish(pose_array)

    listener.waitForTransform('/base', 'k2', now, rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform('/base', 'k2', now)
    pointToPick = Point(x=trans[0], y = trans[1], z = trans[2])
    pose_array = PoseArray()
    pose_array.poses.append(Pose(position = pointToPick))
    pose_array.poses.append(Pose(position=(Point(x=piece_positionmap['34'][0], y=piece_positionmap['34'][1], z=trans[2]))))
    pub_pose.publish(pose_array)

    listener.waitForTransform('/base', 'K4', now, rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform('/base', 'K4', now)
    pointToPick = Point(x=trans[0], y = trans[1], z = trans[2])
    pose_array = PoseArray()
    pose_array.poses.append(Pose(position = pointToPick))
    pose_array.poses.append(Pose(position=(Point(x=piece_positionmap['53'][0], y=piece_positionmap['53'][1], z=trans[2]))))
    pub_pose.publish(pose_array)



def main():

    global board_setup
    global piece_names
    global piece_positionmap
    global start_coordinates

    rospy.init_node("move_pieces")
    rospy.wait_for_service("SpawnPiece")

    listener = tf.TransformListener()
    pub_pose = rospy.Publisher('/chess_baxter/pose', PoseArray, queue_size=20)

    board_setup = rospy.get_param('board_setup')
    piece_names = rospy.get_param('piece_names')
    piece_positionmap = rospy.get_param('piece_target_position_map')
    start_coordinates = rospy.get_param('start_coordinates')
    print(start_coordinates)

    setup_chessboard(listener, pub_pose)
    hardCodedMoves(listener, pub_pose)



if __name__ == '__main__':
    sys.exit(main())

