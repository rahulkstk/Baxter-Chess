#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy
from chess_baxter.srv import SpawnPieceResponse, SpawnPiece
# http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials

all_spawned = False
srv_call = None
orient = None
default_pose = Pose(Point(0.67,0.6,0.794951), orient)

def SpawnPiece_callback(request):
    xml = rospy.get_param('pieces_xml')
    piece_col = list(request.piece)
    print(request.piece)
    print srv_call(request.piece, xml[piece_col[0]], "", default_pose, "world")
    return SpawnPieceResponse(True)

def main():
    global srv_call
    global orient

    rospy.init_node("spawn_chessboard")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    srv_call = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    
    # Table
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')

    table_pose=Pose(position=Point(x=0.9, y=0.4, z=0.0))
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_sdf("cafe_table", table_xml, "/", table_pose, "world")
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    
    
    # ChessBoard
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    board_pose = Pose(Point(0.47,0.55,0.78), orient)
    frame_dist = 0.025
    model_path = "/home/vagrant/rf_ws/src/chess_baxter/models/"
    
    with open(model_path + "chessboard/model.sdf", "r") as f:
        board_xml = f.read().replace('\n', '')

    # Add chessboard into the simulation
    print srv_call("chessboard", board_xml, "", board_pose, "world")

    # Add chesspieces into the simulation
    origin_piece = 0.03125

    pieces_xml = dict()
    list_pieces = 'rnbqkpRNBQKP'
    for each in list_pieces:
        with open(model_path + each+".sdf", "r") as f:
            pieces_xml[each] = f.read().replace('\n', '')

    # board_setup = ['rnbqkbnr', 'pppppppp', '', '', '', '', 'PPPPPPPP', 'RNBQKBNR']
    board_setup = ['********', '********', '**k***p*', '*p******', 'R*****P*', '****K***', '********', '********']

    piece_positionmap = dict()
    piece_names = []
    piece_coordinates = dict()
    for row, each in enumerate(board_setup):
        # print row
        for col, piece in enumerate(each):
            pose = deepcopy(board_pose)
            pose.position.x = board_pose.position.x + frame_dist + origin_piece + row * (2 * origin_piece)
            pose.position.y = board_pose.position.y - 0.55 + frame_dist + origin_piece + col * (2 * origin_piece)
            pose.position.z += 0.018
            piece_positionmap[str(row)+str(col)] = [pose.position.x, pose.position.y, pose.position.z-0.93] #0.93 to compensate Gazebo RViz origin difference
            if piece in list_pieces:
                piece_names.append("%s%d" % (piece,col))
                piece_coordinates[piece + str(col)] = [pose.position.x, pose.position.y, pose.position.z-0.93]
    

    rospy.set_param('board_setup', board_setup) # Board setup
    rospy.set_param('list_pieces', list_pieces) # List of unique pieces
    rospy.set_param('piece_target_position_map', piece_positionmap) # 3D positions for each square in the chessboard
    rospy.set_param('piece_names', piece_names) # Pieces that will be part of the game
    rospy.set_param('pieces_xml', pieces_xml) # File paths to Gazebo models, i.e. SDF files
    rospy.set_param('start_coordinates', piece_coordinates)
    print(piece_positionmap)
    print len(piece_positionmap)
    rospy.Service('SpawnPiece', SpawnPiece, SpawnPiece_callback)
    
    rate = rospy.Rate(10)
    rospy.spin()

if __name__ == '__main__':
    sys.exit(main())
