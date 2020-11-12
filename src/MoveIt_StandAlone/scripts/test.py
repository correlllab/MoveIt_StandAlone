#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "simple_PID_on_wall.py"
__version__  = "2019.04"
__desc__     = "A Finite State Machine for autonomous car motion planning"

"""
~~~ Developmnent Plan ~~~
[ ] Step 1
[ ] Step 2
"""

# === Init Environment =====================================================================================================================
# ~~~ Prepare Paths ~~~
import sys, os.path
SOURCEDIR = os.path.dirname( os.path.abspath( __file__ ) ) # URL, dir containing source file: http://stackoverflow.com/a/7783326
PARENTDIR = os.path.dirname( SOURCEDIR )
# ~~ Path Utilities ~~
def prepend_dir_to_path( pathName ): sys.path.insert( 0 , pathName ) # Might need this to fetch a lib in a parent directory

# ~~~ Imports ~~~
# ~~ Standard ~~
from math import pi , sqrt , sin , cos

# ~~ Special ~~
import numpy as np
import rospy
from std_msgs.msg import String , Float32MultiArray , Bool
from geometry_msgs.msg import PoseStamped, Point, Twist

# ~~ Local ~~
from ur_motion_planning.srv import FK , IK
from ur_motion_planning.msg import FK_req

def load_arr_to_pose( rspArr ):
    """ Reshape 1x16 array into 4x4 homogeneous matrix """
    _DEBUG = 1
    dimLen = 4
    rtnHomog = np.zeros( (4,4) )
    i = j = 0
    for elem in rspArr:
        if _DEBUG:  print "[" + str(j) + "," + str(i) + "]:" , elem
        rtnHomog[j,i] = elem
        i += 1
        if i == dimLen:
            i = 0
            j = (j+1) % dimLen
    return rtnHomog

class FK_IK_Tester:
    """ Test the MoveIt Services """

    def __init__( self ):
        """ Set up vars and publishers """

        rospy.init_node( 'FK_IK_Tester' )
        
        rospy.wait_for_service( 'UR_FK_IK/FKsrv' )
        self.FKsrv = rospy.ServiceProxy( 'UR_FK_IK/FKsrv' , FK )

    def test( self ):
        """ Test the FK and IK services """
        req = FK_req()
        req.q_joints = [ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]
        rsp = load_arr_to_pose( self.FKsrv( req ).rsp.pose )
        print rsp


if __name__ == "__main__":
    node = FK_IK_Tester()
    node.test()