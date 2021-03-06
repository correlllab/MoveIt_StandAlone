#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "test.py"
__version__  = "2020.11"
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
from random import random

# ~~ Special ~~
import numpy as np
import rospy
from std_msgs.msg import String , Float32MultiArray , Bool
from geometry_msgs.msg import PoseStamped, Point, Twist

# ~~ Local ~~
from ur_motion_planning.srv import FK , IK
from ur_motion_planning.msg import FK_req , IK_req

def load_arr_to_pose( rspArr ):
    """ Reshape 1x16 array into 4x4 homogeneous matrix """
    _DEBUG = 0
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

def load_pose_to_arr( rspArr ):
    """ Reshape 4x4 homogeneous matrix into 1x16 array """
    _DEBUG = 0
    dimLen = 4
    i = j = 0
    rtnArr = []
    for k in range(16):
        elem = rspArr[j,i]
        if _DEBUG:  print "[" + str(j) + "," + str(i) + "]:" , elem
        rtnArr.append( elem ) 
        i += 1
        if i == dimLen:
            i = 0
            j = (j+1) % dimLen
    return rtnArr

def rand_lo_hi( lo , hi ):
    return random() * ( hi - lo ) + lo

class FK_IK_Tester:
    """ Test the MoveIt Services """

    def __init__( self ):
        """ Set up vars and publishers """

        rospy.init_node( 'FK_IK_Tester' )
        
        rospy.wait_for_service( 'UR_FK_IK/FKsrv' )
        print "FK AVAILABLE"
        self.FKsrv = rospy.ServiceProxy( 'UR_FK_IK/FKsrv' , FK )

        rospy.wait_for_service( 'UR_FK_IK/IKsrv' )
        print "IK AVAILABLE"
        self.IKsrv = rospy.ServiceProxy( 'UR_FK_IK/IKsrv' , IK )

    def test1( self ):
        """ Test the FK and IK services """

        # Example Usage: Forward Kinematics
        req = FK_req()
        req.q_joints = [ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]
        rsp = load_arr_to_pose( self.FKsrv( req ).rsp.pose )
        print rsp

        

    def test2( self ):

        wins =  0
        Nrpt = 200

        for i in range( Nrpt ):

            print "Iteration" , i+1

            FKreq = FK_req()
            FKreq.q_joints = [ rand_lo_hi( -pi*1.00 , +pi*1.00 ) for i in range(6) ]
            FKrsp = load_arr_to_pose( self.FKsrv( FKreq ).rsp.pose )
            print "Request Joints:" , FKreq.q_joints
            print "Request Pose:\n" , FKrsp

            # Example Usage: Inverse Kinematics
            IKreq = IK_req()
            IKreq.pose   = load_pose_to_arr( FKrsp )
            if 0:
                IKreq.q_seed = [ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]
            else:
                IKreq.q_seed = np.add( FKreq.q_joints , [ rand_lo_hi( -pi , +pi ) for i in range(6) ] )
            print IKreq.pose
            IKrsp = self.IKsrv( IKreq )
            # print dir( IKrsp )
            if IKrsp.rsp.valid > 0:
                wins += 1

                FKchk = FK_req()
                FKchk.q_joints = IKrsp.rsp.q_joints
                FKcrs = load_arr_to_pose( self.FKsrv( FKchk ).rsp.pose )
                print "\tJoint Differene:" , np.linalg.norm( np.subtract( np.array( FKreq.q_joints ) , np.array( IKrsp.rsp.q_joints ) ) )
                print "\tLinear Difference:" , np.linalg.norm( np.subtract( FKrsp[0:3,3] , FKcrs[0:3,3] ) )

            print "Got IK response: " , IKrsp.rsp.q_joints , "is it valid?:" , IKrsp.rsp.valid
            print "\n"

        print wins , "/" , Nrpt , "valid answers"

if __name__ == "__main__":
    node = FK_IK_Tester()
    node.test1()
    print
    print
    node.test2()