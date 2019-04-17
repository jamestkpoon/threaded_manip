#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from mujoco_ros.srv import *

import sys
import numpy as np



def alignment_check(af_tf_msg, axis,axis_lin_ofs, pos_thresh,eul_thresh):
    # get relevant data for other axes
    eul_ = euler_from_quaternion([
        af_tf_msg.orientation.x,
        af_tf_msg.orientation.y,
        af_tf_msg.orientation.z,
        af_tf_msg.orientation.w ])
        
    if axis == 'x':
        abt_ = [ af_tf_msg.position.y, af_tf_msg.position.z ]
        axis_pos_ = af_tf_msg.position.x
        abr_ = [ eul_[1], eul_[2] ]
    elif axis == 'y':
        abt_ = [ af_tf_msg.position.x, af_tf_msg.position.z ]
        axis_pos_ = af_tf_msg.position.y
        abr_ = [ eul_[0], eul_[2] ]
    elif axis == 'z':
        abt_ = [ af_tf_msg.position.x, af_tf_msg.position.y ]
        axis_pos_ = af_tf_msg.position.z
        abr_ = [ eul_[0], eul_[1] ]
    else: return False
    
    # check thresholds
    pos_ok_ = np.all(np.fabs(abt_) <= pos_thresh) and (axis_pos_ <= axis_lin_ofs)
    rot_ok_ = np.all(np.fabs(abr_) <= eul_thresh)
    
    return pos_ok_ and rot_ok_
    


if __name__ == '__main__':

    rospy.init_node('threaded_watchdog_node', anonymous=True)
    
    fastener_name_ = sys.argv[1]
    manip_params_ = rospy.get_param(fastener_name_)
    pair_str_ = "'{}' and '{}'".format(manip_params_['attach_name'],fastener_name_)
    connected_ = False
    
    rospy.wait_for_service('get_relative_pose_bodies')
    obj_pose_svc_ = rospy.ServiceProxy('get_relative_pose_bodies', GetRelativePoseBodies)
    thread_lock_svc_ = rospy.ServiceProxy('thread_lock', ThreadLock)

    rospy.sleep(1.0)
    

            
    while True:
        # get relative pose
        pose_res_ = obj_pose_svc_(GetRelativePoseBodiesRequest(
            bodyA_name=manip_params_['attach_name'], bodyB_name=fastener_name_))
              
        if pose_res_.ok:
            # check relative pose for alignment criteria
            tchk_ = alignment_check(pose_res_.relpose,
                manip_params_['attach_axis'], manip_params_['attach_axis_linear_offset'],
                manip_params_['attach_pos_thresh'], manip_params_['attach_eul_thresh'])
                
            if not connected_ and tchk_: # connect
                connected_ = thread_lock_svc_(ThreadLockRequest(
                    fastener_name=fastener_name_, attach_flag=True,
                    axis=manip_params_['attach_axis'], pitch=manip_params_['pitch'])).ok
                if(connected_): print('  Connected %s' % pair_str_)
                else: print('  Failed to connect %s' % pair_str_)
            elif connected_ and not tchk_: # disconnect
                dc_ok_ = thread_lock_svc_(ThreadLockRequest(
                    fastener_name=fastener_name_, attach_flag=False)).ok
                connected_ = not dc_ok_
                if(dc_ok_): print('  Disconnected %s' % pair_str_)
                else: print('  Failed to disconnect %s' % pair_str_)
        
        rospy.sleep(0.1)
