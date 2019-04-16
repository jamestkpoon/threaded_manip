#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from mujoco_ros.srv import *

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
    
    pos_ok_ = np.all(np.fabs(abt_) <= pos_thresh) and (axis_pos_ <= axis_lin_ofs)
    rot_ok_ = np.all(np.fabs(abr_) <= eul_thresh)
    
    return pos_ok_ and rot_ok_
    
def set_thread_lock(svc, fastener_name,axis,pitch,flag):
    res_ = svc(ThreadLockRequest(fastener_name=fastener_name,
        axis=axis, pitch=pitch, attach_flag=flag))
        
    return res_.ok
    


if __name__ == '__main__':

    rospy.init_node('threaded_watchdog_node', anonymous=True)
    
    manip_params_ = rospy.get_param('')
    connected_ = False
    
    rospy.wait_for_service('/mujoco/get_relative_pose_bodies')
    obj_pose_svc_ = rospy.ServiceProxy('/mujoco/get_relative_pose_bodies', GetRelativePoseBodies)
    thread_lock_svc_ = rospy.ServiceProxy('/mujoco/thread_lock', ThreadLock)

    rospy.sleep(1.0)
    

            
    while True:
        # get relative pose
        pose_res_ = obj_pose_svc_(GetRelativePoseBodiesRequest(
            bodyA_name=manip_params_['attach_name'], bodyB_name=manip_params_['fastener_name']))
              
        if pose_res_.ok:
            # check relative pose for alignment criteria
            tchk_ = alignment_check(pose_res_.relpose,
                manip_params_['attach_axis'], manip_params_['attach_axis_linear_offset'],
                manip_params_['attach_pos_thresh'], manip_params_['attach_eul_thresh'])
                
            if not connected_ and tchk_: # lock
                tl_res_ = set_thread_lock(thread_lock_svc_,
                    manip_params_['fastener_name'], manip_params_['attach_axis'], manip_params_['pitch'], True)
                connected_ = True
                print("  Threaded '%s' and '%s'" % (manip_params_['attach_name'], manip_params_['fastener_name']))
            elif connected_ and not tchk_: # unlock
                tl_res_ = set_thread_lock(thread_lock_svc_,
                    manip_params_['fastener_name'], manip_params_['attach_axis'], manip_params_['pitch'], False)
                connected_ = False
                print("  Un-threaded '%s' and '%s'" % (manip_params_['attach_name'], manip_params_['fastener_name']))
        
        rospy.sleep(0.1)
