#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from mujoco_ros.srv import *
from std_msgs.msg import Float32MultiArray

import numpy as np



def pose_fastener_body_wrt_attach_body(svc, attach_name,fastener_name):
    res_ = svc(GetRelativePoseBodiesRequest(bodyA_name=attach_name, bodyB_name=fastener_name))
    if res_.ok: return res_.relpose
    else: return None
    
def pack_pose_to_6tuple(pose_msg):
    eul_ = euler_from_quaternion([
    pose_msg.orientation.x,
    pose_msg.orientation.y,
    pose_msg.orientation.z,
    pose_msg.orientation.w ])
    
    return [ pose_msg.position.x, pose_msg.position.y, pose_msg.position.z ] + list(eul_)
    


if __name__ == '__main__':

    rospy.init_node('thread_manip_policylrntools_node', anonymous=True)

    # relative pose of attach point -> fastener
    rospy.wait_for_service('/mujoco/get_relative_pose_bodies')
    obj_pose_svc_ = rospy.ServiceProxy('/mujoco/get_relative_pose_bodies', GetRelativePoseBodies)    
    obj_pose_pub_ = rospy.Publisher('/mujoco/policy/relpos_tuple', Float32MultiArray, queue_size=1)
    
    while True:
        af_tf_ = pose_fastener_body_wrt_attach_body(obj_pose_svc_, 'shaft','nut')
        af_tf_tuple_ = pack_pose_to_6tuple(af_tf_)
        obj_pose_pub_.publish(Float32MultiArray(data=af_tf_tuple_))
        
        rospy.sleep(0.2)
