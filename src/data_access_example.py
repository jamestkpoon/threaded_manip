#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import Image
from std_srvs.srv import Empty,EmptyRequest

from ros_numpy import numpify
import numpy as np



if __name__ == '__main__':

    rospy.init_node('mujoco_ros_py', anonymous=True)
    
    # joint position and gripper publishers
    jpos_pub_ = rospy.Publisher('/mujoco/ur5/command/joint_positions', Float32MultiArray, queue_size=1)
    gripper_pub_ = rospy.Publisher('/mujoco/ur5/command/gripper', Bool, queue_size=1)

    # reset service client
    reset_svc_ = rospy.ServiceProxy('/mujoco/reset', Empty)
    
    rospy.sleep(1.0)
    
    
    
    # move joints
    joint_positions_ = [ 0.0, -1.0, 0.0, 0.0, 0.0, 0.0 ]
    jpos_pub_.publish(Float32MultiArray(data=joint_positions_))
    # wait to stop moving
    rospy.sleep(0.1)
    moving_ = rospy.get_param('/mujoco/ur5/moving')
    while moving_:
        rospy.sleep(0.05)
        moving_ = rospy.get_param('/mujoco/ur5/moving')

    # send gripper command, wait a bit
    gripper_close_ = True
    gripper_pub_.publish(Bool(data=gripper_close_))
    rospy.sleep(1.0)

    # get camera image
    img_msg_ = rospy.wait_for_message('/mujoco/ros_cam/rgb', Image)
    img_ = numpify(img_msg_)
    print(img_.shape)
    
    # get relative object positions
    rel_pose_msg_ = rospy.wait_for_message('/mujoco/policy/relpos_tuple', Float32MultiArray)
    print(rel_pose_msg_)
    
    rospy.sleep(3)

    # reset simulation   
    reset_svc_(EmptyRequest())
