#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import Image
from std_srvs.srv import Empty,EmptyRequest

from ros_numpy import numpify
import numpy as np



def wait_for_ur5_to_stop_moving():
    rospy.sleep(0.1)
    moving_ = rospy.get_param('/mujoco/ur5/moving')
    while moving_:
        rospy.sleep(0.05)
        moving_ = rospy.get_param('/mujoco/ur5/moving')

def grab_the_nut(jpos_pub, gripper_pub):
    # move arm above nut
    jpos_pub.publish(Float32MultiArray(
        data=(0.16444053217068055, -0.8873331224531231, 0.9359561695790948, 1.5221841506350873, 1.570794522905687, 0.1644331857506687)))
    wait_for_ur5_to_stop_moving()
    jpos_pub.publish(Float32MultiArray(
        data=(0.16444053217068055, -0.8464304987538576, 1.111644361804554, 1.305593334710362, 1.570794522905687, 0.1644331857506687)))
    wait_for_ur5_to_stop_moving()

    # grab the nut
    gripper_pub.publish(Bool(data=True))
    rospy.sleep(2.0)

    jpos_pub.publish(Float32MultiArray(
        data=(0.16444053217068055, -0.8873331224531231, 0.9359561695790948, 1.5221841506350873, 1.570794522905687, 0.1644331857506687)))
    wait_for_ur5_to_stop_moving()
    
    return (rospy.get_param('/mujoco/ur5/grasped_object') == 'nut')



if __name__ == '__main__':

    rospy.init_node('mujoco_ros_py', anonymous=True)
    
    # joint position and gripper publishers
    jpos_pub_ = rospy.Publisher('/mujoco/ur5/command/joint_positions', Float32MultiArray, queue_size=1)
    gripper_pub_ = rospy.Publisher('/mujoco/ur5/command/gripper', Bool, queue_size=1)


    
    
    

    # reset service client
    reset_svc_ = rospy.ServiceProxy('/mujoco/reset', Empty)    
    rospy.sleep(1.0)    
    reset_svc_(EmptyRequest())
        
        
    
    # grab the nut
    print(grab_the_nut(jpos_pub_, gripper_pub_))
    
    
    # get joint positions and velocities
    def get_joint_positions():
        jstate_ = rospy.wait_for_message('/mujoco/ur5/sensors/joint_state', Float32MultiArray)
        return jstate_.data[:6]
    
#    jstate_ = rospy.wait_for_message('/mujoco/ur5/sensors/joint_state', Float32MultiArray).data
#    joint_positions_ = jstate_[:6]; joint_velocities_ = jstate_[6:]
#    print(joint_positions_, joint_velocities_)
    
    
    
#    # move joints
#    joint_positions_ = [ 0.0, -1.0, 0.0, 0.0, 0.0, 0.0 ]
#    jpos_pub_.publish(Float32MultiArray(data=joint_positions_))
#    # wait to stop moving


#    # send gripper command, wait a bit
#    gripper_close_ = False
#    gripper_pub_.publish(Bool(data=gripper_close_))
#    rospy.sleep(2.0)

#    # get camera image
#    img_msg_ = rospy.wait_for_message('/mujoco/ros_cam/rgb', Image)
#    img_ = numpify(img_msg_)
#    print(img_.shape)
#    
#    # get relative object positions
#    rel_pose_msg_ = rospy.wait_for_message('/mujoco/policy/relpos_tuple', Float32MultiArray).data
#    print(rel_pose_msg_)

    # reset simulation
    rospy.sleep(3)
#    reset_svc_(EmptyRequest())
