#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import Image
from std_srvs.srv import Empty,EmptyRequest
from james_ur_kinematics.srv import *
from mujoco_ros.srv import *

from data_access_example import grab_the_nut

from geometry_msgs.msg import Pose,Point, Quaternion
from tf.transformations import quaternion_from_euler
import numpy as np

import ros_numpy
from PIL import Image as PILImage



canonical_img_dir = '/media/james/Storage/gans/threaded_manip/canonical/'
randomized_img_dir = '/media/james/Storage/gans/threaded_manip/randomized/'

px_r = [ 0.4, 0.65 ]; py_r = [ -0.25, 0.25 ]; pz_r = [ 0.02, 0.25 ]
rr_r = [ -np.pi, np.pi ]; rp_r = [ np.pi * 0.4, np.pi * 0.6 ]; ry_r = [ -np.pi, np.pi ]

N_TD = 1e3
RANDS_PER_POSE = 5

NUT_RELPOS_THR = 0.01
MJ_PAUSE = 0.03



if __name__ == '__main__':
    
    rospy.init_node('rec_gan_td', anonymous=True)
    
    
    
    ### reset simulator, grab nut
    
    reset_svc_ = rospy.ServiceProxy('/mujoco/reset', Empty)

    jpos_pub_ = rospy.Publisher('/mujoco/ur5/command/joint_positions', Float32MultiArray, queue_size=1)
    gripper_pub_ = rospy.Publisher('/mujoco/ur5/command/gripper', Bool, queue_size=1)
        
    def init():
        reset_svc_(EmptyRequest())
        rospy.sleep(0.1)
        
        return grab_the_nut(jpos_pub_, gripper_pub_)
            
        
        
    ### random robot movement

    pose_ranges_ = [ px_r, py_r, pz_r, rr_r, rp_r, ry_r ]
    pose_mindiff_ = [ [ np.min(pose_ranges_[i]), np.ptp(pose_ranges_[i]) ] for i in range(6) ]

    def gen_random_pose(ik_svc):
        while True:
            r_ = np.random.random_sample(6)
            pose6_ = [ pose_mindiff_[i][0] + r_[i]*pose_mindiff_[i][1] for i in range(6) ]
            ql_ = list(quaternion_from_euler(*pose6_[3:]))
            ik_req_ = IKRequest(ee_pose=Pose(
                position=Point(*pose6_[:3]), orientation=Quaternion(*ql_)))
            jangs_ = ik_svc(ik_req_).joint_angles
            if len(jangs_) > 0: return list(jangs_[:6]), pose6_[:3]+ql_, pose6_
            
    def move_to_random_pose(ik_svc, m2j_svc):
        j6_,p7_, p6_ = gen_random_pose(ik_svc)
        m2j_svc(MoveToRequest(ur_state=j6_, gripper_state=True))
        
        return j6_,p7_, p6_
    
    ik_topic_ = '/ur5_kin/IK'
    rospy.wait_for_service(ik_topic_)
    ik_svc_ = rospy.ServiceProxy(ik_topic_, IK)

    m2j_topic_ = '/mujoco/ur5/command/moveTo'
    rospy.wait_for_service(m2j_topic_)
    m2j_svc_ = rospy.ServiceProxy(m2j_topic_, MoveTo)
    
    
    
    body_rel_svc_ = rospy.ServiceProxy('/mujoco/get_relative_pose_bodies', GetRelativePoseBodies)

    def relpos(bA_name, bB_name):
        res_ = body_rel_svc_(GetRelativePoseBodiesRequest(bodyA_name=bA_name, bodyB_name=bB_name))
        return [ res_.relpose.position.x, res_.relpose.position.y, res_.relpose.position.z ]
        
        
    def scenario_ok():
        relpos_a_ = relpos('ee_link', 'nut')        
        move_to_random_pose(ik_svc_, m2j_svc_); rospy.sleep(MJ_PAUSE)
        relpos_b_ = relpos('ee_link', 'nut')
        
        diff_ = np.asarray(relpos_b_) - np.asarray(relpos_a_)
        hyp_ = np.sqrt(np.sum(np.square(diff_)))
        
        return (hyp_ <= NUT_RELPOS_THR)



    ### randomization services
    
    rand_tex_svc_ = rospy.ServiceProxy('/mujoco/policy/rand/textural', Empty)
    rand_phys_svc_ = rospy.ServiceProxy('/mujoco/policy/rand/physical', Empty)
    
    derand_tex_svc_ = rospy.ServiceProxy('/mujoco/rand/undo/textural', Empty)
    derand_phys_svc_ = rospy.ServiceProxy('/mujoco/rand/undo/physical', Empty)
        
        
        
    ### record TD images
    
    def save_camera_image(fp, topic):
        img_ = rospy.wait_for_message(topic, Image)
        img_np_ = ros_numpy.numpify(img_)
        PILImage.fromarray(img_np_).save(fp)   



    i = 0
    
    while i < int(N_TD):
        if init() and scenario_ok():
    
            # save images
            for _ in range(RANDS_PER_POSE):
                i_str_ = str(i); i += 1
                
                # randomize object pose(s) and save a 'canonical' image
                rand_phys_svc_(EmptyRequest()); rospy.sleep(MJ_PAUSE)
                
                save_camera_image(canonical_img_dir+i_str_+'.jpg', '/mujoco/ros_cam/rgb')
                
                # randomize texture(s) and save a 'randomized' image
                rand_tex_svc_(EmptyRequest())
                save_camera_image(randomized_img_dir+i_str_+'.jpg', '/mujoco/ros_cam/rgb')
                
                # de-randomize poses/textures
                derand_phys_svc_(EmptyRequest()); rospy.sleep(MJ_PAUSE)
                derand_tex_svc_(EmptyRequest())
