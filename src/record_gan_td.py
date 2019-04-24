#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import Image
from std_srvs.srv import Empty,EmptyRequest

from data_access_example import grab_the_nut

import ros_numpy
from PIL import Image as PILImage


canonical_img_dir = '/media/james/Storage/gans/threaded_manip/canonical/'
randomized_img_dir = '/media/james/Storage/gans/threaded_manip/randomized/'
N_TD = 10



if __name__ == '__main__':
    
    rospy.init_node('rec_gan_td', anonymous=True)
    
    
    
    ### reset simulator, grab nut
    
    reset_svc_ = rospy.ServiceProxy('/mujoco/reset', Empty)

    jpos_pub_ = rospy.Publisher('/mujoco/ur5/command/joint_positions', Float32MultiArray, queue_size=1)
    gripper_pub_ = rospy.Publisher('/mujoco/ur5/command/gripper', Bool, queue_size=1)
    
    def init():
        reset_svc_(EmptyRequest())
        rospy.sleep(1.0)
        
        return grab_the_nut(jpos_pub_, gripper_pub_)
        
        
        
    ### record images
    
    def save_camera_image(fp, topic):
        img_ = rospy.wait_for_message(topic, Image)
        img_np_ = ros_numpy.numpify(img_)
        PILImage.fromarray(img_np_).save(fp)
    
    
        
    if init():
        
        ## randomization services
        rand_tex_svc_ = rospy.ServiceProxy('/mujoco/policy/rand/textural', Empty)
        rand_phys_svc_ = rospy.ServiceProxy('/mujoco/policy/rand/physical', Empty)
        
        derand_tex_svc_ = rospy.ServiceProxy('/mujoco/rand/undo/textural', Empty)
        derand_phys_svc_ = rospy.ServiceProxy('/mujoco/rand/undo/physical', Empty)
        
        ## loop
        for i in range(N_TD):
            i_str_ = str(i)
            
            # randomize object pose(s) and save a 'canonical' image
            rand_phys_svc_(EmptyRequest())
            rospy.sleep(0.02) # wait for child pose fix
            save_camera_image(canonical_img_dir+i_str_+'.jpg', '/mujoco/ros_cam/rgb')
            
            # randomize texture(s) and save a 'randomized' image
            rand_tex_svc_(EmptyRequest())
            save_camera_image(randomized_img_dir+i_str_+'.jpg', '/mujoco/ros_cam/rgb')
            
            # de-randomize poses/textures
            derand_phys_svc_(EmptyRequest()); derand_tex_svc_(EmptyRequest())
