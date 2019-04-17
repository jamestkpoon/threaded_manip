#!/usr/bin/env python

import rospy
from mujoco_ros.srv import *
from std_srvs.srv import Empty,EmptyRequest



if __name__ == '__main__':

    rospy.init_node('mujoco_randomizer', anonymous=True)

    # randomization parameters
    params_tex_ = rospy.get_param('textural')
    params_phys_ = rospy.get_param('physical')
    
    tex_noise_flat_ = []
    for k in params_tex_.keys(): tex_noise_flat_ += params_tex_[k]
    phys_noise_flat_ = []
    for k in params_phys_.keys(): phys_noise_flat_ += params_phys_[k]
    
    
    
    # randomization services
    rospy.wait_for_service('textural')
    rand_tex_svc_ = rospy.ServiceProxy('textural', RandomizeTexturalAttribute)
    rospy.wait_for_service('physical')
    rand_phys_svc_ = rospy.ServiceProxy('physical', RandomizePhysicalAttribute)
    
    

    def randomize_cb(req):
        # textural
        rand_tex_svc_(RandomizeTexturalAttributeRequest(
            material_names=params_tex_.keys(), noise=tex_noise_flat_))
        # physical
        rand_phys_svc_(RandomizePhysicalAttributeRequest(
            body_names=params_phys_.keys(), noise=phys_noise_flat_))
        
        return []
        
    
       
    rospy.Service('randomize', Empty, randomize_cb)
    
    rospy.spin()
