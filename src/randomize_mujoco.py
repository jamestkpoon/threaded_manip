#!/usr/bin/env python

import rospy
from mujoco_ros.srv import *
from std_srvs.srv import Empty,EmptyRequest



if __name__ == '__main__':

    rospy.init_node('mujoco_randomizer', anonymous=True)
    
    
    
    def read_params(category):
        try:
            params_ = rospy.get_param(category)
            params_flat_ = []
            for k in params_.keys():
                params_flat_ += params_[k]
            return True, params_flat_, params_.keys()
        except: return False, None, None
        
    params_tex_ok_, noise_tex_flat_, keys_tex_ = read_params('textural')
    params_phys_ok_, noise_phys_flat_, keys_phys_ = read_params('physical')
            
        
        
    if params_tex_ok_:
        rospy.wait_for_service('textural')
        rand_tex_svc_ = rospy.ServiceProxy('textural', RandomizeTexturalAttribute)

        def randtex_cb(req):
            rand_tex_svc_(RandomizeTexturalAttributeRequest(
                material_names=keys_tex_, noise=noise_tex_flat_))
                
            return []
                
        rospy.Service('/mujoco/policy/rand/textural', Empty, randtex_cb)



    if params_phys_ok_:
        rospy.wait_for_service('physical')
        rand_phys_svc_ = rospy.ServiceProxy('physical', RandomizePhysicalAttribute)
        
        def randphys_cb(req):
            rand_phys_svc_(RandomizePhysicalAttributeRequest(
                body_names=keys_phys_, noise=noise_phys_flat_,
                hold_6dof_children=[True] * len(keys_phys_)))
            
            return []
        
        rospy.Service('/mujoco/policy/rand/physical', Empty, randphys_cb)
        
        
        
    if params_tex_ok_ or params_phys_ok_:
        rospy.spin()
