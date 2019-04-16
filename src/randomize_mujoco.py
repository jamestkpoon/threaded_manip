#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty,EmptyRequest

import numpy as np



def gen_key_str(mean, noise):
    return 'nug'


if __name__ == '__main__':

    rospy.init_node('mujoco_randomizer', anonymous=True)

    #rospy.wait_for_service('/mujoco/reset')
    #reset_svc_ = rospy.ServiceProxy('/mujoco/reset', Empty)

    # randomization parameter dicts
    params_io_ = rospy.get_param('io')
    params_v_ = rospy.get_param('visual')
    #params_p_ = rospy.get_param('physical')

    print(params_v_)

    def randomize_cb(req):
        
        # load base file
        with open(params_io_['template_fp'], 'r') as f:
            data_ = f.read()
            
        # make visual changes
        for i in range(len(params_v_['keys'])):
            str_ = gen_key_str(params_v_['mean'][i], params_v_['noise'][i])
#            data_ = data_.replace(params_v_['keys'][i], str_)
            
        # make textural changes
        
        # save randomized file
        with open(params_io_['rand_fp'], 'w') as f:
            f.write(data_)
        
        # reset sim to apply new .xml
    #    reset_svc_(EmptyRequest())
        
        return []
        
        

    #rospy.Service('/mujoco/randomize', Empty, randomize_cb)

    #rospy.spin()
    randomize_cb(EmptyRequest())
