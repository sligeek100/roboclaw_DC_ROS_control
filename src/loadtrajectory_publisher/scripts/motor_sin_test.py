#!/usr/bin/env python3

'''
@sli, using trajectory publisher and controller class
'''

from motorctl.scripts.loadController import motorController
# from motorctl import motorController
from loadTrajectory import loadTrajectory
import rospy 

if __name__ =="__main__":
     
    try:
         rospy.init_node("motor_pub_ctl_node")
         
         loadCtl = motorController()
         loadRefPub = loadTrajectory()
         rospy.spin()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")