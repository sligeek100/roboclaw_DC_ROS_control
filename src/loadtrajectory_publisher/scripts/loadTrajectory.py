#!/usr/bin/env python3 

# class for the load reference publisher

# 
from math import pi, cos, sin
from operator import truediv
import diagnostic_msgs 
import diagnostic_updater
import rospy

# from roboclaw_driver.roboclaw_driver_py3 import Roboclaw

# import the service and load flat msg 
from motorctl.msg import motorFlatTarget

# two services 
from loadtrajectory_publisher.srv import trajectory_type, trajectory_mode
from roboclaw_driver.roboclaw_driver_py3 import Roboclaw



# motor state msg
from loadtrajectory_publisher.msg import LoadMotorState


__author__ = "sli100@stevens.edu (Shuai Li ^_*)"


#  only pub load flat msg data NO ros node involved, 
class loadTrajectory:
    def __init__(self):
        
        # create the motor driver interface
        # ~ means private ns
        # no need roboclaw stuff, controller need these 
        # dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        # baud_rate = rospy.get_param("~baud", "460800")
        
        # self.address = int(rospy.get_param("~address", "128"))
        # trajectory type: 0: sin, 1: 
        self.trajectory_type = int(rospy.get_param("~loadtrajectory_type", "0"))
        # sine load reference parameters
        self.t_start = 0
        
        self.lmax = float(rospy.get_param("~cable_lmax", "0.63"))
        self.lmin = float(rospy.get_param("~cable_lmin", "0.2"))
        self.omega = float(rospy.get_param("~omega", 2))
        
        # pub steps 
        self.trajectory_step = 0
        
        # publisher: pub the loadFlatTrajectory msg type 
        self.pub_loadRef = rospy.Publisher("roboclaw/loadReference", motorFlatTarget, queue_size=5)
        # a timer to call the type pub, and decide which trajecotry type.
        self.pubTimer = rospy.Timer(rospy.Duration(0.02), self.pubTimer_cb)
        
        # cmd_pubded boolean
        self.cmd_pub_bool = False
        
        # two services: one for trajectory type, one control the trajecotry_step
        # first seclect the trajectory type service 
        self.trajectory_type_service = rospy.Service("roboclaw/trajectory_type_service", trajectory_type, self.trajectory_type_service_cb)
        
        # second: select the step for this trajectory: init stage or run it.
        self.trajectory_pubMode_service = rospy.Service("roboclaw/trajectory_pubMode_service", trajectory_mode, self.trajectory_pubMode_service_cb)
        
    # service to set the trajectory type: for static 0, or flight test 1
    def trajectory_type_service_cb(self, req):
        if (req.type in [0,1]):
            self.trajectory_type = req.type
            return True, "trajectory type set to:" + str(req.type)
        return False, "check the inputs or the service is unavailable \n"
        
    def trajectory_pubMode_service_cb(self, req):
        if(req.mode in [0,1]):
            self.trajectory_step = req.mode
            return True, "trajectory steps set to:" + str(req.mode)
        return False, "check the input or the service is unavailable \n"
    
    
    # the timer cb function.
    def pubTimer_cb(self, event=None):
        
        msg = motorFlatTarget()
        # check the trajectory type
        if(self.trajectory_type == 0):
            # 0, static control trajectory.
            if(self.trajectory_step ==0): 
                # step =0 , init the static reference traj
                
                if(self.cmd_pub_bool==True):
                    self.cmd_pub_bool = False
                
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "map"
                msg.cable_len.data = self.lmax # see inti pub start from the maximum length position, but notice that, the ticks is 0
                msg.cable_vel.data = 0
                msg.cable_acc.data = 0
                
            elif(self.trajectory_step==1):
                # step =1, start pub this reference trajectory
                if(self.cmd_pub_bool == False):
                    self.t_start = rospy.get_time() # secs
                    self.cmd_pub_bool = True
                
                
                t_trigger = rospy.get_time() - self.t_start
                    
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "map"
                msg.cable_len.data = (self.lmax - self.lmin)/2 * cos(self.omega * t_trigger) + (self.lmax + self.lmin)/2
                msg.cable_vel.data = -self.omega * (self.lmax - self.lmin)/2 * sin(self.omega * t_trigger)
                msg.cable_acc.data = -self.omega * self.omega * (self.lmax - self.lmin)/2 *cos(self.omega * t_trigger)
            else:
                print("Error, select either init 0 or start 1 \n")
                
            # call publisher
            self.pub_loadRef.publish(msg)
        
        # start from a shorten cable will not be used in experiments, too complex procedure.
        elif(self.trajectory_type == 1):
            # 1, init min length, means the minimum length
            # this will be used for the flight. it needs takeoff in shortest cable length
            # still use the sin wave for the test use !! 
            if(self.trajectory_step==0):
                # init the trajectory 
                if(self.cmd_pub_bool == True):
                    self.cmd_pub_bool = False
                    
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "map"
                msg.cable_len.data = self.lmin 
                msg.cable_vel.data = 0
                msg.cable_acc.data = 0
                
            elif(self.trajectory_step==1):
                
                if(self.cmd_pub_bool == False):
                    self.t_start = rospy.get_time() # secs
                    self.cmd_pub_bool = True
                
                t_trigger = rospy.get_time() - self.t_start
                
                # fill the since function type cable length
                msg.header.frame_id = "map"
                msg.header.stamp = rospy.Time.now()
                msg.cable_len.data = (-self.lmax + self.lmin)/2 * cos(self.omega * t_trigger) + (self.lmax + self.lmin)/2
                msg.cable_vel.data = -self.omega * (-self.lmax + self.lmin)/2 * sin(self.omega * t_trigger)
                msg.cable_acc.data= -self.omega * self.omega * (-self.lmax + self.lmin)/2 *cos(self.omega * t_trigger)
            else:
                print("Error, select either init 0 or start 1 \n")
                
            # call publisher
            self.pub_loadRef.publish(msg)
            
            
        else:
            
            print("Error, has no this trajectory type, should selcet 0 or 1 \n")
    

if __name__ == "__main__":
    try:
        rospy.init_node("loadRef_sin_node")
        
        refPub = loadTrajectory()
        rospy.spin()  # spin is needed otherwise it will exit immediately
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")    