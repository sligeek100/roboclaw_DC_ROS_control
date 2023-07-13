#!/usr/bin/env python3
'''
DC motor controller use roboclaw driver 
@sli100
'''
# 5-19-22
# the topic name is changed roboclaw/** 
# 5-21-22
# up to date.

import diagnostic_msgs
import diagnostic_updater

from roboclaw_driver.roboclaw_driver_py3 import Roboclaw
import rospy

from motorctl.msg import motorFlatTarget
# motor state msg
from motorctl.msg import LoadMotorState

# import empty service 
from std_srvs.srv import Empty
# from motorctl.srv import motorMode, motorModeRequest, motorModeResponse

'''
    motorController class:

        -receive the cable length command, 
        -pub current motor position, velocity, infos
        -generate control command to the motor in maximum 10hz.

        safety:
            - maximum speed limits
            - maximum position limits
            - stop running check ?
            - initial position reset ?

'''
class motorController:
    def __init__(self):
        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}
        
        # need the roboclaw interface 
        dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baud", "460800"))
        self.address = int(rospy.get_param("~address", "128"))
        # create connection with roboclaw driver
        self.roboclaw = Roboclaw(dev_name, baud_rate)
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")

        # TODO need to check if address is correct
        try:
            # roboclaw.Open(dev_name, baud_rate)
            # 
            self.roboclaw.Open()
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.FunctionDiagnosticTask("Vitals", self.check_vitals))
    
        self.roboclaw.SpeedM1(self.address, 0)
        self.roboclaw.ResetEncoders(self.address) # cable at the longest position, set as 0.


        # tick speed constraints 
        self.MAX_SPEED = float(rospy.get_param("~max_speed", "1.2"))
        self.TICKS_PER_METER = float(rospy.get_param("~ticks_per_meter", "30921.818"))
        self.NEWTON_PER_MILLIAMP = float(rospy.get_param("~newton_per_milliamp", "0.01"))
        self.MAX_POSITION = float(rospy.get_param("~max_position", "18553")) # 0.6m * tm = 18553.0908 ticks, as the new 
        
        # 
        self.lmax = float(rospy.get_param("~cable_lmax", "0.63"))
        self.lmin = float(rospy.get_param("~cable_lmin", "0.2"))
        rospy.Subscriber("roboclaw/loadReference", motorFlatTarget, self.cmd_load_cb)

        self.received_position = 0 # should be 0, coz 
        self.received_velocity = 0
        self.received_acc = 0

        # 
        self.writeFrequency = float(rospy.get_param("~write_frequency", 10)) # 10 hz write is the maximum frequency.
        rospy.Timer(rospy.Duration(1.0/self.writeFrequency), self.writeCmd)
        # a timer
        rospy.Timer(rospy.Duration(0.2), self.pub_motorState_cb)
        self.motorStatePub = rospy.Publisher("roboclaw/load_motor_state", LoadMotorState, queue_size=5)
        
        # motor control mode
        self.mode_list = ['run', 'stop'] # mode lists
        self.ctl_mode = 'run' # string type, default is run

        # a service to ser the control mode
        self.setCtlMode_srv =  rospy.Service('roboclaw_stop', Empty, self.ctlMode_cb)
        rospy.sleep(1)

        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("address %d", self.address)
        rospy.logdebug("max_speed %f", self.MAX_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
        

    def cmd_load_cb(self, msg):
        
        # saturation of the controller 
        if(msg.cable_vel.data >self.MAX_SPEED):
            msg.cable_vel.data = self.MAX_SPEED
            
        # add the position constraint. 
        if(msg.cable_len.data > self.lmax):
            msg.cable_len.data = self.lmax    # upper bound
        
        if(msg.cable_len.data < self.lmin):
            msg.cable_len.data = self.lmin   # lower bound.  lmin < length  < lmax. 

        self.received_position = (self.lmax - msg.cable_len.data) * self.TICKS_PER_METER  # lmax

        self.received_velocity = abs(msg.cable_vel.data * self.TICKS_PER_METER) # need abs(), it must be a non-negative value
        self.received_acc = abs(msg.cable_acc.data * self.TICKS_PER_METER)

    def writeCmd(self, event=None):
        # 
        if(self.ctl_mode == "run"):
            try:
                
                self.roboclaw.SpeedAccelDeccelPositionM1(self.address, 0, int(self.received_velocity), 0, int(self.received_position), 5)
            except OSError as e:
                rospy.logwarn("SpeedAccelDeccelPositionM1 OSError: %d", e.errno)
                rospy.logdebug(e)

        if(self.ctl_mode == "stop"):
            self.roboclaw.ForwardM1(self.address, 0)

    '''
    cmd 0 Drive M1
    send: [address, 0, value, crc(2 bytes)], receive oxff.
    value range is 0-127, 127 =full speed forward, 64 half speed forward, 0 full stop, and this will make the motor stop with force maintained.
    '''
    # check stuff
    def check_vitals(self, stat):
        try:
            status = self.roboclaw.ReadError(self.address)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", float(self.roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10))
            stat.add("Logic Batt V:", float(self.roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10))
            stat.add("Temp1 C:", float(self.roboclaw.ReadTemp(self.address)[1] / 10))
            stat.add("Temp2 C:", float(self.roboclaw.ReadTemp2(self.address)[1] / 10))
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
        return stat


    # stop the motor 
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            self.roboclaw.ForwardM1(self.address, 0)
            # roboclaw.ForwardM2(self.address, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                self.roboclaw.ForwardM1(self.address, 0)
                # roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)

    # read the motor ticks and compute the cable length and velocity.
    def pub_motorState_cb(self, event=None):
        # print("get motor state\n")
        # read encoder pub the status of the motor
        loadState = LoadMotorState()
        
        try:
            status,enc1, _= self.roboclaw.ReadEncM1(self.address) # enc1 is the motor moving ticks
            status_v, vel1, _ = self.roboclaw.ReadSpeedM1(self.address)
            status_c, cur1, _ = self.roboclaw.ReadCurrents(self.address)
            status_pwm, pwm1, _ = self.roboclaw.ReadPWMs(self.address)

            # convert ticks to m/s etc.
            motor_position_m = (self.lmax - enc1/self.TICKS_PER_METER) # 
            motor_velocity_m = -vel1 / self.TICKS_PER_METER
            # fill the msg 
            loadState.header.stamp = rospy.Time.now()
            loadState.header.frame_id = 'map'
            loadState.position = motor_position_m
            loadState.velocity = motor_velocity_m
            loadState.force = 0.01 * cur1 # approximated force, not accurate parameter.
            loadState.pwm = pwm1
            
            self.motorStatePub.publish(loadState)

            # add the updater, maybe no use and waste resource.
            self.updater.update()
        
        except ValueError:
            pass
        except OSError as e:
            rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
            rospy.logdebug(e)

    def ctlMode_cb(self, req):

        self.ctl_mode = "stop" # from integar to string. 0 or 1 only
        return True, "the motor control been set to" + self.mode_list[req.data]
       

if __name__=="__main__":
    
    try:
        rospy.init_node("Roboclaw_loadControl_node")
        loadCtl = motorController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
    