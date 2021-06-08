import rospy
#from ackermann_msgs.msg import AckermannDrive
import numpy as np
from util.util import euler_to_quaternion, quaternion_to_euler

import rospy
import numpy as np
import argparse
import math

from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd
from std_msgs.msg import String, Bool, Float32, Float64, Char
from pynput.keyboard import Key, Listener, KeyCode

class VehicleController():

    def __init__(self, model_name='gem'):
        # Publisher to publish the control input to the vehicle model
        #self.controlPub = rospy.Publisher("/" + model_name + "/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.model_name = model_name



        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)

        self.enabled = False
        self.accel_flag = False
        self.gear_cmd = PacmodCmd()
        self.accel_cmd = PacmodCmd()
        self.brake_cmd = PacmodCmd()

        self.previousDistance = 0
        self.currentVelocity = 0
        self.desiredVelocity = 0

    def execute(self, currentPose, targetPose):
        """
            This function takes the current state of the vehicle and 
            the target state to compute low-level control input to the vehicle
            Inputs: 
                currentPose: ModelState, the current state of vehicle
                targetPose: The desired state of the vehicle
        """

        currentEuler = quaternion_to_euler(currentPose.pose.orientation.x,
                                           currentPose.pose.orientation.y,
                                           currentPose.pose.orientation.z,
                                           currentPose.pose.orientation.w)
        
        target_x = targetPose[0]
        target_y = targetPose[1]
        target_v = targetPose[2]
        
        k_s = 0.1
        k_ds = 1
        k_n = 0.1

        #compute errors
        xError = (target_x - currentPose.pose.position.x) * np.cos(currentEuler[2]) + (target_y - currentPose.pose.position.y) * np.sin(currentEuler[2])
        yError = -(target_x - currentPose.pose.position.x) * np.sin(currentEuler[2]) + (target_y - currentPose.pose.position.y) * np.cos(currentEuler[2])
        curr_v = np.sqrt(currentPose.twist.linear.x**2 + currentPose.twist.linear.y**2)
        vError = target_v - curr_v
        # print("TaRgetV,CurrV",target_v,curr_v)
        # print("Target X:",xError)
        delta = k_n*yError
        # Checking if the vehicle need to stop
        if target_v > 0:
            v = xError*k_s + vError*k_ds
            # v = target_v
            # print("Velcithy with target > 0",v)
        else:
            v = xError*k_s - 0.2*k_ds
            # if v <= 0:
            #     v = 0
                 

        #Send computed control input to vehicle
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = v
        # print("V:",v)
        newAckermannCmd.steering_angle = delta
        self.controlPub.publish(newAckermannCmd)

    def on_press(self,key):
        if key == KeyCode.from_char('f'):
            print('ACCELERATING')
            self.accel_flag = True
            self.brake_cmd.f64_cmd = 0.0
        if key == KeyCode.from_char('s'):
            print('STOPPING')
            self.accel_flag = False
            self.brake_cmd.f64_cmd = 0.5
        if key == Key.esc:
            return False
        if key == KeyCode.from_char('q'):
            print('DISENGAGED')
            self.enabled = False
            self.accel_cmd.enable = False
            self.accel_cmd.clear = True
            self.accel_cmd.ignore = True
            self.brake_cmd.enable = False
            self.brake_cmd.clear = True
            self.brake_cmd.ignore = True
            self.gear_cmd.ui16_cmd = 2
        if key == KeyCode.from_char('p'):
            print('ENGAGED')
            self.enabled = True
            self.accel_cmd.enable = True
            self.accel_cmd.clear = False
            self.accel_cmd.ignore = False
            self.brake_cmd.enable = True
            self.brake_cmd.clear = False
            self.brake_cmd.ignore = False
            self.accel_cmd.f64_cmd = 0.0
            self.brake_cmd.f64_cmd = 0.5
            self.gear_cmd.ui16_cmd = 2
        if key == KeyCode.from_char('n'):
            print('GEAR: NEUTRAL')
            self.gear_cmd.ui16_cmd = 2
        if key == KeyCode.from_char('d'):
            print('GEAR: DRIVE')
            self.gear_cmd.ui16_cmd = 3
        if key == KeyCode.from_char('r'):
            print('GEAR: REVERSE')
            self.gear_cmd.ui16_cmd = 1

        self.enable_pub.publish(Bool(self.enabled))
        self.gear_pub.publish(self.gear_cmd)
        self.brake_pub.publish(self.brake_cmd)

    def distanceCallback(self,distance):

        #print("accel:",self.accel_cmd.f64_cmd)

        if not self.enabled or not self.accel_flag:
           return 
        
        # print("in Distance callback ",distance)
        # print("Previous:",previousDistance)
        relativeVelocity = (distance.data - self.previousDistance)/0.1
        # if(previousDistance-float(distance.data)!=0 and not math.isnan(distance.data)):
        #     print("Rel Velocity:",relativeVelocity)
        self.desiredVelocity = self.currentVelocity + relativeVelocity
        self.previousDistance = float(distance.data)
        
        # if distance.data < 15:
        #     # print("Settung cmd to 0")
        #     accel_cmd.f64_cmd = 0.0
        #     accel_pub.publish(accel_cmd)
        # else:
        #     accel_cmd.f64_cmd = 0.4
        #     accel_pub.publish(accel_cmd)

        if relativeVelocity > 0 or math.isnan(distance.data):
            # Pedestrian is far away or faster
            print("Relative,distance",relativeVelocity,distance.data)
            self.accel_cmd.f64_cmd = 0.35
            self.brake_cmd.f64_cmd = 0.0
            self.accel_pub.publish(self.accel_cmd)
            self.brake_pub.publish(self.brake_cmd)
        elif relativeVelocity < 0:
            print("Acceleration",self.accel_cmd.f64_cmd)
            print("Relative",relativeVelocity,distance.data)
            if self.currentVelocity <= self.desiredVelocity:
                self.accel_cmd.f64_cmd = self.accel_cmd.f64_cmd
            else:
                self.accel_cmd.f64_cmd -= 0.01

            if self.accel_cmd.f64_cmd <= 0.25:
                self.accel_cmd.f64_cmd = 0.25

            self.accel_pub.publish(self.accel_cmd)

        if distance.data < 10:
            print("Breaking")
            self.accel_cmd.f64_cmd = 0.0
            self.brake_cmd.f64_cmd = 0.5

            self.accel_pub.publish(self.accel_cmd)
            self.brake_pub.publish(self.brake_cmd)

    def vehicleSpeedCallback(self,speed):
        self.currentVelocity = speed.data
        


    def run_model(self,model_name): 

        #controller part
        # rospy.init_node('GEM_Control', anonymous=True)
        rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, self.vehicleSpeedCallback)
        #self.enable_pub.publish(Bool(False))

        # listener = Listener(on_press=self.on_press)
        # listener.start()

        rospy.Subscriber("/mp5/Distance",Float64,self.distanceCallback)

        # Can we make this 100?

        # rate = rospy.Rate(10)
        # print("Accel Main",self.accel_flag)
        # while not rospy.is_shutdown():

            # TODO 
            
            # rate.sleep()  # Wait a while before trying to get a new state   


    
# def speed_control(data):    
#     accel_cmd = PacmodCmd()
    
#     global accel_flag
#     print(accel_flag, "accel")
#     if accel_flag == False:

#         accel_cmd.f64_cmd = 0
#         accel_pub.publish(accel_cmd)

#     else:
#         brake_msg = PacmodCmd()
#         brake_msg.f64_cmd = 0.0
#         brake_pub.publish(brake_msg)

#         accel_cmd.f64_cmd = 0.4
#         accel_pub.publish(accel_cmd)
#         pass






# .32 threshold




     

# if __name__ == "__main__":
#     run_model('gem')


