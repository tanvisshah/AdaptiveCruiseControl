import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

# from sensor_msgs.msg import Image
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs import point_cloud2

from novatel_gps_msgs.msg import NovatelXYZ
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
import copy

class locationGPS:
    def __init__(self,model_name="highbay"):
        if model_name == "highbay":
            self.gpsSub = rospy.Subscriber("/novatel/bestxyz", NovatelXYZ, self.gpsCallback)
        self.x = 0
        self.y = 0
        self.z = 0
    def gpsCallback(self,data):
        # print("x,y,z",x,y,z)
        # print("Callback for GPS")
        # print("x,y,z",data.x,data.y,data.z)
        # print("Num Sats",data.position_type)
        self.x = data.x
        self.y = data.y
        self.z = data.z


    def returnGPSCoord(self):
        return [self.x,self.y,self.z]
