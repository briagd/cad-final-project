from styx_msgs.msg import TrafficLight
import cv2
import torch
from PIL import Image
import rospy
import numpy as np

class TrafficLightDetector():

    def __init__(self):
        # Load the model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path_or_model='best75.pt').autoshape()

        # rospy.logerr('model: ' + str(self.model is not None))

    def detect_state(self, camera_frame):
      
        # img = Image.fromarray(camera_frame, 'RGB')
        # img.save('camera_frame.png')

        # Predict the traffic light status
        pred = self.model(camera_frame)

        # Check all the predictions and get the colour predicted
        for p in pred.pred:
            if p is not None:
                for c in p[:, -1].unique():
                    # c value of 0: green, 1: yellow, 2: red
                    if int(c.item()) != 0:
                        return TrafficLight.RED
        # If the traffic light is green, then we can ignore the traffic light
        return TrafficLight.UNKNOWN

    