from styx_msgs.msg import TrafficLight
from opencv_model import OCVModel
import os
import rospy

class TLClassifier(object):
    def __init__(self, method):
        if method == "opencv_model":
            self.model = OCVModel()
            #rospy.logwarn("I am here")
        else:
            #TODO implement the Classifier using RNN Model
            rospy.logwarn("no model mentioned")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        result = self.model.predict(image)
        return result
