from styx_msgs.msg import TrafficLight
from opencv_model import OCVModel
#from carla import CarlaModel
import os
import rospy

class TLClassifier(object):
    def __init__(self, method):
        if method == "opencv_model":
            self.model = OCVModel()
            #rospy.logwarn("I am here")
        else:
            #TODO implement the Classifier using RNN Model
            curr_dir = os.path.dirname(os.path.realpath(__file__))
            self.model = CarlaModel(curr_dir+"/models/frozen_inference_graph.pb")

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
