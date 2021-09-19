#!/usr/bin/python3
import rospy 
from tensorflow.keras import models
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tensorflow as tf
import rospkg 
import os
import cv2
import numpy as np 
from object_recognition.msg import ObjectRecognitionAction, ObjectRecognitionResult, Object
import actionlib
from roi import ROI
import gpu_setting # Setting for enable memory growth (prevents Tensorflow claiming all GPU memory)

# Action server for object recognition
class ObjectRecognitionServer(object):
  
  # Load the keras model and start the action server
  def __init__(self):
    rospack = rospkg.RosPack()
    path = rospack.get_path("object_recognition")
    model_path = os.path.join(path, "model/model.h5")
    self.model = models.load_model(model_path) # Load the model
    self.image_size = 32 # Defined by the training step
    self.labels = ["barcelona_box", "water_bottle", "brown_cup", "blue_box", "white_ball", "blue_cup", "infinite_money_bowl", "pilot", "cloth", "soap_dispenser", "plant_pot", "water_sprayer"]

    # Initialize model by inputting a fake image (first time seems to be slowest)
    fake_image = np.zeros((self.image_size, self.image_size, 3))
    pred = self.model.predict(np.asarray([fake_image]))[0]

    # Publisher for showing the ROI and Label in the image
    self.image_publisher = rospy.Publisher("/object_recognition/image", Image, queue_size=1)

    self.roi = ROI()
    self.cv_bridge = CvBridge()
    self.object_recognition_server = actionlib.SimpleActionServer("/object_recognition", ObjectRecognitionAction, self.object_recognition_callback, auto_start = False)
    self.object_recognition_server.start()

  # Action server callback function
  def object_recognition_callback(self, goal_msg):
    # Get the latest sensor_msgs Image
    image_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
    # Convert sensor_msg Image to OpenCv Image
    image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8") 

    # Get the region of interests (roi) 
    rois = self.roi.get_rois("camera_link")

    if len(rois) == 0: # If no rois where found, abort action server
      self.object_recognition_server.set_aborted()      
      return

    data = []
    result = ObjectRecognitionResult()    
    image_segmented = image.copy() # Image segmented is for publishing 
    roi_data = []

    for roi in rois:
      roi_image = image[roi.top: roi.bottom, roi.left: roi.right] # Get the roi image
      roi_data.append([(roi.left, roi.top), (roi.right, roi.bottom)])
      resized_image = cv2.resize(roi_image, (self.image_size, self.image_size)) # Resize the roi image to fit the CNN
      normalized_image = resized_image / 255. # Normalize between 0.0-1.0
      data.append(normalized_image)
      obj = Object() 
      obj.x = roi.x
      obj.y = roi.y
      obj.z = roi.z
      result.objects.append(obj)
    
    # Classify the rois 
    predictions = self.model.predict(np.asarray(data))

    # Draw the roi and label, used for debuggin/visualizing 
    for index, prediction in enumerate(predictions):
      result.objects[index].label = self.labels[np.argmax(prediction)]
      cv2.rectangle(image_segmented, roi_data[index][0], roi_data[index][1], (0, 255, 0))

      x, y = roi_data[index][0]
      text_point = (x, y - 5)
      cv2.putText(image_segmented, result.objects[index].label, text_point, cv2.FONT_HERSHEY_COMPLEX,  
                   1.0, (0, 255, 0)) 

    # Convert OpenCv image to sensor_msgs Image
    publish_msg = self.cv_bridge.cv2_to_imgmsg(image_segmented)
    publish_msg.header.frame_id = "camera_link"
    self.image_publisher.publish(publish_msg)

    # Return results and set action server to success
    self.object_recognition_server.set_succeeded(result)

if __name__ == "__main__":
  rospy.init_node("object_recognition")
  object_recognition_server = ObjectRecognitionServer()
  rospy.spin()