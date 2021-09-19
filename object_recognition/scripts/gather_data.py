import rospy 
from sensor_msgs.msg import Image
from roi import ROI
from cv_bridge import CvBridge
import numpy as np
import cv2
import sys
import os
import rospkg
import csv

## Script for gathering data, using an RGB-D camera. 
## Using the ROI class to get region of interest of the object
## Saves the full image containing the object, and saves the crop values in a csv file. 
## Usage: python gather_data.py object_name 
## Will save the data in the object_recognition/data/training/images/object_name path


CAMERA_TOPIC = "/camera/color/image_raw"

class ImageCropper(object):

  # Initializations
  def __init__(self):
    self.data = []
    self.cv_bridge = CvBridge()
    self.roi = ROI()
    self.count = 0
    self.data = []
    self.labels = ["barcelona_box", "water_bottle", "brown_cup", "blue_box", "white_ball", "blue_cup", "infinite_money_bowl", "pilot", "cloth", "soap_dispenser", "plant_pot", "water_sprayer"] # Labels of the possible objects

    if len(sys.argv) != 2: # Check if the current name is given for data gathering
      print(f"Give name of object")
      exit()

    self.current_label = str(sys.argv[1])
    rospack = rospkg.RosPack()
    self.path = rospack.get_path("object_recognition")
    self.path_training = os.path.join(os.path.join(rospack.get_path("object_recognition"), "data/training/images"), self.current_label)

    # If path doesn't exist, create it 
    if not os.path.exists(self.path_training):
      os.makedirs(self.path_training)

    self.label = [0] * len(self.labels)
    # Use a timer callback to request a new image 
    self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

  # Timer callback function
  def timer_callback(self, event):
    # Get the sensor_msgs Image
    image_msg = rospy.wait_for_message(CAMERA_TOPIC, Image)
    # Convert sensor_msgs Image to openCv image
    image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    original_image = image.copy() # Create a copy of the original (used for saving)

    rois = self.roi.get_rois("camera_link") # Get the region of interests (roi)

    if len(rois) > 1:
      color = (0, 0, 255)
    else:
      color = (0, 255, 0)
    for roi in rois:
      left = roi.left
      right = roi.right 
      top = roi.top
      bottom = roi.bottom 
      # Draw the roi on the image
      cv2.rectangle(image, (left, top), (right, bottom), color, 1)
    
    # Show the image with the ROI drawn on it 
    cv2.imshow("Image", image)
    key = cv2.waitKey(1) & 0xFF

    # If 'q' is pressed, save the data and quit
    if key == ord('q'):

      with open(os.path.join(self.path_training, "data.csv"), "w") as f:
        writer = csv.writer(f)
        writer.writerows(self.data)

      rospy.signal_shutdown("done")
      return

    # Save the original image (with no drawing on it) in the correct label folder and append the roi data
    # Only save if there is 1 ROI found, else do not save image
    if key == ord(' ') and len(rois) == 1:
      label = [0] * len(self.labels)
      index = self.labels.index(self.current_label)
      label[index] = 1

      image_path = os.path.join(os.path.join("data/training/images", self.current_label), str(self.count) + ".jpg")
      cv2.imwrite(os.path.join(self.path_training, str(self.count) + ".jpg"), original_image)
      self.data.append([image_path, left, right, top, bottom]) 
      self.count += 1
      print(f"Images {self.count}")

if __name__ == "__main__":
  rospy.init_node("gather_data")
  image_cropper = ImageCropper()
  rospy.spin()