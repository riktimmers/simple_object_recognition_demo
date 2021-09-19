import cv2
import csv
import numpy as np 
import os 
import rospkg 
import sys

if len(sys.argv) != 2:
  exit()

object_name = str(sys.argv[1])

rospack = rospkg.RosPack()
path = rospack.get_path("object_recognition")
data = []

with open(os.path.join(path, "data/training/"+ object_name + "/data.csv"), "rb") as f:
  reader = csv.reader(f)
  
  for row in reader:
    image_path = row[0]
    left = row[1]
    right = row[2]
    top = row[3]
    bottom = row[4]

    image = cv2.imread(path + image_path)
    cv2.imshow("image", image)
    cv2.waitKey(0)
