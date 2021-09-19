#!/usr/bin/python3
import rospy 
from object_recognition.msg import ObjectRecognitionGoal, ObjectRecognitionAction
import actionlib


rospy.init_node("object_recognition_test")

server = actionlib.SimpleActionClient("/object_recognition", ObjectRecognitionAction)

print("Waiting for object recognition server")
server.wait_for_server()
print("Connected to object recognition server")

object_recognition_goal = ObjectRecognitionGoal()

while not rospy.is_shutdown():
  server.send_goal_and_wait(object_recognition_goal)
  state = server.get_state()

  if state == actionlib.GoalStatus.SUCCEEDED:
    result = server.get_result()
  else:
    print("Object regnition failed")
  

