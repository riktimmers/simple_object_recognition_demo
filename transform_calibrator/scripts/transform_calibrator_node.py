#! /usr/bin/python3
import rospy
import tf2_ros
import tf_conversions 
from geometry_msgs.msg import TransformStamped
from tkinter import Tk, Frame, Label, HORIZONTAL, Scale, Text, END, Button
from math import pi
import rospkg 
from os.path import join, exists
from os import mkdir

class TransformCalibrator(object):

  def __init__(self):
    self.parent_link = "base_link"
    self.child_link = "camera_link"
    self.min_x = -1
    self.max_x = 1
    self.min_y = -1
    self.max_y = 1
    self.min_z = -1
    self.max_z = 1
    self.tf_broadcaster = tf2_ros.TransformBroadcaster()
    self.gui = Tk()
    self.gui.title("Transform Calibrator")

    self.ros_pack = rospkg.RosPack()

    self.window = Frame(self.gui, width = 600, height= 500).pack()

    slider_length = 300
        
    self.roll_scale = Scale(self.gui, resolution=0.01, length = slider_length, from_ = -pi, to_ = pi, orient = HORIZONTAL)
    self.roll_scale.place(x = 100, y = 10)
    self.roll_label = Label(self.gui, text = "Roll (radians):").place(x = 10, y = 30)

    self.pitch_scale = Scale(self.gui, resolution=0.01, length = slider_length, from_ = -pi, to_ = pi, orient = HORIZONTAL)
    self.pitch_scale.place(x = 100, y = 50)
    self.pitch_label = Label(self.gui, text = "Pitch (radians):").place(x = 10, y = 70)

    self.yaw_scale = Scale(self.gui, resolution=0.01, length = slider_length, from_ = -pi, to_ = pi, orient = HORIZONTAL)
    self.yaw_scale.place(x = 100, y = 90)
    self.yaw_label = Label(self.gui, text = "Yaw (radians):").place(x = 10, y = 110)
    
    self.x_scale = Scale(self.gui, resolution=0.01, length = slider_length, from_ = self.min_x, to_ = self.max_x, orient = HORIZONTAL)
    self.x_scale.place(x = 100, y = 130)
    self.x_label = Label(self.gui, text = "X (meters):").place(x = 10, y = 150)
    
    self.y_scale = Scale(self.gui, resolution=0.01, length = slider_length, from_ = self.min_y, to_ = self.max_y, orient = HORIZONTAL)
    self.y_scale.place(x = 100, y = 170)
    self.y_label = Label(self.gui, text = "Y (meters):").place(x = 10, y = 190)
    
    self.z_scale = Scale(self.gui, resolution=0.01, length = slider_length, from_ = self.min_z, to_ = self.max_z, orient = HORIZONTAL)
    self.z_scale.place(x = 100, y = 210)
    self.z_label = Label(self.gui, text = "Z (meters):").place(x = 10, y = 230)

    self.button = Button(self.gui, text="Save", command = self.save)
    self.button.place(x=300, y = 260)
    
    self.broadcast_transform()
    self.gui.mainloop()
    
  def save(self):
    x = self.x_scale.get()
    y = self.y_scale.get()
    z = self.z_scale.get()
    roll = self.roll_scale.get()
    pitch = self.pitch_scale.get()
    yaw = self.yaw_scale.get()

    launch_file = f'<launch> \n' + \
                  f'  <node pkg="tf" type="static_transform_publisher" name="static_broadcaster" args="{x} {y} {z} {yaw} {pitch} {roll} {self.parent_link} {self.child_link} 100" /> \n' + \
                  f'</launch>'


    package_path = self.ros_pack.get_path("transform_calibrator")
    launch_path = join(package_path, "launch")

    if not exists(launch_path):
      mkdir(launch_path)

    with open(join(launch_path, "static_transform.launch"), "w") as f:
      f.write(launch_file)
    exit()

  def broadcast_transform(self):
    quaternion = tf_conversions.transformations.quaternion_from_euler(self.roll_scale.get(), self.pitch_scale.get(), self.yaw_scale.get())

    transform_stamped_msg = TransformStamped()
    transform_stamped_msg.header.stamp = rospy.Time.now()
    transform_stamped_msg.header.frame_id = self.parent_link
    transform_stamped_msg.child_frame_id = self.child_link

    transform_stamped_msg.transform.translation.x = self.x_scale.get()
    transform_stamped_msg.transform.translation.y = self.y_scale.get()
    transform_stamped_msg.transform.translation.z = self.z_scale.get()
    transform_stamped_msg.transform.rotation.x = quaternion[0]
    transform_stamped_msg.transform.rotation.y = quaternion[1]
    transform_stamped_msg.transform.rotation.z = quaternion[2]
    transform_stamped_msg.transform.rotation.w = quaternion[3]

    self.tf_broadcaster.sendTransform(transform_stamped_msg)
    self.gui.after(100, self.broadcast_transform)


if __name__ == "__main__":
  rospy.init_node("transform_calibrator_node")
  transform_calibrator = TransformCalibrator()
  rospy.spin()
