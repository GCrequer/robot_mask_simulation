#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tkinter as tk

class JointStatePublisher:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Joint State Publisher")

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.joint_positions = [0.0] * len(self.joint_names)
        
        self.sliders = []
        for i, joint_name in enumerate(self.joint_names):
            label = tk.Label(self.root, text=joint_name)
            label.pack()
            slider = tk.Scale(self.root, from_=-3.14, to=3.14, resolution=0.01, orient=tk.HORIZONTAL, command=lambda val, idx=i: self.update_joint_position(val, idx))
            slider.pack()
            self.sliders.append(slider)

        self.publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rospy.init_node('joint_state_publisher', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

    def update_joint_position(self, val, idx):
        self.joint_positions[idx] = float(val)

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names
        joint_state.position = self.joint_positions

        self.publisher.publish(joint_state)
        self.root.after(100, self.publish_joint_states)  # Schedule next call after 100ms

    def run(self):
        self.root.after(0, self.publish_joint_states)
        self.root.mainloop()

if __name__ == '__main__':
    try:
        jsp = JointStatePublisher()
        jsp.run()
    except rospy.ROSInterruptException:
        pass
