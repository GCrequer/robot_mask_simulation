#!/usr/bin/env python3

import os
from xml_mujoco_modifier import *
from MyRobot import *
from Tiles2EquiCU import *
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
import rospy
import cv2
import tkinter as tk

import threading


config_path = os.path.dirname(os.path.abspath(__file__)).split('src')[0] + 'config/'

robot_calib_name = create_calibration_robot_file(config_path + "ur5e.xml")
scene_calib_name = create_scene_file(config_path + "scene.xml", robot_calib_name)

img = np.zeros((1024,2048,3),np.uint8)
dims = 512
robot = MyRobot(scene_calib_name, size=(dims, dims))
robot.init_outview()
robot.init_capture()

def update_joint(joint_index, value):
    robot.data.ctrl[joint_index] = value
    robot.data.joint(joint_index).qpos = value

def create_slider(frame, label, min_val, max_val, init_val, joint_index):
    tk.Label(frame, text=label).pack()
    slider = tk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL, resolution=0.01, length=400)
    slider.set(init_val)
    slider.pack()
    slider.bind("<Motion>", lambda event: update_joint(joint_index, slider.get()))
    return slider

def tk_ui():
    root = tk.Tk()
    root.title("Camera Pose Sliders")
    frame = tk.Frame(root)
    frame.pack()
    for i in range(6, 9):
        create_slider(frame, f'Slider {i}', -0.3, 0.3, 0, i)
    for i in range(9, 12):
        create_slider(frame, f'Hinge {i}', -np.pi, np.pi, 0, i)
    root.mainloop()

# Start Tkinter UI in a separate thread
tk_thread = threading.Thread(target=tk_ui)
tk_thread.start()


front_gpu, back_gpu, top_gpu, bottom_gpu, left_gpu, right_gpu, dst_gpu = allocate_gpu_memory(dims)
bridge = CvBridge()

def ros_callback_joint(data):
    l = [data.position[i] for i in [2,1,0,3,4,5]]
    for i in range(len(l)):
        robot.data.ctrl[i] = l[i]
        robot.data.joint(i).qpos = l[i]

def ros_callback_image(data):
    global img
    img = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

def main():
    rospy.init_node('camera_superposition', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, ros_callback_joint)
    rospy.Subscriber('/kodak/image_equi', Image, ros_callback_image)
    image_pub = rospy.Publisher('/robot_shape', Image, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        mujoco.mj_step(robot.model, robot.data)

        robot.update_outview()
        robot.update_capture()

        front_gpu = cp.asarray(np.flipud(robot.cams[0]["frame"]))
        back_gpu = cp.asarray(np.flipud(robot.cams[1]["frame"]))
        top_gpu = cp.asarray(np.rot90(np.flipud(robot.cams[2]["frame"]),1))
        bottom_gpu = cp.asarray(np.flipud(np.rot90(robot.cams[3]["frame"],1)))
        left_gpu = cp.asarray(np.fliplr(robot.cams[4]["frame"]))
        right_gpu = cp.asarray(np.flipud(robot.cams[5]["frame"]))

        equi = cube2equi_cuda(front_gpu, back_gpu, top_gpu, bottom_gpu, left_gpu, right_gpu, dst_gpu, dims)
        gray_image = cv2.cvtColor(equi, cv2.COLOR_RGB2GRAY)

        #get the contours in green
        ret,thresh = cv2.threshold(gray_image,127,255,0)
        contours,hierarchy = cv2.findContours(thresh, 1, 2)

        cv2.drawContours(img, contours, -1, (0,255,0), 3)

        image_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        image_pub.publish(image_msg)
        rate.sleep()
        glfw.poll_events()
    
    pos = robot.data.ctrl[6:9]

    euler = robot.data.ctrl[9:12]

    robot_calibrated_name = create_calibrated_robot_file(config_path + "ur5e.xml", pos, euler)
    scene_calibrated_name = create_scene_file(config_path + "scene.xml", robot_calibrated_name)

if __name__ == '__main__':
    main()
    rospy.spin()
    glfw.terminate()
    cv2.destroyAllWindows()

