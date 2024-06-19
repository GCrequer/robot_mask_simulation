#!/usr/bin/env python3

import glfw
import numpy as np
from MyRobot import MyRobot
import mujoco
from Tiles2EquiCU import *
import cupy as cp
import cv2

import rospy
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import os

#scene and all the config in catkin_ws/config
package_path = os.path.dirname(os.path.abspath(__file__)).split('src')[0]
scene_file = package_path + '/config/scene_ur5e_calibrated.xml'

dims = 512
robot = MyRobot(scene_file, size=(dims, dims))
robot.init_capture()

front_gpu, back_gpu, top_gpu, bottom_gpu, left_gpu, right_gpu, dst_gpu = allocate_gpu_memory(dims)
bridge = CvBridge()
def ros_callback(data):
    l = data.position
    for i in range(len(l)):
        robot.data.ctrl[i] = l[i]
        robot.data.joint(i).qpos = l[i]

def main():
    rospy.init_node('mask_estimation', anonymous=True)
    rospy.Subscriber('/robot/joint_states', JointState, ros_callback)
    image_pub = rospy.Publisher('/mask_equirect', Image, queue_size=10)
    rate = rospy.Rate(10)


    while not rospy.is_shutdown():
        mujoco.mj_step(robot.model, robot.data)

        robot.update_capture()

        front_gpu = cp.asarray(np.flipud(robot.cams[0]["frame"]))
        back_gpu = cp.asarray(np.flipud(robot.cams[1]["frame"]))
        top_gpu = cp.asarray(np.rot90(np.flipud(robot.cams[2]["frame"]),1))
        bottom_gpu = cp.asarray(np.flipud(np.rot90(robot.cams[3]["frame"],1)))
        left_gpu = cp.asarray(np.fliplr(robot.cams[4]["frame"]))
        right_gpu = cp.asarray(np.flipud(robot.cams[5]["frame"]))

        equi = cube2equi_cuda(front_gpu, back_gpu, top_gpu, bottom_gpu, left_gpu, right_gpu, dst_gpu, dims)
        image_msg = bridge.cv2_to_imgmsg(equi, encoding="bgr8")
        image_pub.publish(image_msg)
        rate.sleep()

        glfw.poll_events()




if __name__ == '__main__':
    main()
    rospy.spin()
    cv2.destroyAllWindows()
    glfw.terminate()
