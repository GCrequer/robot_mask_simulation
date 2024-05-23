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
import rospkg

rospack = rospkg.RosPack()
package_path = rospack.get_path('equirect_mask_package')
scene_file = package_path + '/config/scene2.xml'

robot = MyRobot(scene_file)
glfw.init()

dims = 512
robot.init_capture((dims,dims))

posy_gpu, negx_gpu, posx_gpu, negz_gpu, negy_gpu, posz_gpu, dst_gpu = allocate_gpu_memory(dims)
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

        posx_gpu = cp.asarray(np.flipud(robot.frames[2]))
        negx_gpu = cp.asarray(np.fliplr(robot.frames[1]))
        negz_gpu = cp.asarray(np.flipud(robot.frames[5]))
        posz_gpu = cp.asarray(np.flipud(robot.frames[0]))
        negy_gpu = cp.asarray(np.fliplr(np.rot90(robot.frames[4], -1)))
        posy_gpu = cp.asarray(np.rot90(np.flipud(robot.frames[3]), 1))

        equi = cube2equi_cuda(posy_gpu, negx_gpu, posx_gpu, negz_gpu, negy_gpu, posz_gpu, dst_gpu, dims)
        gray_image = cv2.cvtColor(equi, cv2.COLOR_RGB2GRAY)
        image_msg = bridge.cv2_to_imgmsg(gray_image, encoding="mono8")
        image_pub.publish(image_msg)
        rate.sleep()



if __name__ == '__main__':
    main()
    rospy.spin()
    cv2.destroyAllWindows()
    glfw.terminate()
