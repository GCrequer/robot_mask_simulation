import glfw
import numpy as np
from MyRobot import MyRobot
import mujoco
from Tiles2EquiCU import *
import cupy as cp
import cv2

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

robot = MyRobot("scene.xml")
glfw.init()

dims = 512
robot.init_capture((dims,dims))

posy_gpu, negx_gpu, posx_gpu, negz_gpu, negy_gpu, posz_gpu, dst_gpu = allocate_gpu_memory(dims)

def ros_callback(data):
    l = [float(x) for x in data.data.split()]
    for i in range(len(l)):
        robot.data.ctrl[i] = l[i]
        robot.data.joint(i).qpos = l[i]

def main():
    rospy.init_node('mask_estimation', anonymous=True)
    rospy.Subscriber('/robot/joint_states', String, ros_callback)
    image_pub = rospy.Publisher('/mask_equirect', Image, queue_size=10)
    rate = rospy.Rate(10)


    while not rospy.is_shutdown():
        mujoco.mj_step(robot.model, robot.data)

        robot.update_capture()

        negx_gpu = cp.asarray(np.flipud(robot.frames[2]))
        posx_gpu = cp.asarray(np.fliplr(robot.frames[1]))
        negz_gpu = cp.asarray(np.fliplr(robot.frames[5]))
        posz_gpu = cp.asarray(np.fliplr(robot.frames[0]))
        posy_gpu = cp.asarray(np.fliplr(np.rot90(robot.frames[4], -1)))
        negy_gpu = cp.asarray(np.rot90(np.flipud(robot.frames[3]), 1))


        equi = cube2equi_cuda(posy_gpu, negx_gpu, posx_gpu, negz_gpu, negy_gpu, posz_gpu, dst_gpu, dims)
        gray_image = cv2.cvtColor(equi, cv2.COLOR_RGB2GRAY)
        image_pub.publish(CvBridge().cv2_to_imgmsg(gray_image, "mono8"))
        rate.sleep()



if __name__ == '__main__':
    main()
    rospy.spin()
    cv2.destroyAllWindows()
    glfw.terminate()