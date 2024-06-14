#!/usr/bin/env python3

import os
from xml_mujoco_modifier import *
from MyRobot import *
from Tiles2EquiCU import *
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
import rospy
import cv2

import threading


config_path = os.path.dirname(os.path.abspath(__file__)).split('src')[0] + '/config/'

robot_calib_name = create_calibration_robot_file(config_path + "ur5e.xml")
scene_calib_name = create_scene_file(config_path + "scene.xml", robot_calib_name)


dims = 512
robot = MyRobot(scene_calib_name, size=(dims, dims))
robot.init_capture()

robot_lock = threading.Lock()
def opencv_ui():
    cv2.namedWindow('camera_pose')
    # position
    for i in range(6, 9):
        cv2.createTrackbar('Joint'+str(i),'camera_pose',int((robot.data.ctrl[i]+1)*255/2),255,lambda value, i=i: update_joint(i, (value*2/255)-1))
    # rotation
    for i in range(9, 12):
        cv2.createTrackbar('Joint'+str(i),'camera_pose',int((robot.data.ctrl[i]+np.pi)*255/(2*np.pi)),255,lambda value, i=i: update_joint(i, (value*2*np.pi/255)-np.pi))

    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

def update_joint(joint_index, value):
    with robot_lock:
        robot.data.ctrl[joint_index] = value
        robot.data.joint(joint_index).qpos = value

# Create a separate thread for the OpenCV UI
opencv_thread = threading.Thread(target=opencv_ui)
opencv_thread.start()

front_gpu, back_gpu, top_gpu, bottom_gpu, left_gpu, right_gpu, dst_gpu = allocate_gpu_memory(dims)
bridge = CvBridge()

def ros_callback_joint(data):
    l = data.position
    with robot_lock:
        for i in range(len(l)):
            robot.data.ctrl[i] = l[i]
            robot.data.joint(i).qpos = l[i]

def ros_callback_image(data):
    global img
    img = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

def main():
    rospy.init_node('camera_superposition', anonymous=True)
    rospy.Subscriber('/robot/joint_states', JointState, ros_callback_joint)
    rospy.Subscriber('/kodak/image_equi', Image, ros_callback_image)
    image_pub = rospy.Publisher('/robot_shape', Image, queue_size=10)
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
        gray_image = cv2.cvtColor(equi, cv2.COLOR_RGB2GRAY)

        #get the contours in green
        ret,thresh = cv2.threshold(gray_image,127,255,0)
        contours,hierarchy = cv2.findContours(thresh, 1, 2)

        cv2.drawContours(img, contours, -1, (0,255,0), 3)

        image_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        image_pub.publish(image_msg)
        rate.sleep()
    
    pos = robot.data.ctrl[6:9]
    euler = robot.data.ctrl[9:12]

    robot_calibrated_name = create_calibrated_robot_file(config_path + "ur5e.xml", pos, euler)
    scene_calibrated_name = create_scene_file(config_path + "scene.xml", robot_calibrated_name)

if __name__ == '__main__':
    main()
    rospy.spin()
    glfw.terminate()
    cv2.destroyAllWindows()

