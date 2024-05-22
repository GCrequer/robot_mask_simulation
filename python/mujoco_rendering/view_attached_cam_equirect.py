import glfw
import cv2
from MyRobot import MyRobot
import mujoco
from Tiles2EquiCU import *
import cupy as cp
import numpy as np

robot = MyRobot("scene.xml")
glfw.init()
dims = 512
robot.init_outview((900, 700))
robot.init_capture((dims,dims))


# Callback
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0
def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS:
        if key == glfw.KEY_BACKSPACE:
            robot.cam_out.azimuth = 45
            robot.cam_out.elevation = 45
            robot.cam_out.distance = 1

            mujoco.mj_resetData(robot.model, robot.data)
            mujoco.mj_forward(robot.model, robot.data)
        
        if key == glfw.KEY_A: robot.data.ctrl[0] -= 0.1; robot.data.joint(0).qpos -= 0.1
        if key == glfw.KEY_W: robot.data.ctrl[1] += 0.1; robot.data.joint(1).qpos += 0.1
        if key == glfw.KEY_Q: robot.data.ctrl[0] += 0.1; robot.data.joint(0).qpos += 0.1
        if key == glfw.KEY_S: robot.data.ctrl[1] -= 0.1; robot.data.joint(1).qpos -= 0.1
        if key == glfw.KEY_E: robot.data.ctrl[2] += 0.1; robot.data.joint(2).qpos += 0.1
        if key == glfw.KEY_D: robot.data.ctrl[2] -= 0.1; robot.data.joint(2).qpos -= 0.1
        if key == glfw.KEY_R: robot.data.ctrl[3] += 0.1; robot.data.joint(3).qpos += 0.1
        if key == glfw.KEY_F: robot.data.ctrl[3] -= 0.1; robot.data.joint(3).qpos -= 0.1
        if key == glfw.KEY_T: robot.data.ctrl[4] += 0.1; robot.data.joint(4).qpos += 0.1
        if key == glfw.KEY_G: robot.data.ctrl[4] -= 0.1; robot.data.joint(4).qpos -= 0.1
        if key == glfw.KEY_Y: robot.data.ctrl[5] += 0.1; robot.data.joint(5).qpos += 0.1
        if key == glfw.KEY_H: robot.data.ctrl[5] -= 0.1; robot.data.joint(5).qpos -= 0.1
        
    


    #move camera up down left right using arrow keys
    if act == glfw.PRESS and key == glfw.KEY_DOWN:
        mujoco.mjv_moveCamera(robot.model, mujoco.mjtMouse.mjMOUSE_MOVE_V, 0.0, -0.1, robot.scene, robot.cam_out)
    if act == glfw.PRESS and key == glfw.KEY_UP:
        mujoco.mjv_moveCamera(robot.model, mujoco.mjtMouse.mjMOUSE_MOVE_V, 0.0, 0.1, robot.scene, robot.cam_out)
    if act == glfw.PRESS and key == glfw.KEY_LEFT:
        mujoco.mjv_moveCamera(robot.model, mujoco.mjtMouse.mjMOUSE_MOVE_H, 0.1, 0.0, robot.scene, robot.cam_out)
    if act == glfw.PRESS and key == glfw.KEY_RIGHT:
        mujoco.mjv_moveCamera(robot.model, mujoco.mjtMouse.mjMOUSE_MOVE_H, -0.1, 0.0, robot.scene, robot.cam_out)


    #print camera configuration (help to initialize the view)
    if act == glfw.PRESS and key == glfw.KEY_P:
        print(f"cam.azimuth = {robot.cam_out.azimuth}; cam.elevation = {robot.cam_out.elevation}; cam.distance = {robot.cam_out.distance}")
def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)
def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mujoco.mjtMouse.mjMOUSE_ZOOM

    mujoco.mjv_moveCamera(robot.model, action, dx/height, dy/height, robot.scene, robot.cam_out)
def scroll(window, xoffset, yoffset):
    robot.cam_out.distance = robot.cam_out.distance + yoffset
#Linking
glfw.set_key_callback(robot.window_out, keyboard)
glfw.set_mouse_button_callback(robot.window_out, mouse_button)
glfw.set_cursor_pos_callback(robot.window_out, mouse_move)
glfw.set_scroll_callback(robot.window_out, scroll)

posy_gpu, negx_gpu, posx_gpu, negz_gpu, negy_gpu, posz_gpu, dst_gpu = allocate_gpu_memory(dims)

cv2.namedWindow("equirectangular", cv2.WINDOW_NORMAL)
# Main loop
while True:
    mujoco.mj_step(robot.model, robot.data)

    robot.render_outview()
    robot.update_capture()

    negx_gpu = cp.asarray(np.fliplr(robot.frames[1]))
    posx_gpu = cp.asarray(np.fliplr(robot.frames[2]))
    negz_gpu = cp.asarray(np.fliplr(robot.frames[5]))
    posz_gpu = cp.asarray(np.fliplr(robot.frames[0]))
    negy_gpu = cp.asarray(np.fliplr(robot.frames[4]))
    posy_gpu = cp.asarray(np.rot90(np.flipup(robot.frames[3]), 1))


    equi = cube2equi_cuda(posy_gpu, negx_gpu, posx_gpu, negz_gpu, negy_gpu, posz_gpu, dst_gpu, dims)

    cv2.imshow("equirectangular", cv2.cvtColor(equi, cv2.COLOR_RGB2BGR))
    #convert cubemap to equirectangular
    glfw.poll_events()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
glfw.terminate()