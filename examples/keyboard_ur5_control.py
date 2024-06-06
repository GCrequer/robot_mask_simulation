import cv2
from MyRobot import MyRobot
import glfw
import mujoco
import numpy as np


robot = MyRobot("scene_color.xml", outview=False, size=(1024,1024))
robot.init_outview()
robot.init_capture()



# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0
# Callback functions
def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS:
        if key == glfw.KEY_BACKSPACE:
            robot.out_cam["cam"].azimuth = 0
            robot.out_cam["cam"].elevation = -0.5
            robot.out_cam["cam"].distance = 1.5

            mujoco.mj_resetData(robot.model, robot.data)
            mujoco.mj_forward(robot.model, robot.data)
    

        if key == glfw.KEY_A: robot.data.ctrl[0] -= 0.1
        if key == glfw.KEY_W: robot.data.ctrl[1] += 0.1
        if key == glfw.KEY_Q: robot.data.ctrl[0] += 0.1
        if key == glfw.KEY_S: robot.data.ctrl[1] -= 0.1
        if key == glfw.KEY_E: robot.data.ctrl[2] += 0.1
        if key == glfw.KEY_D: robot.data.ctrl[2] -= 0.1
        if key == glfw.KEY_R: robot.data.ctrl[3] += 0.1
        if key == glfw.KEY_F: robot.data.ctrl[3] -= 0.1
        if key == glfw.KEY_T: robot.data.ctrl[4] += 0.1
        if key == glfw.KEY_G: robot.data.ctrl[4] -= 0.1
        if key == glfw.KEY_Y: robot.data.ctrl[5] += 0.1
        if key == glfw.KEY_H: robot.data.ctrl[5] -= 0.1
    


    #move camera up down left right using arrow keys
    if act == glfw.PRESS and key == glfw.KEY_DOWN:
        mujoco.mjv_moveCamera(robot.model, mujoco.mjtMouse.mjMOUSE_MOVE_V, 0.0, -0.1, robot.scene, robot.out_cam["cam"])
    if act == glfw.PRESS and key == glfw.KEY_UP:
        mujoco.mjv_moveCamera(robot.model, mujoco.mjtMouse.mjMOUSE_MOVE_V, 0.0, 0.1, robot.scene, robot.out_cam["cam"])
    if act == glfw.PRESS and key == glfw.KEY_LEFT:
        mujoco.mjv_moveCamera(robot.model, mujoco.mjtMouse.mjMOUSE_MOVE_H, 0.1, 0.0, robot.scene, robot.out_cam["cam"])
    if act == glfw.PRESS and key == glfw.KEY_RIGHT:
        mujoco.mjv_moveCamera(robot.model, mujoco.mjtMouse.mjMOUSE_MOVE_H, -0.1, 0.0, robot.scene, robot.out_cam["cam"])

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

    mujoco.mjv_moveCamera(robot.model, action, dx/height, dy/height, robot.scene, robot.out_cam["cam"])

def scroll(window, xoffset, yoffset):
    robot.out_cam["cam"].distance = robot.out_cam["cam"].distance + yoffset

#Linking
glfw.set_key_callback(robot.window, keyboard)
glfw.set_mouse_button_callback(robot.window, mouse_button)
glfw.set_cursor_pos_callback(robot.window, mouse_move)
glfw.set_scroll_callback(robot.window, scroll)




cv2.namedWindow("view1", cv2.WINDOW_NORMAL)

cubemap = np.zeros((robot.window_size[0]*3, robot.window_size[1]*4, 3), dtype=np.uint8)

# Main loop
while not glfw.window_should_close(robot.window):
    time_prev = robot.data.time
    while robot.data.time < time_prev + 0.01:
        mujoco.mj_step(robot.model, robot.data)
    robot.update_outview()
    robot.update_capture()
 

    front = np.flipud(robot.cams[2]["frame"])
    back = np.fliplr(robot.cams[1]["frame"])
    bottom = np.flipud(robot.cams[5]["frame"])
    top = np.flipud(robot.cams[0]["frame"])
    right = np.fliplr(np.rot90(robot.cams[4]["frame"], -1))
    left = np.rot90(np.flipud(robot.cams[3]["frame"]), 1)

    cubemap[0:robot.window_size[0], robot.window_size[1]:2*robot.window_size[1]] = top
    cubemap[robot.window_size[0]:2*robot.window_size[0], 0:robot.window_size[1]] = left
    cubemap[robot.window_size[0]:2*robot.window_size[0], robot.window_size[1]:2*robot.window_size[1]] = front
    cubemap[robot.window_size[0]:2*robot.window_size[0], 2*robot.window_size[1]:3*robot.window_size[1]] = right
    cubemap[robot.window_size[0]:2*robot.window_size[0], 3*robot.window_size[1]:4*robot.window_size[1]] = back
    cubemap[2*robot.window_size[0]:3*robot.window_size[0], robot.window_size[1]:2*robot.window_size[1]] = bottom

    cv2.imshow("view1", cubemap)
    
    glfw.poll_events()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cv2.destroyAllWindows()
glfw.terminate()