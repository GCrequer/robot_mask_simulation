import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import sys
import cv2


# Get the XML path from command line arguments
xml_path = "scene.xml"#str(sys.argv[1])


# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data

cam0 = mj.MjvCamera()
default_distance, default_azimuth, default_elevation = 3, 90, -20
cam0.distance = default_distance
cam0.azimuth = default_azimuth
cam0.elevation = default_elevation

cam1 = mj.MjvCamera()
cam1.type = mj.mjtCamera.mjCAMERA_FIXED.value
cam1.fixedcamid = 0

cam2 = mj.MjvCamera()
cam2.type = mj.mjtCamera.mjCAMERA_FIXED.value
cam2.fixedcamid = 1


opt = mj.MjvOption()                        # visualization options


# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()

window0 = glfw.create_window(1200, 900, "world", None, None)
glfw.make_context_current(window0)

glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom = 100) # maxgeom is the maximum number of meshes to be rendered
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)


# install GLFW mouse and keyboard callbacks
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
            cam0.azimuth = default_azimuth
            cam0.elevation = default_elevation
            cam0.distance = default_distance

            mj.mj_resetData(model, data)
            mj.mj_forward(model, data)
    

        if key == glfw.KEY_A: data.ctrl[0] -= 0.1
        if key == glfw.KEY_W: data.ctrl[1] += 0.1
        if key == glfw.KEY_Q: data.ctrl[0] += 0.1
        if key == glfw.KEY_S: data.ctrl[1] -= 0.1
        if key == glfw.KEY_E: data.ctrl[2] += 0.1
        if key == glfw.KEY_D: data.ctrl[2] -= 0.1
        if key == glfw.KEY_R: data.ctrl[3] += 0.1
        if key == glfw.KEY_F: data.ctrl[3] -= 0.1
        if key == glfw.KEY_T: data.ctrl[4] += 0.1
        if key == glfw.KEY_G: data.ctrl[4] -= 0.1
        if key == glfw.KEY_Y: data.ctrl[5] += 0.1
        if key == glfw.KEY_H: data.ctrl[5] -= 0.1

    


    #move camera up down left right using arrow keys
    if act == glfw.PRESS and key == glfw.KEY_DOWN:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_MOVE_V, 0.0, -0.1, scene, cam0)
    if act == glfw.PRESS and key == glfw.KEY_UP:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_MOVE_V, 0.0, 0.1, scene, cam0)
    if act == glfw.PRESS and key == glfw.KEY_LEFT:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_MOVE_H, 0.1, 0.0, scene, cam0)
    if act == glfw.PRESS and key == glfw.KEY_RIGHT:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_MOVE_H, -0.1, 0.0, scene, cam0)


    #print camera configuration (help to initialize the view)
    if act == glfw.PRESS and key == glfw.KEY_P:
        print(f"cam.azimuth = {cam0.azimuth}; cam.elevation = {cam0.elevation}; cam.distance = {cam0.distance}")


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
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height, dy/height, scene, cam0)

def scroll(window, xoffset, yoffset):
    cam0.distance = cam0.distance + yoffset



#Linking
glfw.set_key_callback(window0, keyboard)
glfw.set_mouse_button_callback(window0, mouse_button)
glfw.set_cursor_pos_callback(window0, mouse_move)
glfw.set_scroll_callback(window0, scroll)


cv2.namedWindow("cam1", cv2.WINDOW_NORMAL)
cv2.namedWindow("cam2", cv2.WINDOW_NORMAL)

frame1 = np.zeros((900, 1200, 3), dtype=np.uint8)
frame2 = np.zeros((900, 1200, 3), dtype=np.uint8)

while not glfw.window_should_close(window0):

    time_prev = data.time
    while (data.time - time_prev < 1.0/60.0):
        mj.mj_forward(model, data)
        mj.mj_step(model, data)
    

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(window0)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam0,  mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    glfw.swap_buffers(window0)

    # Get the image from cam1
    mj.mjv_updateScene(model, data, opt, None, cam1,  mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    mj.mjr_readPixels(frame1, None, viewport, context)
    cv2.cvtColor(frame1, cv2.COLOR_RGB2BGR, frame1)
    cv2.imshow("cam1", frame1)

    # Get the image from cam2
    mj.mjv_updateScene(model, data, opt, None, cam2,  mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    mj.mjr_readPixels(frame2, None, viewport, context)
    cv2.cvtColor(frame2, cv2.COLOR_RGB2BGR, frame2)
    cv2.imshow("cam2", frame2)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

glfw.terminate()
