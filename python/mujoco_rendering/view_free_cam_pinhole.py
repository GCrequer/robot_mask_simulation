import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import cv2


# Get the XML path from command line arguments
xml_path = "scene.xml"#str(sys.argv[1])


# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
cam.fixedcamid = 0
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom = 21) # maxgeom is the maximum number of meshes to be rendered
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# Example on how to set camera configuration
azimut_default = 90
elevation_default = -45
distance_default = 3
lookat_default = np.array([0.0, 0.0, 0])

cam.azimuth = azimut_default
cam.elevation = elevation_default
cam.distance = distance_default
cam.lookat = lookat_default

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
            cam.azimuth = azimut_default
            cam.elevation = elevation_default
            cam.distance = distance_default
            cam.lookat = lookat_default

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
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_MOVE_V, 0.0, -0.1, scene, cam)
    if act == glfw.PRESS and key == glfw.KEY_UP:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_MOVE_V, 0.0, 0.1, scene, cam)
    if act == glfw.PRESS and key == glfw.KEY_LEFT:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_MOVE_H, 0.1, 0.0, scene, cam)
    if act == glfw.PRESS and key == glfw.KEY_RIGHT:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_MOVE_H, -0.1, 0.0, scene, cam)


    #print camera configuration (help to initialize the view)
    if act == glfw.PRESS and key == glfw.KEY_P:
        print(f"cam.azimuth = {cam.azimuth}; cam.elevation = {cam.elevation}; cam.distance = {cam.distance}")
        print(f"cam.azimuth = {model.vis.global_.azimuth}; cam.elevation = {model.vis.global_.elevation}")
        print(f"cam.lookat = np.array([{cam.lookat[0]},{cam.lookat[1]},{cam.lookat[2]}]), fov = {model.vis.global_.fovy}")


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

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)
def scroll(window, xoffset, yoffset):
    model.vis.global_.fovy = model.vis.global_.fovy + yoffset


#Linking
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)


while not glfw.window_should_close(window):

    time_prev = data.time
    while (data.time - time_prev < 1.0/60.0):
        
        mj.mj_step(model, data)
    

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()