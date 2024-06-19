import mujoco
import glfw
import numpy as np

class MyRobot:
    """
    Class for instantiating and capturing mujoco views from cameras, and rendering outview

    Args:
        xml_file (str): path to mujoco xml file
        outview (bool): whether to render outview

    Attributes:
        model (mujoco.MjModel): mujoco model
        data (mujoco.MjData): mujoco data
        scene (mujoco.MjvScene): mujoco scene
        opt (mujoco.MjvOption): mujoco option

        window (glfw.Window): window for view

        cams (list of dict): list of cameras {"cam", "frame", "context", "viewport", "index"}
        out_cam (dict): external camera {"cam", "frame", "context", "viewport", "active"}
            - cam (mujoco.MjvCamera): camera
            - frame (np.array): frame
            - context (mujoco.MjrContext): context
            - viewport (mujoco.MjrRect): viewport
            - index (int): index of camera

    Methods:
        init_capture: initialize capture
        update_capture: update capture

        init_outview: initialize outview
        update_outview: render outview
    """

    def __init__(self, xml_file, outview=False, size=(256,256)):
        glfw.init()


        self.model = mujoco.MjModel.from_xml_path(xml_file)
        self.data = mujoco.MjData(self.model)

        self.scene = mujoco.MjvScene(self.model, maxgeom = 100)
        self.opt = mujoco.MjvOption()
        mujoco.mjv_defaultOption(self.opt)

        self.window_size = size
        self.cams = []

        glfw.window_hint(glfw.VISIBLE, outview)
        self.window = glfw.create_window(self.window_size[0], self.window_size[1], "view", None, None)
        glfw.make_context_current(self.window)

        self._already_visible = outview

        if outview:
            self.init_outview()

    def init_capture(self):
        """
        Initialize capture : instantiate cameras
        """

        for i in range(len(self.model.cam_bodyid)) :
            cam = mujoco.MjvCamera()
            cam.type = 2
            cam.fixedcamid = i

            f = np.zeros((self.window_size[0],self.window_size[1],3), dtype=np.uint8)
            cont = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
            v = mujoco.MjrRect(0,0, self.window_size[0], self.window_size[1])

            self.cams.append({"cam"     :cam,
                              "frame"   :f,
                              "context" :cont,
                              "viewport":v,
                              "index"   :i})
 
    def init_outview(self):
        """
        Initialize outview : instantiate external camera
        """

        if not self._already_visible:
            glfw.window_hint(glfw.VISIBLE, True)
            self.window = glfw.create_window(self.window_size[0], self.window_size[1], "view", None, None)
            glfw.make_context_current(self.window)
            self._already_visible = True

        self.out_cam = {"active"    :True,
                        "cam"       :mujoco.MjvCamera(),
                        "frame"     :np.zeros((self.window_size[0],self.window_size[1],3), dtype=np.uint8),
                        "context"   :mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value),
                        "viewport"  :mujoco.MjrRect(0,0, self.window_size[0], self.window_size[1]),
                        "index"     :0}

        self.button_left = False
        self.button_middle = False
        self.button_right = False
        self.lastx = 0
        self.lasty = 0

                #Linking
        glfw.set_key_callback(self.window, self.keyboard)
        glfw.set_mouse_button_callback(self.window, self.mouse_button)
        glfw.set_cursor_pos_callback(self.window, self.mouse_move)
        glfw.set_scroll_callback(self.window, self.scroll)

            


        # Callback functions
    def keyboard(self, window, key, scancode, act, mods):
        if act == glfw.PRESS:
            if key == glfw.KEY_BACKSPACE:
                self.out_cam["cam"].azimuth = 0
                self.out_cam["cam"].elevation = -0.5
                self.out_cam["cam"].distance = 1.5

                mujoco.mj_resetData(self.model, self.data)
                mujoco.mj_forward(self.model, self.data)
        

            if key == glfw.KEY_A: self.data.ctrl[0] -= 0.1
            if key == glfw.KEY_W: self.data.ctrl[1] += 0.1
            if key == glfw.KEY_Q: self.data.ctrl[0] += 0.1
            if key == glfw.KEY_S: self.data.ctrl[1] -= 0.1
            if key == glfw.KEY_E: self.data.ctrl[2] += 0.1
            if key == glfw.KEY_D: self.data.ctrl[2] -= 0.1
            if key == glfw.KEY_R: self.data.ctrl[3] += 0.1
            if key == glfw.KEY_F: self.data.ctrl[3] -= 0.1
            if key == glfw.KEY_T: self.data.ctrl[4] += 0.1
            if key == glfw.KEY_G: self.data.ctrl[4] -= 0.1
            if key == glfw.KEY_Y: self.data.ctrl[5] += 0.1
            if key == glfw.KEY_H: self.data.ctrl[5] -= 0.1
        


        #move camera up down left right using arrow keys
        if act == glfw.PRESS and key == glfw.KEY_DOWN:
            mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_MOVE_V, 0.0, -0.1, self.scene, self.out_cam["cam"])
        if act == glfw.PRESS and key == glfw.KEY_UP:
            mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_MOVE_V, 0.0, 0.1, self.scene, self.out_cam["cam"])
        if act == glfw.PRESS and key == glfw.KEY_LEFT:
            mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_MOVE_H, 0.1, 0.0, self.scene, self.out_cam["cam"])
        if act == glfw.PRESS and key == glfw.KEY_RIGHT:
            mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_MOVE_H, -0.1, 0.0, self.scene, self.out_cam["cam"])
    def mouse_button(self, window, button, act, mods):


        self.button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        self.button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        self.button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

        # update mouse position
        glfw.get_cursor_pos(window)
    def mouse_move(self, window, xpos, ypos):

        dx = xpos - self.lastx
        dy = ypos - self.lasty
        self.lastx = xpos
        self.lasty = ypos

        # no buttons down: nothing to do
        if (not self.button_left) and (not self.button_middle) and (not self.button_right):
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
        if self.button_right:
            if mod_shift:
                action = mujoco.mjtMouse.mjMOUSE_MOVE_H
            else:
                action = mujoco.mjtMouse.mjMOUSE_MOVE_V
        elif self.button_left:
            if mod_shift:
                action = mujoco.mjtMouse.mjMOUSE_ROTATE_H
            else:
                action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM

        mujoco.mjv_moveCamera(self.model, action, dx/height, dy/height, self.scene, self.out_cam["cam"])
    def scroll(self, window, xoffset, yoffset):
        self.out_cam["cam"].distance = self.out_cam["cam"].distance + yoffset


    def update_outview(self, save=False):
        """
        Render outview : render external camera
        Args:
            save (bool): whether to save frame in out_cam
        """

        glfw.make_context_current(self.window)
        mujoco.mjv_updateScene(self.model, self.data, self.opt, None, self.out_cam["cam"], mujoco.mjtCatBit.mjCAT_ALL.value, self.scene)
        mujoco.mjr_render(self.out_cam["viewport"], self.scene, self.out_cam["context"])

        if save:
            mujoco.mjr_readPixels(self.out_cam["frame"], None, self.out_cam["viewport"], self.out_cam["context"])
        glfw.swap_buffers(self.window)


    def update_capture(self):
        """
        Update capture : render cameras and read pixels
        """
        for c in self.cams:
            mujoco.mjv_updateScene(self.model, self.data, self.opt, None, c["cam"], mujoco.mjtCatBit.mjCAT_ALL.value, self.scene)
            mujoco.mjr_render(c["viewport"], self.scene, c["context"])
            mujoco.mjr_readPixels(c["frame"], None, c["viewport"], c["context"])