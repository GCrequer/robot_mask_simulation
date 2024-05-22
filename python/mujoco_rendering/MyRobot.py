import mujoco
import glfw
import numpy as np
import cv2

class MyRobot:
    def __init__(self, xml_file):
        self.model = mujoco.MjModel.from_xml_path(xml_file)
        self.data = mujoco.MjData(self.model)

        self.scene = mujoco.MjvScene(self.model, maxgeom = 100)
        self.opt = mujoco.MjvOption()
        mujoco.mjv_defaultOption(self.opt)

        self.view = False

    def init_capture(self, size=(256,256)):
        if not self.view:
            glfw.window_hint(glfw.VISIBLE, False)
            self.window_out = glfw.create_window(size[0], size[1], "outview", None, None)
            glfw.make_context_current(self.window_out)

        self.cams = []
        for i in range(len(self.model.cam_bodyid)) :
            c = mujoco.MjvCamera()
            c.type = 2
            c.fixedcamid = i
            self.cams.append(c)

        self.frames, self.contexts, self.viewports = [], [], []
        for i in range(len(self.cams)):
            self.frames.append(np.zeros((size[0],size[1],3), dtype=np.uint8))
            self.contexts.append(mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value))
            self.viewports.append(mujoco.MjrRect(0,0, size[0], size[1]))
 
    def init_outview(self, size=(256,256)):
        glfw.window_hint(glfw.VISIBLE, True)
        self.window_out = glfw.create_window(size[0], size[1], "outview", None, None)
        glfw.make_context_current(self.window_out)
        self.cam_out = mujoco.MjvCamera()

        self.frame_out = np.zeros((size[0],size[0],3))
        self.context_out = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
        self.viewport_out = mujoco.MjrRect(0,0, size[0], size[1])

        self.view = True        

    def render_outview(self):
        glfw.make_context_current(self.window_out)
        mujoco.mjv_updateScene(self.model, self.data, self.opt, None, self.cam_out, mujoco.mjtCatBit.mjCAT_ALL.value, self.scene)
        mujoco.mjr_render(self.viewport_out, self.scene, self.context_out)
        glfw.swap_buffers(self.window_out)

    def update_capture(self):
        for i in range(len(self.cams)):
            mujoco.mjv_updateScene(self.model, self.data, self.opt, None, self.cams[i], mujoco.mjtCatBit.mjCAT_ALL.value, self.scene)
            mujoco.mjr_render(self.viewports[i], self.scene, self.contexts[i])
            mujoco.mjr_readPixels(self.frames[i], None, self.viewports[i], self.contexts[i])


if __name__ == "__main__":
    robot = MyRobot("scene.xml")
    glfw.init()
    robot.init_outview((700, 500))
    robot.init_capture((512,512))

    # Callback
    def keyboard(window, key, scancode, act, mods):
        if act == glfw.PRESS:
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
    glfw.set_key_callback(robot.window_out, keyboard)


    cv2.namedWindow("equirectangular", cv2.WINDOW_NORMAL)
    # Main loop
    while not glfw.window_should_close(robot.window_out):
        time_prev = robot.data.time
        while robot.data.time < time_prev + 0.01:
            mujoco.mj_step(robot.model, robot.data)
        robot.render_outview()
        robot.update_capture()


        #convert cubemap to equirectangular
        glfw.poll_events()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    cv2.destroyAllWindows()
    glfw.terminate()
    

