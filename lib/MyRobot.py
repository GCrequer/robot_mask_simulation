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