import numpy as np
np.int = np.int32
np.bool = np.bool_

from urchin import URDF
from OpenGL.GL import *



robot = URDF.load("../urdf/ur5.urdf", lazy_load_meshes=True)

