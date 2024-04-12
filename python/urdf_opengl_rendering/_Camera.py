# Inspiration: Mirado Rajaomarosata

import numpy as np
from OpenGL.GL import *



class Camera:
    def __init__(self, display, EventHandler):
        self.display = display
        self.EventHandler = EventHandler

        # Pose initial
        self.R = np.array([[ 9.82935677e-01,  1.08210394e-03, -1.83946416e-01],[-1.07459820e-03,  9.99999413e-01,  1.40488693e-04],[ 1.83946460e-01,  5.95771386e-05,  9.82936263e-01]])      
        self.t = np.array([[0.0],[0.0],[-2.]])
    
    def handle_events(self):
        self.R, self.t = self.EventHandler.handle_events()


    def perspective_matrix(self, R=np.array([[ 9.82935677e-01,  1.08210394e-03, -1.83946416e-01],[-1.07459820e-03,  9.99999413e-01,  1.40488693e-04],[ 1.83946460e-01,  5.95771386e-05,  9.82]]), t=np.array([[0.0],[0.0],[-2.]])):
        fov = 90.
        aspect = self.display[0] / float(self.display[1])
        d = 2
        fx = d / (2* np.tan(np.radians(fov) / 2))
        fy = fx
        hx = 1
        hy = hx
        px = 0
        py = 0

        # Matrice des paramètres intrinsèques en coordonnées homogènes
        K_homogeneous = np.array([
            [fx/hx, 0, px, 0],
            [0, fy/hy, py, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)

        # Matrice des paramètres extrinsèques en coordonnées homogènes
        self.R = self.R @ R
        self.t = self.t + t
        Rt_homogeneous = np.vstack([np.hstack([self.R, self.t]), [0, 0, 0, 1]])

        # Matrice de projection en coordonnées homogènes
        P_homogeneous = K_homogeneous @ Rt_homogeneous
        
        # Calcul de P_gl à partir des paramètres extraits
        near = 0.1
        far = 100.0
        top = near * np.tan(np.radians(fov) / 2)
        bottom = -top
        right = top * aspect
        left = -right



        P_gl = np.array([
            [2 * near / (right - left), 0, (right + left) / (right - left), 0],
            [0, 2 * near / (top - bottom), (top + bottom) / (top - bottom), 0],
            [0, 0, -(far + near) / (far - near), -2 * far * near / (far - near)],
            [0, 0, -1, 0]
        ], dtype=np.float32)
                
        P_gl = P_gl @ P_homogeneous
        return P_gl
    

    def orthographic_matrix(self):
        left = -10.0
        right = 10.0
        bottom = -10.0
        top = 10.0

        a = 2.0 / (right - left)
        b = 2.0 / (top - bottom)
        c = -2.0 / (self.far - self.near)

        tx = -(right + left) / (right - left)
        ty = -(top + bottom) / (top - bottom)
        tz = -(self.far + self.near) / (self.far - self.near)

        O = np.array([
            [a, 0, 0, tx],
            [0, b, 0, ty],
            [0, 0, c, tz],
            [0, 0, 0, 1]
        ], dtype=np.float32)

        RT = np.eye(4, dtype=np.float32)
        RT[:3, :3] = self.R
        RT[:3, 3] = self.t.flatten()

        OT = np.dot(O, RT)

        return OT


    def perspective_projection(self, R=np.eye(3), t=np.array([[0.0],[0.0],[0.0]])):
        glMatrixMode(GL_PROJECTION)
        glLoadMatrixf(self.perspective_matrix(R, t).T)
        glMatrixMode(GL_MODELVIEW)

    
